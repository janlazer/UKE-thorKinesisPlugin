#include "BDCStage.h"

#include <QDebug>
#include <chrono>
#include <thread>
#include <algorithm>

// Kinesis C API
#include "Thorlabs.MotionControl.Benchtop.DCServo.h"

// ---- BDCStage: enable only if required (using status bits) ----

static bool isChannelEnabled(uint32_t bits)
{
    return (bits & 0x80000000u) != 0;
}

static void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static int channelIndex(unsigned channel)
{
    return (channel == 2) ? 1 : 0;
}

BDCStage::BDCStage(const std::string& baseSerial)
    : m_serial(baseSerial)
{
}

BDCStage::~BDCStage()
{
    close();
}

void BDCStage::logErr(const char* fn, const std::string& serial, short err, unsigned ch)
{
    if (ch != 0)
        qDebug() << "BDCStage -" << fn << "serial=" << serial.c_str() << "ch=" << ch << "err=" << err;
    else
        qDebug() << "BDCStage -" << fn << "serial=" << serial.c_str() << "err=" << err;
}

bool BDCStage::okOrLog(const char* fn, const std::string& serial, short err, unsigned ch, short* errOut)
{
    if (errOut) *errOut = err;
    if (err == 0) return true;
    logErr(fn, serial, err, ch);
    return false;
}

bool BDCStage::open(const std::string& baseSerial, bool home, short* errOut)
{
    if (!baseSerial.empty())
    {
        m_serial = baseSerial;
        m_serialCStr = m_serial.c_str();
    }
    else
    {
        qDebug() << "BDCStage::open ERROR no serial provided";
        if (errOut) *errOut = -1;
        return false;
    }

    qDebug() << "BDCStage::open serial=" << m_serialCStr << "home=" << home;

    if (m_isOpen)
    {
        qDebug() << "BDCStage::open already open serial=" << m_serialCStr;
        if (errOut) *errOut = 0;
        return false;
    }

    short err = BDC_Open(m_serialCStr);
    if (!okOrLog("BDC_Open", m_serial, err, 0, errOut))
        return false;

    for (short ch = 1; ch <= 2; ++ch)
    {
        const bool loaded = BDC_LoadSettings(m_serialCStr, ch);
        qDebug() << "BDCStage::open BDC_LoadSettings ch=" << ch << "loaded=" << loaded;

        err = BDC_RequestSettings(m_serialCStr, ch);
        qDebug() << "BDCStage::open BDC_RequestSettings ch=" << ch << "err = " << err;
    }
    sleepMs(300);

    for (short ch = 1; ch <= 2; ++ch)
    {
        double stepsPerRev = 0.0;
        double gearBoxRatio = 0.0;
        double pitch = 0.0;

        err = BDC_GetMotorParamsExt(m_serialCStr, ch, &stepsPerRev, &gearBoxRatio, &pitch);
        qDebug() << "BDCStage::open BDC_GetMotorParamsExt ch=" << ch
            << "err=" << err
            << "stepsPerRev=" << stepsPerRev
            << "gearBoxRatio=" << gearBoxRatio
            << "pitch=" << pitch;
        sleepMs(300);
    }

    // Read conversion factors per channel.
    // For linear stages, real units are assumed to be mm.
    for (short ch = 1; ch <= 2; ++ch)
    {
        const int idx = channelIndex(ch);

        double realPosMmPerDevice = 0.0;
        double realVelMmPerDevice = 0.0;
        double realAccMmPerDevice = 0.0;

        err = BDC_GetRealValueFromDeviceUnit(m_serialCStr, ch, 1, &realPosMmPerDevice, 0);
        if (!okOrLog("BDC_GetRealValueFromDeviceUnit(pos)", m_serial, err, ch, errOut))
        {
            BDC_Close(m_serialCStr);
            return false;
        }

        err = BDC_GetRealValueFromDeviceUnit(m_serialCStr, ch, 1, &realVelMmPerDevice, 1);
        if (!okOrLog("BDC_GetRealValueFromDeviceUnit(vel)", m_serial, err, ch, errOut))
        {
            BDC_Close(m_serialCStr);
            return false;
        }

        err = BDC_GetRealValueFromDeviceUnit(m_serialCStr, ch, 1, &realAccMmPerDevice, 2);
        if (!okOrLog("BDC_GetRealValueFromDeviceUnit(acc)", m_serial, err, ch, errOut))
        {
            BDC_Close(m_serialCStr);
            return false;
        }

        factor_position_mm[idx] = realPosMmPerDevice;
        factor_velocity_mm[idx] = realVelMmPerDevice;
        factor_acceleration_mm[idx] = realAccMmPerDevice;
        factor_um[idx] = factor_position_mm[idx] * 1000.0;

        qDebug() << "BDCStage::open factors serial=" << m_serialCStr
            << "ch=" << ch
            << "mm/device=" << factor_position_mm[idx]
            << "um/device=" << factor_um[idx]
            << "mm_s/device=" << factor_velocity_mm[idx]
            << "mm_s2/device=" << factor_acceleration_mm[idx];
    }

    sleepMs(500);

    if (!startPollingAll(errOut))
    {
        close();
        return false;
    }

    if (!enableAll(errOut))
    {
        close();
        return false;
    }

    m_isOpen = true;

    if (home)
    {
        if (!this->homeAllIfNeeded(errOut))
        {
            qDebug() << "BDCStage::open homeAll FAILED serial=" << m_serial.c_str()
                << "err=" << (errOut ? *errOut : -9999);
            return false;
        }
    }

    qDebug() << "BDCStage::open OK serial=" << m_serial.c_str();
    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::startPollingAll(short* errOut)
{
    bool poll1 = BDC_StartPolling(m_serialCStr, 1, 200);
    if (!okOrLog("BDC_StartPolling", m_serial, poll1 ? 0 : -1, 1, errOut))
        return false;

    bool poll2 = BDC_StartPolling(m_serialCStr, 2, 200);
    if (!okOrLog("BDC_StartPolling", m_serial, poll2 ? 0 : -1, 2, errOut))
        return false;

    sleepMs(1000);
    return true;
}

bool BDCStage::enableIfNeeded(unsigned channel, short* errOut)
{
    uint32_t bits = (uint32_t)BDC_GetStatusBits(m_serialCStr, (short)channel);

    if (isChannelEnabled(bits))
    {
        if (errOut) *errOut = 0;
        return true;
    }

    short err = BDC_EnableChannel(m_serialCStr, (short)channel);
    if (!okOrLog("BDC_EnableChannel", m_serial, err, channel, errOut))
        return false;

    sleepMs(300);

    bits = (uint32_t)BDC_GetStatusBits(m_serialCStr, (short)channel);
    if (!isChannelEnabled(bits))
    {
        if (errOut) *errOut = -1;
        return false;
    }

    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::enableAll(short* errOut)
{
    if (!enableIfNeeded(1, errOut))
        return false;

    if (!enableIfNeeded(2, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

void BDCStage::close()
{
    if (!m_isOpen)
        return;

    qDebug() << "BDCStage::close serial=" << m_serialCStr;

    BDC_StopPolling(m_serialCStr, 1);
    BDC_StopPolling(m_serialCStr, 2);
    BDC_Close(m_serialCStr);

    m_isOpen = false;
}

bool BDCStage::homeIfNeeded(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::homeIfNeeded serial=" << m_serialCStr << "ch=" << channel;

    const bool canMoveWithoutHoming = BDC_CanMoveWithoutHomingFirst(m_serialCStr, (short)channel);

    if (canMoveWithoutHoming)
    {
        qDebug() << "BDCStage::homeIfNeeded SKIP (not required) serial=" << m_serialCStr << "ch=" << channel;
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "BDCStage::homeIfNeeded DO HOME serial=" << m_serialCStr << "ch=" << channel;
    return home(channel, errOut);
}

bool BDCStage::homeAllIfNeeded(short* errOut)
{
    qDebug() << "BDCStage::homeAllIfNeeded serial=" << m_serialCStr;

    if (!homeIfNeeded(1, errOut)) return false;
    if (!homeIfNeeded(2, errOut)) return false;

    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::homeAll(short* errOut)
{
    qDebug() << "BDCStage::homeAll serial=" << m_serialCStr;

    if (!home(1, errOut)) return false;
    if (!home(2, errOut)) return false;

    return true;
}

bool BDCStage::home(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::home serial=" << m_serialCStr << "ch=" << channel;

    BDC_ClearMessageQueue(m_serialCStr, (short)channel);

    short err = BDC_Home(m_serialCStr, (short)channel);
    if (!okOrLog("BDC_Home", m_serial, err, channel, errOut))
        return false;

    if (!waitUntilHomed(channel, 120000, errOut))
        return false;

    qDebug() << "BDCStage::home DONE serial=" << m_serialCStr << "ch=" << channel;
    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::moveTo(int32_t pos, unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::moveTo serial=" << m_serialCStr << "ch=" << channel << "pos(device)=" << pos;

    BDC_ClearMessageQueue(m_serialCStr, (short)channel);

    short err = BDC_MoveToPosition(m_serialCStr, (short)channel, (int)pos);
    if (!okOrLog("BDC_MoveToPosition", m_serial, err, channel, errOut))
        return false;

    if (!waitUntilIdle(channel, 120000, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::moveRel(int32_t delta, unsigned channel, short* errOut)
{
    const int32_t cur = getPosition(channel);
    return moveTo(cur + delta, channel, errOut);
}

bool BDCStage::moveToUm(double posUm, unsigned channel, short* errOut)
{
    const int32_t targetDev = umToDevice(posUm, channel, errOut);
    if (errOut && *errOut != 0)
        return false;

    qDebug() << "BDCStage::moveToUm serial=" << m_serialCStr
        << "ch=" << channel
        << "posUm=" << posUm
        << "targetDev=" << targetDev;

    return moveTo(targetDev, channel, errOut);
}

bool BDCStage::moveRelUm(double deltaUm, unsigned channel, short* errOut)
{
    const int32_t deltaDev = umToDevice(deltaUm, channel, errOut);
    if (errOut && *errOut != 0)
        return false;

    qDebug() << "BDCStage::moveRelUm serial=" << m_serialCStr
        << "ch=" << channel
        << "deltaUm=" << deltaUm
        << "deltaDev=" << deltaDev;

    return moveRel(deltaDev, channel, errOut);
}

bool BDCStage::stopImmediate(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::stopImmediate serial=" << m_serialCStr << "ch=" << channel;

    short err = BDC_StopImmediate(m_serialCStr, (short)channel);
    return okOrLog("BDC_StopImmediate", m_serial, err, channel, errOut);
}

int32_t BDCStage::getPosition(unsigned channel) const
{
    return (int32_t)BDC_GetPosition(m_serialCStr, (short)channel);
}

double BDCStage::getPositionUm(unsigned channel, short* errOut) const
{
    const int32_t posDev = getPosition(channel);
    const double posUm = deviceToUm(posDev, channel);
    if (errOut) *errOut = 0;
    return posUm;
}

double BDCStage::umPerDeviceUnit(unsigned channel) const
{
    return factor_um[channelIndex(channel)];
}

double BDCStage::mmPerDeviceUnit(unsigned channel) const
{
    return factor_position_mm[channelIndex(channel)];
}

double BDCStage::deviceToMm(int32_t deviceUnits, unsigned channel) const
{
    return static_cast<double>(deviceUnits) * factor_position_mm[channelIndex(channel)];
}

double BDCStage::deviceToUm(int32_t deviceUnits, unsigned channel) const
{
    return static_cast<double>(deviceUnits) * factor_um[channelIndex(channel)];
}

int32_t BDCStage::mmToDevice(double mm, unsigned channel, short* errOut) const
{
    int deviceUnits = 0;
    short err = BDC_GetDeviceUnitFromRealValue(m_serialCStr, (short)channel, mm, &deviceUnits, 0);
    if (!okOrLog("BDC_GetDeviceUnitFromRealValue(pos)", m_serial, err, channel, errOut))
        return 0;

    return (int32_t)deviceUnits;
}

int32_t BDCStage::umToDevice(double um, unsigned channel, short* errOut) const
{
    const double mm = um / 1000.0;
    return mmToDevice(mm, channel, errOut);
}

void BDCStage::setTriggerConfig(const BDCTriggerConfig& cfg)
{
    m_trig = cfg;
}

const BDCTriggerConfig& BDCStage::triggerConfig() const
{
    return m_trig;
}

bool BDCStage::applyTriggerConfig(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::applyTriggerConfig serial=" << m_serialCStr
        << "ch=" << channel
        << "enabled=" << m_trig.enabled
        << "t1Mode=" << m_trig.trigger1Mode
        << "t2Mode=" << m_trig.trigger2Mode
        << "startFwd=" << m_trig.startPosFwd
        << "intervalFwd=" << m_trig.intervalFwd
        << "countFwd=" << m_trig.pulseCountFwd
        << "startRev=" << m_trig.startPosRev
        << "intervalRev=" << m_trig.intervalRev
        << "countRev=" << m_trig.pulseCountRev
        << "pulseWidthUs=" << m_trig.pulseWidthUs;

    // BDC header: width range 1 us to 1 s
    const __int32 pulseWidth = (__int32)std::max<int32_t>(1, std::min<int32_t>(m_trig.pulseWidthUs, 1000000));

    KMOT_TriggerConfig cfg = {};
    KMOT_TriggerParams params = {};

    if (m_trig.enabled)
    {
        cfg.Trigger1Mode = (KMOT_TriggerPortMode)m_trig.trigger1Mode;
        cfg.Trigger1Polarity = (KMOT_TriggerPortPolarity)m_trig.trigger1Polarity;
        cfg.Trigger2Mode = (KMOT_TriggerPortMode)m_trig.trigger2Mode;
        cfg.Trigger2Polarity = (KMOT_TriggerPortPolarity)m_trig.trigger2Polarity;
    }
    else
    {
        cfg.Trigger1Mode = KMOT_TrigDisabled;
        cfg.Trigger1Polarity = KMOT_TrigPolarityHigh;
        cfg.Trigger2Mode = KMOT_TrigDisabled;
        cfg.Trigger2Polarity = KMOT_TrigPolarityHigh;
    }

    params.TriggerStartPositionFwd = m_trig.startPosFwd;
    params.TriggerIntervalFwd = m_trig.intervalFwd;
    params.TriggerPulseCountFwd = m_trig.pulseCountFwd;
    params.TriggerStartPositionRev = m_trig.startPosRev;
    params.TriggerIntervalRev = m_trig.intervalRev;
    params.TriggerPulseCountRev = m_trig.pulseCountRev;
    params.TriggerPulseWidth = pulseWidth;
    params.CycleCount = m_trig.cycleCount;

    short err = BDC_SetTriggerConfigParamsBlock(m_serialCStr, (short)channel, &cfg);
    if (!okOrLog("BDC_SetTriggerConfigParamsBlock", m_serial, err, channel, errOut))
        return false;

    err = BDC_SetTriggerParamsBlock(m_serialCStr, (short)channel, &params);
    if (!okOrLog("BDC_SetTriggerParamsBlock", m_serial, err, channel, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::disableTrigger(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::disableTrigger serial=" << m_serialCStr << "ch=" << channel;

    KMOT_TriggerConfig cfg = {};
    cfg.Trigger1Mode = KMOT_TrigDisabled;
    cfg.Trigger1Polarity = KMOT_TrigPolarityHigh;
    cfg.Trigger2Mode = KMOT_TrigDisabled;
    cfg.Trigger2Polarity = KMOT_TrigPolarityHigh;

    short err = BDC_SetTriggerConfigParamsBlock(m_serialCStr, (short)channel, &cfg);
    if (!okOrLog("BDC_SetTriggerConfigParamsBlock", m_serial, err, channel, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

// --------- Robust wait using status polling ---------

static bool isMovingFromStatus(uint32_t statusBits)
{
    const uint32_t MOVING_MASK = 0x00000030;
    return (statusBits & MOVING_MASK) != 0;
}

static bool isHomedFromStatus(uint32_t statusBits)
{
    const uint32_t HOMED_MASK = 0x00000400;
    return (statusBits & HOMED_MASK) != 0;
}

bool BDCStage::waitUntilIdle(unsigned channel, int timeoutMs, short* errOut)
{
    const auto t0 = std::chrono::steady_clock::now();
    int32_t lastPos = getPosition(channel);
    int stableCount = 0;

    while (true)
    {
        uint32_t bits = (uint32_t)BDC_GetStatusBits(m_serialCStr, (short)channel);
        int32_t pos = getPosition(channel);

        const bool moving = isMovingFromStatus(bits);
        if (!moving)
        {
            if (pos == lastPos) stableCount++;
            else stableCount = 0;

            if (stableCount >= 5)
            {
                if (errOut) *errOut = 0;
                return true;
            }
        }
        else
        {
            stableCount = 0;
        }

        lastPos = pos;

        const auto now = std::chrono::steady_clock::now();
        const int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (elapsed > timeoutMs)
        {
            qDebug() << "BDCStage::waitUntilIdle TIMEOUT serial=" << m_serialCStr
                << "ch=" << channel
                << "lastPos=" << lastPos
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = -1;
            return false;
        }

        sleepMs(50);
    }
}

bool BDCStage::waitUntilHomed(unsigned channel, int timeoutMs, short* errOut)
{
    const auto t0 = std::chrono::steady_clock::now();

    while (true)
    {
        uint32_t bits = (uint32_t)BDC_GetStatusBits(m_serialCStr, (short)channel);
        const bool homed = isHomedFromStatus(bits);
        const bool moving = isMovingFromStatus(bits);

        if (homed && !moving)
        {
            if (errOut) *errOut = 0;
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        const int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (elapsed > timeoutMs)
        {
            qDebug() << "BDCStage::waitUntilHomed TIMEOUT serial=" << m_serialCStr
                << "ch=" << channel
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = -1;
            return false;
        }

        sleepMs(50);
    }
}