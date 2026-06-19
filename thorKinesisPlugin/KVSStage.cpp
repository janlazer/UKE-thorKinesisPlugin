#include "KVSStage.h"

#include <QDebug>
#include <chrono>
#include <thread>
#include <algorithm>

#include "Thorlabs.MotionControl.VerticalStage.h"

// ---- KVSStage: enable only if required (using status bits) ----

static bool isChannelEnabled(uint32_t bits)
{
    return (bits & 0x80000000u) != 0;   // Channel enabled
}

static void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static int32_t roundToI32(double v)
{
    return static_cast<int32_t>(v >= 0.0 ? (v + 0.5) : (v - 0.5));
}

KVSStage::KVSStage(const std::string& serial)
    : m_serial(serial)
{
}

KVSStage::~KVSStage()
{
    close();
}

void KVSStage::logErr(const char* fn, const std::string& serial, short err)
{
    qDebug() << "KVSStage -" << fn << "serial=" << serial.c_str() << "err=" << err;
}

bool KVSStage::okOrLog(const char* fn, const std::string& serial, short err, short* errOut)
{
    if (errOut) *errOut = err;
    if (err == 0) return true;
    logErr(fn, serial, err);
    return false;
}

bool KVSStage::open(const std::string& serial, bool home, short* errOut)
{
    if (!serial.empty())
    {
        m_serial = serial;
        m_serialCStr = m_serial.c_str();
    }
    else if (m_serial.empty())
    {
        qDebug() << "KVSStage::open ERROR no serial provided";
        if (errOut) *errOut = -1;
        return false;
    }

    qDebug() << "KVSStage::open serial=" << m_serialCStr << "home=" << home;
    if (m_isOpen)
    {
        qDebug() << "KVSStage::open already open serial=" << m_serialCStr;
        if (errOut) *errOut = 0;
        return false;
    }

    short err = KVS_Open(m_serialCStr);
    if (!okOrLog("KVS_Open", m_serial, err, errOut))
        return false;

    const bool loaded = KVS_LoadSettings(m_serialCStr);
    qDebug() << "KVSStage::open KVS_LoadSettings loaded=" << loaded;

    err = KVS_RequestSettings(m_serialCStr);
    qDebug() << "KVSStage::open KVS_RequestSettings err=" << err;

    sleepMs(300);

    double stepsPerRev = 0.0;
    double gearBoxRatio = 0.0;
    double pitch = 0.0;

    err = KVS_GetMotorParamsExt(m_serialCStr, &stepsPerRev, &gearBoxRatio, &pitch);
    qDebug() << "KVSStage::open KVS_GetMotorParamsExt "
        << "err=" << err
        << "stepsPerRev=" << stepsPerRev
        << "gearBoxRatio=" << gearBoxRatio
        << "pitch=" << pitch;
    sleepMs(300);

    // Read conversion factors from Kinesis.
    // For linear stages, real units are assumed to be mm.
    double realPosMmPerDevice = 0.0;
    double realVelMmPerDevice = 0.0;
    double realAccMmPerDevice = 0.0;

    err = KVS_GetRealValueFromDeviceUnit(m_serialCStr, 1, &realPosMmPerDevice, 0);
    if (!okOrLog("KVS_GetRealValueFromDeviceUnit(pos)", m_serial, err, errOut))
    {
        KVS_Close(m_serialCStr);
        return false;
    }

    err = KVS_GetRealValueFromDeviceUnit(m_serialCStr, 1, &realVelMmPerDevice, 1);
    if (!okOrLog("KVS_GetRealValueFromDeviceUnit(vel)", m_serial, err, errOut))
    {
        KVS_Close(m_serialCStr);
        return false;
    }

    err = KVS_GetRealValueFromDeviceUnit(m_serialCStr, 1, &realAccMmPerDevice, 2);
    if (!okOrLog("KVS_GetRealValueFromDeviceUnit(acc)", m_serial, err, errOut))
    {
        KVS_Close(m_serialCStr);
        return false;
    }

    factor_position_mm = realPosMmPerDevice;
    factor_velocity_mm = realVelMmPerDevice;
    factor_acceleration_mm = realAccMmPerDevice;
    factor_um = factor_position_mm * 1000.0;

    qDebug() << "KVSStage::open factors serial=" << m_serialCStr
        << "mm/device=" << factor_position_mm
        << "um/device=" << factor_um
        << "mm_s/device=" << factor_velocity_mm
        << "mm_s2/device=" << factor_acceleration_mm;

    sleepMs(500);

    bool poll = KVS_StartPolling(m_serialCStr, 200);
    if (!okOrLog("KVS_StartPolling", m_serial, poll ? 0 : -1, errOut))
    {
        KVS_Close(m_serialCStr);
        return false;
    }

    sleepMs(1000);

    if (!enable(errOut))
    {
        close();
        return false;
    }

    m_isOpen = true;

    sleepMs(500);
    if (home)
    {
        if (!this->home(errOut))
            return false;
    }

    if (errOut) *errOut = 0;
    return true;
}

void KVSStage::close()
{
    if (!m_isOpen)
        return;

    qDebug() << "KVSStage::close serial=" << m_serialCStr;
    KVS_StopPolling(m_serialCStr);
    KVS_Close(m_serialCStr);

    m_isOpen = false;
}

bool KVSStage::enable(short* errOut)
{
    if (!enableIfNeeded(1, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::enableIfNeeded(unsigned channel, short* errOut)
{
    (void)channel;

    uint32_t bits = (uint32_t)KVS_GetStatusBits(m_serialCStr);

    if (isChannelEnabled(bits))
    {
        if (errOut) *errOut = 0;
        return true;
    }

    short err = KVS_EnableChannel(m_serialCStr);
    if (!okOrLog("KVS_EnableChannel", m_serial, err, errOut))
        return false;

    sleepMs(300);

    bits = (uint32_t)KVS_GetStatusBits(m_serialCStr);
    if (!isChannelEnabled(bits))
    {
        if (errOut) *errOut = -1;
        return false;
    }

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::home(short* errOut)
{
    qDebug() << "KVSStage::home serial=" << m_serialCStr;

    KVS_ClearMessageQueue(m_serialCStr);

    short err = KVS_Home(m_serialCStr);
    if (!okOrLog("KVS_Home", m_serial, err, errOut))
        return false;

    if (!waitUntilHomed(120000, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::homeIfNeeded(short* errOut)
{
    qDebug() << "KVSStage::homeIfNeeded serial=" << m_serialCStr;
    const bool canMoveWithoutHoming = KVS_CanMoveWithoutHomingFirst(m_serialCStr);

    if (canMoveWithoutHoming)
    {
        qDebug() << "KVSStage::homeIfNeeded SKIP (not required) serial=" << m_serialCStr;
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "KVSStage::homeIfNeeded DO HOME serial=" << m_serialCStr;
    return home(errOut);
}

bool KVSStage::moveTo(int32_t pos, short* errOut)
{
    qDebug() << "KVSStage::moveTo serial=" << m_serialCStr << "pos(device)=" << pos;

    KVS_ClearMessageQueue(m_serialCStr);

    short err = KVS_MoveToPosition(m_serialCStr, (int)pos);
    if (!okOrLog("KVS_MoveToPosition", m_serial, err, errOut))
        return false;

    if (!waitUntilIdle(120000, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::moveRel(int32_t delta, short* errOut)
{
    const int32_t cur = getPosition();
    return moveTo(cur + delta, errOut);
}

bool KVSStage::moveToUm(double posUm, short* errOut)
{
    const int32_t targetDev = umToDevice(posUm, errOut);
    if (errOut && *errOut != 0)
        return false;

    qDebug() << "KVSStage::moveToUm serial=" << m_serialCStr
        << "posUm=" << posUm
        << "targetDev=" << targetDev;

    return moveTo(targetDev, errOut);
}

bool KVSStage::moveRelUm(double deltaUm, short* errOut)
{
    const int32_t deltaDev = umToDevice(deltaUm, errOut);
    if (errOut && *errOut != 0)
        return false;

    qDebug() << "KVSStage::moveRelUm serial=" << m_serialCStr
        << "deltaUm=" << deltaUm
        << "deltaDev=" << deltaDev;

    return moveRel(deltaDev, errOut);
}

bool KVSStage::stopImmediate(short* errOut)
{
    short err = KVS_StopImmediate(m_serialCStr);
    return okOrLog("KVS_StopImmediate", m_serial, err, errOut);
}

int32_t KVSStage::getPosition() const
{
    return (int32_t)KVS_GetPosition(m_serialCStr);
}

double KVSStage::getPositionUm(short* errOut) const
{
    const int32_t posDev = getPosition();
    const double posUm = deviceToUm(posDev);

    if (errOut) *errOut = 0;
    return posUm;
}

double KVSStage::deviceToMm(int32_t deviceUnits) const
{
    return static_cast<double>(deviceUnits) * factor_position_mm;
}

double KVSStage::deviceToUm(int32_t deviceUnits) const
{
    return static_cast<double>(deviceUnits) * factor_um;
}

int32_t KVSStage::mmToDevice(double mm, short* errOut) const
{
    int deviceUnits = 0;
    short err = KVS_GetDeviceUnitFromRealValue(m_serialCStr, mm, &deviceUnits, 0);
    if (!okOrLog("KVS_GetDeviceUnitFromRealValue(pos)", m_serial, err, errOut))
        return 0;

    return (int32_t)deviceUnits;
}

int32_t KVSStage::umToDevice(double um, short* errOut) const
{
    const double mm = um / 1000.0;
    return mmToDevice(mm, errOut);
}

void KVSStage::setTriggerConfig(const KVSTriggerConfig& cfg)
{
    m_trig = cfg;
}

const KVSTriggerConfig& KVSStage::triggerConfig() const
{
    return m_trig;
}

bool KVSStage::applyTriggerConfig(short* errOut)
{
    qDebug() << "KVSStage::applyTriggerConfig serial=" << m_serialCStr
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

    // KVS trigger width comment in header:
    // range 1000 (1ms) to 32767000 (32767ms)
    // -> keep internal field as "us" and clamp accordingly.
    const __int32 pulseWidth = (__int32)std::max<int32_t>(1000, std::min<int32_t>(m_trig.pulseWidthUs, 32767000));

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

    short err = KVS_SetTriggerConfigParamsBlock(m_serialCStr, &cfg);
    if (!okOrLog("KVS_SetTriggerConfigParamsBlock", m_serial, err, errOut))
        return false;

    err = KVS_SetTriggerParamsParamsBlock(m_serialCStr, &params);
    if (!okOrLog("KVS_SetTriggerParamsParamsBlock", m_serial, err, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::disableTrigger(short* errOut)
{
    qDebug() << "KVSStage::disableTrigger serial=" << m_serialCStr;

    KMOT_TriggerConfig cfg = {};
    cfg.Trigger1Mode = KMOT_TrigDisabled;
    cfg.Trigger1Polarity = KMOT_TrigPolarityHigh;
    cfg.Trigger2Mode = KMOT_TrigDisabled;
    cfg.Trigger2Polarity = KMOT_TrigPolarityHigh;

    short err = KVS_SetTriggerConfigParamsBlock(m_serialCStr, &cfg);
    if (!okOrLog("KVS_SetTriggerConfigParamsBlock", m_serial, err, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}
// -------- Robust wait using status polling --------

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

bool KVSStage::waitUntilIdle(int timeoutMs, short* errOut)
{
    const auto t0 = std::chrono::steady_clock::now();
    int32_t lastPos = getPosition();
    int stableCount = 0;

    while (true)
    {
        uint32_t bits = (uint32_t)KVS_GetStatusBits(m_serialCStr);
        int32_t pos = getPosition();

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
            qDebug() << "KVSStage::waitUntilIdle TIMEOUT serial=" << m_serialCStr
                << "lastPos=" << lastPos
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = -1;
            return false;
        }

        sleepMs(50);
    }
}

bool KVSStage::waitUntilHomed(int timeoutMs, short* errOut)
{
    const auto t0 = std::chrono::steady_clock::now();

    while (true)
    {
        uint32_t bits = (uint32_t)KVS_GetStatusBits(m_serialCStr);
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
            qDebug() << "KVSStage::waitUntilHomed TIMEOUT serial=" << m_serialCStr
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = -1;
            return false;
        }

        sleepMs(50);
    }
}