#include "KVSStage.h"

#include <QDebug>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <limits>

#include "Thorlabs.MotionControl.VerticalStage.h"

// ---- KVSStage: enable only if required (using status bits) ----

namespace
{
constexpr short kErrInvalidState = -1;
constexpr short kErrInvalidArgument = -3;
constexpr short kErrTimeout = -4;
constexpr short kErrTargetNotReached = -5;
constexpr double kPositionToleranceUm = 0.5;
constexpr int kWaitPollIntervalMs = 50;
constexpr int kMovingStuckTimeoutMs = 10000;
constexpr int kMovingStuckSamples = kMovingStuckTimeoutMs / kWaitPollIntervalMs;
constexpr int kHomeIdleWithoutHomeTimeoutMs = 10000;
constexpr int kHomeIdleWithoutHomeSamples = kHomeIdleWithoutHomeTimeoutMs / kWaitPollIntervalMs;
constexpr int kStablePositionSamples = 5;
constexpr int kMoveStartGraceMs = 1000;
constexpr int kMoveStartGraceSamples = kMoveStartGraceMs / kWaitPollIntervalMs;
constexpr double kTrackSettleCycleMs = 0.1024;
constexpr unsigned kTrackSettleMaxTimeCycles = 0x7FFF;

// ThorlabsDefaultSettings.xml, DeviceSettingsDefinition Name="KVS30":
// Pitch=1.0 mm, StepsPerRev=20000, GearboxRatio=1.
// This yields 0.00005 mm/device-unit = 0.05 um/device-unit.
// If Kinesis settings are corrupted, a 100 um GUI step can otherwise become
// hundreds of millions of device units and drive the stage into the limit.
constexpr double kKvs30ExpectedStepsPerRev = 20000.0;
constexpr double kKvs30ExpectedGearboxRatio = 1.0;
constexpr double kKvs30ExpectedPitchMm = 1.0;
constexpr double kKvs30ExpectedUmPerDeviceUnit = 0.05;
constexpr double kKvs30MotorParamRelTolerance = 0.02;
constexpr double kKvs30ScaleRelTolerance = 0.25;
constexpr double kKvs30MaxSingleRelativeMoveUm = 1000.0;
constexpr int32_t kKvs30MinPositionDeviceUnits = 0;
constexpr int32_t kKvs30MaxPositionDeviceUnits = 600000;
constexpr double kKvs30MinAbsolutePositionUm = 0.0;
constexpr double kKvs30MaxAbsolutePositionUm = 30000.0;
constexpr double kKvs30MoveAccelerationMmS2 = 1.0;
constexpr double kKvs30MoveMaxVelocityMmS = 2.0;
constexpr double kKvs30MotorMaxVelocityMmS = 8.0;
constexpr double kKvs30MotorMaxAccelerationMmS2 = 5.0;
constexpr unsigned short kKvs30TrackSettleTimeCycles = 197;
constexpr unsigned short kKvs30TrackSettleSettledError = 20;
constexpr unsigned short kKvs30TrackSettleMaxTrackingError = 0;

bool isValidTriggerMode(int mode)
{
    switch (mode)
    {
    case KMOT_TrigDisabled:
    case KMOT_TrigIn_GPI:
    case KMOT_TrigIn_RelativeMove:
    case KMOT_TrigIn_AbsoluteMove:
    case KMOT_TrigIn_Home:
    case KMOT_TrigIn_Stop:
    case KMOT_TrigIn_StartScan:
    case KMOT_TrigIn_ShuttleMove:
    case KMOT_TrigOut_GPO:
    case KMOT_TrigOut_InMotion:
    case KMOT_TrigOut_AtMaxVelocity:
    case KMOT_TrigOut_AtPositionSteps:
    case KMOT_TrigOut_Synch:
        return true;
    default:
        return false;
    }
}

bool isValidTriggerPolarity(int polarity)
{
    return polarity == KMOT_TrigPolarityHigh || polarity == KMOT_TrigPolarityLow;
}

bool isCloseRelative(double actual, double expected, double relTolerance)
{
    if (!std::isfinite(actual) || !std::isfinite(expected))
        return false;

    const double scale = (std::max)(1.0, std::abs(expected));
    return std::abs(actual - expected) <= scale * relTolerance;
}

bool isSafeKvs30MotorScale(double stepsPerRev, double gearBoxRatio, double pitch)
{
    return isCloseRelative(stepsPerRev, kKvs30ExpectedStepsPerRev, kKvs30MotorParamRelTolerance)
        && isCloseRelative(gearBoxRatio, kKvs30ExpectedGearboxRatio, kKvs30MotorParamRelTolerance)
        && isCloseRelative(pitch, kKvs30ExpectedPitchMm, kKvs30MotorParamRelTolerance);
}

bool isSafeKvs30PositionScale(double umPerDeviceUnit)
{
    return isCloseRelative(umPerDeviceUnit, kKvs30ExpectedUmPerDeviceUnit, kKvs30ScaleRelTolerance);
}

void logTrackSettleParams(const char* context, const std::string& serial,
    const MOT_BrushlessTrackSettleParameters& params)
{
    const unsigned timeCycles = static_cast<unsigned>(params.time);
    const double timeMs = static_cast<double>(timeCycles) * kTrackSettleCycleMs;

    qDebug() << context
        << "serial=" << serial.c_str()
        << "timeCycles=" << timeCycles
        << "timeMs=" << timeMs
        << "settledError/windowCounts=" << static_cast<unsigned>(params.settledError)
        << "maxTrackingErrorCounts=" << static_cast<unsigned>(params.maxTrackingError);

    if (timeCycles == 0 || timeCycles > kTrackSettleMaxTimeCycles)
    {
        qDebug() << context
            << "WARNING track/settle time is outside Thorlabs documented range 1..0x7FFF cycles"
            << "serial=" << serial.c_str()
            << "timeCycles=" << timeCycles
            << "timeMs=" << timeMs;
    }
}
}

static bool isChannelEnabled(uint32_t bits)
{
    return (bits & 0x80000000u) != 0;   // Channel enabled
}

static void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

KVSStage::KVSStage(const std::string& serial)
    : m_serial(serial)
{
    m_serialCStr = m_serial.c_str();
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

bool KVSStage::validateOpen(short* errOut) const
{
    if (m_isOpen && m_serialCStr && *m_serialCStr)
    {
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "KVSStage - device is not open serial=" << m_serial.c_str();
    if (errOut) *errOut = kErrInvalidState;
    return false;
}

bool KVSStage::open(const std::string& serial, bool home, short* errOut)
{
    if (serial.empty())
    {
        qDebug() << "KVSStage::open ERROR no serial provided";
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    if (m_isOpen)
    {
        if (serial != m_serial)
        {
            qDebug() << "KVSStage::open ERROR already open with another serial=" << m_serial.c_str();
            if (errOut) *errOut = kErrInvalidState;
            return false;
        }

        qDebug() << "KVSStage::open already open serial=" << m_serialCStr;
        if (errOut) *errOut = 0;
        return true;
    }

    m_serial = serial;
    m_serialCStr = m_serial.c_str();

    qDebug() << "KVSStage::open serial=" << m_serialCStr << "home=" << home;

    short err = KVS_Open(m_serialCStr);
    if (!okOrLog("KVS_Open", m_serial, err, errOut))
        return false;

    m_isOpen = true;

    const bool loaded = KVS_LoadNamedSettings(m_serialCStr, "KVS30");
    qDebug() << "KVSStage::open KVS_LoadNamedSettings(KVS30) loaded=" << loaded;
    if (!loaded)
    {
        if (errOut) *errOut = kErrInvalidState;
        close();
        return false;
    }

    err = KVS_RequestSettings(m_serialCStr);
    qDebug() << "KVSStage::open KVS_RequestSettings err=" << err;
    if (!okOrLog("KVS_RequestSettings", m_serial, err, errOut))
    {
        close();
        return false;
    }

    sleepMs(300);

    qDebug() << "KVSStage::open applying KVS30/M safe runtime defaults serial=" << m_serialCStr;

    err = KVS_SetMotorParamsExt(
        m_serialCStr,
        kKvs30ExpectedStepsPerRev,
        kKvs30ExpectedGearboxRatio,
        kKvs30ExpectedPitchMm);
    if (!okOrLog("KVS_SetMotorParamsExt", m_serial, err, errOut))
    {
        close();
        return false;
    }

    err = KVS_SetMotorTravelLimits(
        m_serialCStr,
        kKvs30MinAbsolutePositionUm / 1000.0,
        kKvs30MaxAbsolutePositionUm / 1000.0);
    if (!okOrLog("KVS_SetMotorTravelLimits", m_serial, err, errOut))
    {
        close();
        return false;
    }

    err = KVS_SetMotorVelocityLimits(
        m_serialCStr,
        kKvs30MotorMaxVelocityMmS,
        kKvs30MotorMaxAccelerationMmS2);
    if (!okOrLog("KVS_SetMotorVelocityLimits", m_serial, err, errOut))
    {
        close();
        return false;
    }

    err = KVS_SetStageAxisLimits(
        m_serialCStr,
        kKvs30MinPositionDeviceUnits,
        kKvs30MaxPositionDeviceUnits);
    if (!okOrLog("KVS_SetStageAxisLimits", m_serial, err, errOut))
    {
        close();
        return false;
    }

    KVS_SetLimitsSoftwareApproachPolicy(m_serialCStr, DisallowIllegalMoves);

    int moveAccelerationDevice = 0;
    err = KVS_GetDeviceUnitFromRealValue(
        m_serialCStr,
        kKvs30MoveAccelerationMmS2,
        &moveAccelerationDevice,
        2);
    if (!okOrLog("KVS_GetDeviceUnitFromRealValue(move acc)", m_serial, err, errOut))
    {
        close();
        return false;
    }

    int moveMaxVelocityDevice = 0;
    err = KVS_GetDeviceUnitFromRealValue(
        m_serialCStr,
        kKvs30MoveMaxVelocityMmS,
        &moveMaxVelocityDevice,
        1);
    if (!okOrLog("KVS_GetDeviceUnitFromRealValue(move vel)", m_serial, err, errOut))
    {
        close();
        return false;
    }

    MOT_VelocityParameters moveVelocityParams = {};
    moveVelocityParams.minVelocity = 0;
    moveVelocityParams.acceleration = moveAccelerationDevice;
    moveVelocityParams.maxVelocity = moveMaxVelocityDevice;
    err = KVS_SetVelParamsBlock(m_serialCStr, &moveVelocityParams);
    if (!okOrLog("KVS_SetVelParamsBlock", m_serial, err, errOut))
    {
        close();
        return false;
    }

    MOT_BrushlessTrackSettleParameters safeTrackSettleParams = {};
    safeTrackSettleParams.time = static_cast<WORD>(kKvs30TrackSettleTimeCycles);
    safeTrackSettleParams.settledError = static_cast<WORD>(kKvs30TrackSettleSettledError);
    safeTrackSettleParams.maxTrackingError = static_cast<WORD>(kKvs30TrackSettleMaxTrackingError);
    safeTrackSettleParams.notUsed = 0;
    safeTrackSettleParams.lastNotUsed = 0;
    err = KVS_SetTrackSettleParams(m_serialCStr, &safeTrackSettleParams);
    if (!okOrLog("KVS_SetTrackSettleParams", m_serial, err, errOut))
    {
        close();
        return false;
    }

    err = KVS_RequestSettings(m_serialCStr);
    qDebug() << "KVSStage::open KVS_RequestSettings after runtime defaults err=" << err;
    if (!okOrLog("KVS_RequestSettings(after defaults)", m_serial, err, errOut))
    {
        close();
        return false;
    }
    sleepMs(300);

    MOT_BrushlessTrackSettleParameters trackSettleParams = {};
    err = KVS_RequestTrackSettleParams(m_serialCStr);
    qDebug() << "KVSStage::open KVS_RequestTrackSettleParams err=" << err;
    if (err == 0)
    {
        sleepMs(100);
        const short getTrackSettleErr = KVS_GetTrackSettleParams(m_serialCStr, &trackSettleParams);
        qDebug() << "KVSStage::open KVS_GetTrackSettleParams err=" << getTrackSettleErr;
        if (getTrackSettleErr == 0)
        {
            logTrackSettleParams("KVSStage::open track/settle", m_serial, trackSettleParams);
        }
    }

    double stepsPerRev = 0.0;
    double gearBoxRatio = 0.0;
    double pitch = 0.0;

    err = KVS_GetMotorParamsExt(m_serialCStr, &stepsPerRev, &gearBoxRatio, &pitch);
    qDebug() << "KVSStage::open KVS_GetMotorParamsExt "
        << "err=" << err
        << "stepsPerRev=" << stepsPerRev
        << "gearBoxRatio=" << gearBoxRatio
        << "pitch=" << pitch;
    if (!okOrLog("KVS_GetMotorParamsExt", m_serial, err, errOut))
    {
        close();
        return false;
    }
    if (!isSafeKvs30MotorScale(stepsPerRev, gearBoxRatio, pitch))
    {
        qDebug() << "KVSStage::open UNSAFE KVS30/M motor scale; refusing to move serial=" << m_serialCStr
            << "stepsPerRev=" << stepsPerRev
            << "gearBoxRatio=" << gearBoxRatio
            << "pitch=" << pitch
            << "expectedStepsPerRev=" << kKvs30ExpectedStepsPerRev
            << "expectedGearBoxRatio=" << kKvs30ExpectedGearboxRatio
            << "expectedPitch=" << kKvs30ExpectedPitchMm;
        if (errOut) *errOut = kErrInvalidState;
        close();
        return false;
    }
    sleepMs(300);

    // Read conversion factors from Kinesis.
    // For linear stages, real units are assumed to be mm.
    double realPosMmPerDevice = 0.0;
    double realVelMmPerDevice = 0.0;
    double realAccMmPerDevice = 0.0;

    err = KVS_GetRealValueFromDeviceUnit(m_serialCStr, 1, &realPosMmPerDevice, 0);
    if (!okOrLog("KVS_GetRealValueFromDeviceUnit(pos)", m_serial, err, errOut))
    {
        close();
        return false;
    }

    err = KVS_GetRealValueFromDeviceUnit(m_serialCStr, 1, &realVelMmPerDevice, 1);
    if (!okOrLog("KVS_GetRealValueFromDeviceUnit(vel)", m_serial, err, errOut))
    {
        close();
        return false;
    }

    err = KVS_GetRealValueFromDeviceUnit(m_serialCStr, 1, &realAccMmPerDevice, 2);
    if (!okOrLog("KVS_GetRealValueFromDeviceUnit(acc)", m_serial, err, errOut))
    {
        close();
        return false;
    }

    if (!std::isfinite(realPosMmPerDevice) || realPosMmPerDevice <= 0.0
        || !std::isfinite(realVelMmPerDevice) || realVelMmPerDevice <= 0.0
        || !std::isfinite(realAccMmPerDevice) || realAccMmPerDevice <= 0.0)
    {
        qDebug() << "KVSStage::open invalid conversion factors";
        if (errOut) *errOut = kErrInvalidState;
        close();
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

    if (!isSafeKvs30PositionScale(factor_um))
    {
        qDebug() << "KVSStage::open UNSAFE KVS30/M position scale; refusing to move serial=" << m_serialCStr
            << "um/device=" << factor_um
            << "expectedUm/device=" << kKvs30ExpectedUmPerDeviceUnit;
        if (errOut) *errOut = kErrInvalidState;
        close();
        return false;
    }

    sleepMs(500);

    bool poll = KVS_StartPolling(m_serialCStr, 200);
    if (!okOrLog("KVS_StartPolling", m_serial, poll ? 0 : -1, errOut))
    {
        close();
        return false;
    }

    sleepMs(1000);

    if (!enable(errOut))
    {
        close();
        return false;
    }

    sleepMs(500);
    if (home)
    {
        if (!this->home(errOut))
        {
            close();
            return false;
        }
    }

    if (errOut) *errOut = 0;
    return true;
}

void KVSStage::close()
{
    if (!m_isOpen)
        return;

    qDebug() << "KVSStage::close serial=" << m_serialCStr;
    short ignored = 0;
    disableTrigger(&ignored);
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

    if (!validateOpen(errOut))
        return false;

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

    if (!validateOpen(errOut))
        return false;

    if (!disableTrigger(errOut))
        return false;

    KVS_ClearMessageQueue(m_serialCStr);

    short err = KVS_Home(m_serialCStr);
    if (!okOrLog("KVS_Home", m_serial, err, errOut))
        return false;

    if (!waitUntilHomed(120000, errOut))
    {
        KVS_StopImmediate(m_serialCStr);
        return false;
    }

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::homeIfNeeded(short* errOut)
{
    qDebug() << "KVSStage::homeIfNeeded serial=" << m_serialCStr;

    if (!validateOpen(errOut))
        return false;

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

bool KVSStage::beginMoveTo(int32_t pos, short* errOut)
{
    qDebug() << "KVSStage::beginMoveTo serial=" << m_serialCStr << "pos(device)=" << pos;

    if (!validateOpen(errOut))
        return false;

    const uint32_t statusBits = static_cast<uint32_t>(KVS_GetStatusBits(m_serialCStr));
    if ((statusBits & 0x00000400u) == 0)
    {
        qDebug() << "KVSStage::beginMoveTo refusing KVS30/M move because homed bit is not set serial=" << m_serialCStr
            << "targetDevice=" << pos
            << "statusBits=0x" << QString::number(statusBits, 16);
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    if (pos < kKvs30MinPositionDeviceUnits || pos > kKvs30MaxPositionDeviceUnits)
    {
        qDebug() << "KVSStage::beginMoveTo refusing unsafe KVS30/M target serial=" << m_serialCStr
            << "targetDevice=" << pos
            << "allowedMinDevice=" << kKvs30MinPositionDeviceUnits
            << "allowedMaxDevice=" << kKvs30MaxPositionDeviceUnits;
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    KVS_ClearMessageQueue(m_serialCStr);

    short err = KVS_MoveToPosition(m_serialCStr, (int)pos);
    if (!okOrLog("KVS_MoveToPosition", m_serial, err, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::waitForPosition(int32_t targetPos, int timeoutMs, short* errOut)
{
    if (!validateOpen(errOut))
        return false;

    if (!waitUntilIdle(targetPos, timeoutMs, errOut))
    {
        KVS_StopImmediate(m_serialCStr);
        short ignored = 0;
        disableTrigger(&ignored);
        return false;
    }

    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::moveTo(int32_t pos, short* errOut)
{
    qDebug() << "KVSStage::moveTo serial=" << m_serialCStr << "pos(device)=" << pos;

    if (!beginMoveTo(pos, errOut))
        return false;

    return waitForPosition(pos, 120000, errOut);
}

bool KVSStage::moveRel(int32_t delta, short* errOut)
{
    short err = 0;
    const int32_t cur = getPosition(&err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return false;
    }

    const int64_t target = static_cast<int64_t>(cur) + static_cast<int64_t>(delta);
    if (target < (std::numeric_limits<int32_t>::min)()
        || target > (std::numeric_limits<int32_t>::max)())
    {
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    return moveTo(static_cast<int32_t>(target), errOut);
}

bool KVSStage::moveToUm(double posUm, short* errOut)
{
    if (!std::isfinite(posUm)
        || posUm < kKvs30MinAbsolutePositionUm
        || posUm > kKvs30MaxAbsolutePositionUm)
    {
        qDebug() << "KVSStage::moveToUm refusing unsafe absolute KVS30/M position serial=" << m_serialCStr
            << "posUm=" << posUm
            << "allowedMinUm=" << kKvs30MinAbsolutePositionUm
            << "allowedMaxUm=" << kKvs30MaxAbsolutePositionUm;
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    short err = 0;
    const int32_t targetDev = umToDevice(posUm, &err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return false;
    }

    qDebug() << "KVSStage::moveToUm serial=" << m_serialCStr
        << "posUm=" << posUm
        << "targetDev=" << targetDev;

    return moveTo(targetDev, errOut);
}

bool KVSStage::moveRelUm(double deltaUm, short* errOut)
{
    if (!std::isfinite(deltaUm) || std::abs(deltaUm) > kKvs30MaxSingleRelativeMoveUm)
    {
        qDebug() << "KVSStage::moveRelUm refusing unsafe KVS30/M relative move serial=" << m_serialCStr
            << "deltaUm=" << deltaUm
            << "maxAllowedUm=" << kKvs30MaxSingleRelativeMoveUm;
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    short err = 0;
    const int32_t deltaDev = umToDevice(deltaUm, &err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return false;
    }

    const double convertedBackUm = deviceToUm(deltaDev);
    if (!std::isfinite(convertedBackUm)
        || std::abs(convertedBackUm - deltaUm) > (std::max)(1.0, std::abs(deltaUm)) * kKvs30ScaleRelTolerance)
    {
        qDebug() << "KVSStage::moveRelUm refusing inconsistent KVS30/M conversion serial=" << m_serialCStr
            << "deltaUm=" << deltaUm
            << "deltaDev=" << deltaDev
            << "convertedBackUm=" << convertedBackUm
            << "um/device=" << factor_um;
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    const double expectedDeviceDelta = deltaUm / kKvs30ExpectedUmPerDeviceUnit;
    if (std::abs(static_cast<double>(deltaDev) - expectedDeviceDelta)
        > (std::max)(20.0, std::abs(expectedDeviceDelta) * kKvs30ScaleRelTolerance))
    {
        qDebug() << "KVSStage::moveRelUm refusing unsafe KVS30/M device delta serial=" << m_serialCStr
            << "deltaUm=" << deltaUm
            << "deltaDev=" << deltaDev
            << "expectedDeviceDelta~=" << expectedDeviceDelta;
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    qDebug() << "KVSStage::moveRelUm serial=" << m_serialCStr
        << "deltaUm=" << deltaUm
        << "deltaDev=" << deltaDev;

    return moveRel(deltaDev, errOut);
}

bool KVSStage::stopImmediate(short* errOut)
{
    if (!validateOpen(errOut))
        return false;

    short triggerErr = 0;
    const bool triggerDisabled = disableTrigger(&triggerErr);
    short err = KVS_StopImmediate(m_serialCStr);
    const bool stopped = okOrLog("KVS_StopImmediate", m_serial, err, errOut);
    if (!triggerDisabled && errOut) *errOut = triggerErr;
    return triggerDisabled && stopped;
}

int32_t KVSStage::getPosition(short* errOut) const
{
    if (!validateOpen(errOut))
        return 0;

    if (errOut) *errOut = 0;
    return (int32_t)KVS_GetPosition(m_serialCStr);
}

bool KVSStage::isMoving(short* errOut) const
{
    if (!validateOpen(errOut))
        return false;

    const uint32_t bits = static_cast<uint32_t>(KVS_GetStatusBits(m_serialCStr));
    if (errOut) *errOut = 0;
    return (bits & 0x00000030u) != 0;
}

double KVSStage::getPositionUm(short* errOut) const
{
    short err = 0;
    const int32_t posDev = getPosition(&err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return 0.0;
    }

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
    if (!validateOpen(errOut))
        return 0;

    if (!std::isfinite(mm))
    {
        if (errOut) *errOut = kErrInvalidArgument;
        return 0;
    }

    int deviceUnits = 0;
    short err = KVS_GetDeviceUnitFromRealValue(m_serialCStr, mm, &deviceUnits, 0);
    if (!okOrLog("KVS_GetDeviceUnitFromRealValue(pos)", m_serial, err, errOut))
        return 0;

    return (int32_t)deviceUnits;
}

int32_t KVSStage::umToDevice(double um, short* errOut) const
{
    if (!std::isfinite(um))
    {
        if (errOut) *errOut = kErrInvalidArgument;
        return 0;
    }

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
    if (!validateOpen(errOut))
        return false;

    const KVSTriggerConfig requested = m_trig;

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

    if (!requested.enabled)
        return disableTrigger(errOut);

    if (!isValidTriggerMode(requested.trigger1Mode)
        || !isValidTriggerMode(requested.trigger2Mode)
        || !isValidTriggerPolarity(requested.trigger1Polarity)
        || !isValidTriggerPolarity(requested.trigger2Polarity)
        || requested.pulseWidthUs < 1000
        || requested.pulseWidthUs > 32767000
        || requested.cycleCount <= 0)
    {
        qDebug() << "KVSStage::applyTriggerConfig invalid trigger configuration";
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    const bool needsPositionSteps = requested.trigger1Mode == KMOT_TrigOut_AtPositionSteps
        || requested.trigger2Mode == KMOT_TrigOut_AtPositionSteps;
    if (needsPositionSteps
        && (requested.intervalFwd <= 0
            || requested.intervalRev <= 0
            || requested.pulseCountFwd <= 0
            || requested.pulseCountRev <= 0))
    {
        qDebug() << "KVSStage::applyTriggerConfig invalid position-step parameters";
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    KMOT_TriggerConfig cfg = {};
    KMOT_TriggerParams params = {};

    cfg.Trigger1Mode = static_cast<KMOT_TriggerPortMode>(requested.trigger1Mode);
    cfg.Trigger1Polarity = static_cast<KMOT_TriggerPortPolarity>(requested.trigger1Polarity);
    cfg.Trigger2Mode = static_cast<KMOT_TriggerPortMode>(requested.trigger2Mode);
    cfg.Trigger2Polarity = static_cast<KMOT_TriggerPortPolarity>(requested.trigger2Polarity);

    params.TriggerStartPositionFwd = requested.startPosFwd;
    params.TriggerIntervalFwd = requested.intervalFwd;
    params.TriggerPulseCountFwd = requested.pulseCountFwd;
    params.TriggerStartPositionRev = requested.startPosRev;
    params.TriggerIntervalRev = requested.intervalRev;
    params.TriggerPulseCountRev = requested.pulseCountRev;
    params.TriggerPulseWidth = requested.pulseWidthUs;
    params.CycleCount = requested.cycleCount;

    if (!disableTrigger(errOut))
        return false;

    short err = KVS_SetTriggerParamsParamsBlock(m_serialCStr, &params);
    if (!okOrLog("KVS_SetTriggerParamsParamsBlock", m_serial, err, errOut))
        return false;

    err = KVS_SetTriggerConfigParamsBlock(m_serialCStr, &cfg);
    if (!okOrLog("KVS_SetTriggerConfigParamsBlock", m_serial, err, errOut))
    {
        short ignored = 0;
        disableTrigger(&ignored);
        return false;
    }

    m_trig = requested;
    if (errOut) *errOut = 0;
    return true;
}

bool KVSStage::disableTrigger(short* errOut)
{
    qDebug() << "KVSStage::disableTrigger serial=" << m_serialCStr;

    if (!validateOpen(errOut))
        return false;

    KMOT_TriggerConfig cfg = {};
    cfg.Trigger1Mode = KMOT_TrigDisabled;
    cfg.Trigger1Polarity = KMOT_TrigPolarityHigh;
    cfg.Trigger2Mode = KMOT_TrigDisabled;
    cfg.Trigger2Polarity = KMOT_TrigPolarityHigh;

    short err = KVS_SetTriggerConfigParamsBlock(m_serialCStr, &cfg);
    if (!okOrLog("KVS_SetTriggerConfigParamsBlock", m_serial, err, errOut))
        return false;

    m_trig.enabled = false;
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

bool KVSStage::waitUntilIdle(int32_t targetPos, int timeoutMs, short* errOut)
{
    if (!validateOpen(errOut))
        return false;

    const auto t0 = std::chrono::steady_clock::now();
    int32_t lastPos = getPosition(errOut);
    int stableCount = 0;
    bool seenMoving = false;

    while (true)
    {
        uint32_t bits = (uint32_t)KVS_GetStatusBits(m_serialCStr);
        int32_t pos = getPosition(errOut);

        const bool moving = isMovingFromStatus(bits);
        if (moving) seenMoving = true;

        if (pos == lastPos)
            ++stableCount;
        else
            stableCount = 0;

        if (stableCount >= kStablePositionSamples)
        {
            const int64_t positionError = static_cast<int64_t>(pos) - static_cast<int64_t>(targetPos);
            const double positionErrorUm = std::abs(
                static_cast<double>(positionError) * factor_um);
            if (positionErrorUm <= kPositionToleranceUm)
            {
                if (moving)
                {
                    qDebug() << "KVSStage::waitUntilIdle target reached while moving bit still set serial=" << m_serialCStr
                        << "target=" << targetPos
                        << "actual=" << pos
                        << "statusBits=0x" << QString::number(bits, 16);
                }

                if (errOut) *errOut = 0;
                return true;
            }

            if (!seenMoving && stableCount < kMoveStartGraceSamples)
            {
                lastPos = pos;
                sleepMs(kWaitPollIntervalMs);
                continue;
            }

            if (!moving || stableCount >= kMovingStuckSamples)
            {
                qDebug() << "KVSStage::waitUntilIdle target not reached serial=" << m_serialCStr
                    << "target=" << targetPos
                    << "actual=" << pos
                    << "seenMoving=" << seenMoving
                    << "moving=" << moving
                    << "statusBits=0x" << QString::number(bits, 16);
                if (errOut) *errOut = kErrTargetNotReached;
                return false;
            }
        }

        lastPos = pos;

        const auto now = std::chrono::steady_clock::now();
        const int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (elapsed > timeoutMs)
        {
            qDebug() << "KVSStage::waitUntilIdle TIMEOUT serial=" << m_serialCStr
                << "lastPos=" << lastPos
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = kErrTimeout;
            return false;
        }

        sleepMs(kWaitPollIntervalMs);
    }
}

bool KVSStage::waitUntilHomed(int timeoutMs, short* errOut)
{
    if (!validateOpen(errOut))
        return false;

    const auto t0 = std::chrono::steady_clock::now();
    int idleWithoutHomeCount = 0;
    int32_t lastPos = getPosition(errOut);
    int stableCount = 0;
    bool seenMoving = false;

    while (true)
    {
        uint32_t bits = (uint32_t)KVS_GetStatusBits(m_serialCStr);
        int32_t pos = getPosition(errOut);
        const bool homed = isHomedFromStatus(bits);
        const bool moving = isMovingFromStatus(bits);
        if (moving)
            seenMoving = true;

        if (pos == lastPos)
            ++stableCount;
        else
            stableCount = 0;

        if (homed && stableCount >= kStablePositionSamples)
        {
            if (moving)
            {
                qDebug() << "KVSStage::waitUntilHomed homed while moving bit still set serial=" << m_serialCStr
                    << "pos=" << pos
                    << "statusBits=0x" << QString::number(bits, 16);
            }

            if (errOut) *errOut = 0;
            return true;
        }

        if (!homed && !moving)
        {
            if (stableCount >= kStablePositionSamples)
                ++idleWithoutHomeCount;
            else
                idleWithoutHomeCount = 0;

            if (idleWithoutHomeCount >= kHomeIdleWithoutHomeSamples)
            {
                qDebug() << "KVSStage::waitUntilHomed IDLE without homed bit serial=" << m_serialCStr
                    << "pos=" << pos
                    << "seenMoving=" << seenMoving
                    << "statusBits=0x" << QString::number(bits, 16);
                if (errOut) *errOut = kErrTargetNotReached;
                return false;
            }
        }
        else
        {
            idleWithoutHomeCount = 0;
            if (moving && stableCount >= kMovingStuckSamples)
            {
                qDebug() << "KVSStage::waitUntilHomed STUCK moving bit set but position unchanged serial=" << m_serialCStr
                    << "lastPos=" << lastPos
                    << "statusBits=0x" << QString::number(bits, 16);
                if (errOut) *errOut = kErrTargetNotReached;
                return false;
            }
        }

        lastPos = pos;

        const auto now = std::chrono::steady_clock::now();
        const int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (elapsed > timeoutMs)
        {
            qDebug() << "KVSStage::waitUntilHomed TIMEOUT serial=" << m_serialCStr
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = kErrTimeout;
            return false;
        }

        sleepMs(kWaitPollIntervalMs);
    }
}
