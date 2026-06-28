#include "BDCStage.h"

#include <QDebug>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cmath>
#include <limits>

// Kinesis C API
#include "Thorlabs.MotionControl.Benchtop.DCServo.h"

// ---- BDCStage: enable only if required (using status bits) ----

namespace
{
constexpr short kErrInvalidState = -1;
constexpr short kErrInvalidChannel = -2;
constexpr short kErrInvalidArgument = -3;
constexpr short kErrTimeout = -4;
constexpr short kErrTargetNotReached = -5;
constexpr double kPositionToleranceUm = 0.5;

// ThorlabsDefaultSettings.xml, DeviceSettingsDefinition Name="M30 Series":
// Pitch=1.0 mm, StepsPerRev=10000, GearboxRatio=1, MinPos=-15 mm, MaxPos=+15 mm.
// This yields 0.0001 mm/device-unit = 0.1 um/device-unit.
constexpr const char* kBdcM30SettingsName = "M30 Series";
constexpr double kBdcM30ExpectedStepsPerRev = 10000.0;
constexpr double kBdcM30ExpectedGearboxRatio = 1.0;
constexpr double kBdcM30ExpectedPitchMm = 1.0;
constexpr double kBdcM30ExpectedUmPerDeviceUnit = 0.1;
constexpr double kBdcM30MinTravelMm = -15.0;
constexpr double kBdcM30MaxTravelMm = 15.0;
constexpr int kBdcM30MinPositionDeviceUnits = -150000;
constexpr int kBdcM30MaxPositionDeviceUnits = 150000;
constexpr double kBdcM30MoveAccelerationMmS2 = 5.0;
constexpr double kBdcM30MoveMaxVelocityMmS = 2.3;
constexpr double kBdcM30MotorMaxVelocityMmS = 2.6;
constexpr double kBdcM30MotorMaxAccelerationMmS2 = 5.0;
constexpr double kBdcM30JogStepMm = 0.5;
constexpr double kBdcM30JogAccelerationMmS2 = 4.0;
constexpr double kBdcM30JogMaxVelocityMmS = 2.6;
constexpr double kBdcM30MotorParamRelTolerance = 0.02;
constexpr double kBdcM30ScaleRelTolerance = 0.25;
constexpr double kBdcM30MaxSingleRelativeMoveUm = 1000.0;
constexpr double kBdcM30MaxCoordinateMagnitudeUm = 31000.0;
constexpr double kBdcM30MinTravelUm = 1000.0;
constexpr double kBdcM30MaxTravelUm = 31000.0;

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
    case KMOT_TrigOut_AtPositionStepFwd:
    case KMOT_TrigOut_AtPositionStepRev:
    case KMOT_TrigOut_AtPositionStepBoth:
    case KMOT_TrigOut_AtFwdLimit:
    case KMOT_TrigOut_AtBwdLimit:
    case KMOT_TrigOut_AtLimit:
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

bool isSafeBdcM30MotorScale(double stepsPerRev, double gearBoxRatio, double pitch)
{
    return isCloseRelative(stepsPerRev, kBdcM30ExpectedStepsPerRev, kBdcM30MotorParamRelTolerance)
        && isCloseRelative(gearBoxRatio, kBdcM30ExpectedGearboxRatio, kBdcM30MotorParamRelTolerance)
        && isCloseRelative(pitch, kBdcM30ExpectedPitchMm, kBdcM30MotorParamRelTolerance);
}

bool isSafeBdcM30PositionScale(double umPerDeviceUnit)
{
    return isCloseRelative(umPerDeviceUnit, kBdcM30ExpectedUmPerDeviceUnit, kBdcM30ScaleRelTolerance);
}

bool isSafeBdcM30AxisLimits(double minUm, double maxUm)
{
    if (!std::isfinite(minUm) || !std::isfinite(maxUm) || maxUm <= minUm)
        return false;

    const double travelUm = maxUm - minUm;
    return std::abs(minUm) <= kBdcM30MaxCoordinateMagnitudeUm
        && std::abs(maxUm) <= kBdcM30MaxCoordinateMagnitudeUm
        && travelUm >= kBdcM30MinTravelUm
        && travelUm <= kBdcM30MaxTravelUm;
}

}

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
    if (channel == 1) return 0;
    if (channel == 2) return 1;
    return -1;
}

BDCStage::BDCStage(const std::string& baseSerial)
    : m_serial(baseSerial)
{
    m_serialCStr = m_serial.c_str();
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

bool BDCStage::validateOpen(short* errOut) const
{
    if (m_isOpen && m_serialCStr && *m_serialCStr)
    {
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "BDCStage - device is not open serial=" << m_serial.c_str();
    if (errOut) *errOut = kErrInvalidState;
    return false;
}

bool BDCStage::validateChannel(unsigned channel, short* errOut) const
{
    if (channel == 1 || channel == 2)
    {
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "BDCStage - invalid channel serial=" << m_serial.c_str() << "ch=" << channel;
    if (errOut) *errOut = kErrInvalidChannel;
    return false;
}

bool BDCStage::validateReady(unsigned channel, short* errOut) const
{
    return validateOpen(errOut) && validateChannel(channel, errOut);
}

bool BDCStage::open(const std::string& baseSerial, bool home, short* errOut)
{
    if (baseSerial.empty())
    {
        qDebug() << "BDCStage::open ERROR no serial provided";
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    if (m_isOpen)
    {
        if (baseSerial != m_serial)
        {
            qDebug() << "BDCStage::open ERROR already open with another serial=" << m_serial.c_str();
            if (errOut) *errOut = kErrInvalidState;
            return false;
        }

        qDebug() << "BDCStage::open already open serial=" << m_serialCStr;
        if (errOut) *errOut = 0;
        return true;
    }

    m_serial = baseSerial;
    m_serialCStr = m_serial.c_str();
    for (int idx = 0; idx < 2; ++idx)
    {
        m_positionSafetyReady[idx] = false;
        m_minPositionDevice[idx] = 0;
        m_maxPositionDevice[idx] = 0;
        m_minPositionUm[idx] = 0.0;
        m_maxPositionUm[idx] = 0.0;
    }

    qDebug() << "BDCStage::open serial=" << m_serialCStr << "home=" << home;

    short err = BDC_Open(m_serialCStr);
    if (!okOrLog("BDC_Open", m_serial, err, 0, errOut))
        return false;

    // From this point on close() must always release the SDK handle, even when
    // settings, polling, enabling or homing fail later in this method.
    m_isOpen = true;

    auto convertM30RuntimeDefault = [&](short ch, double value, int unitType, const char* label, int& deviceUnits) -> bool
    {
        deviceUnits = 0;
        err = BDC_GetDeviceUnitFromRealValue(m_serialCStr, ch, value, &deviceUnits, unitType);
        qDebug() << "BDCStage::open BDC_GetDeviceUnitFromRealValue runtime default ch=" << ch
            << "label=" << label
            << "value=" << value
            << "unitType=" << unitType
            << "deviceUnits=" << deviceUnits
            << "err=" << err;
        if (!okOrLog("BDC_GetDeviceUnitFromRealValue(runtime default)", m_serial, err, ch, errOut))
            return false;

        if (deviceUnits <= 0)
        {
            qDebug() << "BDCStage::open invalid M30XY runtime default conversion serial=" << m_serialCStr
                << "ch=" << ch
                << "label=" << label
                << "value=" << value
                << "unitType=" << unitType
                << "deviceUnits=" << deviceUnits;
            if (errOut) *errOut = kErrInvalidState;
            return false;
        }

        return true;
    };

    auto applyM30RuntimeDefaults = [&](short ch) -> bool
    {
        qDebug() << "BDCStage::open applying M30XY safe runtime defaults serial=" << m_serialCStr
            << "ch=" << ch;

        err = BDC_SetMotorTravelMode(m_serialCStr, ch, MOT_Linear);
        if (err != 0)
        {
            qDebug() << "BDCStage::open warning: BDC_SetMotorTravelMode failed; continuing because "
                << kBdcM30SettingsName
                << "named settings and motor-scale validation still guard moves serial=" << m_serialCStr
                << "ch=" << ch
                << "err=" << err;
        }

        err = BDC_SetMotorParamsExt(
            m_serialCStr,
            ch,
            kBdcM30ExpectedStepsPerRev,
            kBdcM30ExpectedGearboxRatio,
            kBdcM30ExpectedPitchMm);
        if (!okOrLog("BDC_SetMotorParamsExt", m_serial, err, ch, errOut))
            return false;

        err = BDC_SetMotorTravelLimits(m_serialCStr, ch, kBdcM30MinTravelMm, kBdcM30MaxTravelMm);
        if (!okOrLog("BDC_SetMotorTravelLimits", m_serial, err, ch, errOut))
            return false;

        err = BDC_SetMotorVelocityLimits(
            m_serialCStr,
            ch,
            kBdcM30MotorMaxVelocityMmS,
            kBdcM30MotorMaxAccelerationMmS2);
        if (!okOrLog("BDC_SetMotorVelocityLimits", m_serial, err, ch, errOut))
            return false;

        err = BDC_SetStageAxisLimits(
            m_serialCStr,
            ch,
            kBdcM30MinPositionDeviceUnits,
            kBdcM30MaxPositionDeviceUnits);
        if (!okOrLog("BDC_SetStageAxisLimits", m_serial, err, ch, errOut))
            return false;

        BDC_SetLimitsSoftwareApproachPolicy(m_serialCStr, ch, DisallowIllegalMoves);

        int moveAccelerationDevice = 0;
        int moveMaxVelocityDevice = 0;
        if (!convertM30RuntimeDefault(ch, kBdcM30MoveAccelerationMmS2, 2, "move acceleration", moveAccelerationDevice)
            || !convertM30RuntimeDefault(ch, kBdcM30MoveMaxVelocityMmS, 1, "move max velocity", moveMaxVelocityDevice))
        {
            return false;
        }

        MOT_VelocityParameters velocityParams = {};
        velocityParams.minVelocity = 0;
        velocityParams.acceleration = moveAccelerationDevice;
        velocityParams.maxVelocity = moveMaxVelocityDevice;

        err = BDC_SetVelParamsBlock(m_serialCStr, ch, &velocityParams);
        if (err != 0)
        {
            qDebug() << "BDCStage::open warning: BDC_SetVelParamsBlock failed; trying BDC_SetVelParams serial="
                << m_serialCStr
                << "ch=" << ch
                << "accDevice=" << moveAccelerationDevice
                << "maxVelDevice=" << moveMaxVelocityDevice
                << "err=" << err;

            const short fallbackErr = BDC_SetVelParams(m_serialCStr, ch, moveAccelerationDevice, moveMaxVelocityDevice);
            qDebug() << "BDCStage::open BDC_SetVelParams fallback ch=" << ch << "err=" << fallbackErr;
        }

        int jogStepDevice = 0;
        int jogAccelerationDevice = 0;
        int jogMaxVelocityDevice = 0;
        if (!convertM30RuntimeDefault(ch, kBdcM30JogStepMm, 0, "jog step", jogStepDevice)
            || !convertM30RuntimeDefault(ch, kBdcM30JogAccelerationMmS2, 2, "jog acceleration", jogAccelerationDevice)
            || !convertM30RuntimeDefault(ch, kBdcM30JogMaxVelocityMmS, 1, "jog max velocity", jogMaxVelocityDevice))
        {
            return false;
        }

        MOT_JogParameters jogParams = {};
        jogParams.mode = MOT_SingleStep;
        jogParams.stepSize = static_cast<unsigned int>(jogStepDevice);
        jogParams.velParams.minVelocity = 0;
        jogParams.velParams.acceleration = jogAccelerationDevice;
        jogParams.velParams.maxVelocity = jogMaxVelocityDevice;
        jogParams.stopMode = MOT_Profiled;

        err = BDC_SetJogParamsBlock(m_serialCStr, ch, &jogParams);
        if (err != 0)
        {
            qDebug() << "BDCStage::open warning: BDC_SetJogParamsBlock failed; continuing because plugin step "
                << "moves use moveRelUm, not Kinesis jog parameters serial=" << m_serialCStr
                << "ch=" << ch
                << "err=" << err;
        }

        return true;
    };

    for (short ch = 1; ch <= 2; ++ch)
    {
        const bool loaded = BDC_LoadNamedSettings(m_serialCStr, ch, kBdcM30SettingsName);
        qDebug() << "BDCStage::open BDC_LoadNamedSettings ch=" << ch
            << "settingsName=" << kBdcM30SettingsName
            << "loaded=" << loaded;
        if (!loaded)
        {
            if (errOut) *errOut = kErrInvalidState;
            close();
            return false;
        }

        err = BDC_RequestSettings(m_serialCStr, ch);
        qDebug() << "BDCStage::open BDC_RequestSettings ch=" << ch << "err = " << err;
        if (!okOrLog("BDC_RequestSettings", m_serial, err, ch, errOut))
        {
            close();
            return false;
        }

        if (!applyM30RuntimeDefaults(ch))
        {
            close();
            return false;
        }

        err = BDC_RequestSettings(m_serialCStr, ch);
        qDebug() << "BDCStage::open BDC_RequestSettings after runtime defaults ch=" << ch << "err = " << err;
        if (!okOrLog("BDC_RequestSettings(after runtime defaults)", m_serial, err, ch, errOut))
        {
            close();
            return false;
        }
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
        if (!okOrLog("BDC_GetMotorParamsExt", m_serial, err, ch, errOut))
        {
            close();
            return false;
        }

        if (!isSafeBdcM30MotorScale(stepsPerRev, gearBoxRatio, pitch))
        {
            qDebug() << "BDCStage::open UNSAFE M30XY motor scale; refusing to move serial=" << m_serialCStr
                << "ch=" << ch
                << "stepsPerRev=" << stepsPerRev
                << "gearBoxRatio=" << gearBoxRatio
                << "pitch=" << pitch
                << "expectedStepsPerRev=" << kBdcM30ExpectedStepsPerRev
                << "expectedGearBoxRatio=" << kBdcM30ExpectedGearboxRatio
                << "expectedPitch=" << kBdcM30ExpectedPitchMm;
            if (errOut) *errOut = kErrInvalidState;
            close();
            return false;
        }

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
            close();
            return false;
        }

        err = BDC_GetRealValueFromDeviceUnit(m_serialCStr, ch, 1, &realVelMmPerDevice, 1);
        if (!okOrLog("BDC_GetRealValueFromDeviceUnit(vel)", m_serial, err, ch, errOut))
        {
            close();
            return false;
        }

        err = BDC_GetRealValueFromDeviceUnit(m_serialCStr, ch, 1, &realAccMmPerDevice, 2);
        if (!okOrLog("BDC_GetRealValueFromDeviceUnit(acc)", m_serial, err, ch, errOut))
        {
            close();
            return false;
        }

        if (!std::isfinite(realPosMmPerDevice) || realPosMmPerDevice <= 0.0
            || !std::isfinite(realVelMmPerDevice) || realVelMmPerDevice <= 0.0
            || !std::isfinite(realAccMmPerDevice) || realAccMmPerDevice <= 0.0)
        {
            qDebug() << "BDCStage::open invalid conversion factors ch=" << ch;
            if (errOut) *errOut = kErrInvalidState;
            close();
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

        if (!isSafeBdcM30PositionScale(factor_um[idx]))
        {
            qDebug() << "BDCStage::open UNSAFE M30XY position scale; refusing to move serial=" << m_serialCStr
                << "ch=" << ch
                << "um/device=" << factor_um[idx]
                << "expectedUm/device=" << kBdcM30ExpectedUmPerDeviceUnit;
            if (errOut) *errOut = kErrInvalidState;
            close();
            return false;
        }

        const int minAxisDevice = BDC_GetStageAxisMinPos(m_serialCStr, ch);
        const int maxAxisDevice = BDC_GetStageAxisMaxPos(m_serialCStr, ch);
        const double minAxisUm = static_cast<double>(minAxisDevice) * factor_um[idx];
        const double maxAxisUm = static_cast<double>(maxAxisDevice) * factor_um[idx];
        const MOT_LimitsSoftwareApproachPolicy softLimitPolicy = BDC_GetSoftLimitMode(m_serialCStr, ch);

        qDebug() << "BDCStage::open axis safety serial=" << m_serialCStr
            << "ch=" << ch
            << "minDevice=" << minAxisDevice
            << "maxDevice=" << maxAxisDevice
            << "minUm=" << minAxisUm
            << "maxUm=" << maxAxisUm
            << "softLimitPolicy=" << static_cast<int>(softLimitPolicy);

        if (maxAxisDevice <= minAxisDevice || !isSafeBdcM30AxisLimits(minAxisUm, maxAxisUm))
        {
            qDebug() << "BDCStage::open UNSAFE M30XY axis limits; refusing to move serial=" << m_serialCStr
                << "ch=" << ch
                << "minDevice=" << minAxisDevice
                << "maxDevice=" << maxAxisDevice
                << "minUm=" << minAxisUm
                << "maxUm=" << maxAxisUm
                << "maxCoordinateMagnitudeUm=" << kBdcM30MaxCoordinateMagnitudeUm
                << "maxTravelUm=" << kBdcM30MaxTravelUm;
            if (errOut) *errOut = kErrInvalidState;
            close();
            return false;
        }

        if (softLimitPolicy == AllowAllMoves)
        {
            qDebug() << "BDCStage::open UNSAFE M30XY soft-limit policy allows all moves; refusing to move serial="
                << m_serialCStr
                << "ch=" << ch;
            if (errOut) *errOut = kErrInvalidState;
            close();
            return false;
        }

        m_minPositionDevice[idx] = static_cast<int32_t>(minAxisDevice);
        m_maxPositionDevice[idx] = static_cast<int32_t>(maxAxisDevice);
        m_minPositionUm[idx] = minAxisUm;
        m_maxPositionUm[idx] = maxAxisUm;
        m_positionSafetyReady[idx] = true;
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

    if (home)
    {
        if (!this->homeAllIfNeeded(errOut))
        {
            qDebug() << "BDCStage::open homeAll FAILED serial=" << m_serial.c_str()
                << "err=" << (errOut ? *errOut : -9999);
            close();
            return false;
        }
    }

    qDebug() << "BDCStage::open OK serial=" << m_serial.c_str();
    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::startPollingAll(short* errOut)
{
    if (!validateOpen(errOut))
        return false;

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
    if (!validateReady(channel, errOut))
        return false;

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

    short ignored = 0;
    disableTrigger(1, &ignored);
    disableTrigger(2, &ignored);

    BDC_StopPolling(m_serialCStr, 1);
    BDC_StopPolling(m_serialCStr, 2);
    BDC_Close(m_serialCStr);

    m_isOpen = false;
    for (int idx = 0; idx < 2; ++idx)
    {
        m_positionSafetyReady[idx] = false;
        m_minPositionDevice[idx] = 0;
        m_maxPositionDevice[idx] = 0;
        m_minPositionUm[idx] = 0.0;
        m_maxPositionUm[idx] = 0.0;
    }
}

bool BDCStage::homeIfNeeded(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::homeIfNeeded serial=" << m_serialCStr << "ch=" << channel;

    if (!validateReady(channel, errOut))
        return false;

    const uint32_t statusBits = static_cast<uint32_t>(BDC_GetStatusBits(m_serialCStr, static_cast<short>(channel)));
    const bool homed = (statusBits & 0x00000400u) != 0;
    const bool moving = (statusBits & 0x00000030u) != 0;
    const bool canMoveWithoutHoming = BDC_CanMoveWithoutHomingFirst(m_serialCStr, (short)channel);

    if (homed && !moving)
    {
        qDebug() << "BDCStage::homeIfNeeded SKIP (homed) serial=" << m_serialCStr
            << "ch=" << channel
            << "canMoveWithoutHoming=" << canMoveWithoutHoming;
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "BDCStage::homeIfNeeded DO HOME serial=" << m_serialCStr
        << "ch=" << channel
        << "statusBits=0x" << QString::number(statusBits, 16)
        << "canMoveWithoutHoming=" << canMoveWithoutHoming;
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

    if (!validateReady(channel, errOut))
        return false;

    if (!disableTrigger(channel, errOut))
        return false;

    BDC_ClearMessageQueue(m_serialCStr, (short)channel);

    short err = BDC_Home(m_serialCStr, (short)channel);
    if (!okOrLog("BDC_Home", m_serial, err, channel, errOut))
        return false;

    if (!waitUntilHomed(channel, 120000, errOut))
    {
        BDC_StopImmediate(m_serialCStr, (short)channel);
        return false;
    }

    qDebug() << "BDCStage::home DONE serial=" << m_serialCStr << "ch=" << channel;
    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::beginMoveTo(int32_t pos, unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::beginMoveTo serial=" << m_serialCStr << "ch=" << channel << "pos(device)=" << pos;

    if (!validateReady(channel, errOut))
        return false;

    const int idx = channelIndex(channel);
    if (idx < 0 || !m_positionSafetyReady[idx])
    {
        qDebug() << "BDCStage::beginMoveTo refusing M30XY move because safety limits are not ready serial="
            << m_serialCStr
            << "ch=" << channel
            << "targetDevice=" << pos;
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    const uint32_t statusBits = static_cast<uint32_t>(BDC_GetStatusBits(m_serialCStr, static_cast<short>(channel)));
    if ((statusBits & 0x00000400u) == 0)
    {
        qDebug() << "BDCStage::beginMoveTo refusing M30XY move because homed bit is not set serial=" << m_serialCStr
            << "ch=" << channel
            << "targetDevice=" << pos
            << "statusBits=0x" << QString::number(statusBits, 16);
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    if (pos < m_minPositionDevice[idx] || pos > m_maxPositionDevice[idx])
    {
        qDebug() << "BDCStage::beginMoveTo refusing unsafe M30XY target serial=" << m_serialCStr
            << "ch=" << channel
            << "targetDevice=" << pos
            << "allowedMinDevice=" << m_minPositionDevice[idx]
            << "allowedMaxDevice=" << m_maxPositionDevice[idx]
            << "allowedMinUm=" << m_minPositionUm[idx]
            << "allowedMaxUm=" << m_maxPositionUm[idx];
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    BDC_ClearMessageQueue(m_serialCStr, (short)channel);

    short err = BDC_MoveToPosition(m_serialCStr, (short)channel, (int)pos);
    if (!okOrLog("BDC_MoveToPosition", m_serial, err, channel, errOut))
        return false;

    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::waitForPosition(int32_t targetPos, unsigned channel, int timeoutMs, short* errOut)
{
    if (!validateReady(channel, errOut))
        return false;

    if (!waitUntilIdle(channel, targetPos, timeoutMs, errOut))
    {
        // A failed blocking move must never continue in the background or
        // leave a position trigger armed.
        BDC_StopImmediate(m_serialCStr, (short)channel);
        short ignored = 0;
        disableTrigger(channel, &ignored);
        return false;
    }

    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::moveTo(int32_t pos, unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::moveTo serial=" << m_serialCStr << "ch=" << channel << "pos(device)=" << pos;

    if (!beginMoveTo(pos, channel, errOut))
        return false;

    return waitForPosition(pos, channel, 120000, errOut);
}

bool BDCStage::moveRel(int32_t delta, unsigned channel, short* errOut)
{
    short err = 0;
    const int32_t cur = getPosition(channel, &err);
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

    return moveTo(static_cast<int32_t>(target), channel, errOut);
}

bool BDCStage::moveToUm(double posUm, unsigned channel, short* errOut)
{
    if (!validateReady(channel, errOut))
        return false;

    const int idx = channelIndex(channel);
    if (idx < 0 || !m_positionSafetyReady[idx])
    {
        qDebug() << "BDCStage::moveToUm refusing M30XY move because safety limits are not ready serial="
            << m_serialCStr
            << "ch=" << channel
            << "posUm=" << posUm;
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    if (!std::isfinite(posUm)
        || posUm < m_minPositionUm[idx]
        || posUm > m_maxPositionUm[idx])
    {
        qDebug() << "BDCStage::moveToUm refusing unsafe absolute M30XY position serial=" << m_serialCStr
            << "ch=" << channel
            << "posUm=" << posUm
            << "allowedMinUm=" << m_minPositionUm[idx]
            << "allowedMaxUm=" << m_maxPositionUm[idx];
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    short err = 0;
    const int32_t targetDev = umToDevice(posUm, channel, &err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return false;
    }

    qDebug() << "BDCStage::moveToUm serial=" << m_serialCStr
        << "ch=" << channel
        << "posUm=" << posUm
        << "targetDev=" << targetDev;

    return moveTo(targetDev, channel, errOut);
}

bool BDCStage::moveRelUm(double deltaUm, unsigned channel, short* errOut)
{
    if (!std::isfinite(deltaUm) || std::abs(deltaUm) > kBdcM30MaxSingleRelativeMoveUm)
    {
        qDebug() << "BDCStage::moveRelUm refusing unsafe M30XY relative move serial=" << m_serialCStr
            << "ch=" << channel
            << "deltaUm=" << deltaUm
            << "maxAllowedUm=" << kBdcM30MaxSingleRelativeMoveUm;
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    short err = 0;
    const int32_t deltaDev = umToDevice(deltaUm, channel, &err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return false;
    }

    const double convertedBackUm = deviceToUm(deltaDev, channel);
    if (!std::isfinite(convertedBackUm)
        || std::abs(convertedBackUm - deltaUm) > (std::max)(1.0, std::abs(deltaUm)) * kBdcM30ScaleRelTolerance)
    {
        qDebug() << "BDCStage::moveRelUm refusing inconsistent M30XY conversion serial=" << m_serialCStr
            << "ch=" << channel
            << "deltaUm=" << deltaUm
            << "deltaDev=" << deltaDev
            << "convertedBackUm=" << convertedBackUm
            << "um/device=" << factor_um[channelIndex(channel)];
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    const double expectedDeviceDelta = deltaUm / kBdcM30ExpectedUmPerDeviceUnit;
    if (std::abs(static_cast<double>(deltaDev) - expectedDeviceDelta)
        > (std::max)(20.0, std::abs(expectedDeviceDelta) * kBdcM30ScaleRelTolerance))
    {
        qDebug() << "BDCStage::moveRelUm refusing unsafe M30XY device delta serial=" << m_serialCStr
            << "ch=" << channel
            << "deltaUm=" << deltaUm
            << "deltaDev=" << deltaDev
            << "expectedDeviceDelta~=" << expectedDeviceDelta;
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    qDebug() << "BDCStage::moveRelUm serial=" << m_serialCStr
        << "ch=" << channel
        << "deltaUm=" << deltaUm
        << "deltaDev=" << deltaDev;

    return moveRel(deltaDev, channel, errOut);
}

bool BDCStage::stopImmediate(unsigned channel, short* errOut)
{
    qDebug() << "BDCStage::stopImmediate serial=" << m_serialCStr << "ch=" << channel;

    if (!validateReady(channel, errOut))
        return false;

    short err = BDC_StopImmediate(m_serialCStr, (short)channel);
    const bool stopped = okOrLog("BDC_StopImmediate", m_serial, err, channel, errOut);
    if (!stopped)
        return false;

    short triggerErr = 0;
    const bool triggerDisabled = !isTriggerEnabled(channel) || disableTrigger(channel, &triggerErr);
    if (!triggerDisabled && errOut) *errOut = triggerErr;
    return triggerDisabled;
}

int32_t BDCStage::getPosition(unsigned channel, short* errOut) const
{
    if (!validateReady(channel, errOut))
        return 0;

    if (errOut) *errOut = 0;
    return (int32_t)BDC_GetPosition(m_serialCStr, (short)channel);
}

bool BDCStage::isMoving(unsigned channel, short* errOut) const
{
    if (!validateReady(channel, errOut))
        return false;

    const uint32_t bits = static_cast<uint32_t>(BDC_GetStatusBits(m_serialCStr, static_cast<short>(channel)));
    if (errOut) *errOut = 0;
    return (bits & 0x00000030u) != 0;
}

double BDCStage::getPositionUm(unsigned channel, short* errOut) const
{
    short err = 0;
    const int32_t posDev = getPosition(channel, &err);
    if (err != 0)
    {
        if (errOut) *errOut = err;
        return 0.0;
    }

    const double posUm = deviceToUm(posDev, channel);
    if (errOut) *errOut = 0;
    return posUm;
}

double BDCStage::umPerDeviceUnit(unsigned channel) const
{
    const int idx = channelIndex(channel);
    if (idx < 0)
    {
        qDebug() << "BDCStage::umPerDeviceUnit invalid channel=" << channel;
        return 0.0;
    }
    return factor_um[idx];
}

double BDCStage::mmPerDeviceUnit(unsigned channel) const
{
    const int idx = channelIndex(channel);
    if (idx < 0)
    {
        qDebug() << "BDCStage::mmPerDeviceUnit invalid channel=" << channel;
        return 0.0;
    }
    return factor_position_mm[idx];
}

double BDCStage::getMaxVelocityMmS(unsigned channel, short* errOut) const
{
    if (!validateReady(channel, errOut))
        return 0.0;

    int acceleration = 0;
    int maxVelocity = 0;
    const short err = BDC_GetVelParams(
        m_serialCStr, static_cast<short>(channel), &acceleration, &maxVelocity);
    if (!okOrLog("BDC_GetVelParams", m_serial, err, channel, errOut))
        return 0.0;

    const double velocityMmS = static_cast<double>(maxVelocity)
        * factor_velocity_mm[channelIndex(channel)];
    if (!std::isfinite(velocityMmS) || velocityMmS <= 0.0)
    {
        if (errOut) *errOut = kErrInvalidState;
        return 0.0;
    }

    if (errOut) *errOut = 0;
    return velocityMmS;
}

double BDCStage::deviceToMm(int32_t deviceUnits, unsigned channel) const
{
    const int idx = channelIndex(channel);
    if (idx < 0) return 0.0;
    return static_cast<double>(deviceUnits) * factor_position_mm[idx];
}

double BDCStage::deviceToUm(int32_t deviceUnits, unsigned channel) const
{
    const int idx = channelIndex(channel);
    if (idx < 0) return 0.0;
    return static_cast<double>(deviceUnits) * factor_um[idx];
}

int32_t BDCStage::mmToDevice(double mm, unsigned channel, short* errOut) const
{
    if (!validateReady(channel, errOut))
        return 0;

    if (!std::isfinite(mm))
    {
        qDebug() << "BDCStage::mmToDevice invalid value=" << mm;
        if (errOut) *errOut = kErrInvalidArgument;
        return 0;
    }

    int deviceUnits = 0;
    short err = BDC_GetDeviceUnitFromRealValue(m_serialCStr, (short)channel, mm, &deviceUnits, 0);
    if (!okOrLog("BDC_GetDeviceUnitFromRealValue(pos)", m_serial, err, channel, errOut))
        return 0;

    return (int32_t)deviceUnits;
}

int32_t BDCStage::umToDevice(double um, unsigned channel, short* errOut) const
{
    if (!std::isfinite(um))
    {
        if (errOut) *errOut = kErrInvalidArgument;
        return 0;
    }

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

bool BDCStage::isTriggerEnabled(unsigned channel) const
{
    const int idx = channelIndex(channel);
    return idx >= 0 && m_triggerEnabled[idx];
}

bool BDCStage::applyTriggerConfig(unsigned channel, short* errOut)
{
    if (!validateReady(channel, errOut))
        return false;

    const BDCTriggerConfig requested = m_trig;

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

    if (!requested.enabled)
        return disableTrigger(channel, true, errOut);

    if (!isValidTriggerMode(requested.trigger1Mode)
        || !isValidTriggerMode(requested.trigger2Mode)
        || !isValidTriggerPolarity(requested.trigger1Polarity)
        || !isValidTriggerPolarity(requested.trigger2Polarity)
        || requested.pulseWidthUs < 1
        || requested.pulseWidthUs > 1000000
        || requested.cycleCount <= 0)
    {
        qDebug() << "BDCStage::applyTriggerConfig invalid trigger configuration";
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    const bool needsFwd = requested.trigger1Mode == KMOT_TrigOut_AtPositionStepFwd
        || requested.trigger1Mode == KMOT_TrigOut_AtPositionStepBoth
        || requested.trigger2Mode == KMOT_TrigOut_AtPositionStepFwd
        || requested.trigger2Mode == KMOT_TrigOut_AtPositionStepBoth;
    const bool needsRev = requested.trigger1Mode == KMOT_TrigOut_AtPositionStepRev
        || requested.trigger1Mode == KMOT_TrigOut_AtPositionStepBoth
        || requested.trigger2Mode == KMOT_TrigOut_AtPositionStepRev
        || requested.trigger2Mode == KMOT_TrigOut_AtPositionStepBoth;

    if ((needsFwd && (requested.intervalFwd <= 0 || requested.pulseCountFwd <= 0))
        || (needsRev && (requested.intervalRev <= 0 || requested.pulseCountRev <= 0)))
    {
        qDebug() << "BDCStage::applyTriggerConfig invalid position-step parameters";
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    const int idx = channelIndex(channel);
    if (idx < 0 || !m_positionSafetyReady[idx])
    {
        qDebug() << "BDCStage::applyTriggerConfig refusing M30XY trigger because safety limits are not ready serial="
            << m_serialCStr
            << "ch=" << channel;
        if (errOut) *errOut = kErrInvalidState;
        return false;
    }

    const auto triggerSequenceInsideSafeRange = [this, idx](int32_t start, int32_t interval, int32_t count, int direction, int64_t* lastOut) {
        const int64_t last = static_cast<int64_t>(start)
            + static_cast<int64_t>(direction) * static_cast<int64_t>(interval) * static_cast<int64_t>(count - 1);
        if (lastOut) *lastOut = last;

        return static_cast<int64_t>(start) >= static_cast<int64_t>(m_minPositionDevice[idx])
            && static_cast<int64_t>(start) <= static_cast<int64_t>(m_maxPositionDevice[idx])
            && last >= static_cast<int64_t>(m_minPositionDevice[idx])
            && last <= static_cast<int64_t>(m_maxPositionDevice[idx]);
    };

    int64_t lastTriggerPos = 0;
    if (needsFwd && !triggerSequenceInsideSafeRange(
        requested.startPosFwd, requested.intervalFwd, requested.pulseCountFwd, +1, &lastTriggerPos))
    {
        qDebug() << "BDCStage::applyTriggerConfig refusing unsafe M30XY forward trigger range serial=" << m_serialCStr
            << "ch=" << channel
            << "start=" << requested.startPosFwd
            << "interval=" << requested.intervalFwd
            << "count=" << requested.pulseCountFwd
            << "last=" << lastTriggerPos
            << "allowedMinDevice=" << m_minPositionDevice[idx]
            << "allowedMaxDevice=" << m_maxPositionDevice[idx];
        if (errOut) *errOut = kErrInvalidArgument;
        return false;
    }

    if (needsRev && !triggerSequenceInsideSafeRange(
        requested.startPosRev, requested.intervalRev, requested.pulseCountRev, -1, &lastTriggerPos))
    {
        qDebug() << "BDCStage::applyTriggerConfig refusing unsafe M30XY reverse trigger range serial=" << m_serialCStr
            << "ch=" << channel
            << "start=" << requested.startPosRev
            << "interval=" << requested.intervalRev
            << "count=" << requested.pulseCountRev
            << "last=" << lastTriggerPos
            << "allowedMinDevice=" << m_minPositionDevice[idx]
            << "allowedMaxDevice=" << m_maxPositionDevice[idx];
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

    // Keep both outputs disabled until all positional parameters have been
    // accepted. This prevents stale parameters from becoming active.
    if (!disableTrigger(channel, true, errOut))
        return false;

    short err = BDC_SetTriggerParamsBlock(m_serialCStr, (short)channel, &params);
    if (!okOrLog("BDC_SetTriggerParamsBlock", m_serial, err, channel, errOut))
        return false;

    err = BDC_SetTriggerConfigParamsBlock(m_serialCStr, (short)channel, &cfg);
    if (!okOrLog("BDC_SetTriggerConfigParamsBlock", m_serial, err, channel, errOut))
    {
        short ignored = 0;
        disableTrigger(channel, true, &ignored);
        return false;
    }

    m_trig = requested;
    m_triggerEnabled[channelIndex(channel)] = true;
    if (errOut) *errOut = 0;
    return true;
}

bool BDCStage::disableTrigger(unsigned channel, short* errOut)
{
    return disableTrigger(channel, false, errOut);
}

bool BDCStage::disableTrigger(unsigned channel, bool force, short* errOut)
{
    qDebug() << "BDCStage::disableTrigger serial=" << m_serialCStr << "ch=" << channel;

    if (!validateReady(channel, errOut))
        return false;

    if (!force && !isTriggerEnabled(channel))
    {
        qDebug() << "BDCStage::disableTrigger skip serial=" << m_serialCStr
            << "ch=" << channel
            << "reason=not-enabled";
        if (errOut) *errOut = 0;
        return true;
    }

    qDebug() << "BDCStage::disableTrigger calling BDC_SetTriggerConfigParams serial="
        << m_serialCStr
        << "ch=" << channel
        << "force=" << force
        << "knownEnabled=" << isTriggerEnabled(channel);

    short err = BDC_SetTriggerConfigParams(
        m_serialCStr,
        (short)channel,
        KMOT_TrigDisabled,
        KMOT_TrigPolarityHigh,
        KMOT_TrigDisabled,
        KMOT_TrigPolarityHigh);

    qDebug() << "BDCStage::disableTrigger returned BDC_SetTriggerConfigParams serial="
        << m_serialCStr
        << "ch=" << channel
        << "err=" << err;

    if (!okOrLog("BDC_SetTriggerConfigParams", m_serial, err, channel, errOut))
        return false;

    m_triggerEnabled[channelIndex(channel)] = false;
    m_trig.enabled = m_triggerEnabled[0] || m_triggerEnabled[1];
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

bool BDCStage::waitUntilIdle(unsigned channel, int32_t targetPos, int timeoutMs, short* errOut)
{
    if (!validateReady(channel, errOut))
        return false;

    const auto t0 = std::chrono::steady_clock::now();
    int32_t lastPos = getPosition(channel, errOut);
    int stableCount = 0;
    bool seenMoving = false;

    while (true)
    {
        uint32_t bits = (uint32_t)BDC_GetStatusBits(m_serialCStr, (short)channel);
        int32_t pos = getPosition(channel, errOut);

        const bool moving = isMovingFromStatus(bits);
        if (moving) seenMoving = true;
        if (!moving)
        {
            if (pos == lastPos) stableCount++;
            else stableCount = 0;

            if (stableCount >= 5)
            {
                const int64_t positionError = static_cast<int64_t>(pos) - static_cast<int64_t>(targetPos);
                const double positionErrorUm = std::abs(
                    static_cast<double>(positionError) * factor_um[channelIndex(channel)]);
                if (positionErrorUm <= kPositionToleranceUm)
                {
                    if (errOut) *errOut = 0;
                    return true;
                }

                if (!seenMoving && stableCount < 20)
                {
                    lastPos = pos;
                    sleepMs(50);
                    continue;
                }

                qDebug() << "BDCStage::waitUntilIdle target not reached serial=" << m_serialCStr
                    << "ch=" << channel
                    << "target=" << targetPos
                    << "actual=" << pos;
                if (errOut) *errOut = kErrTargetNotReached;
                return false;
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
            if (errOut) *errOut = kErrTimeout;
            return false;
        }

        sleepMs(50);
    }
}

bool BDCStage::waitUntilHomed(unsigned channel, int timeoutMs, short* errOut)
{
    if (!validateReady(channel, errOut))
        return false;

    const auto t0 = std::chrono::steady_clock::now();
    int idleWithoutHomeCount = 0;

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

        if (!homed && !moving)
        {
            ++idleWithoutHomeCount;
            if (idleWithoutHomeCount >= 40)
            {
                qDebug() << "BDCStage::waitUntilHomed stopped before homing completed serial=" << m_serialCStr
                    << "ch=" << channel;
                if (errOut) *errOut = kErrTargetNotReached;
                return false;
            }
        }
        else
        {
            idleWithoutHomeCount = 0;
        }

        const auto now = std::chrono::steady_clock::now();
        const int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (elapsed > timeoutMs)
        {
            qDebug() << "BDCStage::waitUntilHomed TIMEOUT serial=" << m_serialCStr
                << "ch=" << channel
                << "statusBits=0x" << QString::number(bits, 16);
            if (errOut) *errOut = kErrTimeout;
            return false;
        }

        sleepMs(50);
    }
}
