#pragma once

#include <string>
#include <cstdint>

enum class BDCTriggerMode : int
{
    Disabled = 0x00,
    PositionStepForward = 0x0D,
    PositionStepReverse = 0x0E,
    PositionStepBoth = 0x0F
};

enum class BDCTriggerPolarity : int
{
    High = 0x01,
    Low = 0x02
};

struct BDCTriggerConfig
{
    bool enabled = false;

    int trigger1Mode = 0;
    int trigger1Polarity = 1;
    int trigger2Mode = 0;
    int trigger2Polarity = 1;

    int32_t startPosFwd = 0;
    int32_t intervalFwd = 0;
    int32_t pulseCountFwd = 1;

    int32_t startPosRev = 0;
    int32_t intervalRev = 0;
    int32_t pulseCountRev = 1;

    int32_t cycleCount = 1;
    int32_t pulseWidthUs = 50000;
};

class BDCStage
{
public:
    explicit BDCStage(const std::string& baseSerial);
    ~BDCStage();

    bool open(const std::string& baseSerial, bool home, short* errOut = nullptr);
    void close();

    // Motion - low level device units
    bool homeAll(short* errOut = nullptr);
    bool home(unsigned channel, short* errOut = nullptr);
    bool homeAllIfNeeded(short* errOut = nullptr);
    bool homeIfNeeded(unsigned channel, short* errOut = nullptr);

    bool beginMoveTo(int32_t pos, unsigned channel, short* errOut = nullptr);
    bool waitForPosition(int32_t targetPos, unsigned channel, int timeoutMs = 120000, short* errOut = nullptr);
    bool moveTo(int32_t pos, unsigned channel, short* errOut = nullptr);
    bool moveRel(int32_t delta, unsigned channel, short* errOut = nullptr);
    bool stopImmediate(unsigned channel, short* errOut = nullptr);

    int32_t getPosition(unsigned channel, short* errOut = nullptr) const;
    bool isMoving(unsigned channel, short* errOut = nullptr) const;

    // Motion - public standard unit: micrometers (µm)
    bool moveToUm(double posUm, unsigned channel, short* errOut = nullptr);
    bool moveRelUm(double deltaUm, unsigned channel, short* errOut = nullptr);
    double getPositionUm(unsigned channel, short* errOut = nullptr) const;

    // Unit conversion helpers
    double umPerDeviceUnit(unsigned channel) const;
    double mmPerDeviceUnit(unsigned channel) const;
    double getMaxVelocityMmS(unsigned channel, short* errOut = nullptr) const;

    double deviceToUm(int32_t deviceUnits, unsigned channel) const;
    double deviceToMm(int32_t deviceUnits, unsigned channel) const;
    int32_t umToDevice(double um, unsigned channel, short* errOut = nullptr) const;
    int32_t mmToDevice(double mm, unsigned channel, short* errOut = nullptr) const;

    // Trigger API
    void setTriggerConfig(const BDCTriggerConfig& cfg);
    const BDCTriggerConfig& triggerConfig() const;

    bool applyTriggerConfig(unsigned channel, short* errOut = nullptr);
    bool disableTrigger(unsigned channel, short* errOut = nullptr);
    bool disableTrigger(unsigned channel, bool force, short* errOut = nullptr);
    bool isTriggerEnabled(unsigned channel) const;

    const std::string& serial() const { return m_serial; }
    bool isOpen() const { return m_isOpen; }

private:
    static void logErr(const char* fn, const std::string& serial, short err, unsigned ch = 0);
    static bool okOrLog(const char* fn, const std::string& serial, short err, unsigned ch = 0, short* errOut = nullptr);

    bool startPollingAll(short* errOut = nullptr);
    bool enableIfNeeded(unsigned channel, short* errOut = nullptr);
    bool enableAll(short* errOut = nullptr);

    bool validateOpen(short* errOut = nullptr) const;
    bool validateChannel(unsigned channel, short* errOut = nullptr) const;
    bool validateReady(unsigned channel, short* errOut = nullptr) const;

    bool waitUntilIdle(unsigned channel, int32_t targetPos, int timeoutMs, short* errOut = nullptr);
    bool waitUntilHomed(unsigned channel, int timeoutMs, short* errOut = nullptr);

private:
    std::string m_serial;
    const char* m_serialCStr = nullptr;
    bool m_isOpen = false;
    BDCTriggerConfig m_trig;
    bool m_triggerEnabled[2] = { false, false };

    // Safety limits read and validated from Kinesis at open time.
    bool m_positionSafetyReady[2] = { false, false };
    int32_t m_minPositionDevice[2] = { 0, 0 };
    int32_t m_maxPositionDevice[2] = { 0, 0 };
    double m_minPositionUm[2] = { 0.0, 0.0 };
    double m_maxPositionUm[2] = { 0.0, 0.0 };

    // Internal scaling: real units assumed mm for linear stages
    double factor_position_mm[2] = { 1.0, 1.0 };       // mm / device unit
    double factor_velocity_mm[2] = { 1.0, 1.0 };       // mm/s / device unit
    double factor_acceleration_mm[2] = { 1.0, 1.0 };   // mm/s^2 / device unit

    // Public standard unit
    double factor_um[2] = { 1000.0, 1000.0 };          // µm / device unit
};
