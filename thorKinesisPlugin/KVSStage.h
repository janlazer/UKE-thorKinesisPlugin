#pragma once
#include <string>
#include <cstdint>

enum class KVSTriggerMode : int
{
    Disabled = 0x00,
    PositionSteps = 0x0D
};

enum class KVSTriggerPolarity : int
{
    High = 0x01,
    Low = 0x02
};

struct KVSTriggerConfig
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

class KVSStage
{
public:
    explicit KVSStage(const std::string& serial);
    ~KVSStage();

    bool open(const std::string& serial, bool home, short* errOut = nullptr);
    void close();

    // Motion - low level device units
    bool home(short* errOut = nullptr);
    bool homeIfNeeded(short* errOut = nullptr);

    bool beginMoveTo(int32_t pos, short* errOut = nullptr);
    bool waitForPosition(int32_t targetPos, int timeoutMs = 120000, short* errOut = nullptr);
    bool moveTo(int32_t pos, short* errOut = nullptr);
    bool moveRel(int32_t delta, short* errOut = nullptr);
    bool stopImmediate(short* errOut = nullptr);

    int32_t getPosition(short* errOut = nullptr) const;
    bool isMoving(short* errOut = nullptr) const;
    bool isHomed(short* errOut = nullptr) const;

    // Motion - public standard unit: micrometers (µm)
    bool moveToUm(double posUm, short* errOut = nullptr);
    bool moveRelUm(double deltaUm, short* errOut = nullptr);
    double getPositionUm(short* errOut = nullptr) const;

    // Unit conversion helpers
    double umPerDeviceUnit() const { return factor_um; }
    double mmPerDeviceUnit() const { return factor_position_mm; }
    double getMaxVelocityMmS(short* errOut = nullptr) const;
    bool setMaxVelocityMmS(double maxVelocityMmS, short* errOut = nullptr);

    double deviceToUm(int32_t deviceUnits) const;
    double deviceToMm(int32_t deviceUnits) const;

    int32_t umToDevice(double um, short* errOut = nullptr) const;
    int32_t mmToDevice(double mm, short* errOut = nullptr) const;

    // Trigger
    void setTriggerConfig(const KVSTriggerConfig& cfg);
    const KVSTriggerConfig& triggerConfig() const;

    bool applyTriggerConfig(short* errOut = nullptr);
    bool disableTrigger(short* errOut = nullptr);

    const std::string& serial() const { return m_serial; }
    bool isOpen() const { return m_isOpen; }

private:
    static void logErr(const char* fn, const std::string& serial, short err);
    static bool okOrLog(const char* fn, const std::string& serial, short err, short* errOut = nullptr);

    bool enable(short* errOut = nullptr);
    bool enableIfNeeded(unsigned channel, short* errOut);
    bool validateOpen(short* errOut = nullptr) const;
    bool waitUntilIdle(int32_t targetPos, int timeoutMs, short* errOut = nullptr);
    bool waitUntilHomed(int timeoutMs, short* errOut = nullptr);

private:
    std::string m_serial;
    const char* m_serialCStr = nullptr;
    bool m_isOpen = false;
    KVSTriggerConfig m_trig;

    // Internal scaling:
    // Kinesis real units for linear stages are assumed to be mm.
    double factor_position_mm = 1.0;       // mm per device unit
    double factor_velocity_mm = 1.0;       // mm/s per device unit
    double factor_acceleration_mm = 1.0;   // mm/s^2 per device unit

    // Public standard unit for this wrapper
    double factor_um = 1000.0;             // µm per device unit
};
