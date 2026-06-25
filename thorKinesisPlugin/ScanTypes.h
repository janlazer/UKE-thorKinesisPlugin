#pragma once

#include <cstdint>
#include <vector>

struct ScanPoint
{
    double xUm = 0.0;
    double yUm = 0.0;
};

struct LaserLine
{
    ScanPoint start;
    ScanPoint end;
};

struct ScanLayer
{
    double zUm = 0.0;
    std::vector<LaserLine> lines;
};

struct ScanJob
{
    std::vector<ScanLayer> layers;

    double velocityMmS = 0.0;
    double focusDiameterUm = 0.0;
    double triggerSpacingUm = 0.0;
    int32_t pulseWidthUs = 0;
};

struct ScanCapabilities
{
    bool supportsScanJobs = false;
    bool supportsHardwarePositionTrigger = false;
    bool supportsHorizontalLines = false;
    bool supportsVerticalLines = false;
    bool supportsDiagonalLines = false;
    bool supportsLayeredZ = false;
    bool controlsScanVelocity = false;

    double maximumTriggerFrequencyHz = 0.0;
};
