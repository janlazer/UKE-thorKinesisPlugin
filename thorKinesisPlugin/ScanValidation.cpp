#include "ScanValidation.h"

#include <cmath>
#include <cstdint>
#include <limits>

namespace
{
bool isFinite(double value)
{
    return std::isfinite(value);
}

ScanValidationResult error(ScanValidationError code, std::size_t layer = 0, std::size_t line = 0)
{
    ScanValidationResult result;
    result.error = code;
    result.layerIndex = layer;
    result.lineIndex = line;
    return result;
}
}

double scanTriggerFrequencyHz(const ScanJob& job)
{
    if (!isFinite(job.velocityMmS)
        || !isFinite(job.triggerSpacingUm)
        || job.velocityMmS <= 0.0
        || job.triggerSpacingUm <= 0.0)
        return 0.0;

    return job.velocityMmS / (job.triggerSpacingUm / 1000.0);
}

ScanValidationResult validateScanJob(const ScanJob& job, double maximumTriggerFrequencyHz)
{
    if (job.layers.empty())
        return error(ScanValidationError::EmptyJob);
    if (!isFinite(job.velocityMmS) || job.velocityMmS <= 0.0)
        return error(ScanValidationError::InvalidVelocity);
    if (!isFinite(job.focusDiameterUm) || job.focusDiameterUm <= 0.0)
        return error(ScanValidationError::InvalidFocusDiameter);
    if (!isFinite(job.triggerSpacingUm) || job.triggerSpacingUm <= 0.0)
        return error(ScanValidationError::InvalidTriggerSpacing);
    if (job.pulseWidthUs <= 0 || job.pulseWidthUs > 1000000)
        return error(ScanValidationError::InvalidPulseWidth);
    if (!isFinite(maximumTriggerFrequencyHz) || maximumTriggerFrequencyHz <= 0.0)
        return error(ScanValidationError::InvalidMaximumTriggerFrequency);

    const double frequencyHz = scanTriggerFrequencyHz(job);
    if (!isFinite(frequencyHz) || frequencyHz > maximumTriggerFrequencyHz + 1e-9)
        return error(ScanValidationError::TriggerFrequencyExceeded);

    const double triggerPeriodUs = 1000000.0 / frequencyHz;
    if (static_cast<double>(job.pulseWidthUs) > triggerPeriodUs + 1e-9)
        return error(ScanValidationError::PulseWidthExceedsTriggerPeriod);

    for (std::size_t layerIndex = 0; layerIndex < job.layers.size(); ++layerIndex)
    {
        const ScanLayer& layer = job.layers[layerIndex];
        if (!isFinite(layer.zUm))
            return error(ScanValidationError::InvalidCoordinate, layerIndex);
        if (layer.lines.empty())
            return error(ScanValidationError::EmptyLayer, layerIndex);

        for (std::size_t lineIndex = 0; lineIndex < layer.lines.size(); ++lineIndex)
        {
            const LaserLine& line = layer.lines[lineIndex];
            if (!isFinite(line.start.xUm)
                || !isFinite(line.start.yUm)
                || !isFinite(line.end.xUm)
                || !isFinite(line.end.yUm))
                return error(ScanValidationError::InvalidCoordinate, layerIndex, lineIndex);

            const double dxUm = line.end.xUm - line.start.xUm;
            const double dyUm = line.end.yUm - line.start.yUm;
            const double lengthUm = std::hypot(dxUm, dyUm);
            if (!isFinite(lengthUm) || lengthUm <= 0.0)
                return error(ScanValidationError::ZeroLengthLine, layerIndex, lineIndex);

            const double pulseCount = std::floor(lengthUm / job.triggerSpacingUm) + 1.0;
            if (!isFinite(pulseCount)
                || pulseCount > static_cast<double>(std::numeric_limits<int32_t>::max()))
                return error(ScanValidationError::PulseCountOverflow, layerIndex, lineIndex);
        }
    }

    return ScanValidationResult{};
}
