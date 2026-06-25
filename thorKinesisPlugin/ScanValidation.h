#pragma once

#include "ScanTypes.h"

#include <cstddef>

enum class ScanValidationError
{
    None,
    EmptyJob,
    InvalidVelocity,
    InvalidFocusDiameter,
    InvalidTriggerSpacing,
    InvalidPulseWidth,
    InvalidMaximumTriggerFrequency,
    TriggerFrequencyExceeded,
    PulseWidthExceedsTriggerPeriod,
    EmptyLayer,
    InvalidCoordinate,
    ZeroLengthLine,
    PulseCountOverflow,
    MissingScanAxis,
    UnsupportedScanGeometry,
    UnsupportedScanVelocity
};

struct ScanValidationResult
{
    ScanValidationError error = ScanValidationError::None;
    std::size_t layerIndex = 0;
    std::size_t lineIndex = 0;

    bool isValid() const { return error == ScanValidationError::None; }
};

double scanTriggerFrequencyHz(const ScanJob& job);
ScanValidationResult validateScanJob(const ScanJob& job, double maximumTriggerFrequencyHz = 20.0);
