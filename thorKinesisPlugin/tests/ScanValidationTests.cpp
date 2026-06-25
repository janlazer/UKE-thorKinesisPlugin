#include "../ScanValidation.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace
{
int failures = 0;

void expect(bool condition, const char* message)
{
    if (condition) return;
    ++failures;
    std::cerr << "FAILED: " << message << '\n';
}

ScanJob validJob(double focusDiameterUm = 50.0)
{
    ScanJob job;
    job.velocityMmS = 2.0;
    job.focusDiameterUm = focusDiameterUm;
    job.triggerSpacingUm = 100.0;
    job.pulseWidthUs = 50000;

    ScanLayer layer;
    layer.zUm = 0.0;
    LaserLine line;
    line.start.xUm = 0.0;
    line.start.yUm = 0.0;
    line.end.xUm = 1000.0;
    line.end.yUm = 0.0;
    layer.lines.push_back(line);
    job.layers.push_back(layer);
    return job;
}
}

int main()
{
    for (double focus : { 10.0, 20.0, 35.0, 50.0, 100.0 })
        expect(validateScanJob(validJob(focus)).isValid(), "supported focus diameter must be accepted");

    ScanJob job = validJob();
    expect(std::abs(scanTriggerFrequencyHz(job) - 20.0) < 1e-9, "frequency calculation");

    job.velocityMmS = 2.01;
    expect(validateScanJob(job).error == ScanValidationError::TriggerFrequencyExceeded,
        "velocity above the 20 Hz limit must fail");

    job = validJob();
    job.triggerSpacingUm = 0.0;
    expect(validateScanJob(job).error == ScanValidationError::InvalidTriggerSpacing,
        "zero spacing must fail");

    job = validJob();
    job.layers[0].lines[0].end.xUm = std::numeric_limits<double>::quiet_NaN();
    expect(validateScanJob(job).error == ScanValidationError::InvalidCoordinate,
        "non-finite coordinates must fail");

    job = validJob();
    job.layers[0].lines[0].end = job.layers[0].lines[0].start;
    expect(validateScanJob(job).error == ScanValidationError::ZeroLengthLine,
        "zero-length line must fail");

    job = validJob();
    job.velocityMmS = 1.0;
    job.pulseWidthUs = 100001;
    expect(validateScanJob(job).error == ScanValidationError::PulseWidthExceedsTriggerPeriod,
        "pulse width beyond the trigger period must fail");

    if (failures == 0)
        std::cout << "All ScanValidation tests passed.\n";
    return failures == 0 ? 0 : 1;
}
