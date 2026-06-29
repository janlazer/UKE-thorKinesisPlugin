#pragma once

#include "interfaces.h"

double scanTriggerFrequencyHz(const ScanJob& job);
ScanValidationResult validateScanJob(const ScanJob& job, double maximumTriggerFrequencyHz = 20.0);
