#pragma once
#include <string>
#include <vector>

struct KinesisDeviceInfo
{
    std::string serial;   // raw serial from TLI list
    int deviceType = 0;   // TLI type id (24=KVS30, 101=BDC rack/controller etc.)
};

std::vector<KinesisDeviceInfo> detectKinesisDevices();