// kinesis
#include "KinesisDetect.h"

#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>

#include <QDebug>

// Include BOTH headers you actually use in this project.
// They both provide TLI_* declarations (duplicated but guarded).
#include "Thorlabs.MotionControl.Benchtop.DCServo.h"
//#include "Thorlabs.MotionControl.VerticalStage.h"

static void appendSerialsFromType(std::vector<KinesisDeviceInfo>& out, int typeId)
{
    char buffer[4096] = { 0 };
    const short err = TLI_GetDeviceListByTypeExt(buffer, sizeof(buffer), typeId);

    qDebug() << "KinesisDetect - TLI_GetDeviceListByTypeExt type" << typeId << "err=" << err
        << "buffer=" << buffer;

    if (err != 0)
    {
        qDebug() << "KinesisDetect - ignoring device list because the SDK returned an error";
        return;
    }

    char* ctx = nullptr;
    char* token = strtok_s(buffer, ",", &ctx);

    while (token)
    {
        std::string serial(token);
        serial.erase(serial.begin(), std::find_if(serial.begin(), serial.end(), [](unsigned char c) {
            return !std::isspace(c);
        }));
        serial.erase(std::find_if(serial.rbegin(), serial.rend(), [](unsigned char c) {
            return !std::isspace(c);
        }).base(), serial.end());

        if (!serial.empty())
        {
            KinesisDeviceInfo info;
            info.serial = serial;
            info.deviceType = typeId;
            out.push_back(info);

            qDebug() << "KinesisDetect - found type" << typeId << "serial=" << serial.c_str();
        }
        token = strtok_s(nullptr, ",", &ctx);
    }
}

std::vector<KinesisDeviceInfo> detectKinesisDevices()
{
    std::vector<KinesisDeviceInfo> devices;

    const short err = TLI_BuildDeviceList();
    if (err != 0)
    {
        qDebug() << "KinesisDetect - TLI_BuildDeviceList failed:" << err;
        return devices;
    }

    // 24 = Vertical Stage controller (KVS30)
    appendSerialsFromType(devices, 24);

    // 101 = Benchtop DC Servo (BDC10x / integrated DC servo units)
    // Note: Depending on Kinesis + device, your M30XY base may show as 101,
    // while per-channel virtual serials might show differently in the UI.
    appendSerialsFromType(devices, 101);

    qDebug() << "KinesisDetect - total devices:" << devices.size();
    return devices;
}
