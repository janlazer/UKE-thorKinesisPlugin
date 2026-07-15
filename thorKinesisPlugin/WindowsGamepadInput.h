#pragma once

#include <QString>
#include <QVector>

#include <array>
#include <memory>

class WindowsGamepadInput
{
public:
    static constexpr int ButtonCount = 32;

    struct DeviceInfo
    {
        QString key;
        QString name;
    };

    struct State
    {
        bool connected = false;
        double axisX = 0.0;
        double axisY = 0.0;
        bool up = false;
        bool down = false;
        bool left = false;
        bool right = false;
        std::array<bool, ButtonCount> buttons{};
    };

    WindowsGamepadInput();
    ~WindowsGamepadInput();

    WindowsGamepadInput(const WindowsGamepadInput&) = delete;
    WindowsGamepadInput& operator=(const WindowsGamepadInput&) = delete;

    QVector<DeviceInfo> refreshDevices(quintptr nativeWindowHandle);
    bool selectDevice(const QString& key, quintptr nativeWindowHandle);
    bool poll(State& state);
    void clearSelection();

    QString selectedDeviceKey() const;
    QString selectedDeviceName() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
