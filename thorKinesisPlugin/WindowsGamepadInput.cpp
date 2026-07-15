#include "WindowsGamepadInput.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <hidsdi.h>
#include <hidpi.h>
#include <Xinput.h>

#include <QtGlobal>

#include <algorithm>
#include <vector>

namespace
{
    constexpr USAGE kGenericDesktopUsagePage = 0x01;
    constexpr USAGE kJoystickUsage = 0x04;
    constexpr USAGE kGamepadUsage = 0x05;
    constexpr USAGE kXUsage = 0x30;
    constexpr USAGE kYUsage = 0x31;
    constexpr USAGE kHatSwitchUsage = 0x39;
    constexpr USAGE kButtonUsagePage = 0x09;

    enum class Backend
    {
        XInput,
        Hid
    };

    struct DeviceRecord
    {
        WindowsGamepadInput::DeviceInfo info;
        Backend backend = Backend::Hid;
        DWORD xinputIndex = 0;
        QString devicePath;
    };

    QString devicePath(HANDLE rawDevice)
    {
        UINT pathLength = 0;
        if (GetRawInputDeviceInfoW(rawDevice, RIDI_DEVICENAME, nullptr, &pathLength) != 0
            || pathLength == 0)
        {
            return QString();
        }

        QVector<ushort> path(static_cast<int>(pathLength + 1), 0);
        if (GetRawInputDeviceInfoW(rawDevice, RIDI_DEVICENAME,
            path.data(), &pathLength) == static_cast<UINT>(-1))
        {
            return QString();
        }
        return QString::fromUtf16(path.constData()).trimmed();
    }

    QString hidProductName(const QString& path)
    {
        HANDLE file = CreateFileW(
            reinterpret_cast<LPCWSTR>(path.utf16()),
            0,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr);
        if (file == INVALID_HANDLE_VALUE)
            return QString();

        WCHAR productName[128]{};
        const bool ok = HidD_GetProductString(file, productName, sizeof(productName)) == TRUE;
        CloseHandle(file);
        return ok
            ? QString::fromUtf16(reinterpret_cast<const ushort*>(productName)).trimmed()
            : QString();
    }

    double normalizedHidAxis(LONG value, LONG minimum, LONG maximum)
    {
        if (maximum <= minimum)
            return 0.0;
        const double unit = static_cast<double>(value - minimum)
            / static_cast<double>(maximum - minimum);
        return qBound(-1.0, unit * 2.0 - 1.0, 1.0);
    }

    LONG signedHidValue(ULONG value, const HIDP_VALUE_CAPS& caps)
    {
        if (caps.LogicalMin >= 0 || caps.BitSize == 0 || caps.BitSize >= 32)
            return static_cast<LONG>(value);

        const ULONG signBit = 1UL << (caps.BitSize - 1);
        if ((value & signBit) == 0)
            return static_cast<LONG>(value);
        const ULONG mask = (1UL << caps.BitSize) - 1UL;
        return static_cast<LONG>(value | ~mask);
    }

    void applyHatValue(ULONG value, const HIDP_VALUE_CAPS& caps,
        WindowsGamepadInput::State& state)
    {
        const LONG position = signedHidValue(value, caps) - caps.LogicalMin;
        if (position < 0 || position > 7)
            return;

        state.up = position == 0 || position == 1 || position == 7;
        state.right = position == 1 || position == 2 || position == 3;
        state.down = position == 3 || position == 4 || position == 5;
        state.left = position == 5 || position == 6 || position == 7;
    }

    void setButton(WindowsGamepadInput::State& state, int index, bool pressed)
    {
        if (index >= 0 && index < WindowsGamepadInput::ButtonCount)
            state.buttons[static_cast<size_t>(index)] = pressed;
    }
}

struct WindowsGamepadInput::Impl
{
    QVector<DeviceRecord> records;
    QString selectedKey;
    QString selectedName;
    Backend selectedBackend = Backend::Hid;
    DWORD selectedXInputIndex = 0;

    HANDLE hidFile = INVALID_HANDLE_VALUE;
    HANDLE readEvent = nullptr;
    OVERLAPPED readOperation{};
    PHIDP_PREPARSED_DATA preparsedData = nullptr;
    HIDP_CAPS caps{};
    std::vector<HIDP_BUTTON_CAPS> buttonCaps;
    std::vector<HIDP_VALUE_CAPS> valueCaps;
    std::vector<UCHAR> inputReport;
    bool readPending = false;
    bool reportReady = false;
    DWORD readyReportBytes = 0;
    State lastState;

    ~Impl()
    {
        releaseDevice();
    }

    void releaseDevice()
    {
        if (hidFile != INVALID_HANDLE_VALUE)
        {
            CancelIoEx(hidFile, &readOperation);
            CloseHandle(hidFile);
            hidFile = INVALID_HANDLE_VALUE;
        }
        if (readEvent)
        {
            CloseHandle(readEvent);
            readEvent = nullptr;
        }
        if (preparsedData)
        {
            HidD_FreePreparsedData(preparsedData);
            preparsedData = nullptr;
        }
        readOperation = OVERLAPPED{};
        buttonCaps.clear();
        valueCaps.clear();
        inputReport.clear();
        readPending = false;
        reportReady = false;
        readyReportBytes = 0;
        lastState = State{};
    }

    bool queueRead()
    {
        if (hidFile == INVALID_HANDLE_VALUE || !readEvent || inputReport.empty())
            return false;

        ResetEvent(readEvent);
        readOperation = OVERLAPPED{};
        readOperation.hEvent = readEvent;
        DWORD bytesRead = 0;
        if (ReadFile(hidFile, inputReport.data(),
            static_cast<DWORD>(inputReport.size()), &bytesRead, &readOperation))
        {
            readPending = false;
            reportReady = true;
            readyReportBytes = bytesRead;
            return true;
        }

        if (GetLastError() == ERROR_IO_PENDING)
        {
            readPending = true;
            reportReady = false;
            readyReportBytes = 0;
            return true;
        }

        readPending = false;
        return false;
    }

    bool openHid(const QString& path)
    {
        releaseDevice();
        hidFile = CreateFileW(
            reinterpret_cast<LPCWSTR>(path.utf16()),
            GENERIC_READ,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            nullptr,
            OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED,
            nullptr);
        if (hidFile == INVALID_HANDLE_VALUE)
            return false;

        if (!HidD_GetPreparsedData(hidFile, &preparsedData)
            || HidP_GetCaps(preparsedData, &caps) != HIDP_STATUS_SUCCESS
            || caps.InputReportByteLength == 0)
        {
            releaseDevice();
            return false;
        }

        buttonCaps.resize(caps.NumberInputButtonCaps);
        if (!buttonCaps.empty())
        {
            USHORT count = static_cast<USHORT>(buttonCaps.size());
            if (HidP_GetButtonCaps(HidP_Input, buttonCaps.data(), &count,
                preparsedData) != HIDP_STATUS_SUCCESS)
            {
                buttonCaps.clear();
            }
            else
            {
                buttonCaps.resize(count);
            }
        }

        valueCaps.resize(caps.NumberInputValueCaps);
        if (!valueCaps.empty())
        {
            USHORT count = static_cast<USHORT>(valueCaps.size());
            if (HidP_GetValueCaps(HidP_Input, valueCaps.data(), &count,
                preparsedData) != HIDP_STATUS_SUCCESS)
            {
                valueCaps.clear();
            }
            else
            {
                valueCaps.resize(count);
            }
        }

        inputReport.assign(caps.InputReportByteLength, 0);
        readEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr);
        lastState = State{};
        lastState.connected = true;
        if (!readEvent || !queueRead())
        {
            releaseDevice();
            return false;
        }
        return true;
    }

    void parseButtons(State& state, ULONG reportLength)
    {
        for (const HIDP_BUTTON_CAPS& buttonCap : buttonCaps)
        {
            if (buttonCap.UsagePage != kButtonUsagePage)
                continue;

            ULONG usageCount = HidP_MaxUsageListLength(
                HidP_Input, buttonCap.UsagePage, preparsedData);
            if (usageCount == 0)
                continue;

            std::vector<USAGE> usages(usageCount);
            if (HidP_GetUsages(HidP_Input,
                buttonCap.UsagePage,
                buttonCap.LinkCollection,
                usages.data(),
                &usageCount,
                preparsedData,
                reinterpret_cast<PCHAR>(inputReport.data()),
                reportLength) != HIDP_STATUS_SUCCESS)
            {
                continue;
            }

            for (ULONG index = 0; index < usageCount; ++index)
                setButton(state, static_cast<int>(usages[index]) - 1, true);
        }
    }

    void parseValueUsage(const HIDP_VALUE_CAPS& valueCap, USAGE usage,
        State& state, ULONG reportLength)
    {
        ULONG rawValue = 0;
        if (HidP_GetUsageValue(HidP_Input,
            valueCap.UsagePage,
            valueCap.LinkCollection,
            usage,
            &rawValue,
            preparsedData,
            reinterpret_cast<PCHAR>(inputReport.data()),
            reportLength) != HIDP_STATUS_SUCCESS)
        {
            return;
        }

        if (usage == kHatSwitchUsage)
        {
            applyHatValue(rawValue, valueCap, state);
            return;
        }

        const LONG value = signedHidValue(rawValue, valueCap);
        if (usage == kXUsage)
            state.axisX = normalizedHidAxis(value, valueCap.LogicalMin, valueCap.LogicalMax);
        else if (usage == kYUsage)
            state.axisY = normalizedHidAxis(value, valueCap.LogicalMin, valueCap.LogicalMax);
    }

    void parseValues(State& state, ULONG reportLength)
    {
        for (const HIDP_VALUE_CAPS& valueCap : valueCaps)
        {
            if (valueCap.UsagePage != kGenericDesktopUsagePage)
                continue;

            if (valueCap.IsRange)
            {
                for (USAGE usage = valueCap.Range.UsageMin;
                    usage <= valueCap.Range.UsageMax; ++usage)
                {
                    parseValueUsage(valueCap, usage, state, reportLength);
                    if (usage == valueCap.Range.UsageMax)
                        break;
                }
            }
            else
            {
                parseValueUsage(valueCap, valueCap.NotRange.Usage, state, reportLength);
            }
        }
    }

    void parseReport(DWORD reportLength)
    {
        State state;
        state.connected = true;
        parseButtons(state, reportLength);
        parseValues(state, reportLength);
        lastState = state;
    }
};

WindowsGamepadInput::WindowsGamepadInput()
    : m_impl(std::make_unique<Impl>())
{
}

WindowsGamepadInput::~WindowsGamepadInput() = default;

QVector<WindowsGamepadInput::DeviceInfo> WindowsGamepadInput::refreshDevices(
    quintptr nativeWindowHandle)
{
    Q_UNUSED(nativeWindowHandle);
    m_impl->records.clear();

    for (DWORD index = 0; index < XUSER_MAX_COUNT; ++index)
    {
        XINPUT_STATE state{};
        if (XInputGetState(index, &state) != ERROR_SUCCESS)
            continue;

        DeviceRecord record;
        record.backend = Backend::XInput;
        record.xinputIndex = index;
        record.info.key = QStringLiteral("xinput:%1").arg(index);
        record.info.name = QStringLiteral("XInput Controller %1").arg(index + 1);
        m_impl->records.append(record);
    }

    UINT rawDeviceCount = 0;
    if (GetRawInputDeviceList(nullptr, &rawDeviceCount,
        sizeof(RAWINPUTDEVICELIST)) == 0 && rawDeviceCount > 0)
    {
        std::vector<RAWINPUTDEVICELIST> rawDevices(rawDeviceCount);
        if (GetRawInputDeviceList(rawDevices.data(), &rawDeviceCount,
            sizeof(RAWINPUTDEVICELIST)) != static_cast<UINT>(-1))
        {
            for (const RAWINPUTDEVICELIST& rawDevice : rawDevices)
            {
                if (rawDevice.dwType != RIM_TYPEHID)
                    continue;

                RID_DEVICE_INFO info{};
                info.cbSize = sizeof(info);
                UINT infoSize = sizeof(info);
                if (GetRawInputDeviceInfoW(rawDevice.hDevice, RIDI_DEVICEINFO,
                    &info, &infoSize) == static_cast<UINT>(-1))
                {
                    continue;
                }
                if (info.hid.usUsagePage != kGenericDesktopUsagePage
                    || (info.hid.usUsage != kJoystickUsage
                        && info.hid.usUsage != kGamepadUsage))
                {
                    continue;
                }

                const QString path = devicePath(rawDevice.hDevice);
                if (path.isEmpty())
                    continue;

                DeviceRecord record;
                record.backend = Backend::Hid;
                record.devicePath = path;
                record.info.key = QStringLiteral("hid:") + path.toLower();
                QString productName = hidProductName(path);
                if (productName.isEmpty())
                    productName = QStringLiteral("HID Gamepad");
                record.info.name = QStringLiteral("%1 (HID %2:%3)")
                    .arg(productName)
                    .arg(info.hid.dwVendorId, 4, 16, QLatin1Char('0'))
                    .arg(info.hid.dwProductId, 4, 16, QLatin1Char('0'));
                m_impl->records.append(record);
            }
        }
    }

    QVector<DeviceInfo> devices;
    devices.reserve(m_impl->records.size());
    for (const DeviceRecord& record : m_impl->records)
        devices.append(record.info);
    return devices;
}

bool WindowsGamepadInput::selectDevice(const QString& key, quintptr nativeWindowHandle)
{
    if (key.isEmpty())
    {
        clearSelection();
        return false;
    }

    refreshDevices(nativeWindowHandle);
    const auto found = std::find_if(
        m_impl->records.cbegin(), m_impl->records.cend(),
        [&key](const DeviceRecord& record) { return record.info.key == key; });
    if (found == m_impl->records.cend())
    {
        clearSelection();
        return false;
    }

    m_impl->releaseDevice();
    m_impl->selectedKey = found->info.key;
    m_impl->selectedName = found->info.name;
    m_impl->selectedBackend = found->backend;
    m_impl->selectedXInputIndex = found->xinputIndex;
    if (found->backend == Backend::XInput)
        return true;

    if (!m_impl->openHid(found->devicePath))
    {
        clearSelection();
        return false;
    }
    return true;
}

bool WindowsGamepadInput::poll(State& state)
{
    state = State{};
    if (m_impl->selectedKey.isEmpty())
        return false;

    if (m_impl->selectedBackend == Backend::XInput)
    {
        XINPUT_STATE input{};
        if (XInputGetState(m_impl->selectedXInputIndex, &input) != ERROR_SUCCESS)
            return false;

        state.connected = true;
        state.axisX = input.Gamepad.sThumbLX < 0
            ? static_cast<double>(input.Gamepad.sThumbLX) / 32768.0
            : static_cast<double>(input.Gamepad.sThumbLX) / 32767.0;
        state.axisY = input.Gamepad.sThumbLY < 0
            ? -static_cast<double>(input.Gamepad.sThumbLY) / 32768.0
            : -static_cast<double>(input.Gamepad.sThumbLY) / 32767.0;
        state.up = (input.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP) != 0;
        state.down = (input.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN) != 0;
        state.left = (input.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) != 0;
        state.right = (input.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;
        setButton(state, 0, (input.Gamepad.wButtons & XINPUT_GAMEPAD_A) != 0);
        setButton(state, 1, (input.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0);
        setButton(state, 2, (input.Gamepad.wButtons & XINPUT_GAMEPAD_X) != 0);
        setButton(state, 3, (input.Gamepad.wButtons & XINPUT_GAMEPAD_Y) != 0);
        setButton(state, 4, (input.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0);
        setButton(state, 5, (input.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0);
        setButton(state, 6, (input.Gamepad.wButtons & XINPUT_GAMEPAD_BACK) != 0);
        setButton(state, 7, (input.Gamepad.wButtons & XINPUT_GAMEPAD_START) != 0);
        setButton(state, 8, (input.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB) != 0);
        setButton(state, 9, (input.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) != 0);
        setButton(state, 10, input.Gamepad.bLeftTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD);
        setButton(state, 11, input.Gamepad.bRightTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD);
        return true;
    }

    if (m_impl->hidFile == INVALID_HANDLE_VALUE)
        return false;

    if (m_impl->readPending)
    {
        DWORD bytesRead = 0;
        if (GetOverlappedResult(m_impl->hidFile, &m_impl->readOperation,
            &bytesRead, FALSE))
        {
            m_impl->readPending = false;
            m_impl->reportReady = true;
            m_impl->readyReportBytes = bytesRead;
        }
        else if (GetLastError() != ERROR_IO_INCOMPLETE)
        {
            return false;
        }
    }

    if (m_impl->reportReady)
    {
        m_impl->parseReport(m_impl->readyReportBytes);
        m_impl->reportReady = false;
        m_impl->readyReportBytes = 0;
        if (!m_impl->queueRead())
            return false;
    }

    state = m_impl->lastState;
    state.connected = true;
    return true;
}

void WindowsGamepadInput::clearSelection()
{
    m_impl->releaseDevice();
    m_impl->selectedKey.clear();
    m_impl->selectedName.clear();
    m_impl->selectedXInputIndex = 0;
}

QString WindowsGamepadInput::selectedDeviceKey() const
{
    return m_impl->selectedKey;
}

QString WindowsGamepadInput::selectedDeviceName() const
{
    return m_impl->selectedName;
}
