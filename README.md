# UKE Thorlabs Kinesis Plugin

A Qt 5 multi-axis stage and scan plugin for UKE SmartLab. It controls supported
Thorlabs Kinesis stages, presents each detected axis separately, and provides
coordinated motion, hardware-triggered scans, saved positions, and optional
gamepad control.

## Supported Hardware

The current implementation contains dedicated backends for:

- Thorlabs M30XY stages through the Benchtop DC Servo API
- Thorlabs KVS vertical stages through the Vertical Stage API

Other Kinesis devices are not automatically supported merely because they are
visible in the Kinesis device list.

## Features

- Selective initialization of detected X, Y, and Z axes.
- Absolute and relative movement in micrometers.
- Homing, stop controls, velocity configuration, and continuous position
  updates.
- Travel-limit validation and coordinated multi-axis movement.
- Named position configurations with a position manager.
- Scan-job validation and line-trigger configuration.
- Per-axis trigger start, spacing, count, pulse width, and velocity settings.
- Native Windows HID/XInput gamepad support with configurable axes, speed
  modifiers, Z limits, and trigger output.
- Implements `IScanStage`, `IStageMultiAxis`, `IStage`, and `IHardware`.

## Requirements

- Windows 10 or newer, x64
- Visual Studio 2022 with the MSVC v143 toolset and Windows SDK
- Qt 5.15.2 for MSVC 2019 x64
- Qwt built for the same Qt/MSVC configuration
- Thorlabs Kinesis installed in `C:\Program Files\Thorlabs\Kinesis`
- A compatible UKE SmartLab checkout

The project links against `Thorlabs.MotionControl.VerticalStage` and
`Thorlabs.MotionControl.Benchtop.DCServo`. Their runtime DLLs and the Kinesis
device manager must be available when SmartLab starts.

## Build

The recommended option is to place this repository next to
`UKE-smartLab` and build `UKE-smartLab\smartLab.sln`.

To build the plugin directly:

```powershell
MSBuild.exe thorKinesisPlugin\thorlabsKinesisPlugin.vcxproj `
  /p:Configuration=Release /p:Platform=x64
```

Set `QWTDIR` and `QWTLIB` for the Qwt installation. The plugin is written to
the matching `x64\<Configuration>\plugins` directory.

## Use

1. Install Kinesis, connect the stages, and confirm that Kinesis can see them.
2. Start SmartLab and detect the Thorlabs Kinesis plugin.
3. Select the axes to initialize and home them where required.
4. Verify axis names, serial numbers, travel limits, and movement direction.
5. Use manual controls, saved positions, scan jobs, or the optional gamepad.

Motion and trigger settings affect physical equipment. Confirm limits,
direction mappings, emergency-stop behavior, and laser interlocks before
enabling automated movement or trigger output.
