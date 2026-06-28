<#
Read or restore conservative Thorlabs M30XY / Benchtop DC Servo settings
directly through the Thorlabs Kinesis C API.

This is intentionally separate from the plugin.

The script does not move, home, enable, or disable the stage.

It can restore per selected channel:
  - M30 named settings
  - motor scale: 10000 device units / mm
  - travel range: -15..+15 mm = -150000..+150000 device units
  - software-limit policy: DisallowIllegalMoves
  - move velocity: 5 mm/s^2 acceleration, 2.3 mm/s max velocity
  - motor velocity limits: 5 mm/s^2, 2.6 mm/s
  - jog mode: single-step, 0.5 mm step, 4 mm/s^2, 2.6 mm/s, profiled stop

Examples:
  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Query
  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Restore
  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Action Restore -Persist
  powershell -ExecutionPolicy Bypass -File .\m30xy_restore_motion_defaults.ps1 -Serial 101517004 -Channels 1 -Action Query
#>

param(
    [Parameter(Mandatory = $true)]
    [string]$Serial,
    [ValidateSet("Query", "Restore")]
    [string]$Action = "Query",
    [int[]]$Channels = @(1, 2),
    [string]$SettingsName = "M30 Series",
    [int]$MinDeviceUnits = -150000,
    [int]$MaxDeviceUnits = 150000,
    [double]$MinTravelMm = -15.0,
    [double]$MaxTravelMm = 15.0,
    [double]$StepsPerRev = 10000.0,
    [double]$GearboxRatio = 1.0,
    [double]$PitchMm = 1.0,
    [double]$MoveAccelerationMmS2 = 5.0,
    [double]$MoveMaxVelocityMmS = 2.3,
    [double]$MotorMaxVelocityMmS = 2.6,
    [double]$MotorMaxAccelerationMmS2 = 5.0,
    [double]$JogStepMm = 0.5,
    [double]$JogAccelerationMmS2 = 4.0,
    [double]$JogMaxVelocityMmS = 2.6,
    [switch]$SkipLoadNamedSettings,
    [switch]$Persist,
    [switch]$Force,
    [string]$KinesisDir
)

$ErrorActionPreference = "Stop"

function Find-KinesisDir {
    param([string]$RequestedDir)

    if ($RequestedDir) {
        if (Test-Path -LiteralPath $RequestedDir) {
            return (Resolve-Path -LiteralPath $RequestedDir).Path
        }
        throw "Kinesis directory does not exist: $RequestedDir"
    }

    foreach ($candidate in @(
        "C:\Program Files\Thorlabs\Kinesis",
        "C:\Program Files (x86)\Thorlabs\Kinesis"
    )) {
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }
    throw "Kinesis installation not found."
}

function Add-NativeBindings {
    param([string]$Directory)

    $env:PATH = "$Directory;$env:PATH"
    $code = @'
using System;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential)]
public struct BdcM30VelocityParams
{
    public int minVelocity;
    public int acceleration;
    public int maxVelocity;
}

[StructLayout(LayoutKind.Sequential)]
public struct BdcM30JogParams
{
    public short mode;
    public UInt32 stepSize;
    public BdcM30VelocityParams velParams;
    public short stopMode;
}

public static class BdcM30RestoreNative
{
    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl)]
    public static extern short TLI_BuildDeviceList();

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_Open(string serialNo);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_Close(string serialNo);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern bool BDC_LoadNamedSettings(string serialNo, short channel, string settingsName);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern bool BDC_PersistSettings(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_RequestSettings(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern UInt32 BDC_GetStatusBits(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern int BDC_GetPosition(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetMotorTravelMode(string serialNo, short channel, short travelMode);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetMotorParamsExt(string serialNo, short channel, ref double stepsPerRev, ref double gearBoxRatio, ref double pitch);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetMotorParamsExt(string serialNo, short channel, double stepsPerRev, double gearBoxRatio, double pitch);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetMotorTravelLimits(string serialNo, short channel, ref double minPosition, ref double maxPosition);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetMotorTravelLimits(string serialNo, short channel, double minPosition, double maxPosition);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetMotorVelocityLimits(string serialNo, short channel, ref double maxVelocity, ref double maxAcceleration);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetMotorVelocityLimits(string serialNo, short channel, double maxVelocity, double maxAcceleration);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern int BDC_GetStageAxisMinPos(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern int BDC_GetStageAxisMaxPos(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetStageAxisLimits(string serialNo, short channel, int minPosition, int maxPosition);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetSoftLimitMode(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void BDC_SetLimitsSoftwareApproachPolicy(string serialNo, short channel, short limitsSoftwareApproachPolicy);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetLimitSwitchParams(string serialNo, short channel, ref short clockwiseHardwareLimit, ref short anticlockwiseHardwareLimit, ref UInt32 clockwisePosition, ref UInt32 anticlockwisePosition, ref short softLimitMode);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetDeviceUnitFromRealValue(string serialNo, short channel, double realUnit, ref int deviceUnit, int unitType);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetRealValueFromDeviceUnit(string serialNo, short channel, int deviceUnit, ref double realUnit, int unitType);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_RequestVelParams(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetVelParamsBlock(string serialNo, short channel, ref BdcM30VelocityParams velocityParams);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetVelParamsBlock(string serialNo, short channel, ref BdcM30VelocityParams velocityParams);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_RequestJogParams(string serialNo, short channel);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_GetJogParamsBlock(string serialNo, short channel, ref BdcM30JogParams jogParams);

    [DllImport("Thorlabs.MotionControl.Benchtop.DCServo.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short BDC_SetJogParamsBlock(string serialNo, short channel, ref BdcM30JogParams jogParams);
}
'@
    if (-not ("BdcM30RestoreNative" -as [type])) {
        Add-Type -TypeDefinition $code
    }
}

function Assert-Ok {
    param([string]$Name, [int]$Code)
    if ($Code -ne 0) {
        throw "$Name failed with code $Code."
    }
}

function Get-ValidChannels {
    param([int[]]$RequestedChannels)

    $result = @()
    foreach ($channel in $RequestedChannels) {
        if ($channel -ne 1 -and $channel -ne 2) {
            throw "Invalid BDC channel $channel. Valid channels are 1 and 2."
        }
        if ($result -notcontains $channel) {
            $result += $channel
        }
    }
    if ($result.Count -eq 0) {
        throw "No channels selected."
    }
    return $result
}

function Convert-ToDeviceUnits {
    param(
        [int]$Channel,
        [double]$Value,
        [int]$UnitType,
        [string]$Label
    )
    $device = 0
    $err = [BdcM30RestoreNative]::BDC_GetDeviceUnitFromRealValue($Serial, [int16]$Channel, $Value, [ref]$device, $UnitType)
    Assert-Ok "BDC_GetDeviceUnitFromRealValue(ch=$Channel, $Label)" $err
    return $device
}

function Convert-FromDeviceUnits {
    param(
        [int]$Channel,
        [int]$DeviceUnits,
        [int]$UnitType,
        [string]$Label
    )
    $real = 0.0
    $err = [BdcM30RestoreNative]::BDC_GetRealValueFromDeviceUnit($Serial, [int16]$Channel, $DeviceUnits, [ref]$real, $UnitType)
    if ($err -ne 0) {
        return $null
    }
    return $real
}

function Read-BdcChannelState {
    param([int]$Channel)

    [void][BdcM30RestoreNative]::BDC_RequestSettings($Serial, [int16]$Channel)
    Start-Sleep -Milliseconds 150

    $spr = 0.0
    $gear = 0.0
    $pitch = 0.0
    $motorErr = [BdcM30RestoreNative]::BDC_GetMotorParamsExt($Serial, [int16]$Channel, [ref]$spr, [ref]$gear, [ref]$pitch)

    $mmPerDevice = 0.0
    $posFactorErr = [BdcM30RestoreNative]::BDC_GetRealValueFromDeviceUnit($Serial, [int16]$Channel, 1, [ref]$mmPerDevice, 0)
    $velMmPerDevice = 0.0
    $velFactorErr = [BdcM30RestoreNative]::BDC_GetRealValueFromDeviceUnit($Serial, [int16]$Channel, 1, [ref]$velMmPerDevice, 1)
    $accMmPerDevice = 0.0
    $accFactorErr = [BdcM30RestoreNative]::BDC_GetRealValueFromDeviceUnit($Serial, [int16]$Channel, 1, [ref]$accMmPerDevice, 2)

    $travelMin = 0.0
    $travelMax = 0.0
    $travelErr = [BdcM30RestoreNative]::BDC_GetMotorTravelLimits($Serial, [int16]$Channel, [ref]$travelMin, [ref]$travelMax)

    $velMax = 0.0
    $accMax = 0.0
    $motorVelErr = [BdcM30RestoreNative]::BDC_GetMotorVelocityLimits($Serial, [int16]$Channel, [ref]$velMax, [ref]$accMax)

    $cwHard = [int16]0
    $ccwHard = [int16]0
    $cwPos = [uint32]0
    $ccwPos = [uint32]0
    $softLimitMode = [int16]0
    $limitErr = [BdcM30RestoreNative]::BDC_GetLimitSwitchParams($Serial, [int16]$Channel, [ref]$cwHard, [ref]$ccwHard, [ref]$cwPos, [ref]$ccwPos, [ref]$softLimitMode)

    [void][BdcM30RestoreNative]::BDC_RequestVelParams($Serial, [int16]$Channel)
    Start-Sleep -Milliseconds 150
    $vel = New-Object BdcM30VelocityParams
    $velErr = [BdcM30RestoreNative]::BDC_GetVelParamsBlock($Serial, [int16]$Channel, [ref]$vel)

    [void][BdcM30RestoreNative]::BDC_RequestJogParams($Serial, [int16]$Channel)
    Start-Sleep -Milliseconds 150
    $jog = New-Object BdcM30JogParams
    $jogErr = [BdcM30RestoreNative]::BDC_GetJogParamsBlock($Serial, [int16]$Channel, [ref]$jog)

    $axisMinDevice = [BdcM30RestoreNative]::BDC_GetStageAxisMinPos($Serial, [int16]$Channel)
    $axisMaxDevice = [BdcM30RestoreNative]::BDC_GetStageAxisMaxPos($Serial, [int16]$Channel)
    $positionDevice = [BdcM30RestoreNative]::BDC_GetPosition($Serial, [int16]$Channel)

    [pscustomobject]@{
        Serial = $Serial
        Channel = $Channel
        StatusBitsHex = ("0x{0:X8}" -f [uint32][BdcM30RestoreNative]::BDC_GetStatusBits($Serial, [int16]$Channel))
        PositionDeviceUnits = $positionDevice
        PositionUmApprox = if ($posFactorErr -eq 0) { $positionDevice * $mmPerDevice * 1000.0 } else { $null }
        MotorParamsErr = $motorErr
        StepsPerRev = $spr
        GearboxRatio = $gear
        PitchMm = $pitch
        PositionFactorErr = $posFactorErr
        UmPerDeviceUnit = $mmPerDevice * 1000.0
        VelocityFactorErr = $velFactorErr
        MmSPerDeviceUnit = $velMmPerDevice
        AccelerationFactorErr = $accFactorErr
        MmS2PerDeviceUnit = $accMmPerDevice
        AxisMinDeviceUnits = $axisMinDevice
        AxisMaxDeviceUnits = $axisMaxDevice
        AxisMinUmApprox = if ($posFactorErr -eq 0) { $axisMinDevice * $mmPerDevice * 1000.0 } else { $null }
        AxisMaxUmApprox = if ($posFactorErr -eq 0) { $axisMaxDevice * $mmPerDevice * 1000.0 } else { $null }
        SoftLimitPolicy = [BdcM30RestoreNative]::BDC_GetSoftLimitMode($Serial, [int16]$Channel)
        LimitSwitchErr = $limitErr
        CwHardwareLimit = [int]$cwHard
        CcwHardwareLimit = [int]$ccwHard
        CwSoftwareLimitPosition = [uint32]$cwPos
        CcwSoftwareLimitPosition = [uint32]$ccwPos
        LimitSwitchSoftLimitMode = [int]$softLimitMode
        TravelLimitsErr = $travelErr
        TravelMinMm = $travelMin
        TravelMaxMm = $travelMax
        MotorVelocityLimitsErr = $motorVelErr
        MotorMaxVelocityMmS = $velMax
        MotorMaxAccelerationMmS2 = $accMax
        VelocityErr = $velErr
        MoveMinVelocityDevice = [int]$vel.minVelocity
        MoveAccelerationDevice = [int]$vel.acceleration
        MoveAccelerationMmS2Approx = Convert-FromDeviceUnits -Channel $Channel -DeviceUnits ([int]$vel.acceleration) -UnitType 2 -Label "move acceleration"
        MoveMaxVelocityDevice = [int]$vel.maxVelocity
        MoveMaxVelocityMmSApprox = Convert-FromDeviceUnits -Channel $Channel -DeviceUnits ([int]$vel.maxVelocity) -UnitType 1 -Label "move max velocity"
        JogErr = $jogErr
        JogMode = [int]$jog.mode
        JogStepDevice = [uint32]$jog.stepSize
        JogStepUmApprox = if ($posFactorErr -eq 0) { [double]$jog.stepSize * $mmPerDevice * 1000.0 } else { $null }
        JogAccelerationDevice = [int]$jog.velParams.acceleration
        JogAccelerationMmS2Approx = Convert-FromDeviceUnits -Channel $Channel -DeviceUnits ([int]$jog.velParams.acceleration) -UnitType 2 -Label "jog acceleration"
        JogMaxVelocityDevice = [int]$jog.velParams.maxVelocity
        JogMaxVelocityMmSApprox = Convert-FromDeviceUnits -Channel $Channel -DeviceUnits ([int]$jog.velParams.maxVelocity) -UnitType 1 -Label "jog max velocity"
        JogStopMode = [int]$jog.stopMode
    }
}

function Restore-BdcChannelDefaults {
    param([int]$Channel)

    $bits = [uint32][BdcM30RestoreNative]::BDC_GetStatusBits($Serial, [int16]$Channel)
    if (($bits -band 0x00000030) -ne 0) {
        throw "Channel $Channel is moving according to status bits 0x$('{0:X8}' -f $bits). Refusing to write settings."
    }

    if (-not $SkipLoadNamedSettings) {
        $loaded = [BdcM30RestoreNative]::BDC_LoadNamedSettings($Serial, [int16]$Channel, $SettingsName)
        Write-Host "BDC_LoadNamedSettings('$SettingsName', ch=$Channel) returned: $loaded"
        if (-not $loaded) {
            throw "BDC_LoadNamedSettings('$SettingsName', ch=$Channel) returned false."
        }
        Start-Sleep -Milliseconds 500
        Assert-Ok "BDC_RequestSettings(ch=$Channel)" ([BdcM30RestoreNative]::BDC_RequestSettings($Serial, [int16]$Channel))
        Start-Sleep -Milliseconds 250
    }

    Assert-Ok "BDC_SetMotorTravelMode(ch=$Channel, linear)" ([BdcM30RestoreNative]::BDC_SetMotorTravelMode($Serial, [int16]$Channel, 1))
    Assert-Ok "BDC_SetMotorParamsExt(ch=$Channel)" ([BdcM30RestoreNative]::BDC_SetMotorParamsExt($Serial, [int16]$Channel, $StepsPerRev, $GearboxRatio, $PitchMm))
    Start-Sleep -Milliseconds 150

    Assert-Ok "BDC_SetMotorTravelLimits(ch=$Channel)" ([BdcM30RestoreNative]::BDC_SetMotorTravelLimits($Serial, [int16]$Channel, $MinTravelMm, $MaxTravelMm))
    Assert-Ok "BDC_SetMotorVelocityLimits(ch=$Channel)" ([BdcM30RestoreNative]::BDC_SetMotorVelocityLimits($Serial, [int16]$Channel, $MotorMaxVelocityMmS, $MotorMaxAccelerationMmS2))
    Assert-Ok "BDC_SetStageAxisLimits(ch=$Channel)" ([BdcM30RestoreNative]::BDC_SetStageAxisLimits($Serial, [int16]$Channel, $MinDeviceUnits, $MaxDeviceUnits))

    # 0 = DisallowIllegalMoves. This is safer than truncating/allowing while recovering bad settings.
    [BdcM30RestoreNative]::BDC_SetLimitsSoftwareApproachPolicy($Serial, [int16]$Channel, 0)

    $moveAccDevice = Convert-ToDeviceUnits -Channel $Channel -Value $MoveAccelerationMmS2 -UnitType 2 -Label "move acceleration"
    $moveVelDevice = Convert-ToDeviceUnits -Channel $Channel -Value $MoveMaxVelocityMmS -UnitType 1 -Label "move max velocity"
    $velParams = New-Object BdcM30VelocityParams
    $velParams.minVelocity = 0
    $velParams.acceleration = $moveAccDevice
    $velParams.maxVelocity = $moveVelDevice
    Assert-Ok "BDC_SetVelParamsBlock(ch=$Channel)" ([BdcM30RestoreNative]::BDC_SetVelParamsBlock($Serial, [int16]$Channel, [ref]$velParams))

    $jogStepDevice = Convert-ToDeviceUnits -Channel $Channel -Value $JogStepMm -UnitType 0 -Label "jog step"
    if ($jogStepDevice -lt 0) {
        throw "Converted jog step is negative ($jogStepDevice). Refusing to write jog parameters."
    }
    $jogAccDevice = Convert-ToDeviceUnits -Channel $Channel -Value $JogAccelerationMmS2 -UnitType 2 -Label "jog acceleration"
    $jogVelDevice = Convert-ToDeviceUnits -Channel $Channel -Value $JogMaxVelocityMmS -UnitType 1 -Label "jog max velocity"

    $jogParams = New-Object BdcM30JogParams
    $jogParams.mode = 2
    $jogParams.stepSize = [uint32]$jogStepDevice
    $jogParams.velParams.minVelocity = 0
    $jogParams.velParams.acceleration = $jogAccDevice
    $jogParams.velParams.maxVelocity = $jogVelDevice
    $jogParams.stopMode = 2
    Assert-Ok "BDC_SetJogParamsBlock(ch=$Channel)" ([BdcM30RestoreNative]::BDC_SetJogParamsBlock($Serial, [int16]$Channel, [ref]$jogParams))

    if ($Persist) {
        $persisted = [BdcM30RestoreNative]::BDC_PersistSettings($Serial, [int16]$Channel)
        Write-Host "BDC_PersistSettings(ch=$Channel) returned: $persisted"
        if (-not $persisted) {
            throw "BDC_PersistSettings(ch=$Channel) returned false."
        }
        Start-Sleep -Milliseconds 1000
    }
}

$selectedChannels = Get-ValidChannels -RequestedChannels $Channels
$resolvedKinesisDir = Find-KinesisDir -RequestedDir $KinesisDir
Add-NativeBindings -Directory $resolvedKinesisDir

if ($Action -eq "Restore" -and -not $Force) {
    $persistText = if ($Persist) { " and persist them to the controller" } else { "" }
    Write-Warning "This will write M30XY/BDC motion defaults$persistText for serial $Serial channels $($selectedChannels -join ','). It sends no move/home/enable command."
    $answer = Read-Host "Type RESTORE to continue"
    if ($answer -ne "RESTORE") {
        Write-Host "Canceled."
        exit 2
    }
}

[void][BdcM30RestoreNative]::TLI_BuildDeviceList()
$open = [BdcM30RestoreNative]::BDC_Open($Serial)
if ($open -ne 0) {
    throw "BDC_Open failed with code $open. Close Kinesis/SmartLab/other clients and retry."
}

try {
    Write-Host ""
    Write-Host "Before:"
    foreach ($channel in $selectedChannels) {
        Read-BdcChannelState -Channel $channel | Format-List
    }

    if ($Action -eq "Restore") {
        foreach ($channel in $selectedChannels) {
            Restore-BdcChannelDefaults -Channel $channel
        }
        Start-Sleep -Milliseconds 500
        Write-Host ""
        Write-Host "After:"
        foreach ($channel in $selectedChannels) {
            Read-BdcChannelState -Channel $channel | Format-List
        }
    }
}
finally {
    try { [void][BdcM30RestoreNative]::BDC_Close($Serial) } catch {}
}
