<#
Restore conservative KVS30/M motion settings directly through the Thorlabs
Kinesis C API. This is intentionally separate from the plugin.

The script does not move, home, enable, or disable the stage.

It can restore:
  - KVS30/M named settings
  - motor scale: 20000 device units / mm
  - travel range: 0..30 mm = 0..600000 device units
  - software-limit policy: DisallowIllegalMoves
  - move velocity: 1 mm/s^2 acceleration, 2 mm/s max velocity
  - motor velocity limits: 5 mm/s^2, 8 mm/s
  - track/settle params, default 197 cycles (= 20.1728 ms), window 20 counts

Examples:
  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Query
  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Restore
  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Restore -Persist
  powershell -ExecutionPolicy Bypass -File .\kvs30_restore_motion_defaults.ps1 -Action Restore -Persist -Force
#>

param(
    [string]$Serial = "24522994",
    [ValidateSet("Query", "Restore")]
    [string]$Action = "Query",
    [string]$SettingsName = "KVS30/M",
    [int]$MinDeviceUnits = 0,
    [int]$MaxDeviceUnits = 600000,
    [double]$MinTravelMm = 0.0,
    [double]$MaxTravelMm = 30.0,
    [double]$StepsPerRev = 20000.0,
    [double]$GearboxRatio = 1.0,
    [double]$PitchMm = 1.0,
    [double]$MoveAccelerationMmS2 = 1.0,
    [double]$MoveMaxVelocityMmS = 2.0,
    [double]$MotorMaxVelocityMmS = 8.0,
    [double]$MotorMaxAccelerationMmS2 = 5.0,
    [int]$TrackSettleTimeCycles = 197,
    [int]$TrackSettleSettledError = 20,
    [int]$TrackSettleMaxTrackingError = 0,
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
public struct KvsVelocityParams
{
    public int minVelocity;
    public int acceleration;
    public int maxVelocity;
}

[StructLayout(LayoutKind.Sequential)]
public struct KvsTrackSettleParams
{
    public ushort time;
    public ushort settledError;
    public ushort maxTrackingError;
    public ushort notUsed;
    public ushort lastNotUsed;
}

public static class KvsRestoreNative
{
    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl)]
    public static extern short TLI_BuildDeviceList();

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_Open(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void KVS_Close(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern bool KVS_LoadNamedSettings(string serialNo, string settingsName);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern bool KVS_PersistSettings(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern UInt32 KVS_GetStatusBits(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern int KVS_GetPosition(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetMotorTravelMode(string serialNo, short travelMode);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetMotorParamsExt(string serialNo, double stepsPerRev, double gearBoxRatio, double pitch);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetMotorParamsExt(string serialNo, ref double stepsPerRev, ref double gearBoxRatio, ref double pitch);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetMotorTravelLimits(string serialNo, double minPosition, double maxPosition);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetMotorTravelLimits(string serialNo, ref double minPosition, ref double maxPosition);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetMotorVelocityLimits(string serialNo, double maxVelocity, double maxAcceleration);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetMotorVelocityLimits(string serialNo, ref double maxVelocity, ref double maxAcceleration);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetStageAxisLimits(string serialNo, int minPosition, int maxPosition);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern int KVS_GetStageAxisMinPos(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern int KVS_GetStageAxisMaxPos(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetSoftLimitMode(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern void KVS_SetLimitsSoftwareApproachPolicy(string serialNo, short limitsSoftwareApproachPolicy);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetDeviceUnitFromRealValue(string serialNo, double realUnit, ref int deviceUnit, int unitType);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetRealValueFromDeviceUnit(string serialNo, int deviceUnit, ref double realUnit, int unitType);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_RequestVelParams(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetVelParamsBlock(string serialNo, ref KvsVelocityParams velocityParams);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetVelParamsBlock(string serialNo, ref KvsVelocityParams velocityParams);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_RequestTrackSettleParams(string serialNo);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_GetTrackSettleParams(string serialNo, ref KvsTrackSettleParams settleParams);

    [DllImport("Thorlabs.MotionControl.VerticalStage.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    public static extern short KVS_SetTrackSettleParams(string serialNo, ref KvsTrackSettleParams settleParams);
}
'@
    if (-not ("KvsRestoreNative" -as [type])) {
        Add-Type -TypeDefinition $code
    }
}

function Assert-Ok {
    param([string]$Name, [int]$Code)
    if ($Code -ne 0) {
        throw "$Name failed with code $Code."
    }
}

function Assert-WordRange {
    param([string]$Name, [int]$Value, [int]$Min = 0)
    if ($Value -lt $Min -or $Value -gt 65535) {
        throw "$Name must be between $Min and 65535, got $Value."
    }
}

function Convert-ToDeviceUnits {
    param(
        [double]$Value,
        [int]$UnitType,
        [string]$Label
    )
    $device = 0
    $err = [KvsRestoreNative]::KVS_GetDeviceUnitFromRealValue($Serial, $Value, [ref]$device, $UnitType)
    Assert-Ok "KVS_GetDeviceUnitFromRealValue($Label)" $err
    return $device
}

function Read-KvsState {
    $spr = 0.0
    $gear = 0.0
    $pitch = 0.0
    $motorErr = [KvsRestoreNative]::KVS_GetMotorParamsExt($Serial, [ref]$spr, [ref]$gear, [ref]$pitch)

    $mmPerDevice = 0.0
    $posFactorErr = [KvsRestoreNative]::KVS_GetRealValueFromDeviceUnit($Serial, 1, [ref]$mmPerDevice, 0)

    $travelMin = 0.0
    $travelMax = 0.0
    $travelErr = [KvsRestoreNative]::KVS_GetMotorTravelLimits($Serial, [ref]$travelMin, [ref]$travelMax)

    $velMax = 0.0
    $accMax = 0.0
    $motorVelErr = [KvsRestoreNative]::KVS_GetMotorVelocityLimits($Serial, [ref]$velMax, [ref]$accMax)

    [void][KvsRestoreNative]::KVS_RequestVelParams($Serial)
    Start-Sleep -Milliseconds 150
    $vel = New-Object KvsVelocityParams
    $velErr = [KvsRestoreNative]::KVS_GetVelParamsBlock($Serial, [ref]$vel)

    [void][KvsRestoreNative]::KVS_RequestTrackSettleParams($Serial)
    Start-Sleep -Milliseconds 150
    $settle = New-Object KvsTrackSettleParams
    $settleErr = [KvsRestoreNative]::KVS_GetTrackSettleParams($Serial, [ref]$settle)

    [pscustomobject]@{
        Serial = $Serial
        StatusBitsHex = ("0x{0:X8}" -f [uint32][KvsRestoreNative]::KVS_GetStatusBits($Serial))
        PositionDeviceUnits = [KvsRestoreNative]::KVS_GetPosition($Serial)
        MotorParamsErr = $motorErr
        StepsPerRev = $spr
        GearboxRatio = $gear
        PitchMm = $pitch
        RealValueFactorErr = $posFactorErr
        UmPerDeviceUnit = $mmPerDevice * 1000.0
        AxisMinDeviceUnits = [KvsRestoreNative]::KVS_GetStageAxisMinPos($Serial)
        AxisMaxDeviceUnits = [KvsRestoreNative]::KVS_GetStageAxisMaxPos($Serial)
        SoftLimitPolicy = [KvsRestoreNative]::KVS_GetSoftLimitMode($Serial)
        TravelLimitsErr = $travelErr
        TravelMinMm = $travelMin
        TravelMaxMm = $travelMax
        MotorVelocityLimitsErr = $motorVelErr
        MotorMaxVelocityMmS = $velMax
        MotorMaxAccelerationMmS2 = $accMax
        VelocityErr = $velErr
        MoveMinVelocityDevice = [int]$vel.minVelocity
        MoveAccelerationDevice = [int]$vel.acceleration
        MoveMaxVelocityDevice = [int]$vel.maxVelocity
        TrackSettleErr = $settleErr
        TrackSettleTimeCycles = [int]$settle.time
        TrackSettleTimeMs = [Math]::Round([double]$settle.time * 0.1024, 4)
        TrackSettleSettledError = [int]$settle.settledError
        TrackSettleMaxTrackingError = [int]$settle.maxTrackingError
    }
}

function Restore-KvsDefaults {
    Assert-WordRange "TrackSettleTimeCycles" $TrackSettleTimeCycles 1
    if ($TrackSettleTimeCycles -gt 32767) {
        throw "TrackSettleTimeCycles $TrackSettleTimeCycles exceeds the Thorlabs documented 0x7FFF maximum."
    }
    Assert-WordRange "TrackSettleSettledError" $TrackSettleSettledError 0
    Assert-WordRange "TrackSettleMaxTrackingError" $TrackSettleMaxTrackingError 0

    $bits = [uint32][KvsRestoreNative]::KVS_GetStatusBits($Serial)
    if (($bits -band 0x00000030) -ne 0) {
        throw "Stage is moving according to status bits 0x$('{0:X8}' -f $bits). Refusing to write settings."
    }

    if (-not $SkipLoadNamedSettings) {
        $loaded = [KvsRestoreNative]::KVS_LoadNamedSettings($Serial, $SettingsName)
        Write-Host "KVS_LoadNamedSettings('$SettingsName') returned: $loaded"
        if (-not $loaded) {
            throw "KVS_LoadNamedSettings('$SettingsName') returned false."
        }
        Start-Sleep -Milliseconds 500
    }

    Assert-Ok "KVS_SetMotorTravelMode(linear)" ([KvsRestoreNative]::KVS_SetMotorTravelMode($Serial, 1))
    Assert-Ok "KVS_SetMotorParamsExt" ([KvsRestoreNative]::KVS_SetMotorParamsExt($Serial, $StepsPerRev, $GearboxRatio, $PitchMm))
    Assert-Ok "KVS_SetMotorTravelLimits" ([KvsRestoreNative]::KVS_SetMotorTravelLimits($Serial, $MinTravelMm, $MaxTravelMm))
    Assert-Ok "KVS_SetMotorVelocityLimits" ([KvsRestoreNative]::KVS_SetMotorVelocityLimits($Serial, $MotorMaxVelocityMmS, $MotorMaxAccelerationMmS2))
    Assert-Ok "KVS_SetStageAxisLimits" ([KvsRestoreNative]::KVS_SetStageAxisLimits($Serial, $MinDeviceUnits, $MaxDeviceUnits))

    # 0 = DisallowIllegalMoves. This is safer than truncating/allowing when recovering bad settings.
    [KvsRestoreNative]::KVS_SetLimitsSoftwareApproachPolicy($Serial, 0)

    $accDevice = Convert-ToDeviceUnits -Value $MoveAccelerationMmS2 -UnitType 2 -Label "move acceleration"
    $velDevice = Convert-ToDeviceUnits -Value $MoveMaxVelocityMmS -UnitType 1 -Label "move max velocity"
    $velParams = New-Object KvsVelocityParams
    $velParams.minVelocity = 0
    $velParams.acceleration = $accDevice
    $velParams.maxVelocity = $velDevice
    Assert-Ok "KVS_SetVelParamsBlock" ([KvsRestoreNative]::KVS_SetVelParamsBlock($Serial, [ref]$velParams))

    $settleParams = New-Object KvsTrackSettleParams
    $settleParams.time = [uint16]$TrackSettleTimeCycles
    $settleParams.settledError = [uint16]$TrackSettleSettledError
    $settleParams.maxTrackingError = [uint16]$TrackSettleMaxTrackingError
    $settleParams.notUsed = 0
    $settleParams.lastNotUsed = 0
    Assert-Ok "KVS_SetTrackSettleParams" ([KvsRestoreNative]::KVS_SetTrackSettleParams($Serial, [ref]$settleParams))

    if ($Persist) {
        $persisted = [KvsRestoreNative]::KVS_PersistSettings($Serial)
        Write-Host "KVS_PersistSettings returned: $persisted"
        if (-not $persisted) {
            throw "KVS_PersistSettings returned false."
        }
        Start-Sleep -Milliseconds 3000
    }
}

$resolvedKinesisDir = Find-KinesisDir -RequestedDir $KinesisDir
Add-NativeBindings -Directory $resolvedKinesisDir

if ($Action -eq "Restore" -and -not $Force) {
    $persistText = if ($Persist) { " and persist them to the controller" } else { "" }
    Write-Warning "This will write KVS30/M motion defaults$persistText. It sends no move/home/enable command."
    $answer = Read-Host "Type RESTORE to continue"
    if ($answer -ne "RESTORE") {
        Write-Host "Canceled."
        exit 2
    }
}

[void][KvsRestoreNative]::TLI_BuildDeviceList()
$open = [KvsRestoreNative]::KVS_Open($Serial)
if ($open -ne 0) {
    throw "KVS_Open failed with code $open. Close Kinesis/other clients and retry."
}

try {
    Write-Host ""
    Write-Host "Before:"
    Read-KvsState | Format-List

    if ($Action -eq "Restore") {
        Restore-KvsDefaults
        Start-Sleep -Milliseconds 500
        Write-Host ""
        Write-Host "After:"
        Read-KvsState | Format-List
    }
}
finally {
    try { [KvsRestoreNative]::KVS_Close($Serial) } catch {}
}
