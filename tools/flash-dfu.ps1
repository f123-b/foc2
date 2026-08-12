[CmdletBinding()]
param(
    [ValidateSet('abz-low-speed-v3', 'abz-low-speed-v2', 'abz-low-speed-v1', 'baseline-speed-22')]
    [string]$Profile = 'abz-low-speed-v3',
    [switch]$IUnderstandMotorMayStart
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$profiles = @{
    'abz-low-speed-v3' = @{
        Label = 'ABZ dither-resistant breakaway and bounded-integrator v3'
        Bin = 'output\firmware\abz-low-speed-v3-16C7FC68\FOCStudioFirmware-abz-low-speed-v3.bin'
        Hex = 'output\firmware\abz-low-speed-v3-16C7FC68\FOCStudioFirmware-abz-low-speed-v3.hex'
        BinSha256 = '4F94AD292F947A2823140CCA739DA0DC7BB8F96EC2973145012C9A67E6453F15'
        HexSha256 = '16C7FC683EF64573BF9C36050B82F571FD0EBAA42A83CA80F038D561CBB62011'
        StLinkChecksum = [uint32]0x0176D6EC
    }
    'abz-low-speed-v2' = @{
        Label = 'ABZ count-aware low-speed and position trajectory v2'
        Bin = 'output\firmware\abz-low-speed-v2-7B639E84\FOCStudioFirmware-abz-low-speed-v2.bin'
        Hex = 'output\firmware\abz-low-speed-v2-7B639E84\FOCStudioFirmware-abz-low-speed-v2.hex'
        BinSha256 = '154490A28F9F9B9E23EF99C41D1EBEFDB33808525F87B86CC329F32F5FDB354E'
        HexSha256 = '7B639E8419B05ACA2D485666B38BF139AACE8E20DB76E619B4180925DA7321F0'
        StLinkChecksum = [uint32]0x0176BF16
    }
    'abz-low-speed-v1' = @{
        Label = 'ABZ low-speed and position trajectory v1'
        Bin = 'output\firmware\abz-low-speed-v1-B79DB48A\FOCStudioFirmware-abz-low-speed-v1.bin'
        Hex = 'output\firmware\abz-low-speed-v1-B79DB48A\FOCStudioFirmware-abz-low-speed-v1.hex'
        BinSha256 = '4FC81E559240377B96A2D9C7A01D51683CF6545F3D751077DDE9D96F86383BA8'
        HexSha256 = 'B79DB48A96B1C7B32C67BD962E75A66E0ED49C07B2F165BC01CC056DF906CC50'
        StLinkChecksum = [uint32]0x01739645
    }
    'baseline-speed-22' = @{
        Label = 'verified speed baseline 22 rollback'
        Bin = 'output\firmware\speed-baseline-22-1CE11A09\FOCStudioFirmware-speed-baseline-22.bin'
        Hex = 'output\firmware\speed-baseline-22-1CE11A09\FOCStudioFirmware-speed-baseline-22.hex'
        BinSha256 = '828AED340EEBBECB992CF91BD26FC70BEA63261D056875CBE24A5C1CD104072B'
        HexSha256 = '1CE11A09F22C67384EAC35C24A075BACC1B81C31C2D346F5B7B3C47AEAAA8569'
        StLinkChecksum = [uint32]0x01717CDF
    }
}
$selectedProfile = $profiles[$Profile]
$firmwareBin = Join-Path $projectRoot $selectedProfile.Bin
$firmwareHex = Join-Path $projectRoot $selectedProfile.Hex
$expectedBinSha256 = $selectedProfile.BinSha256
$expectedHexSha256 = $selectedProfile.HexSha256
$expectedStLinkChecksum = $selectedProfile.StLinkChecksum

if (!$IUnderstandMotorMayStart) {
    throw 'Flash blocked. Disconnect motor phases or remove mechanical load, then rerun with -IUnderstandMotorMayStart.'
}
if (!(Test-Path -LiteralPath $firmwareBin) -or !(Test-Path -LiteralPath $firmwareHex)) {
    throw "Firmware for profile '$Profile' was not found. Build it or restore its delivery folder first."
}

$actualBinSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $firmwareBin).Hash
$actualHexSha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $firmwareHex).Hash
if ($actualBinSha256 -ne $expectedBinSha256 -or $actualHexSha256 -ne $expectedHexSha256) {
    throw @"
Flash blocked because the files do not match profile '$Profile'.
BIN expected: $expectedBinSha256
BIN actual:   $actualBinSha256
HEX expected: $expectedHexSha256
HEX actual:   $actualHexSha256
"@
}

Write-Host "Verified firmware profile: $Profile ($($selectedProfile.Label))" -ForegroundColor Green
Write-Host "  BIN $actualBinSha256"
Write-Host "  HEX $actualHexSha256"
Write-Host ('  ST-LINK checksum 0x{0:X8}' -f $expectedStLinkChecksum)
Write-Host 'Disconnect motor power and mechanical load before programming the FOC Studio controller.' -ForegroundColor Yellow

$stLinkCli = (Get-Command ST-LINK_CLI.exe -ErrorAction SilentlyContinue).Source
if (!$stLinkCli) {
    throw 'ST-LINK_CLI.exe was not found on PATH. Install ST-LINK tools or pass it on PATH.'
}
if (Test-Path -LiteralPath $stLinkCli) {
    # This is the same program/verify operation shown in STM32 ST-LINK Utility.
    & $stLinkCli -c SWD -P $firmwareHex -V after_programming -Rst
    if ($LASTEXITCODE -ne 0) {
        throw 'ST-LINK flash or verification failed.'
    }
    Write-Host ('ST-LINK flash and verification succeeded: 0x{0:X8}' -f $expectedStLinkChecksum) -ForegroundColor Green
    exit 0
}

$cubeCandidates = @(
    (Get-Command STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -ErrorAction SilentlyContinue),
    'E:\cubemxpergrammer\bin\STM32_Programmer_CLI.exe',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
    'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
) | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1

if ($cubeCandidates) {
    & $cubeCandidates -c port=USB1 -w $firmwareHex -v -rst
    if ($LASTEXITCODE -ne 0) {
        throw 'STM32CubeProgrammer flash or verification failed.'
    }
    Write-Host "Flash and verification succeeded: $actualHexSha256" -ForegroundColor Green
    exit 0
}

$dfuUtil = Get-Command dfu-util.exe -ErrorAction SilentlyContinue
if ($dfuUtil) {
    & $dfuUtil.Source -a 0 -s '0x08000000:leave' -D $firmwareBin
    if ($LASTEXITCODE -ne 0) { throw 'dfu-util flash failed.' }
    Write-Host "Flash completed without readback verification: $actualBinSha256" -ForegroundColor Yellow
    exit 0
}

throw 'Neither STM32CubeProgrammer CLI nor dfu-util.exe was found.'
