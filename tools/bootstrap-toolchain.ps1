[CmdletBinding()]
param(
    [switch]$Force,
    [switch]$Quiet
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$toolsRoot = Join-Path $projectRoot 'tools'
$downloadsRoot = Join-Path $toolsRoot 'downloads'

function Write-Status([string]$Message) {
    if (!$Quiet) { Write-Host $Message -ForegroundColor Cyan }
}

function Get-Sha256([string]$Path) {
    $sha256 = [System.Security.Cryptography.SHA256]::Create()
    $stream = [System.IO.File]::OpenRead($Path)
    try {
        return ([System.BitConverter]::ToString($sha256.ComputeHash($stream))).Replace('-', '')
    } finally {
        $stream.Dispose()
        $sha256.Dispose()
    }
}

function Install-ToolArchive {
    param(
        [string]$Name,
        [string]$Uri,
        [string]$ArchiveName,
        [string]$ExpectedSha256,
        [string]$Destination,
        [string]$RequiredRelativePath
    )

    $requiredPath = Join-Path $Destination $RequiredRelativePath
    if ((Test-Path -LiteralPath $requiredPath) -and !$Force) {
        Write-Status "$Name is already available."
        return
    }
    if ((Test-Path -LiteralPath $Destination) -and !$Force) {
        throw "$Name directory is incomplete: $Destination. Remove it or rerun with -Force."
    }

    New-Item -ItemType Directory -Force -Path $downloadsRoot | Out-Null
    $archive = Join-Path $downloadsRoot $ArchiveName
    $archiveIsEmpty = (Test-Path -LiteralPath $archive) -and
            (Get-Item -LiteralPath $archive).Length -eq 0
    if (!(Test-Path -LiteralPath $archive) -or $Force -or $archiveIsEmpty) {
        Write-Status "Downloading $Name..."
        $partial = "$archive.partial"
        # Keep a partial archive so interrupted downloads can be resumed safely.
        $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
        if ($curl) {
            & $curl.Source --fail --location --retry 5 --retry-delay 2 --connect-timeout 30 --continue-at - --output $partial $Uri
            if ($LASTEXITCODE -ne 0) { throw "Failed to download $Name." }
        } else {
            Invoke-WebRequest -Uri $Uri -OutFile $partial
        }
        Move-Item -LiteralPath $partial -Destination $archive -Force
    }

    $actualHash = (Get-Sha256 $archive).ToUpperInvariant()
    if ($ExpectedSha256 -and $actualHash -ne $ExpectedSha256.ToUpperInvariant()) {
        throw "$Name checksum mismatch. Expected $ExpectedSha256, got $actualHash."
    }

    $staging = Join-Path $downloadsRoot ("extract-" + [guid]::NewGuid().ToString('N'))
    try {
        Expand-Archive -LiteralPath $archive -DestinationPath $staging -Force
        $candidate = Get-ChildItem -LiteralPath $staging -Recurse -File |
            Where-Object { $_.FullName.EndsWith($RequiredRelativePath) } |
            Select-Object -First 1
        if (!$candidate) { throw "$Name archive does not contain $RequiredRelativePath." }

        $sourceRoot = $candidate.FullName
        foreach ($part in ($RequiredRelativePath -split '[\\/]')) {
            $sourceRoot = Split-Path -Parent $sourceRoot
        }
        if (Test-Path -LiteralPath $Destination) {
            Remove-Item -LiteralPath $Destination -Recurse -Force
        }
        Move-Item -LiteralPath $sourceRoot -Destination $Destination
        if (!(Test-Path -LiteralPath $requiredPath)) {
            throw "$Name installation did not produce $requiredPath."
        }
    } finally {
        if (Test-Path -LiteralPath $staging) {
            Remove-Item -LiteralPath $staging -Recurse -Force
        }
    }
}

Install-ToolArchive `
    -Name 'Arm GNU Toolchain 12.2.Rel1' `
    -Uri 'https://developer.arm.com/-/media/Files/downloads/gnu/12.2.rel1/binrel/arm-gnu-toolchain-12.2.rel1-mingw-w64-i686-arm-none-eabi.zip' `
    -ArchiveName 'arm-gnu-toolchain-12.2.rel1-mingw-w64-i686-arm-none-eabi.zip' `
    -ExpectedSha256 'C5215E37E70A2FA3233CD1F348AB74896281C40D1C531FC719CA6BA11EB99290' `
    -Destination (Join-Path $toolsRoot 'arm-gnu-toolchain') `
    -RequiredRelativePath 'bin\arm-none-eabi-gcc.exe'

Install-ToolArchive `
    -Name 'Tup v0.8' `
    -Uri 'https://gittup.org/tup/win32/tup-v0.8.zip' `
    -ArchiveName 'tup-v0.8.zip' `
    -ExpectedSha256 '8A1C3775C95A1CC23D261D02F71E194ED5DEEB6849280177EE93558721DFB1E0' `
    -Destination (Join-Path $toolsRoot 'tup') `
    -RequiredRelativePath 'bin\tup.exe'

Write-Status 'FOC Studio firmware toolchain is ready.'
