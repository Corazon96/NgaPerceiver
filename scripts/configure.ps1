param(
    [string]$QtPath = "",
    [string]$Generator = "Visual Studio 17 2022",
    [string]$Arch = "x64"
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

$BuildDir = Join-Path $PSScriptRoot "..\build"

if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

Push-Location $BuildDir

$CMakeArgs = @("-G", $Generator, "-A", $Arch)

if ($QtPath) {
    $CMakeArgs += "-DCMAKE_PREFIX_PATH=$QtPath"
}

Write-Host "Configuring with CMake..."
& cmake .. @CMakeArgs

if ($LASTEXITCODE -eq 0) {
    Write-Host "Configuration successful."
} else {
    Write-Error "Configuration failed."
}

Pop-Location
