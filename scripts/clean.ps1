$ErrorActionPreference = 'Stop'
$BuildDir = Join-Path $PSScriptRoot "..\build"

if (Test-Path $BuildDir) {
    Write-Host "Removing build directory: $BuildDir"
    Remove-Item -Path $BuildDir -Recurse -Force
    Write-Host "Clean complete."
} else {
    Write-Host "Build directory does not exist."
}
