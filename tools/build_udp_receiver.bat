@echo off
REM ============================================================================
REM Build UDP Receiver Tool
REM ============================================================================
REM
REM Usage - One-line command from project root (PowerShell):
REM   cmd /c "if not exist build\tools md build\tools && call ""C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"" && cl /EHsc /std:c++17 /O2 /Fe:build\tools\udp_receiver.exe tools\udp_receiver.cpp /I include /link ws2_32.lib"
REM
REM Or run the bat script:
REM   .\tools\build_udp_receiver.bat
REM
REM Output: build\tools\udp_receiver.exe
REM
REM Run:
REM   .\build\tools\udp_receiver.exe [port]   (default port: 5000)
REM
REM ============================================================================

echo Building UDP Receiver...

REM Check if MSVC is available
where cl >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: MSVC compiler (cl.exe) not found.
    echo Please run this script from a Visual Studio Developer Command Prompt.
    pause
    exit /b 1
)

REM Create output directory
if not exist "build\tools" md "build\tools"

REM Compile
cl /EHsc /std:c++17 /O2 /Fe:build\tools\udp_receiver.exe tools\udp_receiver.cpp /link ws2_32.lib

if %errorlevel% equ 0 (
    echo.
    echo Build successful!
    echo Output: build\tools\udp_receiver.exe
    echo.
    echo Usage: build\tools\udp_receiver.exe [port]
    echo Default port: 5000
) else (
    echo.
    echo Build failed!
)

pause
