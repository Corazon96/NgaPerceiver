# LingerPerceiver

LingerPerceiver is a high-performance LiDAR visualization and recording tool designed for Livox Mid-360 sensors. It leverages Qt for the UI, VTK/PCL for 3D rendering, and the Livox SDK2 for hardware communication.

## Features

*   **Real-time Visualization:** View point clouds with intensity coloring and adjustable point size.
*   **IMU Integration:** Real-time pose estimation using 6-DOF IMU data (handled by `DataProcessor`).
*   **Recording & Replay:**
    *   Record live data to compact `.lgv` binary files (managed by `Replayer`).
    *   Replay recordings with variable speed (0.5x - 8.0x).
    *   Seek bar with scrubbing support (real-time preview while dragging).
    *   Pause/Resume functionality.
*   **Filters:**
    *   Distance Filter (Min/Max range).
    *   Voxel Grid Filter (Downsampling).
    *   Adjustable Integration Time (Retention) for visual trails.
*   **Device Management:** Auto-discovery, Connect/Disconnect, Start/Stop Acquisition.

## Architecture

The project follows a modular architecture:
*   **DeviceManager:** Acts as a facade, coordinating hardware interaction, data processing, and replay/recording.
*   **DataProcessor:** Handles IMU data ingestion, timestamp synchronization, and pose interpolation.
*   **Replayer:** Manages file I/O for both recording and playback, including speed control and seeking.
*   **Processor:** Handles point cloud filtering and map accumulation in a background thread.
*   **Renderer:** Uses VTK for high-performance 3D rendering, with a dedicated worker thread to prevent UI freezing.

## Build Instructions (Windows)

1.  **Prerequisites:**
    *   Visual Studio 2022 (C++17 support).
    *   CMake 3.10+.
    *   Qt 5.12.7 (MSVC 2017 64-bit).
    *   Dependencies in `3rdParty/`: PCL 1.15, VTK 9.4, Livox-SDK2.

2.  **Configure & Build:**
    Use the provided PowerShell scripts:
    ```powershell
    # Configure (adjust Qt path if needed)
    .\scripts\configure.ps1 -QtPath "D:/Qt/Qt5.12.7/5.12.7/msvc2017_64"

    # Build (Release)
    .\scripts\build_utf8.ps1 -Configuration Release
    ```

3.  **Run:**
    The executable `linger_perceiver.exe` will be in `build/Release/`.
    Ensure `mid360_config.json` is present in the same directory (automatically copied during build).

## Usage

*   **Connect:** Select a discovered device and click "Connect".
*   **Start:** Click "Start" to begin data streaming.
*   **Record:** Click "Record" (requires active acquisition) to save data.
*   **Replay:** Click "Replay" (requires disconnected device) to open a `.lgv` file. Use the control bar to seek, pause, or change speed.
*   **Controls:**
    *   Left Mouse: Rotate.
    *   Middle Mouse: Pan.
    *   Right Mouse: Zoom.
    *   `R`: Reset Camera.

## License

[License Information Here]
