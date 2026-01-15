# LingerPerceiver

[中文](#中文) | [English](#english)

---

## English

### Overview

LingerPerceiver is a high-performance LiDAR visualization and recording tool designed for Livox Mid-360 sensors. It leverages Qt for the UI, VTK/PCL for 3D rendering, and the Livox SDK2 for hardware communication.

### Features

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

### Architecture

The project follows a modular architecture:
*   **DeviceManager:** Acts as a facade, coordinating hardware interaction, data processing, and replay/recording.
*   **DataProcessor:** Handles IMU data ingestion, timestamp synchronization, and pose interpolation.
*   **Replayer:** Manages file I/O for both recording and playback, including speed control and seeking.
*   **Processor:** Handles point cloud filtering and map accumulation in a background thread.
*   **Renderer:** Uses VTK for high-performance 3D rendering, with a dedicated worker thread to prevent UI freezing.

### Build Instructions (Windows)

#### Prerequisites

Visual Studio 2022 (C++17 support) and CMake 3.10+ are required. The following third-party libraries are used:

*   **Qt 5.12.7** (MSVC 2017 64-bit)
*   **PCL 1.15.1**
*   **VTK 9.4**
*   **Livox SDK2**
*   **Eigen3**
*   **RapidJSON**
*   **spdlog**

> **Note:** The `3rdParty/` directory is **not included** in this repository due to its large size. You need to install or place these libraries separately and configure the build system accordingly. This is a common practice in open-source projects to reduce repository size and allow developers to use their own optimized builds.

#### Setup Third-Party Libraries

**Option 1: Install via Package Manager (Recommended for Windows)**
- Install Qt5: Download from [Qt Official Website](https://www.qt.io/download)
- Install PCL: Use `vcpkg` or download pre-built binaries
- Install VTK: Use `vcpkg` or build from source
- Install Livox SDK2: Clone from [Livox SDK2 Repository](https://github.com/Livox-SDK/Livox-SDK2)

**Option 2: Place Pre-built Libraries in `3rdParty/`**
Create the following structure:
```
3rdParty/
  PCL-1.15.1/
  VTK-9.4/
  Livox-SDK2/
  Eigen3/
  rapidjson/
  spdlog/
```

#### Configure & Build

Use the provided PowerShell scripts:

```powershell
# Configure (adjust Qt path if needed)
.\scripts\configure.ps1 -QtPath "D:/Qt/Qt5.12.7/5.12.7/msvc2017_64"

# Build (Release)
.\scripts\build_utf8.ps1 -Configuration Release
```

#### Run

The executable `linger_perceiver.exe` will be in `build/Release/`. Configuration files in `config/` are automatically copied during build.

### Usage

*   **Connect:** Select a discovered device and click "Connect".
*   **Start:** Click "Start" to begin data streaming.
*   **Record:** Click "Record" (requires active acquisition) to save data.
*   **Replay:** Click "Replay" (requires disconnected device) to open a `.lgv` file. Use the control bar to seek, pause, or change speed.
*   **Controls:**
    *   Left Mouse: Rotate
    *   Middle Mouse: Pan
    *   Right Mouse: Zoom
    *   `R`: Reset Camera

### Configuration

Configuration files are located in the `config/` directory:
- `app_config.json` - Application settings
- `mid360_config.json` - Livox Mid-360 network settings

### License

[License Information Here]

---

## 中文

### 简介

LingerPerceiver 是一款为 Livox Mid-360 传感器设计的高性能激光雷达可视化和录制工具。它采用 Qt 构建用户界面，使用 VTK/PCL 进行 3D 渲染，通过 Livox SDK2 与硬件通信。

### 功能特性

*   **实时可视化:** 支持强度着色和可调节的点大小来显示点云。
*   **IMU 集成:** 基于 6-DOF IMU 数据的实时位姿估计（由 `DataProcessor` 处理）。
*   **录制和回放:**
    *   将实时数据录制为紧凑的 `.lgv` 二进制文件（由 `Replayer` 管理）。
    *   支持可变速度回放（0.5x - 8.0x）。
    *   支持搜索条拖动预览（实时预览）。
    *   暂停/继续功能。
*   **滤波功能:**
    *   距离滤波器（最小/最大范围）。
    *   体素网格滤波器（下采样）。
    *   可调节的积分时间（保留时间）用于视觉追踪。
*   **设备管理:** 自动发现、连接/断开连接、启动/停止采集。

### 软件架构

项目采用模块化架构设计：
*   **DeviceManager:** 充当外观模式角色，协调硬件交互、数据处理和回放/录制功能。
*   **DataProcessor:** 处理 IMU 数据采集、时间戳同步和位姿插值。
*   **Replayer:** 管理录制和回放的文件 I/O，包括速度控制和搜索功能。
*   **Processor:** 在后台线程中处理点云滤波和地图累积。
*   **Renderer:** 使用 VTK 进行高性能 3D 渲染，采用专用工作线程防止 UI 卡顿。

### 编译说明（Windows）

#### 环境要求

需要 Visual Studio 2022（支持 C++17）和 CMake 3.10+。使用以下第三方库：

*   **Qt 5.12.7** (MSVC 2017 64-bit)
*   **PCL 1.15.1**
*   **VTK 9.4**
*   **Livox SDK2**
*   **Eigen3**
*   **RapidJSON**
*   **spdlog**

> **说明:** `3rdParty/` 目录**不包含**在本仓库中，因为这些库文件体积较大。这是开源项目的常见做法，可以减小仓库大小，同时允许开发者使用自己优化的构建版本。

#### 配置第三方库

**方案 1: 通过包管理器安装（Windows 推荐）**
- 安装 Qt5: 从 [Qt 官方网站](https://www.qt.io/download) 下载
- 安装 PCL: 使用 `vcpkg` 或下载预编译二进制文件
- 安装 VTK: 使用 `vcpkg` 或从源码编译
- 安装 Livox SDK2: 从 [Livox SDK2 仓库](https://github.com/Livox-SDK/Livox-SDK2) 克隆

**方案 2: 在 `3rdParty/` 目录放置预编译库**
创建以下目录结构：
```
3rdParty/
  PCL-1.15.1/
  VTK-9.4/
  Livox-SDK2/
  Eigen3/
  rapidjson/
  spdlog/
```

#### 编译步骤

使用提供的 PowerShell 脚本：

```powershell
# 配置（根据需要调整 Qt 路径）
.\scripts\configure.ps1 -QtPath "D:/Qt/Qt5.12.7/5.12.7/msvc2017_64"

# 编译（Release）
.\scripts\build_utf8.ps1 -Configuration Release
```

#### 运行

可执行文件 `linger_perceiver.exe` 将位于 `build/Release/` 目录。`config/` 目录中的配置文件会在编译时自动复制。

### 使用说明

*   **连接:** 选择发现的设备并点击"连接"。
*   **启动:** 点击"启动"开始数据流。
*   **录制:** 点击"录制"（需要活跃的采集）保存数据。
*   **回放:** 点击"回放"（需要设备未连接）打开 `.lgv` 文件。使用控制栏进行搜索、暂停或改变速度。
*   **控制:**
    *   左键鼠标: 旋转
    *   中键鼠标: 平移
    *   右键鼠标: 缩放
    *   `R`: 重置相机

### 配置文件

配置文件位于 `config/` 目录中：
- `app_config.json` - 应用程序设置
- `mid360_config.json` - Livox Mid-360 网络设置

### 许可证

[许可证信息]
