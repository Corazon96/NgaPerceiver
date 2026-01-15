# LingerPerceiver ARM64 部署指南

本文档记录了将 LingerPerceiver 部署到 Raspberry Pi CM5 的完整流程和注意事项。

## 目录

- [硬件配置](#硬件配置)
- [部署包内容](#部署包内容)
- [部署步骤](#部署步骤)
- [运行程序](#运行程序)
- [性能监控](#性能监控)
- [配置说明](#配置说明)
- [常见问题](#常见问题)
- [重新编译](#重新编译)

---

## 硬件配置

| 项目 | 规格 |
|------|------|
| **型号** | Raspberry Pi Compute Module 5 Rev 1.0 |
| **CPU** | ARM aarch64, 4核 @ 1.9 GHz |
| **内存** | 1GB ARM + 4MB GPU |
| **系统** | Debian 13 (trixie), Kernel 6.12 |
| **架构** | aarch64 (ARM64) |

---

## 部署包内容

部署包位于 `build_arm64/deploy/`，包含以下文件：

```
deploy/
├── linger_perceiver_service    # 可执行文件 (~1.4MB)
├── run.sh                      # 启动脚本
├── monitor_linger.sh           # 性能监控脚本
├── hardware_info.sh            # 硬件信息查看脚本
├── config/
│   ├── app_config.json         # 应用配置（滤波器、UDP等）
│   └── mid360_config.json      # Livox Mid-360 配置
├── lib/                        # 依赖库
│   ├── libpcl_*.so.*           # PCL 点云库
│   ├── libgomp.so.*            # OpenMP
│   └── ...
└── *.lgv                       # 测试回放文件
```

---

## 部署步骤

### 1. 首次部署（完整包）

```powershell
# Windows PowerShell
scp -r "d:\nga\work\pro\LingerPerceiver0.0.1\build_arm64\deploy" linger@192.168.1.99:~/linger_perceiver
```

### 2. 验证部署

```bash
# 树莓派终端
cd ~/linger_perceiver
ls -la

# 注意：scp 已自动保留执行权限，通常不需要手动 chmod
# 如果权限不对，可以运行：
# chmod +x linger_perceiver_service run.sh monitor_linger.sh hardware_info.sh
```

### 3. 快速更新（仅可执行文件）

```powershell
# Windows PowerShell - 代码更新后只需传输可执行文件
scp "d:\nga\work\pro\LingerPerceiver0.0.1\build_arm64\deploy\linger_perceiver_service" linger@192.168.1.99:~/linger_perceiver/
```

---

## 运行程序

### 基本用法

```bash
cd ~/linger_perceiver

# 查看帮助
./run.sh -h

# 回放模式（测试用）
./run.sh -r berthing.lgv

# 实时采集模式（自动搜索 Livox 配置）
./run.sh

# 指定特定配置文件（可选）
./run.sh -c ./config/mid360_config.json

# 后台运行
nohup ./run.sh -r berthing.lgv > linger.log 2>&1 &
```

### 退出程序

- **正常退出**: 按 `Ctrl+C`，等待 3-5 秒让线程清理完成
- **强制退出**: 快速按两次 `Ctrl+C`
- **后台进程**: `kill $(pgrep -f linger_perceiver_service)`

### 为什么需要 run.sh？

直接运行 `./linger_perceiver_service` 会报错找不到共享库。

`run.sh` 的作用是设置 `LD_LIBRARY_PATH`，告诉系统在 `lib/` 目录中查找依赖库：

```bash
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:$LD_LIBRARY_PATH"
```

如果想直接运行，需要手动设置：
```bash
export LD_LIBRARY_PATH=~/linger_perceiver/lib:$LD_LIBRARY_PATH
./linger_perceiver_service -r berthing.lgv --app-config ./config/app_config.json
```

---

## 性能监控

### 实时监控

```bash
./monitor_linger.sh
```

### 关键指标

| 指标 | 正常范围 | 危险阈值 | 查看命令 |
|------|----------|----------|----------|
| **CPU 温度** | < 70°C | > 80°C | `vcgencmd measure_temp` |
| **CPU 使用率** | < 300% | > 350% | `top` 或 `htop` |
| **内存使用** | < 700MB | > 900MB | `free -h` |
| **进程状态** | Running | - | `pgrep -f linger_perceiver` |

### 快速查看

```bash
# 一次性查看所有关键指标
echo "CPU温度: $(vcgencmd measure_temp)"
free -h | grep Mem
ps aux | grep linger_perceiver_service | grep -v grep
```

### 查看硬件信息

```bash
./hardware_info.sh
```

---

## 配置说明

### app_config.json

UDP 通信配置（修改后需重启程序）：

```json
{
    "udp": {
        "target_ip": "192.168.1.50",  // Windows 接收端 IP
        "target_port": 5000,           // UDP 端口
        "sensor_id": 3,                // 传感器 ID
        "enabled": true
    }
}
```

### mid360_config.json

Livox Mid-360 雷达配置，通常不需要修改。

### 修改配置

```bash
nano ~/linger_perceiver/config/app_config.json
```

---

## 常见问题

### Q1: 程序报错找不到 .so 文件

**原因**: 没有使用 `run.sh` 启动

**解决**:
```bash
./run.sh -r berthing.lgv  # 使用 run.sh
```

### Q2: UDP 消息收不到

**检查项**:
1. 确认 `app_config.json` 中 `target_ip` 是 Windows 的 IP
2. 确认防火墙没有阻止 UDP 端口 5000
3. 确认 Windows 上的 `udp_receiver.exe` 正在运行

```bash
# 测试网络连通性
ping 192.168.1.50
```

### Q3: 程序退出慢

**正常现象**: 按 Ctrl+C 后需要 3-5 秒等待线程清理

**强制退出**: 快速按两次 Ctrl+C

### Q4: CPU 温度过高

**解决方案**:
1. 增加散热片或风扇
2. 降低环境温度
3. 降低处理帧率（修改代码）

### Q5: 回放速度太快/太慢

回放默认按原始采集速度播放。如需调整，需修改代码中的回放速度参数。

---

## 重新编译

### 前置条件

- Docker Desktop (Windows)
- WSL2 + QEMU ARM64 支持

### 编译命令

```powershell
cd d:\nga\work\pro\LingerPerceiver0.0.1

# 完整构建（首次需要构建 Docker 镜像，约 1-2 小时）
.\docker\build_arm64.ps1

# 强制重建 Docker 镜像
.\docker\build_arm64.ps1 -RebuildImage

# 清理构建缓存
.\docker\build_arm64.ps1 -Clean
```

### 编译输出

编译完成后，部署包自动生成在 `build_arm64/deploy/`。

### Docker 镜像

- 镜像名: `linger-arm64-builder`
- 包含: PCL 1.15, Livox SDK2, Boost, Eigen3, OpenMPI
- 首次构建后会被缓存，后续编译很快

---

## 开机自启动（可选）

### 创建 systemd 服务

```bash
sudo tee /etc/systemd/system/linger.service << 'EOF'
[Unit]
Description=LingerPerceiver Service
After=network.target

[Service]
Type=simple
User=linger
WorkingDirectory=/home/linger/linger_perceiver
ExecStart=/home/linger/linger_perceiver/run.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
```

### 启用服务

```bash
sudo systemctl daemon-reload
sudo systemctl enable linger
sudo systemctl start linger
```

### 管理命令

```bash
sudo systemctl status linger    # 查看状态
sudo systemctl stop linger      # 停止
sudo systemctl restart linger   # 重启
journalctl -u linger -f         # 查看日志
```

---

## 网络配置

### 固定 IP（推荐生产环境）

```bash
sudo nano /etc/dhcpcd.conf

# 添加以下内容
interface eth0
static ip_address=192.168.1.99/24
static routers=192.168.1.1
static domain_name_servers=8.8.8.8
```

### 当前网络配置

| 设备 | IP 地址 | 说明 |
|------|---------|------|
| 树莓派 | 192.168.1.99 | 运行 linger_perceiver_service |
| Windows | 192.168.1.50 | 运行 udp_receiver.exe |
| Livox Mid-360 | 192.168.1.1xx | 雷达（默认 DHCP） |

---

## 附录：快速命令参考

```bash
# === 部署 ===
scp -r build_arm64/deploy linger@192.168.1.99:~/linger_perceiver

# === 运行 ===
./run.sh -r berthing.lgv          # 回放
./run.sh                          # 实时采集（自动搜索 Livox 配置）

# === 监控 ===
./monitor_linger.sh               # 实时监控
./hardware_info.sh                # 硬件信息
vcgencmd measure_temp             # CPU 温度
free -h                           # 内存使用
htop                              # 进程监控

# === 服务管理 ===
sudo systemctl start linger       # 启动
sudo systemctl stop linger        # 停止
sudo systemctl status linger      # 状态

# === Windows UDP 接收 ===
cd d:\nga\work\pro\LingerPerceiver0.0.1\build\tools
.\udp_receiver.exe
```
