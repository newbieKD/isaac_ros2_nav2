# 企業版 Isaac Sim + ROS2 + Nav2 完整環境

[![Docker Build](https://img.shields.io/badge/Docker-Build%20Passing-green)](https://hub.docker.com/r/newbiekd/isaac_sim_lab_ros2_nav2)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-4.5.0-blue)](https://developer.nvidia.com/isaac-sim)
[![Isaac Lab](https://img.shields.io/badge/Isaac%20Lab-2.1.0-blue)](https://isaac-sim.github.io/IsaacLab/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)](https://docs.ros.org/en/humble/)

## 📑 目錄

- [環境需求](#環境需求)
- [專案介紹](#專案介紹)
- [檔案結構](#檔案結構)
- [快速開始](#快速開始)
- [完整測試流程](#完整測試流程)
  - [User Mode 啟動測試](#1-user-mode-啟動測試)
  - [X11VNC 遠端桌面測試](#2-x11vnc-遠端桌面測試)
  - [Isaac Sim 啟動測試](#3-isaac-sim-啟動測試)
  - [IsaacLab 功能測試](#4-isaaclab-功能測試)
  - [ROS2 功能測試](#5-ros2-功能測試)
- [常見問題排解](#常見問題排解)

## 環境需求

- **操作系統**: Ubuntu 20.04+ / Windows 11 (WSL2) / macOS (Docker Desktop)
- **硬體需求**:
  - NVIDIA GPU (支援 CUDA 11.8+)
  - 記憶體: 16GB+ (建議 32GB)
  - 儲存空間: 50GB+ 可用空間
- **軟體需求**:
  - Docker Engine 20.10+
  - Docker Compose 2.0+
  - NVIDIA Container Toolkit
- **網路需求**: 企業環境需確保可存取 Docker Hub 和 NVIDIA NGC

## 專案介紹

本專案提供一個**完整測試且可用於企業環境**的 Isaac Sim + ROS2 + Nav2 Docker 容器環境。

### � 主要特色

- ✅ **User Mode 啟動**: 支援非 root 用戶啟動，符合企業安全要求
- ✅ **X11VNC 遠端桌面**: 內建 VNC 服務，支援遠端 GUI 操作
- ✅ **GPU 加速**: 完整 NVIDIA GPU 支援和 CUDA 運算
- ✅ **完整整合**: Isaac Sim 4.5.0 + Isaac Lab 2.1.0 + ROS2 Humble + Nav2
- ✅ **企業友善**: 支援 Proxy、防火牆、自訂 UID/GID 等企業環境需求

### 📦 包含組件

- **Isaac Sim 4.5.0** - NVIDIA 3D 機器人模擬平台
- **Isaac Lab 2.1.0** - 機器人學習和訓練框架
- **ROS2 Humble** - 機器人作業系統 
- **Nav2** - ROS2 導航堆疊
- **X11VNC** - 遠端桌面服務
- **CUDA Toolkit** - GPU 運算支援

## �📁 檔案結構

```
h1_ws_corporate/
├── README.md                               # 本文件
├── VNC_README.md                           # VNC 詳細說明
└── docker/
    ├── Dockerfile                          # 主要建構檔案
    ├── compose.yaml                        # Docker Compose 配置
    ├── cyclonedx.xml                       # ROS2 DDS 設定
    ├── .bashrc                            # Bash 配置和別名
    └── modules/                           # 模組化安裝腳本
        ├── install_ros.sh                  # ROS2 Humble + Nav2
        ├── install_x11_opengl_vulkan.sh    # 圖形界面支援
        ├── install_cuda_toolkit.sh         # CUDA 工具包
        ├── install_isaac_sim.sh            # Isaac Sim 4.5.0
        ├── install_isaac_lab.sh            # Isaac Lab 2.1.0
        ├── install_isaac_ros.sh            # Isaac ROS
        ├── install_vnc.sh                  # VNC 套件安裝
        ├── start_xvfb_vnc.sh              # VNC 啟動腳本
        ├── stop_xvfb_vnc.sh               # VNC 停止腳本
        └── cleanup_vnc.sh                  # VNC 清理腳本
```

## 🚀 快速開始

### 1. 建構映像
```bash
cd h1_ws_corporate/docker
docker compose build
```

### 2. 啟動容器 (開發模式)
```bash
xhost +local:docker
docker compose up -d
```

### 3. 進入容器
```bash
docker compose exec h1-ws bash
```

## 🧪 完整測試流程

**本專案已通過完整的 User Mode（非 root）測試流程，包含 GPU、X11VNC 以及 ROS2、Isaac Sim、IsaacLab 等主要功能皆已驗證通過。**

### 1. User Mode 啟動測試

測試容器以非 root 用戶模式正確啟動，確保企業安全要求：

```bash
# 啟動純 User Mode 容器 (UID:1017)
docker run --rm -it \
  --gpus all \
  --runtime=nvidia \
  --network=host \
  --user 1017:1017 \
  ghcr.io/newbiekd/isaac_ros2_nav2:3.1.0 bash
  
```

**驗證項目**:
- ✅ 非 root 用戶可正常啟動
- ✅ GPU 裝置正確掛載和存取
- ✅ 網路功能正常
- ✅ 所有必要權限已正確設定

**注意事項**:
- 請確認您的用戶 UID (可用 `id -u` 查看) 並替換 `1017:1017`
- `--network=host` 確保 VNC 和其他服務正常運作

### 2. X11VNC 遠端桌面測試

測試 X11VNC 遠端桌面功能，支援遠端 GUI 操作：

```bash
# 進入容器後執行
vnc-display
start-xvfb-vnc

# 檢查 VNC 狀態
vnc-user-status

# (option)停止 VNC 服務
stop-xvfb-vnc
```

**驗證 VNC 圖形介面**:
```bash
# 測試 ROS2 圖形化工具
ros2 run rviz2 rviz2

# 測試其他 GUI 應用
xclock
# 或
xeyes
```

**VNC 連線資訊**:
- **地址**: `<主機 IP>:5900`
- **密碼**: `0000` (預設)
- **顯示**: `:1` (1920x1080)

**注意事項**:
- VNC 使用虛擬顯示器 Xvfb，不需實體螢幕
- 所有 VNC 操作皆在 user 權限下執行
- 若連線失敗，請檢查防火牆設定

### 3. Isaac Sim 啟動測試

測試兩種 Isaac Sim 啟動方式，確保 3D 模擬環境正常運作：

#### 方式一：串流模式 (Streaming)
```bash
cd ~/isaacsim
./isaac-sim.streaming.sh
```

#### 方式二：標準模式
```bash
cd ~/isaacsim
./isaac-sim.sh
```

**驗證項目**:
- ✅ Isaac Sim 4.5.0 正確啟動
- ✅ GPU 加速正常運作
- ✅ 3D 渲染功能正常
- ✅ WebRTC 串流功能 (streaming 模式)

**注意事項**:
- 串流模式支援Isaac Sim WebRTC Streaming Client存取
- 標準模式需透過 VNC 查看 GUI
- 首次啟動可能需要較長時間載入

### 4. IsaacLab 功能測試

測試 IsaacLab 強化學習訓練環境，確保機器人學習框架正常運作：

```bash
cd ~/IsaacLab

# 執行 H1 人形機器人訓練任務
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Rough-H1-v0 \
  --headless \
  --livestream 2

# 目前IsaacLab 2.1.0的train file有錯誤，因此"--livestream 2"無法work，需要額外更改train.py
# 可使用以下code驗證是否能訓練即可
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Rough-H1-v0 \
  --headless 
```



**WebRTC 連線方式**:
- 串流模式支援Isaac Sim WebRTC Streaming Client存取
- 支援即時觀看訓練過程

**驗證項目**:
- ✅ IsaacLab 2.1.0 正確載入
- ✅ 強化學習環境正常初始化
- ✅ GPU 訓練加速功能正常
- ✅ WebRTC 串流正常顯示

**注意事項**:
- `--headless` 模式不需顯示器，適合伺服器環境
- `--livestream 2` 啟用 WebRTC 串流
- 訓練過程會產生檢查點檔案，需確保有足夠儲存空間

### 5. ROS2 功能測試

測試 ROS2 Humble 和 Nav2 導航功能：

#### 重要提醒
> **⚠️ 容器本身並不包含 ROS2 workspace，需要自行複製進去**

#### 複製 ROS2 Workspace
```bash
# 將本地 workspace 複製到容器中
docker cp <本地workspace路徑>/src <container_name>:/home/ros2-essentials/humble_ws/src

# 範例：
docker cp /path/to/JDP_code/humble_ws/src <container_name>>:/home/ros2-essentials/humble_ws/src
```

#### ROS2 環境設定和測試
```bash
# 進入 ROS2 workspace
cd /home/ros2-essentials/humble_ws/

# 建構所有套件
colcon build

# 設定環境變數
source /home/ros2-essentials/humble_ws/install/setup.bash

# 建構特定套件
colcon build --packages-select humanoid_navigation
source install/setup.bash

# 啟動導航服務
ros2 launch humanoid_navigation h1_policy_improved.launch.py
```

#### 基本 ROS2 功能驗證
```bash
# 檢查 ROS2 安裝
ros2 --version

# 列出可用節點
ros2 node list

# 檢查主題
ros2 topic list

# 測試發布訂閱
ros2 run demo_nodes_cpp talker
# (在另一終端) ros2 run demo_nodes_cpp listener
```

**驗證項目**:
- ✅ ROS2 Humble 正確安裝和配置
- ✅ Nav2 導航堆疊可正常載入
- ✅ colcon 建構工具正常運作
- ✅ 自訂 ROS2 套件可正常建構和執行

**注意事項**:
- ROS2 workspace 需要手動複製，容器不預載專案代碼
- 建構過程可能需要較長時間，請確保有足夠計算資源
- 某些導航功能可能需要額外的感測器資料或地圖檔案

---

## 🔧 常用指令別名

容器內建以下便捷指令：

### VNC 相關
- `start-xvfb-vnc` - 啟動 X11VNC 服務
- `stop-xvfb-vnc` - 停止 X11VNC 服務
- `cleanup-vnc` - 清理 VNC 相關檔案
- `vnc-user-status` - 檢查用戶 VNC 程序狀態
- `vnc-user-logs` - 查看 VNC 日誌
- `vnc-display` - 設定顯示到 VNC (:1)
- `x11-display` - 恢復 X11 轉發顯示

### 系統資訊
- `gpu-info` - 顯示 GPU 資訊
- `system-info` - 顯示系統資訊

詳細 VNC 說明請參考：[VNC_README.md](./VNC_README.md)

## 常見問題排解

### Q1: 容器啟動時權限錯誤
**問題**: `docker: Error response from daemon: failed to create task for container: failed to create shim task`

**解決方案**:
```bash
# 檢查當前用戶 UID
id -u

# 使用正確的 UID:GID 啟動
docker run --user $(id -u):$(id -g) ...
```

### Q2: VNC 連線失敗
**問題**: 無法連線到 VNC 服務 (port 5900)

**解決方案**:
```bash
# 檢查 VNC 程序是否運行
vnc-user-status

# 檢查端口是否被占用
netstat -tlnp | grep :5900

# 重新啟動 VNC
stop-xvfb-vnc
start-xvfb-vnc
```

### Q3: GPU 不可用
**問題**: Isaac Sim 無法使用 GPU 加速

**解決方案**:
```bash
# 檢查 NVIDIA 驅動
nvidia-smi

# 確認 Docker 啟動參數包含
--gpus all --runtime=nvidia
```

### Q4: X11 轉發問題
**問題**: GUI 應用無法顯示

**解決方案**:
```bash
# 主機端允許 X11 轉發
xhost +local:docker

# 容器內設定正確的 DISPLAY
export DISPLAY=:0  # 用於 X11 轉發
# 或
vnc-display      # 用於 VNC 顯示
```


### Q5: 檔案掛載權限問題
**問題**: 掛載的檔案或目錄權限錯誤

**解決方案**:
```bash
# 確保主機端檔案權限正確
sudo chown -R $(id -u):$(id -g) /path/to/mounted/folder

# 或在 compose.yaml 中設定正確的 user
user: "${UID:-1000}:${GID:-1000}"
```

## 🏢 企業環境注意事項


### 防火牆設定
需要開放的連接埠：
- **5900**: VNC 遠端桌面
- **8211**: Isaac Sim WebRTC 串流

### 安全考量
- 使用非 root 用戶執行所有服務
- VNC 密碼請修改為更安全的密碼
- 建議在內網環境使用，避免直接暴露於公網

---

## 📞 支援與貢獻

如有問題或建議，請提交 Issue 或 Pull Request。

**維護團隊**: Binghua Cai  
**更新日期**: 2025年7月 

