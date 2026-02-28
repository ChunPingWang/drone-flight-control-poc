# PX4 SITL + Gazebo Harmonic 固定翼模擬環境

Ubuntu 24.04 上的 PX4 SITL 開發環境，搭配 Gazebo Harmonic 模擬器，用於固定翼無人機模擬。

## Gazebo Sim 模擬畫面

![Gazebo Sim - RC Cessna](docs/images/gazebo-sim-rc-cessna.png)

Gazebo Harmonic 模擬器中的 RC Cessna 固定翼飛機模型。右側面板顯示 World 設定（物理引擎：gz-physics-dartsim-plugin、碰撞偵測：ODE）以及 Entity Tree 中的場景元素（ground_plane、sunUTC、rc_cessna_0）。

## 快速啟動

```bash
# 啟動 RC Cessna（預設）
./start-sitl.sh

# 啟動進階固定翼
./start-sitl.sh advanced_plane

# 無 GUI 模式
HEADLESS=1 ./start-sitl.sh
```

或直接使用 make：

```bash
cd PX4-Autopilot
make px4_sitl gz_rc_cessna
```

## 可用的固定翼 Make Target

| Target | 說明 |
|---|---|
| `gz_rc_cessna` | RC Cessna 小型飛機 |
| `gz_advanced_plane` | 進階固定翼飛機 |

## PX4 SITL Shell 常用指令

SITL 啟動後會進入 PX4 shell（`pxh>`），可使用以下指令：

```bash
# 起飛 / 降落
commander takeoff
commander land

# 查看感測器狀態
sensor_accel_sim status
sensor_gyro_sim status

# 查看飛行模式
commander status

# GPS 狀態
gps status
```

## 環境變數

| 變數 | 用途 | 範例 |
|---|---|---|
| `HEADLESS=1` | 不啟動 Gazebo GUI | `HEADLESS=1 make px4_sitl gz_rc_cessna` |
| `PX4_GZ_WORLD=windy` | 指定 Gazebo 世界場景 | 模擬風場環境 |
| `PX4_SIM_SPEED_FACTOR=2` | 加速模擬 | 2 倍速運行 |
| `PX4_HOME_LAT` | 起飛緯度 | `PX4_HOME_LAT=25.034` |
| `PX4_HOME_LON` | 起飛經度 | `PX4_HOME_LON=121.564` |
| `PX4_HOME_ALT` | 起飛高度 (m) | `PX4_HOME_ALT=10` |

## QGroundControl 地面站

```bash
./QGroundControl-x86_64.AppImage
```

QGC 會自動透過 UDP 14550 連接到 PX4 SITL，無需額外設定。

## 驗證步驟

1. `make px4_sitl gz_rc_cessna` → Gazebo 視窗出現 Cessna 模型
2. PX4 shell 中 `commander takeoff` → 飛機起飛
3. 啟動 QGroundControl → 自動連接，顯示飛機位置和狀態
4. `commander land` → 飛機降落

## 環境安裝紀錄

```bash
# 1. Clone PX4-Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# 2. 執行自動化安裝腳本（安裝 Gazebo Harmonic、cmake、ninja 等）
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# 3. 建立 PX4 專用 Python venv（Ubuntu 24.04 PEP 668 限制）
python3 -m venv PX4-Autopilot/.venv
PX4-Autopilot/.venv/bin/pip install -r PX4-Autopilot/Tools/setup/requirements.txt

# 4. 重新登入（讓 dialout group 生效）

# 5. 首次建置（約 5-15 分鐘）
cd PX4-Autopilot && make px4_sitl gz_rc_cessna
```

## 已知問題：ESP-IDF PATH 衝突

如果系統安裝了 ESP-IDF，其 Python venv 會被 CMake 優先使用，導致找不到 `kconfiglib` 等模組。
`start-sitl.sh` 已自動處理此問題（排除 espressif 路徑並啟用 PX4 venv）。

手動建置時需先執行：
```bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v espressif | grep -v 'esp/esp-idf' | tr '\n' ':' | sed 's/:$//')
source PX4-Autopilot/.venv/bin/activate
```
