# ArduPlane EKF3 感測器融合與 PID 調校

## EKF3 概述

ArduPilot EKF3（`AP_NavEKF3`）同樣使用**誤差狀態擴展卡爾曼濾波器**，但在架構上與 PX4 EKF2 有顯著差異。

### 核心架構特點

- **感測器親和性（Sensor Affinity）**：不同 EKF 車道可綁定不同的感測器實例（例如 Lane 1 用 GPS 1，Lane 2 用 GPS 2），由 `EK3_AFFINITY` 位元遮罩控制
- **車道切換（Lane Switching）**：透過累積車道誤差比較（`EK3_ERR_THRESH`）決定是否切換至表現更好的車道
- **DCM 後備**：ArduPlane 背景運行 DCM 估計器，當 EKF3 健康度下降時可回退

### 狀態向量（24 個元素）

| 索引 | 狀態 | 說明 |
|------|------|------|
| 0-3 | `quat` | 旋轉四元數 NED→Body |
| 4-6 | `velocity` | NED 速度 (m/s) |
| 7-9 | `position` | NED 位置 (m) |
| 10-12 | `gyro_bias` | 角度增量偏差 (rad) |
| 13-15 | `accel_bias` | 速度增量偏差 (m/s) |
| 16-18 | `earth_magfield` | 地球座標磁場 (Gauss) |
| 19-21 | `body_magfield` | 機體座標磁場 (Gauss) |
| 22-23 | `wind_vel` | NE 風速 (m/s) |

> **與 PX4 EKF2 的差異**：EKF3 有 24 個狀態，**不包含地形狀態**（地形由獨立模組處理）。PX4 EKF2 在 index 24 包含地形高度，共 25 個元素。

---

## EKF3 vs ArduPilot 內部的 EKF2

ArduPilot 本身也有 EKF2（`AP_NavEKF2`），注意這與 PX4 的 EKF2 是完全不同的實作：

| 特性 | ArduPilot EKF2 | ArduPilot EKF3 |
|------|---------------|----------------|
| 感測器親和性 | 無 — 所有核心使用主感測器 | 有 — 核心可綁定非主感測器 |
| 車道切換 | 基本健康度判斷 | 累積誤差閾值（`EK3_ERR_THRESH`） |
| 陀螺儀比例因子 | 有估計 | 移除（簡化） |
| 風速估計時機 | 較晚開始 | 較早開始（反應更快） |
| 額外感測器支援 | 標準感測器 | Beacon、輪速計、視覺里程計 |
| 狀態 | 已棄用 | **ArduPilot 4.1+ 預設** |

---

## EKF3 關鍵參數

### 核心設定

| 參數 | 說明 |
|------|------|
| `EK3_ENABLE` | 啟用 EKF3（設為 1） |
| `AHRS_EKF_TYPE` | 設為 3 使用 EKF3 |
| `EK3_IMU_MASK` | 選擇使用的 IMU |

### 感測器親和性

| 參數 | 說明 |
|------|------|
| `EK3_AFFINITY` | 位元遮罩：bit 0=氣壓計, bit 1=磁力計, bit 2=GPS, bit 3=空速計 |
| `EK3_ERR_THRESH` | 車道切換誤差閾值（預設 0.2） |

### 量測雜訊

| 參數 | 說明 |
|------|------|
| `EK3_VELNE_M_NSE` | 水平速度雜訊 |
| `EK3_VELD_M_NSE` | 垂直速度雜訊 |
| `EK3_POSNE_M_NSE` | 水平位置雜訊 |
| `EK3_ALT_M_NSE` | 高度雜訊 |
| `EK3_MAG_M_NSE` | 磁力計雜訊 |
| `EK3_EAS_M_NSE` | 空速雜訊 |

### 過程雜訊

| 參數 | 說明 |
|------|------|
| `EK3_GYRO_P_NSE` | 陀螺儀過程雜訊 |
| `EK3_ACC_P_NSE` | 加速度計過程雜訊 |
| `EK3_GBIAS_P_NSE` | 陀螺儀偏差過程雜訊 |
| `EK3_WIND_P_NSE` | 風速過程雜訊 |

### 創新門檻

| 參數 | 說明 |
|------|------|
| `EK3_VEL_I_GATE` | 速度拒絕閾值 (SD) |
| `EK3_POS_I_GATE` | 位置拒絕閾值 (SD) |
| `EK3_HGT_I_GATE` | 高度拒絕閾值 (SD) |

---

## PID 控制器架構

ArduPlane 同樣使用串級控制，但將**前饋（FF）視為主要控制機制**，PID 提供修正：

```
目標位置 → [導航控制器] → 目標姿態 → [姿態控制器] → 目標角速度 → [角速度控制器] → 舵面輸出
              外環              中間環（P 控制器）         內環（FF 主導 + PID 修正）
```

### 角速度控制器參數

| 功能 | Roll | Pitch | Yaw |
|------|------|-------|-----|
| 前饋 | `RLL_RATE_FF` | `PTCH_RATE_FF` | `YAW_RATE_FF` |
| 比例 | `RLL_RATE_P` | `PTCH_RATE_P` | `YAW_RATE_P` |
| 積分 | `RLL_RATE_I` | `PTCH_RATE_I` | `YAW_RATE_I` |
| 微分 | `RLL_RATE_D` | `PTCH_RATE_D` | `YAW_RATE_D` |
| 積分限制 | `RLL_RATE_IMAX` | `PTCH_RATE_IMAX` | `YAW_RATE_IMAX` |
| 擺率限制 | `RLL_RATE_SMAX` | `PTCH_RATE_SMAX` | `YAW_RATE_SMAX` |
| 目標濾波 | `RLL_RATE_FLTT` | `PTCH_RATE_FLTT` | `YAW_RATE_FLTT` |
| 誤差濾波 | `RLL_RATE_FLTE` | `PTCH_RATE_FLTE` | `YAW_RATE_FLTE` |
| D 項濾波 | `RLL_RATE_FLTD` | `PTCH_RATE_FLTD` | `YAW_RATE_FLTD` |

### 姿態控制器參數

| 參數 | 說明 |
|------|------|
| `RLL2SRV_TCONST` | Roll 時間常數（預設 0.5 s） |
| `PTCH2SRV_TCONST` | Pitch 時間常數（預設 0.5 s） |
| `RLL2SRV_RMAX` | 最大滾轉速率（預設 0 = 無限） |
| `PTCH2SRV_RMAX_UP` | 最大抬頭速率 |
| `PTCH2SRV_RMAX_DN` | 最大低頭速率 |
| `PTCH2SRV_RLL` | 轉彎時的俯仰補償 |

### 調校方法

ArduPlane 的調校以**前饋為核心**：

1. **從日誌分析確定 FF 增益**（需求角速度與舵面偏轉的比值）
2. **設 I = FF**（提供約 1 秒的回應時間）
3. **逐步增加 P**（以 0.01 為單位，直到震盪，再減 25-50%）
4. **逐步增加 D**（以 0.001 為單位，直到震盪，再減半）
5. **D 項會主動調校並濾波**（不像 PX4 預設為 0）
6. 可使用 `AUTOTUNE_LEVEL` 參數啟用自動調校

### SITL 中即時調參

```bash
# 在 MAVProxy console 中
param set RLL_RATE_FF 0.3
param set RLL_RATE_P 0.08
param set RLL_RATE_I 0.3
param set RLL_RATE_D 0.01
param show RLL_RATE_*
param save
```

---

## PX4 vs ArduPlane 控制器哲學差異

| 面向 | PX4 | ArduPlane |
|------|-----|-----------|
| 姿態環 | 時間常數法（隱式產生 P 增益） | 明確的姿態→角速度轉換參數 |
| 控制主體 | 平衡的 PID + FF | FF 為主，PID 為修正 |
| 輸出濾波 | 最少 | 擺率限制 + 三組低通濾波器 |
| 空速增益調整 | `FW_ARSP_SCALE_EN` | `SCALING_SPEED` |
| D 項使用 | 預設 0，很少調 | 主動使用並濾波 |
| 轉彎俯仰補償 | 內建於姿態控制器 | 明確的 `PTCH2SRV_RLL` 參數 |

---

## 參考資料

- [ArduPilot EKF3 概述](https://ardupilot.org/plane/docs/common-apm-navigation-extended-kalman-filter-overview.html)
- [ArduPilot EKF3 Affinity & Lane Switching](https://ardupilot.org/plane/docs/common-ek3-affinity-lane-switching.html)
- [ArduPlane Roll/Pitch/Yaw Controller Tuning](https://ardupilot.org/plane/docs/new-roll-and-pitch-tuning.html)
- [ArduPilot Methodic Configurator Tuning Guide](https://ardupilot.github.io/MethodicConfigurator/TUNING_GUIDE_ArduPlane.html)
- [ArduPilot EKF3 Source (GitHub)](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_core.h)
