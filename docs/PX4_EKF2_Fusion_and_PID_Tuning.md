# PX4 EKF2 感測器融合與 PID 調校

## EKF2 概述

PX4 使用 **EKF2**（Error-State Extended Kalman Filter），以誤差狀態公式估計旋轉向量並修正名義狀態，提升數值穩定性。

### 核心架構特點

- **延遲融合時間軸**：各感測器資料經 FIFO 緩衝，依 `EKF2_*_DELAY` 參數在適當延遲時取出融合
- **輸出預測器**：互補濾波器從延遲融合時間軸前推至當前時刻，避免重算所有 IMU 資料
- **多實例支援**：可同時運行 N = (IMU 數量) × (磁力計數量) 個 EKF 實例，選擇器自動選擇最佳

### 狀態向量（25 個元素 / 24 自由度）

| 索引 | 狀態 | 自由度 | 說明 |
|------|------|--------|------|
| 0-3 | `quat_nominal` | 3 | 旋轉四元數 NED→Body（4 元素，3 DOF） |
| 4-6 | `vel` | 3 | NED 速度 (m/s) |
| 7-9 | `pos` | 3 | NED 位置 (m) |
| 10-12 | `gyro_bias` | 3 | 陀螺儀偏差 (rad/s) |
| 13-15 | `accel_bias` | 3 | 加速度計偏差 (m/s²) |
| 16-18 | `mag_I` | 3 | 地球磁場 NED (Gauss) |
| 19-21 | `mag_B` | 3 | 機體磁場偏差 (Gauss) |
| 22-23 | `wind_vel` | 2 | 風速 NE (m/s) |
| 24 | `terrain` | 1 | 地形高度 (m) |

> 協方差矩陣為 24×24（四元數使用 3 DOF 誤差狀態）。

---

## 固定翼專用融合

### 空速融合

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EKF2_ARSP_THR` | 0.0 m/s | 空速高於此值時融合（設 > 0 啟用） |
| `EKF2_ASP_DELAY` | 100 ms | 空速量測延遲補償 |
| `EKF2_TAS_GATE` | 5.0 SD | 真空速創新一致性門檻 |
| `EKF2_EAS_NOISE` | 1.4 m/s | 等效空速量測雜訊 |

### 合成側滑融合（協調飛行假設）

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EKF2_FUSE_BETA` | 0 | 啟用合成側滑融合 |
| `EKF2_BETA_GATE` | 5.0 SD | 創新門檻 |
| `EKF2_BETA_NOISE` | 0.3 m/s | 側滑雜訊 |

### 風速估計

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EKF2_WIND_NSD` | 0.05 m/s²/√Hz | 風速過程雜訊頻譜密度 |

> 對固定翼而言，同時啟用空速融合（`EKF2_ARSP_THR > 0`）和側滑融合（`EKF2_FUSE_BETA = 1`）才能獲得可靠的風速估計。

---

## PID 控制器架構

PX4 固定翼使用**串級 PID + 前饋（FF）**控制：

```
目標位置 → [位置控制器] → 目標姿態 → [姿態控制器] → 目標角速度 → [角速度控制器] → 舵面輸出
              外環              中間環（時間常數）          內環（PID + FF）
```

### 角速度控制器參數

| 功能 | Roll | Pitch | Yaw |
|------|------|-------|-----|
| 前饋 | `FW_RR_FF` (0.5) | `FW_PR_FF` (0.5) | `FW_YR_FF` (0.3) |
| 比例 | `FW_RR_P` (0.05) | `FW_PR_P` (0.08) | `FW_YR_P` (0.05) |
| 積分 | `FW_RR_I` (0.1) | `FW_PR_I` (0.1) | `FW_YR_I` (0.1) |
| 微分 | `FW_RR_D` (0.0) | `FW_PR_D` (0.0) | `FW_YR_D` (0.0) |
| 積分限制 | `FW_RR_IMAX` (0.2) | `FW_PR_IMAX` (0.4) | `FW_YR_IMAX` (0.2) |

### 姿態控制器參數

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `FW_R_TC` | 0.4 s | Roll 時間常數（越小反應越快） |
| `FW_P_TC` | 0.4 s | Pitch 時間常數 |
| `FW_R_RMAX` | 70 deg/s | 最大滾轉速率 |
| `FW_P_RMAX_POS` | 60 deg/s | 最大抬頭速率 |
| `FW_P_RMAX_NEG` | 60 deg/s | 最大低頭速率 |

### 調校方法

1. **先調 Roll 軸**（比 Pitch 安全），再調 Pitch
2. **順序**：前饋（FF）→ 比例（P）→ 積分（I）
3. 將增益加倍直到不穩定，再減半
4. **微分（D）預設為 0**，固定翼通常不需要調
5. 可使用 `FW_AT_*` 參數啟用自動調校

### SITL 中即時調參

```bash
# 在 PX4 shell (pxh>) 中
param set FW_RR_P 0.05
param set FW_RR_I 0.01
param set FW_RR_FF 0.5
param show FW_RR_*
param save
```

---

## 參考資料

- [PX4 EKF2 Navigation Filter Guide](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)
- [PX4 Fixed-Wing PID Tuning Guide](https://docs.px4.io/main/en/config_fw/pid_tuning_guide_fixedwing)
- PX4 原始碼：`src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h`
- PX4 原始碼：`src/modules/fw_rate_control/fw_rate_control_params.c`
- PX4 原始碼：`src/modules/fw_att_control/fw_att_control_params.c`
