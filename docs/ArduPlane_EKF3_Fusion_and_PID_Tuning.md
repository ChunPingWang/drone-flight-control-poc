# ArduPlane Fixed-Wing EKF3 感測器融合架構與 PID 調參策略

> GPS + INS 航向與姿態控制技術參考文件 | February 2026

---

## 第一部分：EKF3 感測器融合架構

### 1.1 EKF3 總覽

ArduPilot 的狀態估算核心是 **EKF3（Extended Kalman Filter 3）**，實作於 AP_NavEKF3 函式庫中。EKF3 是 EKF2 的演進版本（EKF2 已於 ArduPilot 4.1+ 被標記為 deprecated），核心改進在於 **Sensor Affinity（感測器親和性）** 機制——EKF3 為每個 IMU 運行獨立的 **Lane（通道）**，並能自動評估各通道的健康度，動態切換到最健康的通道。

EKF3 是一個 **24 狀態的擴展卡爾曼濾波器**（每個 Lane 獨立運行），估算的狀態向量結構如下：

| 索引 | 狀態 | 維度 | 說明 |
|------|------|------|------|
| 0–3 | `quaternion` | 4 | 四元數姿態（roll, pitch, yaw） |
| 4–6 | `velocity` | 3 | NED 座標系下的速度（北、東、下） |
| 7–9 | `position` | 3 | NED 座標系下的位置 |
| 10–12 | `gyro_bias` | 3 | 陀螺儀偏差估計 |
| 13–15 | `accel_bias_z` | 1 + 2 reserved | Z 軸加速度計偏差估計（NE 軸受限） |
| 16–18 | `earth_mag` | 3 | 地球磁場向量（NED 座標系） |
| 19–21 | `body_mag` | 3 | 機體磁場偏差（body 座標系） |
| 22–23 | `wind_vel` | 2 | 風速估計（NE 方向，固定翼關鍵狀態） |

**與 PX4 EKF2 的差異：** 兩者的 24 狀態結構高度相似。主要差異在於 ArduPilot EKF3 對加速度計偏差的處理——EKF3 預設只估算 Z 軸加速度計偏差（`EK3_ACC_BIAS_LIM`），NE 軸透過 IMU 間互相比較而非 EKF 內部估計。此外，EKF3 的 **多 Lane 架構** 是根本性的設計差異。

**Sensor Affinity 機制（EKF3 獨有）：**

EKF3 為每個 IMU 運行一個獨立的 Lane（Pixhawk 6X 有 3 個 IMU = 3 個 Lane）。每個 Lane 可以被指定使用不同的感測器組合：

- `EK3_IMU_MASK`：指定哪些 IMU 運行 EKF Lane（位元遮罩，預設 7 = 全部三個）
- `EK3_GPS_TYPE`、`EK3_MAG_CAL`：每個 Lane 可獨立選擇 GPS 和磁力計融合模式
- ArduPilot 的 `AP_AHRS` 層持續評估各 Lane 的健康度（innovation 一致性、偏差穩定度），自動選擇最佳 Lane 作為主要輸出

**對固定翼的意義**：風速估計（states 22–23）是固定翼特有的關鍵狀態。有了風速估計，EKF3 才能從 GPS 地速推算空速、在 GPS 短暫丟失時靠慣性+風速模型維持導航，並補償氣壓計的靜壓位置誤差。

---

### 1.2 Multi-Lane 架構與 DCM Fallback

EKF3 的多通道架構是其核心設計哲學：

```
IMU 0 → EKF3 Lane 0 → ┐
IMU 1 → EKF3 Lane 1 → ├─→ AP_AHRS 選擇最佳 Lane → 輸出給控制迴路
IMU 2 → EKF3 Lane 2 → ┘
                          ↕
                      DCM Fallback（備援）
```

**Lane 選擇邏輯：**

AP_AHRS 根據以下指標選擇主要 Lane：
- Innovation test ratios（各感測器的 innovation 是否在預期範圍內）
- 陀螺儀偏差估計的穩定度
- 加速度計偏差估計的穩定度
- GPS 位置/速度融合的健康度

如果所有 EKF3 Lane 都失效（例如所有 IMU 資料異常），ArduPlane 會回退到 **DCM（Direction Cosine Matrix）**——一個輕量級的姿態估計器。DCM 不做位置/速度估計，但能維持基本的姿態（roll/pitch/yaw），讓飛機在緊急狀態下維持 Stabilized-like 的控制。

**DCM 在固定翼的特殊角色：** 當 EKF3 因 GPS 異常而拒絕融合 GPS 資料時，ArduPlane 可以利用 DCM 作為交叉校驗。這是 PX4 沒有的第二層安全網。

**啟用/停用 EKF3：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `AHRS_EKF_TYPE` | 3 | 3 = 使用 EKF3（推薦），2 = 使用 EKF2（deprecated） |
| `EK3_ENABLE` | 1 | 啟用 EKF3 |
| `EK3_IMU_MASK` | 7 | 使用哪些 IMU 的位元遮罩（7 = IMU 0+1+2 全部啟用） |

---

### 1.3 融合時間線與延遲補償

EKF3 處理感測器延遲的方式與 PX4 EKF2 類似，但實作細節不同：

```
感測器量測時間 ←── 延遲 ──→ EKF 融合時間點 ←── 狀態前推 ──→ 當前時間
                              ↑                              ↑
                         EKF 在此進行                    輸出給控制迴路
                         狀態更新                        的估算值
```

**延遲補償機制：**

EKF3 使用 **環形緩衝區（Ring Buffer）** 儲存 IMU 預測的歷史狀態。當延遲感測器（如 GPS）的量測到達時，EKF3：

1. 在環形緩衝區中找到對應該量測時間戳的歷史狀態
2. 計算 innovation（量測值 - 預測值）
3. 在歷史時間點計算 Kalman Gain 並更新狀態
4. 將修正量前推到當前時間

**關鍵差異（vs PX4 EKF2）：** PX4 EKF2 使用互補濾波器（Complementary Filter）將融合時間線的狀態傳播到當前時間。EKF3 則直接在環形緩衝區中修正歷史狀態，然後用 IMU delta 重新積分到當前時間。

**延遲參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EK3_GPS_DELAY` | 220ms | GPS 量測延遲（ArduPilot 預設較 PX4 保守） |
| `EK3_FLOW_DELAY` | 10ms | 光流感測器延遲 |
| `EK3_RNG_DELAY` | 50ms | 測距儀延遲 |
| `EK3_BCN_DELAY` | 50ms | Beacon 延遲 |

**注意：** ArduPilot 沒有 PX4 的 `EKF2_DELAY_MAX` 概念。每個感測器延遲獨立設定，環形緩衝區長度自動根據最大延遲調整。

---

### 1.4 IMU：預測（Prediction）階段

IMU 資料驅動 EKF3 的狀態預測，與 PX4 EKF2 的原理相同：

**預測方程（簡化表示）：**

```
姿態預測：q(k+1) = q(k) ⊗ Δq(ω - bias_gyro, dt)
速度預測：v(k+1) = v(k) + R(q) * (a - bias_accel) * dt + g * dt
位置預測：p(k+1) = p(k) + v(k) * dt
```

**EKF3 的 IMU 處理特點：**

- **IMU 間的互相校驗：** 多 Lane 架構讓 EKF3 可以比較不同 IMU 的預測結果。如果某個 IMU 的 delta angle/velocity 明顯偏離其他 IMU，該 Lane 的健康度評分會降低
- **加速度計偏差估計的保守處理：** EKF3 預設只估算 Z 軸加速度計偏差（`EK3_ABL_LIM` 控制限幅），水平軸偏差主要透過 IMU 間比較處理。這比 PX4 EKF2 的全三軸估計更保守但更穩定
- **陀螺儀 delta angle 限制：** `EK3_GYRO_P_NSE`（預設 0.015 rad/s）控制陀螺儀過程雜訊模型

**IMU 要求：** ArduPilot 預設以 400Hz 讀取 IMU（ChibiOS RTOS 排程），EKF3 以 IMU 頻率運行預測。最低要求 50Hz，建議 200Hz 以上。

---

### 1.5 GPS 融合：位置與速度觀測

GPS 提供絕對位置和速度，是 EKF3 在戶外飛行中最重要的觀測源。

**EKF3 的 GPS 融合模式（`EK3_GPS_TYPE`）：**

| 值 | 模式 | 說明 |
|----|------|------|
| 0 | GPS 3D velocity + 2D position + height | 標準模式（預設） |
| 1 | GPS 3D velocity + 2D position | 不使用 GPS 高度 |
| 2 | GPS 2D velocity + 2D position + height | 不使用垂直速度 |
| 3 | No GPS | 完全不使用 GPS |

**Innovation Gate 機制：**

EKF3 使用與 PX4 類似的 innovation gate，但參數命名不同：

```
Innovation = GPS_measurement - EKF_prediction
Test Ratio = innovation² / (innovation_variance * gate²)
如果 Test Ratio > 1.0 → 拒絕該量測
```

**GPS 融合的關鍵參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EK3_GPS_DELAY` | 220ms | GPS 量測延遲 |
| `EK3_VELNE_M_NSE` | 0.5 m/s | GPS 水平速度量測雜訊 |
| `EK3_VELD_M_NSE` | 0.7 m/s | GPS 垂直速度量測雜訊 |
| `EK3_POSNE_M_NSE` | 0.5 m | GPS 水平位置量測雜訊 |
| `EK3_GPS_V_GATE` | 300 | GPS 速度 innovation 閘門（百分比 × 100） |
| `EK3_GPS_P_GATE` | 300 | GPS 位置 innovation 閘門 |
| `EK3_GPS_CHECK` | 31 | GPS 品質檢查位元遮罩 |

**注意：ArduPilot 的 gate 值是 PX4 gate 值的百分比表示。** 例如 EK3_GPS_V_GATE = 300 等於 PX4 的 gate = 3.0。在 ArduPilot 中，500 對應 5 sigma，是合理的默認範圍。

**GPS Blending（多 GPS 混合）：**

ArduPilot 支援多 GPS 接收器的資料混合（`GPS_AUTO_SWITCH`）：

| 值 | 模式 | 說明 |
|----|------|------|
| 0 | 使用第一個 GPS | 不切換 |
| 1 | 自動切換到最佳 GPS | 基於 GPS 報告的精度 |
| 2 | GPS Blending | 加權混合多 GPS（推薦用於雙 GPS 配置） |
| 4 | GPS Blend + 使用第二 GPS 作為 yaw | 雙天線 GPS heading |

---

### 1.6 磁力計融合：航向觀測

ArduPilot EKF3 的磁力計融合比 PX4 更細緻，提供多種校準與融合模式。

**磁力計校準模式（`EK3_MAG_CAL`）：**

| 值 | 模式 | 說明 |
|----|------|------|
| 0 | When flying | 飛行中估計 mag bias（預設，推薦固定翼） |
| 1 | When manoeuvring | 只在機動時估計（適合大型慢速飛機） |
| 2 | Never | 不估計 mag bias（使用初始校準值） |
| 3 | After first climb | 第一次爬升後開始估計 |
| 4 | Always | 地面+飛行全程估計（適合室外固定翼） |
| 5 | External yaw | 使用外部 yaw 源（GPS heading、雙天線 GPS） |

**EKF3 的磁力計融合策略：**

EKF3 支援兩種磁力計融合方式（自動選擇）：
- **3D 磁場融合：** 估計地球磁場三軸分量 + 機體磁場偏差（6 狀態），適合磁場環境良好時
- **Magnetic heading 融合：** 只融合航向角，適合磁場干擾時

**EK3_MAG_MASK（ArduPlane 4.6+）：** 位元遮罩，控制哪些 EKF Lane 使用哪個磁力計。例如：
- Lane 0 → 外部磁力計（GPS 模組上的）
- Lane 1 → 內部磁力計（Pixhawk 板載）
- 如果外部磁力計被干擾，Lane 1 自動接管

**GSF Yaw 備份（與 PX4 類似）：**

EKF3 同樣運行 GSF（Gaussian Sum Filter）航向備份估計器：
- 多個小型 EKF 只用 IMU + GPS 速度估計 yaw
- 不依賴磁力計
- 當主 EKF 的磁力計航向失效時，用 GSF 重置 yaw
- 適合固定翼在飛行中（有足夠水平速度時），即使磁力計完全失效也能維持航向

**固定翼航向估計的特點：** 在高速飛行中，GPS 速度方向是非常好的航向參考。ArduPlane 會自動在高速時降低磁力計權重，增加 GPS 速度方向的貢獻。

**調參要點：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EK3_MAG_CAL` | 0 | 磁力計校準模式 |
| `EK3_MAG_M_NSE` | 0.05 Gauss | 磁力計量測雜訊 |
| `EK3_MAG_I_GATE` | 300 | 磁力計 innovation 閘門 |
| `EK3_YAW_M_NSE` | 0.5 rad | 航向雜訊 |
| `COMPASS_USE`, `COMPASS_USE2`, `COMPASS_USE3` | 1 | 啟用/停用各磁力計 |

---

### 1.7 氣壓計融合：高度觀測

氣壓計在 ArduPlane 中是高度估計的主要來源。

**靜壓位置誤差（與 PX4 相同的問題）：**

固定翼飛行中，機身周圍氣流改變氣壓計感受壓力，導致高度讀數偏移。ArduPilot 的補償方式：

1. **自動補償：** EKF3 結合風速估計和姿態資訊進行補償
2. **手動校正曲線（`ARSPD_PSTC_RATIO`、Plane 4.6+）：** 允許使用者定義空速→氣壓校正比例

**高度源選擇：**

ArduPilot 使用 `EK3_SRC1_POSZ` 控制高度源（Source Set 概念）：

| 值 | 高度源 | 說明 |
|----|--------|------|
| 1 | Barometer | 預設，穩定但有靜壓誤差 |
| 2 | Rangefinder | 近地面精確 |
| 3 | GPS | 較嘈雜但無靜壓誤差 |
| 4 | Beacon | 室內定位 |
| 6 | ExternalNav | 外部導航系統 |

**Source Set（ArduPilot 獨有功能）：**

ArduPilot 4.1+ 引入了 **三組 Source Set**（`EK3_SRC1_*`, `EK3_SRC2_*`, `EK3_SRC3_*`），允許在飛行中透過 RC 開關（`RC*_OPTION = 90/91`）切換不同的感測器源組合。例如：
- Source Set 1：GPS 位置 + Baro 高度（戶外正常飛行）
- Source Set 2：GPS 位置 + GPS 高度（需要絕對高度時）
- Source Set 3：ExternalNav 位置 + ExternalNav 高度（室內）

這是 PX4 目前沒有的功能彈性。

**氣壓計相關參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `EK3_ALT_M_NSE` | 2.0 m | 氣壓計高度量測雜訊 |
| `EK3_HGT_I_GATE` | 500 | 高度 innovation 閘門 |
| `EK3_SRC1_POSZ` | 1 | 主要高度源（1=Baro） |
| `EK3_SRC1_VELZ` | 0 | 垂直速度源（0=None，3=GPS） |
| `BARO_GND_TEMP` | 0 | 地面溫度校正 |

---

### 1.8 空速融合：固定翼的關鍵觀測

空速計對固定翼至關重要——它是唯一能直接量測相對空氣速度的感測器，也是 EKF3 估計風速的核心觀測。

**空速融合的觀測模型（與 PX4 相同原理）：**

```
TAS_predicted = |V_ned - V_wind|
             = |[vn - wn, ve - we, vd]|

其中：
V_ned = EKF 估計的 NED 速度（from GPS+IMU）
V_wind = EKF 估計的風速（states 22-23，假設只有水平風）
TAS = True Airspeed
```

EKF3 將空速計量測（IAS）轉換為 TAS 後與預測值比較，計算 innovation 用於更新風速和速度狀態。

**空速融合啟用條件：**

ArduPlane 的空速融合比 PX4 更自動化：
- 只要 `ARSPD_TYPE` ≠ 0 且 `ARSPD_USE` = 1，空速計就會被 EKF3 使用
- 不需要像 PX4 那樣手動設定 `EKF2_ARSP_THR` 閾值
- ArduPilot 內部根據空速計的健康狀態自動啟用/停用融合

**合成空速（Synthetic Airspeed）：**

ArduPlane 支援在沒有空速計時的替代方案：
- `EK3_DRAG_BCOEF_X/Y`：啟用阻力模型融合（用 GPS 速度 + 推力模型推算空速）
- `AHRS_WIND_MAX`：風速估計上限
- ArduPlane 也隱含地假設協調飛行（零側滑），但不需要像 PX4 那樣顯式設定 `EKF2_FUSE_BETA`

**空速計健康度監測（ArduPlane 特有）：**

ArduPlane 持續監測空速計的一致性：
- 如果空速計讀數與 GPS/IMU 推算的空速差異過大，`ARSPD_OPTIONS` 可設定自動停用空速計
- `ARSPD_AUTOCAL`：飛行中自動校準空速計的比例因子和偏移量

**空速相關參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `ARSPD_TYPE` | 0 | 空速計類型（0=None, 1=Analog, 2=MS4525DO...） |
| `ARSPD_USE` | 1 | 啟用空速計 |
| `ARSPD_RATIO` | 1.9936 | 空速計比例因子 |
| `ARSPD_AUTOCAL` | 0 | 自動校準空速計 |
| `EK3_EAS_M_NSE` | 1.4 m/s | 空速量測雜訊 |
| `EK3_EAS_I_GATE` | 400 | 空速 innovation 閘門 |
| `EK3_DRAG_BCOEF_X` | 0 | X 軸阻力模型係數（0=停用） |

---

### 1.9 GPS 丟失時的行為（Dead Reckoning）

GPS 丟失時，EKF3 的處理機制比 PX4 EKF2 多一層策略：

```
GPS 正常 ──→ GPS 丟失 ──→ EKF3 Dead Reckoning ──→ DCM Fallback
              ↑                ↑                        ↑
         innovation 檢測     wind model + IMU        所有 EKF Lane
         GPS drift 檢測      空速計輔助推算           都失敗時啟用
```

**Dead Reckoning 階段：**

1. EKF3 繼續用 IMU 積分預測位置/速度（會漂移）
2. 如果有空速計：空速 + 風速估計 + 姿態 → 推算地速，大幅減緩漂移
3. 磁力計/GSF 維持航向
4. `EK3_GPS_TOUT_MAX`（預設 5s）：GPS 丟失後，EKF 等待多久才進入 dead reckoning

**DCM Fallback 階段（ArduPlane 特有安全網）：**

當所有 EKF3 Lane 都判定位置/速度不可靠時：
- ArduPlane 自動切換到 DCM 姿態估計
- DCM 提供基本的 roll/pitch/yaw，但無位置/速度
- 飛行模式會被限制（例如無法執行 Mission，但 FBWA/Stabilize 仍可操作）
- 這讓飛手可以手動飛回

**有空速計的固定翼在 GPS 丟失時的優勢（與 PX4 相同原理）：**

空速量測 + 風速估計 + 姿態 → 推算地速向量。這顯著減緩 GPS 丟失時的位置漂移。空速計在固定翼上不只是防失速裝置，更是 GPS 丟失時的導航冗餘關鍵。

---

### 1.10 EKF3 融合的故障偵測與處理

**Innovation 一致性檢查：** 與 PX4 相同，每次量測更新計算 innovation test ratio。超過 gate 值的量測被拒絕。

**Sensor Affinity 的自動切換：**

這是 EKF3 相比 PX4 EKF2 最大的優勢之一。當特定感測器在某個 Lane 上持續產生異常 innovation：

1. 該 Lane 的健康度評分降低
2. AP_AHRS 切換到更健康的 Lane
3. 切換過程對飛行控制是透明的（控制器看到的估計值平滑過渡）

**Affinity 配置示例（Pixhawk 6X 三 IMU、雙磁力計、雙 GPS）：**

```
Lane 0 (IMU 0): GPS 1 + 外部 Mag → 預設主要
Lane 1 (IMU 1): GPS 1 + 內部 Mag → 磁干擾備援
Lane 2 (IMU 2): GPS 2 + 外部 Mag → GPS 故障備援
```

**GPS Glitch 偵測：**

ArduPilot 在 EKF 之外還有一層 GPS glitch 偵測（在 AP_GPS 層）：
- 計算 GPS 位置跳變速度是否超過物理可能的加速度
- 持續的 GPS glitch 會觸發 `GPS_GLITCH` failsafe

**日誌分析：**

ArduPilot 飛行後分析工具：
- **MAVExplorer**：內建於 Mission Planner，即時查看 EKF 狀態
- **Mission Planner > Flight Data > Log Analysis**：自動化 EKF 健康度檢查
- **UAV Log Viewer**（https://plot.ardupilot.org）：Web-based 日誌可視化
- **MAVProxy + MAVExplorer**：命令列分析工具

關鍵日誌 message：
- `XKF1`：EKF3 姿態/速度/位置輸出
- `XKF2`：Innovation 值
- `XKF3`：Innovation variances
- `XKF4`：EKF 狀態（fusion mode, GPS status）
- `NKF0`–`NKF5`：詳細 EKF 內部狀態（每個 Lane）

---

### 1.11 EKF3 固定翼完整參數速查

以下列出固定翼場景中最重要的 EKF3 參數：

| 參數 | 預設值 | 建議調整範圍 | 說明 |
|------|--------|-------------|------|
| **架構設定** | | | |
| `AHRS_EKF_TYPE` | 3 | 3 | 使用 EKF3 |
| `EK3_ENABLE` | 1 | 1 | 啟用 EKF3 |
| `EK3_IMU_MASK` | 7 | 3/7 | IMU 選擇位元遮罩 |
| **延遲補償** | | | |
| `EK3_GPS_DELAY` | 220ms | 100–300 | GPS 量測延遲 |
| **GPS 融合** | | | |
| `EK3_GPS_TYPE` | 0 | 0/1 | GPS 融合模式 |
| `EK3_VELNE_M_NSE` | 0.5 m/s | 0.3–1.0 | GPS 水平速度雜訊 |
| `EK3_VELD_M_NSE` | 0.7 m/s | 0.3–1.0 | GPS 垂直速度雜訊 |
| `EK3_POSNE_M_NSE` | 0.5 m | 0.3–2.0 | GPS 水平位置雜訊 |
| `EK3_GPS_V_GATE` | 300 | 200–500 | GPS 速度 innovation 閘門 |
| `EK3_GPS_P_GATE` | 300 | 200–500 | GPS 位置 innovation 閘門 |
| **磁力計** | | | |
| `EK3_MAG_CAL` | 0 | 0/3/5 | 磁力計校準/融合模式 |
| `EK3_MAG_M_NSE` | 0.05 Gauss | 0.02–0.1 | 磁力計量測雜訊 |
| `EK3_MAG_I_GATE` | 300 | 200–500 | 磁力計 innovation 閘門 |
| **氣壓計** | | | |
| `EK3_ALT_M_NSE` | 2.0 m | 1.0–5.0 | 氣壓計高度雜訊 |
| `EK3_HGT_I_GATE` | 500 | 300–500 | 高度 innovation 閘門 |
| `EK3_SRC1_POSZ` | 1 | 1/3 | 高度源（1=Baro, 3=GPS） |
| **空速 / 風速（固定翼）** | | | |
| `EK3_EAS_M_NSE` | 1.4 m/s | 0.5–2.0 | 空速量測雜訊 |
| `EK3_EAS_I_GATE` | 400 | 300–500 | 空速 innovation 閘門 |
| `EK3_DRAG_BCOEF_X` | 0 | 25–200 | 阻力模型係數（0=停用） |
| **過程雜訊** | | | |
| `EK3_GYRO_P_NSE` | 0.015 rad/s | 0.01–0.03 | 陀螺儀過程雜訊 |
| `EK3_ACC_P_NSE` | 0.25 m/s² | 0.1–0.5 | 加速度計過程雜訊 |
| `EK3_WIND_P_NSE` | 0.1 m/s | 0.05–0.3 | 風速過程雜訊 |

---

## 第二部分：固定翼 PID 調參策略

### 2.1 控制架構回顧

ArduPlane 的控制迴路同樣是三層串級結構，但命名和實作與 PX4 不同：

```
               ┌──────────────────────────────────────────────────────────┐
               │                     導航層                               │
               │  L1 Navigation Controller → 目標 roll/pitch 角度         │
               │  (NAVL1_PERIOD, NAVL1_DAMPING)                          │
               │  TECS → 油門 + 俯仰協調                                 │
               └────────────────────────┬─────────────────────────────────┘
                                        ↓
               ┌──────────────────────────────────────────────────────────┐
               │                     姿態層                               │
               │  Attitude Controller → 目標角速率                        │
               │  (P 控制器，時間常數隱含在增益比例中)                      │
               └────────────────────────┬─────────────────────────────────┘
                                        ↓
               ┌──────────────────────────────────────────────────────────┐
               │                     角速率層                              │
               │  Rate Controller → 舵面偏轉                              │
               │  (PID，RLL_RATE_*, PTCH_RATE_*, YAW_RATE_*)             │
               └──────────────────────────────────────────────────────────┘
```

**ArduPlane vs PX4 控制器差異：**

| 面向 | ArduPlane | PX4 |
|------|-----------|-----|
| Rate Controller | PID（P + I + D + FF） | FF + PID |
| 姿態層 | 隱含在 Rate 增益比例中 | 顯式 P 控制器（`FW_R_TC`, `FW_P_TC`） |
| 導航層 | L1（`NAVL1_*`）+ TECS | L1（`FW_L1_*`）+ TECS |
| FF 角色 | FF 是 Rate PID 的一部分 | FF 是主力，PID 修正 |
| Auto-Tune | AUTOTUNE 飛行模式 | QGC 內啟用 |

**調參順序同樣重要：永遠從最內層（Rate）開始，往外調。**

---

### 2.2 前置條件：Trim（配平）與 Servo Setup

**Servo Setup（舵面設定）：**

在 PID 調參之前，ArduPlane 需要先確認舵面設定正確：

1. **`SERVOx_FUNCTION`**：指定每個伺服的功能（4=Aileron, 19=Elevator, 21=Rudder...）
2. **`SERVOx_REVERSED`**：確認舵面方向正確（這是新手最常犯的錯誤之一）
3. **`SERVOx_MIN/MAX/TRIM`**：設定舵面行程和中位

**Trim 調整方法：**

ArduPlane 的 Trim 有兩層：
1. **伺服 Trim（`SERVOx_TRIM`）：** 物理中位，確保舵面在中性位置
2. **飛行 Trim（`TRIM_THROTTLE`, `TRIM_ARSPD_CM`, `LIM_PITCH_MIN/MAX`）：** 飛行姿態中性

**手動 Trim 流程：**
1. 在 MANUAL 模式飛行，找到穩定巡航狀態
2. 用遙控器 Trim 開關微調到穩定直飛
3. 在 Mission Planner 中按 **「Save Trim」** 按鈕（或使用 `RC*_OPTION = 150` Trim 儲存開關）
4. 確認 `SERVOx_TRIM` 值已更新

---

### 2.3 Rate Controller（角速率控制器）—— 最內層

ArduPlane 的 Rate Controller 使用標準的 PID + FF 結構：

**控制器方程：**

```
舵面輸出 = FF * rate_target 
         + P * rate_error 
         + I * ∫rate_error * dt 
         + D * d(rate_filtered)/dt
```

**各增益的角色：**

| 增益 | 代號（Roll） | 代號（Pitch） | 代號（Yaw） | 作用 |
|------|-------------|--------------|------------|------|
| Feedforward | `RLL_RATE_FF` | `PTCH_RATE_FF` | `YAW_RATE_FF` | 前饋：主要控制力 |
| Proportional | `RLL_RATE_P` | `PTCH_RATE_P` | `YAW_RATE_P` | 比例：即時誤差修正 |
| Integral | `RLL_RATE_I` | `PTCH_RATE_I` | `YAW_RATE_I` | 積分：穩態誤差消除 |
| Derivative | `RLL_RATE_D` | `PTCH_RATE_D` | `YAW_RATE_D` | 微分：阻尼高頻震盪 |
| I Max | `RLL_RATE_IMAX` | `PTCH_RATE_IMAX` | `YAW_RATE_IMAX` | 積分限幅 |
| Filter | `RLL_RATE_FLTT` | `PTCH_RATE_FLTT` | `YAW_RATE_FLTT` | D 項低通濾波截止頻率 |
| Target Filter | `RLL_RATE_FLTE` | `PTCH_RATE_FLTE` | `YAW_RATE_FLTE` | Error 低通濾波截止頻率 |
| Slew Rate | `RLL_RATE_SMAX` | `PTCH_RATE_SMAX` | `YAW_RATE_SMAX` | 最大 slew rate（防 servo buzz） |

**ArduPlane vs PX4 增益命名對照：**

| ArduPlane | PX4 | 說明 |
|-----------|-----|------|
| `RLL_RATE_FF` | `FW_RR_FF` | Roll rate feedforward |
| `RLL_RATE_P` | `FW_RR_P` | Roll rate proportional |
| `RLL_RATE_I` | `FW_RR_I` | Roll rate integral |
| `RLL_RATE_D` | （無直接對應） | Roll rate derivative |
| `PTCH_RATE_FF` | `FW_PR_FF` | Pitch rate feedforward |
| `PTCH_RATE_P` | `FW_PR_P` | Pitch rate proportional |

**Roll Rate 調參步驟：**

1. **先調 FF（`RLL_RATE_FF`）**：
   - 起始值：0.3（ArduPlane 建議比 PX4 稍低起步）
   - 在 FBWA（Fly By Wire A）模式下飛行，給 roll 指令
   - 逐步增加（0.3 → 0.5 → 0.8...），直到飛機能跟上 roll 指令但不過衝
   - 如果出現過衝但接近正確範圍，降低 10–20%

2. **再調 P（`RLL_RATE_P`）**：
   - 起始值：0.04
   - 逐步增加（0.04 → 0.08 → 0.12...），直到快速抖動出現
   - 出現抖動後，減半

3. **調 D（`RLL_RATE_D`）**：
   - 起始值：0.0（ArduPlane D 項通常從零開始）
   - 小幅增加（0.0 → 0.002 → 0.005），觀察是否改善阻尼
   - D 增益過高會放大感測器雜訊 → 高頻抖動
   - 如果不確定，保持 0.0 也可以

4. **最後調 I（`RLL_RATE_I`）**：
   - **ArduPlane 慣例：I 通常設為與 FF 相同的值**
   - 例如 `RLL_RATE_FF = 0.5`，則 `RLL_RATE_I = 0.5`
   - 這確保穩態時積分項能完全補償所需的舵面偏轉
   - 如果出現低頻振盪，降低 I 值

**Pitch Rate 調參步驟（`PTCH_RATE_FF/P/I/D`）：**

與 Roll 相同流程，但注意：
- Pitch 對 Trim 更敏感，確保 `TRIM_PITCH_CD` 正確
- Pitch 的 FF 值通常比 Roll 高（因為升降舵需要更大力矩）
- I 增益設為 FF 值，與 Roll 相同慣例

**Yaw Rate（`YAW_RATE_*`）：**

固定翼 Yaw 主要靠 Roll 控制轉彎，Rudder 用於：
- `YAW_DAMPER_ENABLE`：Dutch roll 阻尼器（推薦啟用）
- `KFF_RDDRMIX`：Roll-to-Yaw 混控（coordinated turn）
- 地面方向控制（`GROUND_STEER_ALT` 以下啟用）

---

### 2.4 Attitude Controller（姿態控制器）—— 中間層

ArduPlane 的姿態層不像 PX4 有獨立的 `FW_R_TC`/`FW_P_TC` 時間常數。姿態響應速度主要由以下參數控制：

**Roll 姿態控制：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `RLL2SRV_TCONST` | 0.5s | Roll 時間常數（與 PX4 的 `FW_R_TC` 直接對應） |
| `RLL2SRV_RMAX` | 0 deg/s | 最大 roll rate（0=自動計算） |

**Pitch 姿態控制：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `PTCH2SRV_TCONST` | 0.5s | Pitch 時間常數（與 PX4 的 `FW_P_TC` 直接對應） |
| `PTCH2SRV_RMAX_UP` | 0 deg/s | 最大上仰 pitch rate |
| `PTCH2SRV_RMAX_DN` | 0 deg/s | 最大下俯 pitch rate |

**時間常數的物理意義與 PX4 相同：** `RLL2SRV_TCONST = 0.5s` 表示 10° roll 誤差 → 目標 20°/s roll rate。

**調參要點：**
- 預設 0.5s 適合大多數機體
- 大型慢速飛機：增大到 0.6–1.0s
- 小型靈活飛機：減小到 0.3–0.4s
- 時間常數太小 + Rate 增益不足 = 慢速振盪

---

### 2.5 振盪診斷

| 現象 | 頻率 | 原因 | 解決方案 |
|------|------|------|----------|
| 快速抖動 / servo buzz | >2 Hz | Rate P/D 增益太高 | 減小 `RLL_RATE_P/D`, `PTCH_RATE_P/D` |
| 中頻振盪 | 1–2 Hz | Rate P 太高或 D 項濾波不當 | 減小 P，降低 `*_RATE_FLTT` 截止頻率 |
| 慢速擺盪 | <1 Hz | 姿態層太快，Rate 跟不上 | 增大 `RLL2SRV_TCONST`, `PTCH2SRV_TCONST` |
| 穩態偏差 | 不振盪 | I 增益不足或 Trim 不對 | 增大 I（=FF），或重新 Save Trim |
| 降落時擺盪 | 低空速 | 低速舵面效率差，增益相對過高 | 降低 `*_RATE_SMAX`，啟用 airspeed scaling |
| Servo buzz（高頻嗡聲） | >5 Hz | D 增益太高或 SMAX 太高 | 設定 `*_RATE_SMAX = 150`，降低 D |

---

### 2.6 AUTOTUNE（自動調參飛行模式）

ArduPlane 有專門的 **AUTOTUNE 飛行模式**（與 PX4 的 QGC 內啟用方式不同），是 ArduPlane 最強大的調參工具之一。

**使用流程：**

1. **前提條件：**
   - 飛機在 FBWA 模式下能穩定飛行
   - Trim 已正確設定
   - 天氣良好（低風）
   - 飛行高度足夠（建議 >50m AGL）

2. **啟動 AUTOTUNE：**
   - 設定飛行模式開關的某個位置為 AUTOTUNE（`FLTMODEx = 15`）
   - 飛行中切到 AUTOTUNE 模式
   - ArduPlane 會自動執行以下動作：
     a. 給 roll 方向一個步進輸入
     b. 觀測飛機響應
     c. 調整 Roll Rate PID 增益
     d. 重複直到收斂
     e. 對 Pitch 重複上述流程

3. **AUTOTUNE 期間飛手要做的事：**
   - 用遙控器維持高度和方向（AUTOTUNE 只控制姿態軸）
   - 如果飛機行為異常，切回 FBWA
   - AUTOTUNE 會用 roll stick 輸入作為觸發——飛手給滿桿 roll，鬆開，AUTOTUNE 觀測回復動態

4. **完成：**
   - AUTOTUNE 收斂後（Mission Planner 會顯示訊息）
   - 在 AUTOTUNE 模式下 **降落** → 參數自動儲存
   - 如果切回其他模式降落 → 參數不儲存（方便測試比較）

**AUTOTUNE 調的參數：**

AUTOTUNE 會自動設定以下參數：
- `RLL_RATE_FF`, `RLL_RATE_P`, `RLL_RATE_I`, `RLL_RATE_D`
- `PTCH_RATE_FF`, `PTCH_RATE_P`, `PTCH_RATE_I`, `PTCH_RATE_D`
- `RLL2SRV_TCONST`, `PTCH2SRV_TCONST`

**AUTOTUNE 控制參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `AUTOTUNE_LEVEL` | 6 | 調參激進度（1-10，6=中等，10=最激進） |
| `AUTOTUNE_OPTIONS` | 0 | 位元遮罩控制行為 |

**AUTOTUNE 的限制：**
- 不調 Yaw/Rudder（需手動）
- 不調 L1 導航和 TECS
- 風大時結果不可靠
- 需要足夠飛行空間讓飛機做大幅度機動

**建議策略：** 先用 AUTOTUNE 取得基線，然後根據飛行日誌手動微調。AUTOTUNE Level 6 通常給出保守但穩定的結果；如果覺得響應太慢，Level 7–8 可以更激進。

---

### 2.7 導航層：L1 Controller（外層）

ArduPlane 使用 L1 Navigation Controller 將航點轉換為目標 roll 角。ArduPilot 4.2+ 還引入了更新的 **S-Curve Navigation**，但 L1 仍是主要導航器。

**L1 核心參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `NAVL1_PERIOD` | 20s | L1 導航週期（等同 PX4 的 `FW_L1_PERIOD`） |
| `NAVL1_DAMPING` | 0.75 | L1 阻尼 |
| `WP_LOITER_RAD` | 60m | 盤旋半徑 |
| `WP_RADIUS` | 90m | 航點到達半徑 |

**調參要點（與 PX4 相同原則）：**
- `NAVL1_PERIOD` 過小 → 蛇行
- `NAVL1_PERIOD` 過大 → 轉彎半徑大
- 典型範圍：12–25s
- 先確保 Rate + Attitude 層穩定後再調 L1

---

### 2.8 空速控制器：TECS

ArduPlane 的 TECS（Total Energy Control System）原理與 PX4 相同——將高度和空速作為能量管理問題統一處理：

```
總能量 = 動能（空速²）+ 位能（高度）
油門 → 改變總能量
俯仰角 → 在動能和位能之間分配
```

**關鍵參數：**

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `TRIM_THROTTLE` | 45% | 巡航油門 |
| `THR_MIN` | 0% | 最小油門 |
| `THR_MAX` | 75% | 最大油門 |
| `TRIM_ARSPD_CM` | 1500 (cm/s = 15 m/s) | 巡航空速目標 |
| `ARSPD_FBW_MIN` | 9 m/s | FBWA/FBWB 最低空速 |
| `ARSPD_FBW_MAX` | 22 m/s | FBWA/FBWB 最高空速 |
| `TECS_CLMB_MAX` | 5 m/s | 最大爬升率 |
| `TECS_SINK_MIN` | 2 m/s | 最小下降率 |
| `TECS_SINK_MAX` | 5 m/s | 最大下降率 |
| `TECS_TIME_CONST` | 5s | TECS 時間常數（響應速度） |
| `TECS_THR_DAMP` | 0.5 | 油門阻尼 |
| `TECS_INTEG_GAIN` | 0.1 | TECS 積分增益 |
| `TECS_SPDWEIGHT` | 1.0 | 速度/高度權重（1.0=平衡，2.0=優先速度） |

**TECS 調參要點：**

1. `TRIM_THROTTLE` 是起點——在水平巡航時量測平均油門值
2. `TECS_CLMB_MAX` 和 `TECS_SINK_MIN` 需要實際飛行數據來設定
3. `TECS_SPDWEIGHT`：對固定翼安全性極為重要
   - 1.0 = 平衡高度和速度控制
   - 2.0 = 優先保持空速（推薦用於容易失速的小型飛機）
   - 0.0 = 優先保持高度

---

### 2.9 ArduPlane 特有功能

以下是 ArduPlane 提供而 PX4 沒有（或不完善）的固定翼調參功能：

**2.9.1 FBWA / FBWB 飛行模式**

| 模式 | 說明 | 用途 |
|------|------|------|
| FBWA | Fly By Wire A：遙控器控制 roll/pitch 角度，ArduPlane 穩定 | PID 調參的首選模式 |
| FBWB | Fly By Wire B：遙控器控制空速和爬升率 | TECS 調參的首選模式 |

PX4 的 Stabilized 模式大致對應 FBWA，但 ArduPlane 的 FBWA 在舵面偏轉上限（`LIM_ROLL_CD`、`LIM_PITCH_MIN/MAX`）和空速保護上更完善。

**2.9.2 Airspeed Scaling**

ArduPlane 會根據空速自動縮放 PID 增益：

```
effective_gain = base_gain * (SCALING_SPEED / current_airspeed)²
```

`SCALING_SPEED`（預設 15 m/s）是增益設計的參考空速。低速時增益自動增大（補償低舵面效率），高速時減小（避免過靈敏）。

PX4 也有類似機制，但 ArduPlane 的實作更成熟且參數化更完整。

**2.9.3 Landing 自動降落**

ArduPlane 的自動降落（`DO_LAND_START` + `LAND`）比 PX4 更成熟：
- Approach 階段：空速控制 + 下滑角
- Flare 階段（`LAND_FLARE_SEC`）：拉平進入地效
- Touchdown：自動收油門、保持方向

**2.9.4 Hand Launch Detection**

ArduPlane 支援手擲起飛偵測（`TKOFF_THR_DELAY`）：
- 飛機偵測到加速度超過閾值後才啟動馬達
- 手擲時不需要預先給油門

---

### 2.10 完整調參流程總結

以下是 ArduPlane 固定翼從零到可靠自主飛行的調參順序：

**Phase 1：地面準備**
1. 選擇最接近的 Frame Type（`FRAME_CLASS`）
2. 感測器校準：IMU（Level）、磁力計（Compass）、空速計（ARSPD_OFFSET）
3. **確認舵面方向正確**（Manual 模式搖桿測試）
4. 設定 `SERVOx_MIN/MAX/TRIM`

**Phase 2：手動飛行 + Trim**
5. MANUAL 模式試飛，確認舵面效率和方向
6. 調整 Trim 直到穩定直飛
7. 使用 **Save Trim** 功能儲存
8. 切 FBWA 模式，確認自動改平正常

**Phase 3：Rate Controller 調參**
9. 先調 Roll Rate（FF → P → D → I）
10. 再調 Pitch Rate（FF → P → D → I）
11. 確認 Yaw Damper（`YAW_DAMPER_ENABLE`）
12. **或直接使用 AUTOTUNE 飛行模式**

**Phase 4：Attitude Controller**
13. 調整 `RLL2SRV_TCONST` 和 `PTCH2SRV_TCONST`（通常預設即可）
14. 確認無快/慢速振盪

**Phase 5：FBWB + TECS**
15. 切 FBWB 模式，觀察高度和空速保持
16. 調整 `TRIM_THROTTLE`（巡航油門）
17. 調整 `TECS_TIME_CONST`（響應速度）
18. 設定 `TECS_SPDWEIGHT`（速度/高度優先度）
19. 設定空速範圍（`ARSPD_FBW_MIN/MAX`）

**Phase 6：AUTO + Mission 測試**
20. 調整 L1（`NAVL1_PERIOD`, `NAVL1_DAMPING`）
21. 規劃簡單航點任務（RTL → 簡單 Waypoints → Loiter）
22. 分析飛行日誌，確認 EKF3 innovation 正常
23. 測試 RTL（Return to Launch）
24. 測試自動降落（`DO_LAND_START`）
25. 逐步增加任務複雜度

---

### 2.11 飛行日誌分析重點

每次飛行後應檢查以下指標：

**EKF3 健康度：**
- Innovation test ratios（日誌中 `NKF4` 的 `SV`, `SP`, `SH` 欄位）：應持續 < 1.0
- GPS fusion status（`NKF4.SS`）：確認持續融合中
- Lane switch events（`EKF3.primary`）：頻繁切換表示感測器問題
- Sensor bias 估計值穩定（`NKF1` 中的 gyro/accel bias）

**控制器性能：**
- `ATT` 日誌：DesRoll vs Roll, DesPitch vs Pitch（命令 vs 實際）
- `RATE` 日誌：RDes vs R, PDes vs P（角速率命令 vs 實際）
- 舵面輸出飽和（`RCOU` 日誌中 servo 輸出是否持續在極限值）
- Vibration levels（`VIBE` 日誌）：X/Y/Z 加速度震動

**工具：**
- **Mission Planner > Log Analysis**：自動化飛行品質檢查
- **UAV Log Viewer**（https://plot.ardupilot.org）：Web-based，支援拖放 .bin/.log
- **MAVExplorer**：Mission Planner 內建，支援圖表和 FFT 分析
- **MAVProxy**：命令列工具，支援腳本化分析

---

## 附錄：固定翼參數速查表

### A. EKF3 核心參數

| 參數 | 預設 | 說明 |
|------|------|------|
| `AHRS_EKF_TYPE` | 3 | EKF 類型（3=EKF3） |
| `EK3_GPS_DELAY` | 220ms | GPS 延遲 |
| `EK3_VELNE_M_NSE` | 0.5 | GPS 水平速度雜訊 (m/s) |
| `EK3_POSNE_M_NSE` | 0.5 | GPS 水平位置雜訊 (m) |
| `EK3_MAG_CAL` | 0 | 磁力計校準模式 |
| `EK3_MAG_M_NSE` | 0.05 | 磁力計雜訊 (Gauss) |
| `EK3_ALT_M_NSE` | 2.0 | 氣壓計高度雜訊 (m) |
| `EK3_EAS_M_NSE` | 1.4 | 空速量測雜訊 (m/s) |
| `EK3_WIND_P_NSE` | 0.1 | 風速過程雜訊 (m/s) |
| `EK3_SRC1_POSZ` | 1 | 高度源 (1=Baro) |

### B. Rate Controller 參數

| 參數 | 預設 | 起始調參值 | 說明 |
|------|------|-----------|------|
| `RLL_RATE_FF` | 0.3 | 0.3 | Roll rate feedforward |
| `RLL_RATE_P` | 0.04 | 0.04 | Roll rate proportional |
| `RLL_RATE_I` | 0.3 | = FF | Roll rate integral |
| `RLL_RATE_D` | 0.0 | 0.0 | Roll rate derivative |
| `PTCH_RATE_FF` | 0.3 | 0.4 | Pitch rate feedforward |
| `PTCH_RATE_P` | 0.04 | 0.04 | Pitch rate proportional |
| `PTCH_RATE_I` | 0.3 | = FF | Pitch rate integral |
| `PTCH_RATE_D` | 0.0 | 0.0 | Pitch rate derivative |
| `YAW_RATE_FF` | 0.15 | 0.15 | Yaw rate feedforward |
| `YAW_RATE_P` | 0.02 | 0.02 | Yaw rate proportional |
| `YAW_RATE_I` | 0.15 | = FF | Yaw rate integral |

### C. Attitude Controller 參數

| 參數 | 預設 | 說明 |
|------|------|------|
| `RLL2SRV_TCONST` | 0.5s | Roll 時間常數 |
| `PTCH2SRV_TCONST` | 0.5s | Pitch 時間常數 |
| `RLL2SRV_RMAX` | 0 deg/s | 最大 roll rate (0=自動) |
| `PTCH2SRV_RMAX_UP` | 0 deg/s | 最大上仰 rate |
| `PTCH2SRV_RMAX_DN` | 0 deg/s | 最大下俯 rate |
| `LIM_ROLL_CD` | 4500 (45°) | 最大 roll 角度 (centidegrees) |
| `LIM_PITCH_MIN` | -25° | 最小 pitch 角度 |
| `LIM_PITCH_MAX` | 20° | 最大 pitch 角度 |

### D. Navigation + TECS 參數

| 參數 | 預設 | 說明 |
|------|------|------|
| `NAVL1_PERIOD` | 20s | L1 導航週期 |
| `NAVL1_DAMPING` | 0.75 | L1 阻尼 |
| `TRIM_THROTTLE` | 45% | 巡航油門 |
| `TRIM_ARSPD_CM` | 1500 | 巡航空速 (cm/s) |
| `ARSPD_FBW_MIN` | 9 m/s | 最低空速 |
| `ARSPD_FBW_MAX` | 22 m/s | 最高空速 |
| `TECS_CLMB_MAX` | 5 m/s | 最大爬升率 |
| `TECS_SINK_MIN` | 2 m/s | 最小下降率 |
| `TECS_TIME_CONST` | 5s | TECS 時間常數 |
| `TECS_SPDWEIGHT` | 1.0 | 速度/高度權重 |

### E. Failsafe 參數（固定翼重要安全設定）

| 參數 | 預設 | 說明 |
|------|------|------|
| `FS_SHORT_ACTN` | 0 | RC 短暫丟失動作（0=continue, 1=RTL） |
| `FS_LONG_ACTN` | 0 | RC 長時間丟失動作 |
| `FS_SHORT_TIMEOUT` | 1.5s | 短暫丟失超時 |
| `FS_LONG_TIMEOUT` | 5s | 長時間丟失超時 |
| `THR_FAILSAFE` | 1 | 油門 failsafe 啟用 |
| `FS_GCS_ENABL` | 0 | GCS 連線丟失 failsafe |
| `BATT_FS_LOW_ACT` | 0 | 低電壓 failsafe 動作 |

---

## 附錄 F：ArduPlane 飛行模式速查

| 模式 | 說明 | 用途 |
|------|------|------|
| MANUAL | 直接 RC 控制 | 初始測試、Trim 設定 |
| STABILIZE | 自動改平，手動油門 | 基本穩定飛行 |
| FBWA | Fly By Wire A：角度控制 | **PID 調參首選** |
| FBWB | Fly By Wire B：空速/爬升率控制 | **TECS 調參首選** |
| AUTOTUNE | 自動 PID 調參 | **自動調參** |
| CRUISE | 混合 FBWB + heading hold | 長距離巡航 |
| AUTO | 執行 Mission | 自主任務 |
| RTL | Return to Launch | 安全返航 |
| LOITER | 定點盤旋 | 等待/觀察 |
| GUIDED | GCS/Companion 指定目標 | AI 整合首選 |
| QSTABILIZE/QHOVER/QLOITER | QuadPlane VTOL 模式 | VTOL 配置 |

---

> **資料來源：** ArduPilot Official Documentation (ardupilot.org/plane), AP_NavEKF3 Source Code (github.com/ArduPilot/ardupilot), ArduPilot Developer Wiki, ArduPlane 4.6 Release Notes
