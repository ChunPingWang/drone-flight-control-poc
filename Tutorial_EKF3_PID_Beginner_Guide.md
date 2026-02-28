# ArduPlane EKF3 感測器融合與 PID 調參 — 初學者教學指南

> 透過純軟體模擬理解固定翼飛控核心概念 | 無需硬體即可學習

---

## 目錄

1. [前言：為什麼要學這些？](#1-前言為什麼要學這些)
2. [環境準備](#2-環境準備)
3. [第一課：什麼是 EKF？](#3-第一課什麼是-ekf)
4. [第二課：感測器融合——多個感測器如何協同工作](#4-第二課感測器融合多個感測器如何協同工作)
5. [第三課：Multi-Lane 架構——EKF3 的安全網](#5-第三課multi-lane-架構ekf3-的安全網)
6. [第四課：PID 控制器——讓飛機聽話](#6-第四課pid-控制器讓飛機聽話)
7. [第五課：TECS 與 L1 導航——自主飛行的大腦](#7-第五課tecs-與-l1-導航自主飛行的大腦)
8. [第六課：動手實驗——執行模擬程式](#8-第六課動手實驗執行模擬程式)
9. [第七課：理解模擬結果](#9-第七課理解模擬結果)
10. [附錄：名詞解釋](#10-附錄名詞解釋)

---

## 1. 前言：為什麼要學這些？

想像你在操控一架固定翼無人機。飛機在空中以 15 m/s 的速度飛行，遇到風、GPS 信號不穩、感測器雜訊等各種干擾。飛控系統需要在這些不完美的條件下，精確知道飛機在哪裡（**狀態估算**）、讓飛機穩定飛行（**PID 控制**），以及按照規劃的航線飛行（**導航**）。

這份教學將帶你理解三個核心問題：

| 問題 | 技術 | 對應模組 |
|------|------|----------|
| 飛機現在在哪裡？姿態如何？ | EKF3 感測器融合 | AP_NavEKF3 |
| 如何讓飛機保持穩定？ | PID 控制器 | Rate / Attitude Controller |
| 如何讓飛機按航線飛？ | L1 導航 + TECS | Navigation Layer |

**本教學的特色：** 所有概念都附帶 Python 模擬程式，你可以在自己的電腦上執行，透過圖表直觀理解這些概念，完全不需要真實硬體。

---

## 2. 環境準備

### 2.1 系統需求

- Python 3.7 以上
- 以下 Python 套件：
  - `numpy`（數值計算）
  - `matplotlib`（繪圖）
  - `scipy`（科學計算）

### 2.2 安裝步驟

```bash
# 1. 確認 Python 版本
python3 --version

# 2. 安裝必要套件
pip3 install numpy matplotlib scipy

# 3. 克隆本專案
git clone https://github.com/ChunPingWang/drone-flight-control-poc.git
cd drone-flight-control-poc

# 4. 執行所有模擬（一鍵驗證）
python3 simulations/run_all.py
```

### 2.3 專案結構

```
drone-flight-control-poc/
├── ArduPlane_EKF3_Fusion_and_PID_Tuning.md  # 技術參考文件（進階）
├── Tutorial_EKF3_PID_Beginner_Guide.md      # 本教學文件（入門）
└── simulations/
    ├── run_all.py                           # 一鍵執行所有模擬
    ├── sim1_ekf_state_estimation.py         # 模擬 1: EKF 狀態估算
    ├── sim2_pid_tuning.py                   # 模擬 2: PID 調參
    ├── sim3_sensor_fusion.py                # 模擬 3: 多感測器融合
    ├── sim4_tecs_navigation.py              # 模擬 4: TECS 與導航
    └── output_*.png                         # 模擬產生的圖表
```

---

## 3. 第一課：什麼是 EKF？

### 3.1 日常比喻

想像你閉著眼睛走路：

1. **你的「內部預測」**：根據自己的步幅和方向，你大概知道走了多遠（類似 **IMU** 的角色）
2. **偶爾睜開眼睛**：你看到周圍環境，修正自己的位置估計（類似 **GPS** 的角色）
3. **結合兩者**：你的大腦自動融合「步行預測」和「視覺觀測」得到最佳位置估計

**EKF（Extended Kalman Filter，擴展卡爾曼濾波器）** 就是飛控系統的「大腦」，做的事情和你走路時的大腦一模一樣。

### 3.2 EKF 的兩個步驟

EKF 不斷重複兩個步驟：

```
┌──────────────┐       ┌──────────────┐
│   預測步驟    │ ────→ │   更新步驟    │
│  (IMU 驅動)  │       │ (GPS 修正)    │
│              │       │              │
│ 用加速度計和  │       │ 用 GPS 位置   │
│ 陀螺儀推算    │       │ 修正預測誤差   │
│ 下一刻的狀態  │       │              │
└──────────────┘       └──────────────┘
        ↑                      │
        └──────────────────────┘
              不斷循環
```

### 3.3 ArduPilot 的 EKF3

ArduPilot 使用的是 **EKF3**（第三代），它追蹤 **24 個狀態**：

| 狀態 | 白話說明 | 數量 |
|------|----------|------|
| 四元數姿態 | 飛機的傾斜角度（前後左右） | 4 |
| NED 速度 | 飛機往北/東/下飛多快 | 3 |
| NED 位置 | 飛機在哪裡 | 3 |
| 陀螺儀偏差 | 陀螺儀的「零點漂移」修正量 | 3 |
| 加速度計偏差 | 加速度計的「零點漂移」修正量 | 3 |
| 地球磁場 | 當地的地磁方向 | 3 |
| 機體磁場偏差 | 飛機本身的磁場干擾 | 3 |
| **風速估計** | 風從哪個方向吹、多快（固定翼專用！） | 2 |

> **初學者要點：** 你不需要記住所有 24 個狀態。最重要的是理解 EKF 同時追蹤「飛機在哪、飛多快、朝哪個方向、風怎麼吹」。

### 3.4 動手驗證

執行模擬 1 來觀察 EKF 如何工作：

```bash
python3 simulations/sim1_ekf_state_estimation.py
```

你會看到：
- **2D 軌跡圖**：EKF 估計的軌跡（紅色虛線）緊密跟隨真實軌跡（藍色實線）
- **位置誤差圖**：即使有 GPS 雜訊，EKF 估計誤差維持在 GPS 雜訊範圍內
- **Innovation 圖**：EKF 計算的「預期 vs 實際」差異，正常時在零附近波動

---

## 4. 第二課：感測器融合——多個感測器如何協同工作

### 4.1 為什麼需要多個感測器？

每種感測器都有優缺點：

| 感測器 | 優點 | 缺點 |
|--------|------|------|
| **IMU**（加速度計+陀螺儀） | 高頻率(400Hz)、低延遲 | 會累積漂移 |
| **GPS** | 絕對位置、不會漂移 | 低頻率(5Hz)、有延遲(220ms) |
| **磁力計** | 提供航向（像指南針） | 容易受磁場干擾 |
| **氣壓計** | 高度量測穩定 | 受風速影響（靜壓誤差） |
| **空速計** | 量測相對空氣速度 | 只有固定翼才有，會結冰 |

**感測器融合**就是把這些不同特性的感測器「融合」在一起，取長補短。

### 4.2 Innovation 和 Gate 機制

EKF 怎麼知道感測器資料是否可信？靠的是 **Innovation**（創新量）：

```
Innovation = 感測器量測值 - EKF 預測值
```

如果 Innovation 太大（超過 Gate 閾值），表示這筆量測可能有問題，EKF 會**拒絕**它。

**生活比喻**：你閉眼走路，大腦預測你在房間中央。突然睜開眼看到自己在走廊上——你不會馬上相信，因為這和預期差太多（Innovation 太大），可能是你看錯了。

### 4.3 GPS Glitch 情境

執行 GPS 故障模擬：

```bash
python3 simulations/sim1_ekf_state_estimation.py
```

觀察 GPS Glitch 圖表：
- 在 10-15 秒之間，GPS 突然「跳」了 50 公尺
- EKF 的 Innovation Test Ratio 飆升超過 1.0
- EKF **拒絕**這些異常的 GPS 資料
- 飛機的位置估計依然穩定（靠 IMU 慣性推算）

> **初學者要點：** Innovation Gate 就是 EKF 的「品質管控」——只接受合理的感測器資料，拒絕明顯異常的資料。

### 4.4 風速估計（固定翼的特殊技能）

固定翼飛機有一項特殊能力：**估計風速**。

原理很簡單：
```
地速（GPS 量測）= 空速（空速計量測）+ 風速
→ 風速 = 地速 - 空速
```

如果飛機以 15 m/s 空速朝北飛，但 GPS 顯示地速只有 10 m/s，那就是有 5 m/s 的北風（逆風）。

執行模擬 3 觀察風速估計的收斂過程：

```bash
python3 simulations/sim3_sensor_fusion.py
```

---

## 5. 第三課：Multi-Lane 架構——EKF3 的安全網

### 5.1 概念

EKF3 最大的特色是 **Multi-Lane（多通道）架構**：

```
IMU 0 → EKF Lane 0 → ┐
IMU 1 → EKF Lane 1 → ├→ 自動選擇最佳 Lane → 輸出
IMU 2 → EKF Lane 2 → ┘
```

每個 IMU 運行一個獨立的 EKF。系統持續比較所有 Lane 的「健康度」，自動使用最好的那個。

### 5.2 為什麼這很重要？

**比喻**：你有三個 GPS 導航 App 同時在運行。如果其中一個 App 開始顯示你在海上（明顯錯誤），你的手機自動切換到另一個正常的 App。你甚至不需要知道切換發生了。

### 5.3 模擬驗證

執行模擬 3 的 Multi-Lane 部分：

```bash
python3 simulations/sim3_sensor_fusion.py
```

觀察圖表：
- **Lane 0** 在 IMU 故障期間（15-25 秒）位置開始漂移
- **健康度分數圖**：Lane 0 的分數大幅下降
- **Primary Lane 歷史**：系統自動從 Lane 0 切換到更健康的 Lane
- **最終輸出**：即使 Lane 0 壞了，輸出仍然準確

---

## 6. 第四課：PID 控制器——讓飛機聽話

### 6.1 什麼是 PID？

PID 控制器讓飛機的實際姿態追蹤目標姿態。它由四個部分組成（ArduPlane 使用 **FF + PID**）：

| 部分 | 全名 | 作用 | 比喻 |
|------|------|------|------|
| **FF** | Feedforward | 提前給出大概的控制量 | 你知道要轉 90 度，先打一個大概的方向盤角度 |
| **P** | Proportional | 根據當前誤差修正 | 偏了多少就修正多少 |
| **I** | Integral | 消除長期穩態誤差 | 如果一直往右偏，就持續增加向左的修正 |
| **D** | Derivative | 阻尼震盪 | 如果修正太快（開始甩頭），就減緩修正速度 |

### 6.2 控制器層級

ArduPlane 的控制系統是三層結構，像俄羅斯套娃：

```
外層：L1 Navigation → 「該往左轉 30 度」 → 目標 roll 角
         ↓
中層：Attitude Controller → 「要以 20 deg/s 滾轉」 → 目標角速率
         ↓
內層：Rate Controller → 「副翼偏轉 0.6」 → 舵面輸出
```

**調參順序：永遠從最內層開始！**（先 Rate → 再 Attitude → 最後 Navigation）

### 6.3 調參步驟（Roll 為例）

文件建議的順序：

```
步驟 1: 先調 FF（前饋）
   └─ 從 0.3 開始，逐步增加到飛機能跟上指令

步驟 2: 再調 P（比例）
   └─ 從 0.04 開始，增加到出現快速抖動，然後減半

步驟 3: 調 D（微分，可選）
   └─ 從 0 開始，小幅增加看是否改善阻尼

步驟 4: 最後調 I（積分）
   └─ ArduPlane 慣例：I = FF 的值
```

### 6.4 振盪診斷

調參時最常遇到的問題就是振盪。快速判斷表：

| 你觀察到... | 可能原因 | 解法 |
|-------------|----------|------|
| 快速抖動（像在發抖） | P 或 D 太高 | 降低 P 和 D |
| 慢速擺盪（像在搖擺） | 時間常數太小 | 增大 TCONST |
| 一直偏向一邊 | I 不足或 Trim 不對 | 增大 I 或重新 Trim |
| 低速著陸時晃動 | 低速舵面效率差 | 啟用 Airspeed Scaling |

### 6.5 動手驗證

```bash
python3 simulations/sim2_pid_tuning.py
```

觀察四種不同增益配置的效果：
1. **只有 FF**：飛機大致跟上，但有穩態偏差
2. **FF + P**：追蹤更緊密，響應更快
3. **FF + P + I**：穩態偏差消除
4. **完整 PIDF**：D 提供額外阻尼，最平滑

### 6.6 Airspeed Scaling

固定翼有一個特殊問題：同樣的舵面偏轉，在高速和低速時效果不同。

```
低速（9 m/s）：空氣力小 → 需要更大舵面偏轉 → 增益放大 2.78 倍
高速（22 m/s）：空氣力大 → 需要更小舵面偏轉 → 增益縮小到 0.46 倍
```

ArduPlane 自動處理這個：`effective_gain = base_gain * (15 / airspeed)^2`

---

## 7. 第五課：TECS 與 L1 導航——自主飛行的大腦

### 7.1 L1 導航：水平面怎麼飛

L1 控制器告訴飛機「該轉多少度」來追蹤航線：

核心參數是 **NAVL1_PERIOD**：
- **太小（10s）**：飛機反應過激，航跡出現蛇形（S-curve）
- **剛好（20s）**：平滑追蹤航線，轉彎流暢
- **太大（35s）**：反應遲鈍，轉彎半徑過大

### 7.2 TECS：垂直面怎麼飛

TECS（Total Energy Control System）把高度和速度當作**能量問題**：

```
總能量 = 動能（速度²）+ 位能（高度）
```

兩個控制手段：
- **油門**：改變總能量（加油=加總能量，收油=減總能量）
- **俯仰角**：在動能和位能之間分配（抬頭=速度換高度，低頭=高度換速度）

**SPDWEIGHT 參數**（速度/高度優先度）：

| 值 | 行為 | 適用場景 |
|----|------|----------|
| 0 | 優先保持高度 | 航測等需要精確高度的任務 |
| 1 | 平衡（預設） | 一般飛行 |
| 2 | 優先保持空速 | 容易失速的小型飛機（推薦！） |

### 7.3 動手驗證

```bash
python3 simulations/sim4_tecs_navigation.py
```

觀察：
- **L1 導航圖**：三種 NAVL1_PERIOD 的航跡比較
- **TECS 圖**：三種 SPDWEIGHT 在爬升時的高度和速度變化

---

## 8. 第六課：動手實驗——執行模擬程式

### 8.1 一鍵執行所有模擬

```bash
cd drone-flight-control-poc
python3 simulations/run_all.py
```

預期輸出：
```
  [OK] PASS: Sim 1: EKF State Estimation
  [OK] PASS: Sim 2: PID Tuning Strategy
  [OK] PASS: Sim 3: Multi-Sensor Fusion & Multi-Lane
  [OK] PASS: Sim 4: TECS & L1 Navigation

  ALL SIMULATIONS PASSED!
```

### 8.2 個別執行（建議學習順序）

| 順序 | 指令 | 學習重點 |
|------|------|----------|
| 1 | `python3 simulations/sim1_ekf_state_estimation.py` | EKF 基礎、GPS 融合、Glitch 防護 |
| 2 | `python3 simulations/sim3_sensor_fusion.py` | Multi-Lane、風速估計、高度源 |
| 3 | `python3 simulations/sim2_pid_tuning.py` | PID 各增益效果、振盪診斷 |
| 4 | `python3 simulations/sim4_tecs_navigation.py` | L1 導航、TECS 能量管理 |

### 8.3 自行修改實驗（進階）

鼓勵你修改模擬程式中的參數，觀察效果：

**實驗 1：改變 GPS 雜訊**
在 `sim1_ekf_state_estimation.py` 中找到：
```python
gps_noise_std = 0.5  # 試試改成 2.0 或 5.0
```

**實驗 2：改變 PID 增益**
在 `sim2_pid_tuning.py` 中修改 configs 陣列：
```python
'ff': 0.5, 'p': 0.08  # 試試 p=0.3 看快速振盪
```

**實驗 3：改變 L1 Period**
在 `sim4_tecs_navigation.py` 中：
```python
periods = [10, 20, 35]  # 試試加入 5 或 50
```

---

## 9. 第七課：理解模擬結果

### 9.1 模擬 1 的圖表解讀

| 圖表 | 正常應看到 | 如果異常... |
|------|-----------|------------|
| 2D 軌跡 | 紅色虛線緊跟藍色實線 | EKF 融合有問題 |
| 位置誤差 | 小於 GPS 雜訊的 2-3 倍 | 增益設定不當 |
| Innovation | 在零附近隨機波動 | 感測器有系統性偏差 |
| Test Ratio | 基本上 < 1.0 | Gate 閾值太緊或感測器異常 |

### 9.2 模擬 2 的圖表解讀

| 圖表 | 正常應看到 | 如果異常... |
|------|-----------|------------|
| FF Only | 大致跟蹤但有偏差 | FF 值不合適 |
| FF + P | 更緊密跟蹤 | P 可能太大或太小 |
| FF + P + I | 穩態偏差消除 | I 設太大會低頻振盪 |
| Full PID | 最平滑的響應 | D 太大會高頻抖動 |

### 9.3 從模擬到真實飛行

這些模擬驗證了**理論概念**。真實飛行中的差異：

| 模擬 | 真實飛行 |
|------|----------|
| 完美的數學模型 | 空氣動力非線性、結構振動 |
| 已知的雜訊分布 | 雜訊特性隨環境變化 |
| 即時計算 | 受限於飛控計算能力 |
| 4 狀態簡化 EKF | 完整 24 狀態 EKF3 |

**建議的學習路徑**：

```
本教學（概念理解）
    ↓
ArduPilot SITL 模擬器（完整模擬環境）
    ↓
Mission Planner 地面站操作
    ↓
真實飛機調參飛行
```

---

## 10. 附錄：名詞解釋

| 名詞 | 英文 | 白話解釋 |
|------|------|----------|
| EKF | Extended Kalman Filter | 一種數學方法，融合多個不完美的感測器得到最佳估計 |
| IMU | Inertial Measurement Unit | 加速度計+陀螺儀，量測飛機的加速度和旋轉 |
| GPS | Global Positioning System | 衛星定位，提供絕對位置 |
| NED | North-East-Down | 座標系統，以北、東、下為三個軸 |
| Innovation | - | 感測器量測值和 EKF 預測值的差，用來判斷資料品質 |
| Gate | - | 閾值，Innovation 超過 Gate 就拒絕該量測 |
| Lane | - | EKF3 的通道，每個 IMU 一個 Lane |
| PID | Proportional-Integral-Derivative | 經典控制器，用比例、積分、微分三項控制 |
| FF | Feedforward | 前饋，根據目標值直接計算近似控制量 |
| TECS | Total Energy Control System | 用能量觀點統一管理高度和速度 |
| L1 | L1 Navigation Controller | 水平面航路追蹤控制器 |
| FBWA | Fly By Wire A | ArduPlane 的穩定飛行模式，遙控器控制角度 |
| AUTOTUNE | Auto Tune | ArduPlane 自動調參飛行模式 |
| TAS | True Airspeed | 真實空速，修正了密度高度的空速 |
| DCM | Direction Cosine Matrix | 備用的姿態估計器，EKF 失效時啟用 |
| Trim | - | 配平，讓飛機在無操控時保持穩定直飛 |
| SITL | Software In The Loop | ArduPilot 的完整軟體模擬器 |

---

## 延伸資源

- **ArduPilot 官方文件**：https://ardupilot.org/plane/
- **ArduPilot SITL 模擬器**：https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- **UAV Log Viewer**：https://plot.ardupilot.org
- **本專案技術參考文件**：`ArduPlane_EKF3_Fusion_and_PID_Tuning.md`

---

> 本教學文件搭配 `simulations/` 目錄下的 Python 模擬程式使用。所有模擬已通過驗證，可在任何安裝了 Python 3.7+ 和 numpy/matplotlib 的環境中執行。
