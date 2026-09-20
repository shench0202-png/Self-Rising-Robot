# Self Rising Robot（自起身機器人）

本專案包含雙自由度自起身機器人的 MuJoCo 模擬、Behavior Cloning（BC）預訓練、PPO 強化學習，以及 ESP32-C3 實機部署程式。

- `RL_MuJoCo/`：MuJoCo 模型、Gymnasium 環境、BC/PPO 訓練、評估與模型匯出。
- `ESP32/esp32c3_supermini/`：MPU6050 姿態估測、PPO 推論與兩顆伺服馬達控制。

目前 MuJoCo 環境與 ESP32-C3 韌體已通過基本載入及編譯測試；完整落地起身效果仍需依實際機構、供電與伺服方向進行驗證。

## 系統流程

```text
MuJoCo robo1.xml
  ↓
腳本專家起身軌跡
  ↓
Behavior Cloning 預訓練
  ↓
PPO 接續訓練
  ↓
export_policy_header.py
  ↓
policy_network.h
  ↓
ESP32-C3 + MPU6050 + Servo 1/2
```

PPO actor 使用四個輸入與兩個輸出：

```text
observation = [roll, pitch, servo1_target, servo2_target]
action      = [servo1_delta, servo2_delta]
```

控制頻率為 50 Hz，每一步以 `target += action × 0.08 rad` 更新伺服目標。

## 資料夾內容

### ESP32 韌體

`ESP32/esp32c3_supermini/`

- `esp32c3_supermini.ino`：主程式、50 Hz PPO 排程、自動啟動與序列埠指令。
- `RobotRuntime.cpp/.h`：四元數 MEKF、Euler/PPO 輸入處理、LEDC 伺服輸出。
- `Receive.cpp/.h`：MPU6050 讀取、濾波與陀螺儀零偏校正。
- `policy_network.h`：由 `robo1_getup_ppo_beckup3.zip` 匯出的 PPO actor。

目前韌體重點：

- 僅使用 MPU6050 加速度計與陀螺儀，不使用磁力計。
- 傾角持續大於等於 30°、250 ms 後自動啟動 PPO；回到 20°內重新解鎖。
- 單次起身最長 14 秒。
- Servo 1 輸出限制為 10°～170°。
- Servo 2 已依目前實機需求反轉方向。
- 使用 ESP32 Arduino Core 內建 LEDC，不需要 `ESP32Servo.h`。

### BC、PPO 與 MuJoCo

`RL_MuJoCo/`

- `robo1.xml`、`assets/`：MuJoCo 機器人模型與 STL。
- `robo1_env.py`：Gymnasium 環境、觀測、動作、reward 與終止條件。
- `getup_reference.py`：四種倒地姿態的專家目標角序列。
- `pretrain_robo1_from_scripted.py`：蒐集專家資料並進行 BC 預訓練。
- `train_robo1.py`：PPO 從頭訓練或接續既有模型微調。
- `play_robo1_policy.py`、`eval_robo1_policy.py`：播放與評估模型。
- `export_policy_header.py`：將 Stable-Baselines3 PPO actor 匯出為 C header。
- `robo1_getup_ppo_bc_good.zip`：保留的 BC 預訓練模型。
- `robo1_getup_ppo_beckup3.zip`：目前部署到 ESP32 的 PPO 模型。

## Python 環境

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r .\RL_MuJoCo\requirements.txt
cd .\RL_MuJoCo
```

## BC 預訓練

```powershell
python pretrain_robo1_from_scripted.py
```

## PPO 訓練

接續 BC 模型訓練：

```powershell
python train_robo1.py --model-in robo1_getup_ppo_bc_good.zip --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

從頭訓練：

```powershell
python train_robo1.py --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

## MuJoCo 播放與評估

```powershell
python play_robo1_policy.py --pose roll_pos
python eval_robo1_policy.py
```

若要播放目前 ESP32 使用的模型：

```powershell
Copy-Item robo1_getup_ppo_beckup3.zip robo1_getup_ppo.zip
python play_robo1_policy.py --pose roll_pos
```

## 匯出 ESP32 模型

```powershell
python export_policy_header.py robo1_getup_ppo_beckup3.zip -o policy_network.h
```

匯出後將 `policy_network.h` 放入 `ESP32/esp32c3_supermini/`。

## ESP32-C3 接線

| ESP32-C3 SuperMini | 裝置 |
|---|---|
| IO6 | GY-521 SDA |
| IO7 | GY-521 SCL |
| IO10 | Servo 1 訊號 |
| IO20 | Servo 2 訊號 |
| 3V3 / GND | GY-521 VCC / GND |

兩顆伺服必須使用合適的外部電源，外部電源 GND、ESP32-C3 GND 與伺服 GND 必須共地。請勿使用 ESP32-C3 的 3V3 腳直接供電給伺服。

## 序列埠指令

| 指令 | 功能 |
|---|---|
| `G` | 啟動 PPO 起身 |
| `S` | 停止並回到中心 |
| `Z` | 將伺服目標歸零 |
| `H` | 顯示說明 |

## 參考資料

- [MuJoCo：MJCF 模型文件](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [Stable-Baselines3：PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [Gymnasium：自訂環境](https://gymnasium.farama.org/introduction/create_custom_env/)
- [Espressif：Arduino-ESP32 LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [MPU-6050 Product Specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

