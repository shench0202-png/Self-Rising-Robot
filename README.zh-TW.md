# ESP32、BC、PPO 與 MuJoCo 程式備份

這個資料夾是 `SelfRisingRobot` 目前可用程式的獨立副本。原始專案檔案沒有被移動或刪除。

## 資料夾內容

- `ESP32/esp32c3_supermini/`：目前修改完成的 ESP32-C3 韌體，包含 MPU6050 四元數 EKF、PPO actor、LEDC 伺服輸出、自動傾倒觸發、Servo 1 限位與 Servo 2 反轉。
- `RL_MuJoCo/pretrain_robo1_from_scripted.py`：使用腳本專家資料進行 Behavior Cloning 預訓練。
- `RL_MuJoCo/train_robo1.py`：PPO 從頭訓練或接續 BC/PPO 模型訓練。
- `RL_MuJoCo/robo1_env.py`：Gymnasium/MuJoCo 訓練環境、觀測、動作與 reward。
- `RL_MuJoCo/robo1.xml` 與 `RL_MuJoCo/assets/`：MuJoCo 機器人模型及 STL。
- `RL_MuJoCo/getup_reference.py`、`scripted_getup.py`、`search_all_getup.py`：專家起身軌跡與搜尋工具。
- `RL_MuJoCo/play_robo1_policy.py`、`eval_robo1_policy.py`：模型播放與評估。
- `RL_MuJoCo/export_policy_header.py`：將 PPO actor 匯出成 ESP32 使用的 `policy_network.h`。
- `RL_MuJoCo/robo1_getup_ppo_bc_good.zip`：保留的 BC 預訓練模型。
- `RL_MuJoCo/robo1_getup_ppo_beckup3.zip`：目前 ESP32 `policy_network.h` 對應的 PPO 模型。

## Python 環境

在這個資料夾的上一層執行：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r .\Export_ESP32_BC_PPO_MuJoCo_20260920\RL_MuJoCo\requirements.txt
cd .\Export_ESP32_BC_PPO_MuJoCo_20260920\RL_MuJoCo
```

## BC 預訓練

```powershell
python pretrain_robo1_from_scripted.py
```

## PPO 接續訓練

```powershell
python train_robo1.py --model-in robo1_getup_ppo_bc_good.zip --timesteps 200000 --n-envs 6 --model-out robo1_getup_ppo
```

## MuJoCo 播放

```powershell
python play_robo1_policy.py --pose roll_pos
```

若要播放目前部署的模型，可先複製或重新命名模型：

```powershell
Copy-Item robo1_getup_ppo_beckup3.zip robo1_getup_ppo.zip
python play_robo1_policy.py --pose roll_pos
```

## 匯出 ESP32 policy header

```powershell
python export_policy_header.py robo1_getup_ppo_beckup3.zip -o policy_network.h
```

ESP32 Arduino IDE 設定：`ESP32C3 Dev Module`、`USB CDC On Boot = Enabled`。伺服需使用外部電源並與 ESP32 共地。
