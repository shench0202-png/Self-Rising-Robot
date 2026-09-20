# Self Rising Robot（自起身機器人）

本專案包含雙自由度自起身機器人的 MuJoCo 模擬、Behavior Cloning（BC）預訓練、PPO 強化學習，以及 ESP32-C3 實機部署程式。

- `RL_MuJoCo/`：MuJoCo 模型、Gymnasium 環境、BC/PPO 訓練、評估與模型匯出。
- `ESP32/esp32c3_supermini/`：MPU6050 姿態估測、PPO 推論與兩顆伺服馬達控制。

目前 MuJoCo 環境與 ESP32-C3 韌體已通過基本載入及編譯測試；完整落地起身效果仍需依實際機構、供電與伺服方向進行驗證。

## 模擬顯示

MuJoCo 起身模擬展示將放置於此區。

> 目前專案內尚未加入模擬截圖或影片。

## 實機展示

ESP32-C3、MPU6050 與雙伺服實機起身展示將放置於此區。

> 目前專案內尚未加入實機照片或影片。

## 參考資料

- [MuJoCo：MJCF 模型文件](https://mujoco.readthedocs.io/en/stable/modeling.html)
- [Stable-Baselines3：PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)
- [Gymnasium：自訂環境](https://gymnasium.farama.org/introduction/create_custom_env/)
- [Espressif：Arduino-ESP32 LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [MPU-6050 Product Specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
