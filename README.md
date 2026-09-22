# Self Rising Robot（自起身機器人）

本專案包含雙自由度自起身機器人的 MuJoCo 模擬、Behavior Cloning（BC）預訓練、PPO 強化學習，以及 ESP32-C3 實機部署程式。

- `RL_MuJoCo/`：MuJoCo 模型、Gymnasium 環境、BC/PPO 訓練、評估與模型匯出。
- `ESP32/esp32c3_supermini/`：MPU6050 姿態估測、PPO 推論與兩顆伺服馬達控制。

目前 MuJoCo 環境與 ESP32-C3 韌體已通過基本載入及編譯測試；
完整落地起身效果也已驗證。

這專案主要用來了解PPO的基礎，以及了解reward的設計
也了解RL的環境設置與建造

## 模擬顯示

MuJoCo 起身模擬展示將放置於此區。

> 目前專案內尚未加入模擬截圖或影片。

## 實機展示

[![ESP32-C3 雙伺服自起身實機測試](https://img.youtube.com/vi/RmcQNsX-z0Y/hqdefault.jpg)](https://youtube.com/shorts/RmcQNsX-z0Y?feature=share)

[ESP32-C3 雙伺服自起身實機測試（YouTube Shorts）](https://youtube.com/shorts/RmcQNsX-z0Y?feature=share)

點擊預覽圖或文字連結，即可在 YouTube 播放。

## 參考資料

主要參考:
- [HomeMadeGarbage：AIエージェント Codex で 強化学習 -起き上がりロボット-](https://youtu.be/LWi4ya8LXFA)
- [HomeMadeGarbage：SelfRisingRobot GitHub](https://github.com/homemadegarbage/SelfRisingRobot)

RL學習:
- https://hrl.boyuai.com/chapter/2/sac%E7%AE%97%E6%B3%95/#141-%E7%AE%80%E4%BB%8B
- https://hackmd.io/@shaoeChen/Bywb8YLKS/https%3A%2F%2Fhackmd.io%2F%40shaoeChen%2FHkH2hSKuS
