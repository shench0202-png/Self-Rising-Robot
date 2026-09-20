# ESP32-C3 SuperMini 起身控制

此資料夾是獨立 Arduino sketch。它只使用 MPU6050 的三軸陀螺儀與三軸加速度計，透過六狀態 multiplicative quaternion EKF 估測姿態，再直接呼叫 `policy_network.h` 的 `forward_policy()`；沒有 M5Atom、Wi-Fi、磁力計或外部 EKF 函式庫相依性。兩顆伺服馬達預設會在開機時依序做 10°～170° 行程測試，第一次上電請先卸下連桿或架空機構。

## 目前模型

`policy_network.h` 是 `RL/robo1_getup_ppo_beckup3.zip` 的 PPO actor，模型 SHA-256 為 `DB7AD57A19FB3A08A87B8BDC870383D16A44477288508751813F13B740BB7246`。輸入順序為 `[roll_rad, pitch_rad, servo1_target_rad, servo2_target_rad]`，輸出為兩個 `[-1,1]` action。每 20 ms 更新 `target += 0.08 * action`；為了配合 1 號伺服 10°～170°的實體限制，1 號 target 限制為 ±1.3962634 rad（±80°），2 號仍為 ±1.55 rad。這不是 SAC 模型。

## 接線

請以板子上的 `IO` 標籤辨認 GPIO；不同賣家的 SuperMini 外觀可能不同。

| ESP32-C3 SuperMini | 接到 | 說明 |
|---|---|---|
| `3V3` | GY-521 `VCC` | 感測器與 I²C 均維持 3.3 V；先確認你手上模組接受 3.3 V VCC |
| `GND` | GY-521 `GND` | 共地 |
| `IO6` | GY-521 `SDA` | I²C data |
| `IO7` | GY-521 `SCL` | I²C clock |
| `GND` | GY-521 `AD0` | 固定 I²C 位址 0x68；如果模組已有下拉，可不另接 |
| `IO10` | Servo 1 訊號線 | 50 Hz PWM |
| `IO20` | Servo 2 訊號線 | 50 Hz PWM；此腳位同時是 UART0 RX，本程式不用 UART0 |
| 板載 `IO8` | 板載 LED | 低電位點亮，不需外接 |
| `GND` | 獨立伺服器電源負極 | 兩路訊號必須共地 |
| 獨立伺服器電源正極 | Servo 1/2 電源正極 | 依伺服器額定電壓選擇，常見為 5 V；勿由 `3V3` 供應兩顆伺服器 |

伺服器電源正極只接伺服器。初次 USB 調試時，不要把外部伺服器電源正極接到 SuperMini `5V` 或 USB 5 V；先量測電源與極性。若你的伺服器不接受 3.3 V 訊號高電位，需要適合 PWM 的電平轉換。I²C pull-up 必須到 3.3 V，不能到 5 V。

Type-C 的原生 USB 訊號使用 GPIO18（D-）與 GPIO19（D+）；目前伺服 PWM 使用 GPIO10／GPIO20，不會占用 USB 資料腳。保持 Arduino IDE 的 `USB CDC On Boot = Enabled`，讓程式的 `Serial` 指令與遙測走 Type-C。原本接在 IO4／IO5 的 GY-521 SDA／SCL 必須改接 IO6／IO7。

## Arduino IDE

1. 安裝 Espressif `esp32` boards package；伺服輸出直接使用 ESP32 Core 內建的 LEDC 硬體 PWM，不需要安裝 `ESP32Servo` library。
2. 開啟 `esp32c3_supermini/esp32c3_supermini.ino`，選擇 `ESP32C3 Dev Module`。若板子有專用 board definition，以實際板子為準。
3. 以 USB-C 連接電腦，將 `USB CDC On Boot` 設為 `Enabled`，選擇正確埠並上傳。
4. 開啟 115200 baud Serial Monitor。上傳困難時按住 BOOT、按一下 RESET，進入下載模式後重試。

此 sketch 只需同資料夾中的 `.ino`、`Receive.*`、`RobotRuntime.*`、`NeuralNetwork.*`、`policy_network.h`。`NeuralNetwork.*` 是沿用模組的相依檔，實際推論直接使用匯出函式。

## 程式設定

在 `esp32c3_supermini.ino`：

```cpp
const bool ENABLE_POLICY_CONTROL = true;
const bool ENABLE_SERVO_OUTPUT = true;
const bool ENABLE_IMU_AUTO_START = true;
const float AUTO_START_TILT_RAD = 0.5235988f;  // 30 deg
const float AUTO_REARM_TILT_RAD = 0.3490659f;  // 20 deg
const unsigned long AUTO_START_HOLD_MS = 250;
const uint8_t SDA_PIN = 6;
const uint8_t SCL_PIN = 7;
const uint8_t SERVO1_PIN = 10;
const uint8_t SERVO2_PIN = 20;
const uint8_t STATUS_LED_PIN = 8;
const int SERVO1_MIN_DEG = 10;
const int SERVO1_MAX_DEG = 170;
const int SERVO2_MIN_DEG = 0;
const int SERVO2_MAX_DEG = 180;
```

開機時先讓 1 號伺服執行 `90° → 10° → 90° → 170° → 90°`，再讓 2 號執行相同序列；不再直接跳到目標角度，而是每 20 ms 移動 1°（約 50°/s），到達各位置後停留 300 ms，另一顆維持 90°。完成後才校正 MPU6050 並進入主迴圈。板載 LED 正常時每 0.5 秒切換一次亮滅，也就是每秒完成一次閃爍；初始化失敗時快閃。伺服 PWM 由 ESP32-C3 LEDC 以 50 Hz、14-bit 解析度輸出，設定為 1500 µs 中心、約 500–2500 µs 全範圍；因此 10°約為 611 µs、170°約為 2389 µs。1 號伺服的最終 LEDC 角度命令仍固定限制在 10°～170°，PPO 即使要求更大的角度也不會超出此範圍；2 號伺服的一般輸出維持 0°～180°。實際機構的零點、正負方向、允許行程，請調整 `SERVO*_OFFSET_US`、`SERVO*_PULSE_RANGE_US`、`SERVO*_MIN_DEG`、`SERVO*_MAX_DEG`、`SERVO*_INVERT`。上電校正 gyro 時保持 GY-521 靜止。

## 六軸 Quaternion EKF

EKF 名義狀態是 WXYZ 四元數與三軸 gyro bias；六維誤差狀態為三軸小角度誤差及三軸 bias 誤差。每次更新先用 gyro 做 quaternion／covariance prediction，再用正規化 accelerometer 重力方向修正 roll、pitch 與可觀測的 bias。當加速度大小不在 0.70–1.30 g 時會跳過重力修正，避免起身撞擊直接拉歪姿態。因為程式完全不使用磁力計，絕對 yaw 不可觀測，會隨 gyro bias 緩慢漂移；目前 PPO 只使用 roll、pitch。

## PPO Euler 85°保護

原始四元數與 `roll/pitch/yaw` 遙測保持不變；只有送入 PPO 的 `policy_roll/policy_pitch` 使用狀態式保護。當 policy pitch 到達 +85°或 -85°時，程式記住當下的 roll，將 policy pitch 固定在該側的 85°，並保持 roll 不受 ZYX Euler 跨越 90°後約 180°的表示跳變影響。回程時必須同時滿足 `|pitch| <= 80°` 且原始 roll 已回到鎖定前的分支才解除，避免在臨界點反覆切換。Serial `P` 行最後一欄在原始 Euler 接近奇異點或此保護仍鎖定時為 `1`。

這項保護需要先經過 85°附近才能記住正確 roll。若控制器開機時機身已在 pitch 超過 90°的位置，單靠當下四元數無法區分「pitch 翻過 90°」和真正的 roll 旋轉；應先以已知方向啟動並完成姿態方向測試。這項保護也改變了 PPO 原本的 principal-Euler 觀測，因此第一次測試必須先關閉伺服輸出，再進行架空測試。

## 驗證順序

1. 斷開機構連桿或架空機器，確認開機時兩顆伺服依序完成 10°～170° 行程測試、板載 LED 每秒閃一次，Serial 顯示 `SERVO_STARTUP_TEST_DONE` 與 `MPU6050 detected.`。
2. 依次擺出 `roll_pos`、`roll_neg`、`pitch_pos`、`pitch_neg`，確認 Serial `P` 行的 policy roll/pitch 分別約為 `(±1.57,0)`、`(0,±1.48)` rad；pitch 的 `1.48 rad` 約為 85°。必要時改 `POLICY_SWAP_ROLL_PITCH` 與正負號；也要確認感測器實際安裝方向。
3. 保持機器架空，將 IMU 的 body-Z 相對重力方向傾斜超過 30°並維持 250 ms；Serial 應依序顯示 `AUTO_TRIGGER_PENDING`、`AUTO_TRIGGER`、`POLICY_START`，`P` 行第二欄從 `0` 變成 `1`，target/action 開始變化。也可以輸入 `G` 手動啟動；`S` 停止，`Z` 將 target 歸零，`H` 顯示指令。
4. 確認伺服電源、中心、行程與方向都正確後，才做落地起身測試。實機速度、供電、摩擦與機構尺寸尚未與 MuJoCo 驗證一致。

程式預設使用 IMU tilt 自動觸發：超過 30°並維持 250 ms 後開始 PPO；一次啟動後會先解除武裝，必須回到 20°內才會顯示 `AUTO_TRIGGER_ARMED` 並允許下一次自動觸發，避免超時或臨界角度造成反覆啟動。`G` 可隨時手動啟動。每次 policy 最長 14 秒，姿態穩定直立 700 ms 時停止。`S` 或 `Z` 會解除自動觸發武裝，回到 20°內才重新武裝。這些韌體條件與 MuJoCo 的連續 1 秒完整 `goal_pose` 判定不同。
