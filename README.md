# 16-IR Line Follower

An ESP32-based 16-sensor IR line follower robot using a CD74HC4067 16-channel multiplexer and dual BTS7960 motor drivers, implementing PID control for smooth and accurate line tracking.

***

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32 (Arduino Core v3.x) |
| Motor Driver | 2× BTS7960 (Left + Right) |
| Sensor MUX | CD74HC4067 — 16-channel analog MUX |
| Sensors | 16× IR sensors via MUX |
| Buttons | Calibrate (GPIO 15), Start (GPIO 33) |

***

## Pin Definitions

### Left Motor Driver (BTS7960)
| Signal | GPIO |
|--------|------|
| R_EN   | 23   |
| L_EN   | 4    |
| RPWM   | 25   |
| LPWM   | 32   |

### Right Motor Driver (BTS7960)
| Signal | GPIO |
|--------|------|
| R_EN   | 18   |
| L_EN   | 19   |
| RPWM   | 21   |
| LPWM   | 22   |

### 16-Channel MUX (CD74HC4067)
| Signal | GPIO |
|--------|------|
| EN     | 26   |
| S0     | 27   |
| S1     | 14   |
| S2     | 16   |
| S3     | 13   |
| SIG    | 34 (ADC input) |

> **Note:** Pin assignments are intentionally swapped from the physical motor labels to correct for crossed motor wiring on the chassis.

***

## PID Parameters

| Parameter | Value |
|-----------|-------|
| Kp        | 0.15  |
| Ki        | 0.0   |
| Kd        | 1.2   |
| Base Speed | 150 (PWM) |
| Max PWM   | 255   |

Ki is set to 0 — this is effectively **PD control**. Tune Kp and Kd for your track.

***

## How It Works

1. **Calibration** — Press the CALIBRATE button. Sweep the robot over the line for 4 seconds. The firmware records min/max ADC values per sensor to normalize readings.
2. **Start** — Press the START button. The robot begins the PID loop.
3. **Sensor Reading** — Raw 12-bit ADC values are read from all 16 sensors via the MUX, mapped to 0–1000, and thresholded at 500 to produce binary (on/off line) values.
4. **Position Calculation** — A weighted average of active sensors gives a position value from 0 to 15000. Center = **7500**.
5. **PID Loop** — Error = `position - 7500`. PID output adjusts left/right motor speeds.
6. **Special Cases:**
   - **Cross junction** (≥8 sensors active) → forces straight ahead (returns 7500)
   - **Line lost** (0 sensors active) → recovers based on last known position (hard left, hard right, or straight)
   - **Sharp/90° turns** (`|error| > 4500`) → one motor reverses for an in-place pivot

***

## Files

| File | Description |
|------|-------------|
| `main.ino` | Full PID line follower — calibration, sensor reading, position calculation, motor control |
| `test.ino` | Motor direction test — drives forward and backward in 10s intervals at full speed |

***

## Flashing

Requires **Arduino IDE** or **PlatformIO** with ESP32 board support.

1. Install ESP32 Arduino Core v3.x
2. Select board: `ESP32 Dev Module`
3. Flash `main.ino`
4. Open Serial Monitor at **115200 baud** to watch calibration and debug output

***

## Tuning Tips

- Increase **Kd** if the robot oscillates on straight sections
- Increase **Kp** if it's slow to correct on curves
- Adjust the hard-turn threshold (`4500`) if 90° turns are being triggered too early or too late
- Raise **BASE_SPEED** after tuning PID gains — higher speed needs higher Kd

***

## License

GPL-3.0 — see [LICENSE](LICENSE)
