# THLogger

Autonomous temperature, humidity, and vibration data-logger running on Zephyr RTOS.  
Logs are stored persistently in on-chip RRAM and can be retrieved over a serial shell at any time.

---

## Hardware

| Component | Part | Role |
|-----------|------|------|
| MCU board | [Seeed Studio XIAO nRF54L15](https://wiki.seeedstudio.com/xiao_nrf54l15_getting_started/) | Application processor (nRF54L15, Arm Cortex-M33, 1.5 MB RRAM) |
| Expansion board | [Seeed Studio Expansion Base for XIAO](https://wiki.seeedstudio.com/Seeeduino-XIAO-Expansion-Board/) | Carrier with RTC battery holder, Grove connectors |
| Temp/humidity | DHT20 (Aosong) — I²C 0x38 | Capacitive RH + temperature sensor |
| IMU | LSM6DSO (STMicroelectronics) — I²C 0x6A | 3-axis accelerometer for vibration measurement |
| RTC | PCF8563 (NXP) — I²C 0x51 | Battery-backed real-time clock (CR1220 on expansion board) |

All three sensors share the XIAO I²C bus operating at 400 kHz.

---

## What it does

Every **5 minutes** the firmware:

1. **Vibration snapshot** — Runs the LSM6DSO accelerometer at 104 Hz for 10 seconds (1 000 samples per axis).  
   Computes two metrics from the AC component (gravity removed):
   - **RMS acceleration** — combined across X/Y/Z using one-pass variance: $\sqrt{\sigma_x^2 + \sigma_y^2 + \sigma_z^2}$
   - **Peak amplitude** — half the observed min/max range per axis, combined as a Euclidean norm

2. **Environment sample** — Fetches temperature (°C) and relative humidity (%) from the DHT20 via the Zephyr sensor driver.

3. **Log entry** — Stores one record in a circular ring buffer in on-chip RRAM (NVS):

   | Field | Type | Description |
   |-------|------|-------------|
   | `timestamp_s` | `int64_t` | UTC Unix seconds |
   | `temp_val1/2` | `int32_t × 2` | Temperature (Zephyr `sensor_value`) |
   | `hum_val1/2` | `int32_t × 2` | Humidity (Zephyr `sensor_value`) |
   | `vibration_rms_mg` | `int32_t` | RMS vibration in milli-g |
   | `vibration_peak_mg` | `int32_t` | Peak vibration in milli-g |

4. **LED blink** — The on-board LED blinks 500 ms to confirm each successful sample.

### Timekeeping

- On first use, set the time once with `thtime set`.  
  The firmware writes it to the **PCF8563 RTC** (battery-backed) and to NVS.
- On every subsequent power-up the RTC is read automatically — timestamps remain correct across resets without any user intervention.
- A software clock derived from `k_uptime_get()` tracks sub-second elapsed time between RTC reads.

### Storage

```
RRAM layout (1 428 KB total)
├── 0x000000 – 0x0E5000   Application code      (916 KB)
└── 0x0E5000 – 0x165000   NVS storage partition (512 KB)
```

- **Capacity:** 10 000 entries × 40 B = 400 KB — well within the 512 KB partition.  
- **Logging duration at 5 min/entry:** ~34.7 days of continuous data.  
- Ring-buffer wrapping: oldest entry is silently overwritten when the buffer is full.

---

## Serial shell interface

Connect at **115 200 baud** (USB CDC-ACM or UART).  
The Zephyr shell is available immediately after boot.

### Log commands

| Command | Description |
|---------|-------------|
| `thlog show` | Print all stored entries in chronological order |
| `thlog clear` | Erase all log entries (resets write index) |

Example output of `thlog show`:

```
Timestamp (UTC)        Temp (C)    Humidity (%)    VibRMS (mg)  VibPeak (mg)
------------------------------------------------------------------------
2026-06-21 19:54:11      27.01        65.83               3           12
2026-06-21 20:04:40      26.48        67.57               4           15
...
250 entries (max 10000).
```

### Time commands

| Command | Description |
|---------|-------------|
| `thtime set YYYY-MM-DD HH:MM:SS` | Set the UTC clock and program the PCF8563 RTC |
| `thtime get` | Print the current derived UTC time |

Example:

```
uart:~$ thtime set 2026-06-23 14:30:00
PCF8563 RTC updated.
Time set to: 2026-06-23 14:30:00 UTC

uart:~$ thtime get
Current time: 2026-06-23 14:30:05 UTC
```

> **Note:** `thtime set` only needs to be run once after initial flashing.  
> The PCF8563 battery backup keeps the time correct across all subsequent power cycles.

---

## Log visualisation

`THLogShow.py` reads `thlog.txt` (produced by pasting `thlog show` output into a file) and renders a dual-axis line graph of temperature and humidity.

**Requirements:** Python 3, `pandas`, `matplotlib`

```bash
pip install pandas matplotlib
python THLogShow.py
```

The x-axis shows sample index (log order); left y-axis shows temperature (°C) in red; right y-axis shows humidity (%) in blue.

---

## Building and flashing

Requires [nRF Connect SDK](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/getting_started.html) v3.3.1 or later.

```bash
# Full build (pristine)
west build --pristine -- -DBOARD=xiao_nrf54l15/nrf54l15/cpuapp

# Flash
west flash
```

---

## Source structure

```
THLogger/
├── CMakeLists.txt
├── prj.conf                        Zephyr Kconfig
├── sysbuild.conf
├── boards/
│   └── xiao_nrf54l15_nrf54l15_cpuapp.overlay   DTS: sensors, RTC, NVS partition
├── src/
│   ├── main.c                      NVS, software clock, shell commands, main loop
│   ├── imu.h / imu.c               LSM6DSO vibration driver (raw I²C)
│   ├── pcf8563.h / pcf8563.c       PCF8563 RTC wrapper (Unix ↔ rtc_time)
│   └── dht20.h / dht20.c           DHT20 sensor wrapper
└── THLogShow.py                    Python log visualisation script
```

### Module responsibilities

| Module | Responsibility |
|--------|---------------|
| `imu` | Initialise LSM6DSO; collect 1 000 samples at 104 Hz; compute RMS and peak vibration in milli-g |
| `pcf8563` | Initialise PCF8563; convert Unix timestamps to/from `struct rtc_time`; read/write the hardware clock |
| `dht20` | Initialise the Zephyr DHT20 driver; fetch temperature and humidity in one call |
| `main` | Orchestrate the 5-minute sampling loop; manage NVS ring buffer; expose shell commands |

---

## Dependencies (Zephyr Kconfig)

| Symbol | Purpose |
|--------|---------|
| `CONFIG_I2C` | I²C bus driver |
| `CONFIG_SENSOR` | Zephyr sensor subsystem (DHT20) |
| `CONFIG_DHT20` | Aosong DHT20 driver |
| `CONFIG_RTC` | Zephyr RTC subsystem |
| `CONFIG_RTC_PCF8563` | NXP PCF8563 driver (auto-enabled by DTS node) |
| `CONFIG_NVS` | Non-volatile storage |
| `CONFIG_FLASH` / `CONFIG_FLASH_MAP` | Flash access for NVS |
| `CONFIG_SHELL` | Serial shell interface |
| `CONFIG_GPIO` | LED control |
