# Spirob Aktuation

ESP32-basiertes Firmware- und Python-Tooling-Projekt zur Steuerung eines dualen Seilzug-Aktuators (SpiRob) mit Kraftregelung, binärer Telemetrie und MuJoCo-Digital-Twin.

---

## Inhaltsverzeichnis

1. [Systemübersicht](#systemübersicht)
2. [Hardware](#hardware)
3. [Firmware-Architektur](#firmware-architektur)
4. [Build & Flash (PlatformIO)](#build--flash-platformio)
5. [Kommunikationsprotokoll](#kommunikationsprotokoll)
6. [Python-Tooling](#python-tooling)
7. [MuJoCo Digital Twin](#mujoco-digital-twin)
8. [Wichtige Konventionen & Fallstricke](#wichtige-konventionen--fallstricke)

---

## Systemübersicht

```
┌─────────────────────────────────────────────────────────┐
│  PC / Python                                            │
│  ┌──────────────────┐   ┌───────────────────────────┐  │
│  │ live_monitor /   │   │  spirob_digital_twin.py   │  │
│  │ system_id /      │◄──┤  (MuJoCo GUI → Hardware)  │  │
│  │ step_response    │   └───────────────────────────┘  │
│  └────────┬─────────┘                                   │
│           │ UART 460800 (sys_id) / 115200 (prod)        │
└───────────┼─────────────────────────────────────────────┘
            │
┌───────────▼─────────────────────────────────────────────┐
│  ESP32                                                  │
│  ┌──────────────┐  ┌─────────────────┐                 │
│  │ ForceSensor  │  │  EndStopSwitch  │                 │
│  │ (NAU7802     │  │  (debounced,    │                 │
│  │  via I2C-Mux)│  │   GPIO 26/27)   │                 │
│  └──────┬───────┘  └────────┬────────┘                 │
│         │                   │                           │
│  ┌──────▼───────────────────▼──────┐                   │
│  │       ForceControlLoop          │                   │
│  │  (PID · Speed · Manual modes)   │                   │
│  └──────────────┬──────────────────┘                   │
│                 │                                       │
│  ┌──────────────▼──────────────────┐                   │
│  │  MotorDriver × 2                │                   │
│  │  SC-Servo UART@1Mbps            │                   │
│  │  Servo ID 1 (Motor 0)           │                   │
│  │  Servo ID 2 (Motor 1)           │                   │
│  └─────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────┘
```

---

## Hardware

| Komponente | Detail |
|---|---|
| Mikrocontroller | ESP32 (esp32dev) |
| Motoren | 2× SC-Servo (Feetech STS/SMS), UART @ 1 Mbit/s, Serial2 (Pins 16 RX / 17 TX) |
| Kraftsensoren | 2× NAU7802 (I2C @ 0x2A) hinter TCA9548A I²C-Mux |
| Endstops | GPIO 26 (Motor 0) und GPIO 27 (Motor 1), INPUT_PULLUP, active-low |
| Seilwindenumfang | Ø 44 mm → 1 Schritt = `π·44/4096 ≈ 0.0337 mm` |
| Debug-UART | 115200 Baud (production), 460800 Baud (system_id) |

### Motor-Mapping

| Index (Python/CLI) | Servo-ID (Firmware) | Mux-Kanal | Endstop-Pin | Richtung invertiert |
|---|---|---|---|---|
| 0 | 1 | 0 | 26 | ja |
| 1 | 2 | 1 | 27 | nein |

---

## Firmware-Architektur

### Schichten

```
main.cpp / main_SystemIdentification.cpp   ← Einstiegspunkt (je Umgebung einer)
├── ForceControlLoop      ← Regelt Kraft via PID oder gibt Speed direkt durch
│   ├── MotorDriver       ← SC-Servo-Protokoll (Position, Speed, Diagnose)
│   ├── ForceSensor       ← Abstrakte Basis; Implementierungen: HX711, NAU7802
│   └── PidController     ← Generischer PID
└── EndStopSwitch         ← Debounced Limit-Switch (Safety)
```

### Schlüsseldateien

| Datei | Funktion |
|---|---|
| [src/main.cpp](src/main.cpp) | **Production**: Dual-Motor PID, ASCII-CLI @ 115200 |
| [src/main_SystemIdentification.cpp](src/main_SystemIdentification.cpp) | **System-ID**: Binäre Telemetrie 600 Hz, Step-Response, Homing @ 460800 |
| [src/ForceControlLoop.h/.cpp](src/ForceControlLoop.h) | PID-Kraftregelschleife; `ControlMode` ≠ `DriverMode` |
| [src/MotorDriver.h/.cpp](src/MotorDriver.h) | SC-Servo-Treiber; Konstruktor nimmt **Servo-ID** (1 oder 2), nicht Array-Index |
| [src/ForceSensor.h](src/ForceSensor.h) | Abstrakte Schnittstelle; `getForce()` → Newton, intern kg |
| [src/ForceSensorAnu78025.h/.cpp](src/ForceSensorAnu78025.h) | NAU7802-Implementierung mit I²C-Mux |
| [src/EndStopSwitch.h/.cpp](src/EndStopSwitch.h) | `isRawTriggered()` = sofort; `isTriggered()` = entprellt (5 ms) |
| [platformio.ini](platformio.ini) | Umgebungs-Isolation via `build_src_filter` |

### Kraftsensor-Kalibrierung

Ausschließlich zur Compile-Zeit als `#define` am Dateianfang der jeweiligen `main_*.cpp`:

```cpp
#define FORCE_OFFSET_0   153859L
#define FORCE_SCALE_0    105.13830f
```

Kein NVS / EEPROM-Persistenz. Werte müssen nach Hardware-Umbau neu ermittelt werden.

---

## Build & Flash (PlatformIO)

### Umgebungen

| Umgebung | Quelldate | Baud | Zweck |
|---|---|---|---|
| `production` | `main.cpp` | 115200 | Dual-Motor Kraftregelung, manuelles CLI |
| `one_motor_system_id` | `main_SystemIdentification.cpp` | 460800 | System-ID, Homing, binäre Telemetrie |
| `object_force` | `main_object_force.cpp` | 115200 | Reine Kraftmessung ohne Motor |
| `test_hx711` | `test_hx711.cpp` | 115200 | Komponententest HX711 |
| `test_anu78025` | `test_anu78025.cpp` | 115200 | Komponententest NAU7802 |
| `test_vl6180x` | `test_vl6180x.cpp` | 115200 | Komponententest VL6180X |
| `test_motor` | `test_motor.cpp` | 115200 | Komponententest MotorDriver |
| `test_config` | `test_config.cpp` | 115200 | Komponententest ConfigManager |
| `test_one_motor_force_control` | `test_one_motor_force_control.cpp` | 115200 | Einzelmotor PID-Test |

### Typischer Workflow

```bash
# System-ID Firmware flashen und Monitor öffnen
pio run -e one_motor_system_id -t upload
pio device monitor -b 460800

# Production Firmware
pio run -e production -t upload
pio device monitor -b 115200

# Nur kompilieren (kein Upload)
pio run -e one_motor_system_id
```

---

## Kommunikationsprotokoll

Vollständige Spezifikation: [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md)

### Python → ESP32: ASCII-Kommandos (`\n`-terminiert)

#### Production (`main.cpp`) @ 115200

```
set <id|all> <N>           # Kraft-Sollwert in Newton
start <id|all>             # PID-Regelung aktivieren
stop <id|all>              # Regelung deaktivieren
pid <id> <kp> <ki> <kd>    # PID-Parameter tunen
status                     # Status ausgeben
fast_print                 # 10Hz-Konsolenausgabe umschalten
```

#### System-ID (`main_SystemIdentification.cpp`) @ 460800

```
f <m|all> <N>              # Kraft-Sollwert (z.B. f 0 5.0)
start <m|all>              # PID-Kraftregelung starten
pid <m> <kp> <ki> <kd>     # PID tunen (z.B. pid 0 30 0.5 0)
v <m|all> <speed>          # Direkter Speed (-500…500, deaktiviert PID)
step <m> <speed> <ms>      # Kraft-limitierte Step-Response (blockierend)
stop [m]                   # Motor(en) stoppen
c <m|all>                  # Homing / Referenzfahrt
r <m|all>                  # Seil auf 0 mm zurückfahren
n <m|all>                  # Seiltracking auf 0 zurücksetzen
fl <N>                     # Force-Limit für Step-Response setzen
help                       # Befehlsübersicht
```

### ESP32 → Python: Binäre Telemetrie (~600 Hz)

**Status-Paket (22 Bytes)**

```
0xAA 0x55 | uint32 ts_us | float force0_N | float force1_N | float rope0_mm | float rope1_mm
```

```python
import struct
ts_us, f0, f1, r0, r1 = struct.unpack('<I ff ff', pkt[2:22])
```

**Step-End-Paket (3 Bytes)**

```
0xBB 0x66 | uint8 motor_index
```

---

## Python-Tooling

### Abhängigkeiten ([pyproject.toml](pyproject.toml))

```
pyserial, customtkinter, polars, matplotlib, scipy, pyqt6, pyqtgraph, mujoco
```

Ausführen mit:
```bash
uv run <script>.py
```

### Skripte

| Skript | Funktion |
|---|---|
| [live_monitor_simple.py](live_monitor_simple.py) | Echtzeit-Telemetrieanzeige + interaktive CLI-Kommandos (Terminal) |
| [system_identification.py](system_identification.py) | OOP-Datenerfassung; speichert Parquet-Dateien |
| [step_response_automation.py](step_response_automation.py) | Automatisierte Sprung-Antwort-Messungen |
| [main.py](main.py) | Tkinter/CustomTkinter GUI für manuelle Steuerung |
| [spirob_digital_twin.py](spirob_digital_twin.py) | MuJoCo-Simulation → Live-Hardware-Bridge |
| [analyze_step_response.py](analyze_step_response.py) | Auswertung gespeicherter Messungen (**⚠ noch Einmotor-Schema**) |

### Parquet-Datenformat (Dual-Motor)

| Spalte | Typ | Einheit |
|---|---|---|
| `timestamp_s` | float64 | s |
| `timestamp_us` | uint32 | µs |
| `force0_N` | float32 | N |
| `force1_N` | float32 | N |
| `rope0_mm` | float32 | mm |
| `rope1_mm` | float32 | mm |

---

## MuJoCo Digital Twin

**Datei:** [spirob_digital_twin.py](spirob_digital_twin.py)  
**Modell:** [spiral_chain.xml](spiral_chain.xml)

Öffnet den MuJoCo-Viewer mit dem SpiRob-Modell. Sobald Aktuator-Slider in der GUI bewegt werden, werden die resultierenden Kraft-Sollwerte (Newton) direkt an die echte Hardware gesendet.

```
MuJoCo GUI  →  data.ctrl[0/1] (N)  →  f 0 <N> / f 1 <N>  →  ESP32 PID
                                                         ↑
                                                  start all beim Start
```

**Konfigurierbare Konstanten (oben im Skript):**

| Variable | Standard | Bedeutung |
|---|---|---|
| `PORT` | `/dev/ttyUSB0` | Serieller Port |
| `BAUDRATE` | `460800` | Baudrate |
| `SEND_HZ` | `50` | Max. Kommandorate an ESP32 |
| `FORCE_DEADBAND` | `0.05 N` | Minimale Änderung vor neuem Befehl |

Beim Schließen des Viewers wird automatisch `stop` gesendet.

---

## Wichtige Konventionen & Fallstricke

1. **Motor-Index vs. Servo-ID**: CLI/Python nutzen Indices `0`/`1`; `MotorDriver`-Konstruktor erwartet Servo-ID `1`/`2`. Nicht verwechseln.
2. **Enum-Verwechslung**: `ForceControlLoop::ControlMode` (PID/Speed/Manual) ≠ `MotorDriver::DriverMode` (Position/Wheel). Kontext prüfen vor Cast.
3. **Sensor-Einheiten**: `ForceSensor` speichert intern kg; `getForce()` gibt **Newton** zurück.
4. **Non-blocking ADC**: `ForceSensor::update()` im selben Loop-Zyklus vor `getForce()` aufrufen.
5. **Endstop im Blocking-Loop**: `isRawTriggered()` statt `isTriggered()` für sofortigen Notstopp in Homing/Step-Response.
6. **Kalibrierung Motor 1**: `FORCE_SCALE_1` / `FORCE_OFFSET_1` in `main.cpp` sind Platzhalter – vor Feldeinsatz kalibrieren.
7. **`analyze_step_response.py`**: Erwartet noch das alte Einmotor-Schema; muss auf Dual-Motor-Parquet-Spalten migriert werden.
8. **Neues Build-Filter-Prinzip**: Beim Hinzufügen neuer `.cpp`-Dateien müssen alle relevanten Umgebungen in [platformio.ini](platformio.ini) angepasst werden.
