#!/usr/bin/env python3
"""
SpiRob Digital Twin – MuJoCo ↔ Hardware Bridge

Opens the MuJoCo viewer. When you drag tendon actuator sliders in the GUI,
the resulting force setpoints (Newtons) are forwarded to the real ESP32
hardware via serial force-control commands.

Requires the `one_motor_system_id` firmware (460800 baud, binary telemetry).
"""

import mujoco as mj
import mujoco.viewer as viewer
import time
import struct
import numpy as np
import serial
import tkinter as tk

# ── Serial config (same as live_monitor_simple.py) ──────────────────────────
PORT = "/dev/ttyUSB0"
BAUDRATE = 460800
HEADER = b"\xaa\x55"
STEP_END = b"\xbb\x66"
STRUCT_FMT = "<I ff ff"  # uint32 ts_us, float force[2], float rope_mm[2]
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)  # 20

SEND_HZ = 50  # max command rate to ESP32
SEND_INTERVAL = 1.0 / SEND_HZ
FORCE_DEADBAND = 0.05  # N – ignore changes smaller than this


# ── Helpers ──────────────────────────────────────────────────────────────────

def send_cmd(ser: serial.Serial, cmd: str) -> None:
    """Send an ASCII command (auto-appends newline)."""
    ser.write((cmd + "\n").encode("ascii"))


def drain_telemetry(ser: serial.Serial):
    """Read all pending binary packets; return latest status tuple or None."""
    latest = None
    while ser.in_waiting >= 2 + STRUCT_SIZE:
        b0 = ser.read(1)
        if b0 == b"\xaa":
            b1 = ser.read(1)
            if b1 == b"\x55":
                pkt = ser.read(STRUCT_SIZE)
                if len(pkt) == STRUCT_SIZE:
                    latest = struct.unpack(STRUCT_FMT, pkt)
        elif b0 == b"\xbb":
            ser.read(2)  # step-end: marker byte + motor index
    return latest


# ── PID tuning GUI ───────────────────────────────────────────────────────────
# Live Kp/Ki/Kd sliders so gains can be tried out without reflashing the ESP32.
# main_SystemIdentification.cpp's "pid" command needs a specific motor (0 or 1)
# - unlike production's "pid all ...", there's no "all" here - so "Beide" just
# sends the command twice.

PID_KP_RANGE = (0.0, 30.0)
PID_KI_RANGE = (0.0, 1.0)
PID_KD_RANGE = (0.0, 5.0)
PID_GUI_SEND_HZ = 5  # max rate for "pid ..." commands while dragging a slider
PID_GUI_SEND_INTERVAL = 1.0 / PID_GUI_SEND_HZ

# Initial slider positions - mirror the firmware's current DEFAULT_KP/KI/KD in
# main_SystemIdentification.cpp, but are otherwise independent (dragging a
# slider just sends a new "pid" command, it doesn't read the value back).
PID_KP_INIT = 10.0
PID_KI_INIT = 0.05
PID_KD_INIT = 0.2


class PidTuningGUI:
    """Small always-on-top slider panel. Call poll() once per simulation frame -
    it pumps Tk's event loop cooperatively instead of blocking on mainloop()."""

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self._closed = False
        self._last_sent = None
        self._last_send_t = 0.0

        self.root = tk.Tk()
        self.root.title("PID Tuning")
        self.root.attributes("-topmost", True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.motor_var = tk.StringVar(value="Beide")
        tk.OptionMenu(self.root, self.motor_var, "Motor 0", "Motor 1", "Beide").grid(
            row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 2)
        )

        self.kp_var = self._add_slider("Kp", 1, PID_KP_RANGE, 0.01, PID_KP_INIT)
        self.ki_var = self._add_slider("Ki", 2, PID_KI_RANGE, 0.001, PID_KI_INIT)
        self.kd_var = self._add_slider("Kd", 3, PID_KD_RANGE, 0.01, PID_KD_INIT)

    def _add_slider(self, label, row, value_range, resolution, default):
        tk.Label(self.root, text=label).grid(row=row, column=0, sticky="w", padx=6)
        var = tk.DoubleVar(value=default)
        tk.Scale(
            self.root, variable=var, from_=value_range[0], to=value_range[1],
            resolution=resolution, orient=tk.HORIZONTAL, length=260,
        ).grid(row=row, column=1, padx=6, pady=2)
        return var

    def _on_close(self):
        self._closed = True
        self.root.destroy()

    def poll(self) -> bool:
        """Returns False once the window has been closed (nothing left to do)."""
        if self._closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self._closed = True
            return False

        now = time.time()
        if now - self._last_send_t < PID_GUI_SEND_INTERVAL:
            return True

        gains = (self.kp_var.get(), self.ki_var.get(), self.kd_var.get())
        if gains != self._last_sent:
            motor_ids = {"Motor 0": [0], "Motor 1": [1], "Beide": [0, 1]}[self.motor_var.get()]
            kp, ki, kd = gains
            for m in motor_ids:
                send_cmd(self.ser, f"pid {m} {kp:.3f} {ki:.4f} {kd:.3f}")
            self._last_sent = gains
            self._last_send_t = now
        return True


# ── MuJoCo model setup ──────────────────────────────────────────────────────

spec = mj.MjSpec.from_file("spiral_chain.xml")

cylinder = spec.worldbody.add_body(name="cylinder", pos=[-0.11, 0.00, 0.11])
cylinder.add_geom(
    name="cyl_geom",
    type=mj.mjtGeom.mjGEOM_CYLINDER,
    size=[0.05, 0.15, 0.05],
    euler=[90, 0, 0],
    rgba=[0.2, 0.8, 0.5, 1],
    density=1000,
)

spirob = spec.body("seg_0")
model = spec.compile()
data = mj.MjData(model)
print("MuJoCo-Modell geladen.")

data.ctrl[0] = -5.0
data.ctrl[1] = -5.0


# ── Serial connection ───────────────────────────────────────────────────────

ser = serial.Serial(PORT, BAUDRATE, timeout=0.001)
time.sleep(0.1)
ser.reset_input_buffer()
print(f"Verbunden: {PORT} @ {BAUDRATE}")

send_cmd(ser, "start all")
print("Kraftregelung gestartet (start all)")

pid_gui = PidTuningGUI(ser)


# ── Main loop ───────────────────────────────────────────────────────────────

prev_f = [None, None]
last_send_t = 0.0

try:
    with mj.viewer.launch_passive(model, data) as v:
        t0 = time.time()
        while v.is_running():
            step_start = time.time()

            # 1) Read actuator ctrl values from GUI (Newtons)
            f0 = -1 * float(data.ctrl[0])
            f1 = -1 * float(data.ctrl[1])

            # 2) Forward to hardware (throttled, with dead-band)
            now = time.time()
            if now - last_send_t >= SEND_INTERVAL:
                if prev_f[0] is None or abs(f0 - prev_f[0]) > FORCE_DEADBAND:
                    send_cmd(ser, f"f 0 {f0:.2f}")
                    prev_f[0] = f0
                if prev_f[1] is None or abs(f1 - prev_f[1]) > FORCE_DEADBAND:
                    send_cmd(ser, f"f 1 {f1:.2f}")
                    prev_f[1] = f1
                last_send_t = now

            # 2b) Pump the PID tuning GUI (cooperative, non-blocking)
            pid_gui.poll()

            # 3) Drain hardware telemetry (prevents serial buffer overflow)
            hw = drain_telemetry(ser)
            if hw:
                _ts, hf0, hf1, hr0, hr1 = hw
                print(
                    f"\rHW: {hf0:6.2f}N {hf1:6.2f}N | "
                    f"{hr0:7.1f}mm {hr1:7.1f}mm  ",
                    end="",
                    flush=True,
                )

            # 4) Step simulation & sync viewer
            mj.mj_step(model, data)
            v.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

finally:
    send_cmd(ser, "stop")
    print("\nMotoren gestoppt.")
    ser.close()
    print("Serielle Verbindung geschlossen.")





