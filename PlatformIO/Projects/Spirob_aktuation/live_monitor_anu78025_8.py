import argparse
import struct
import time
from collections import deque

from PyQt6 import QtCore, QtWidgets
import pyqtgraph as pg
import serial

HEADER = b"\xAA\x55"
SENSOR_COUNT = 8
FRAME_SIZE = 2 + 4 + SENSOR_COUNT * 4


def read_frames(ser):
    buffer = bytearray()
    while True:
        data = ser.read(1024)
        if data:
            buffer.extend(data)
        else:
            time.sleep(0.001)
            continue

        while True:
            idx = buffer.find(HEADER)
            if idx < 0:
                buffer.clear()
                break
            if len(buffer) < idx + FRAME_SIZE:
                if idx > 0:
                    del buffer[:idx]
                break

            frame = buffer[idx:idx + FRAME_SIZE]
            del buffer[:idx + FRAME_SIZE]

            timestamp_us = struct.unpack_from("<I", frame, 2)[0]
            forces = struct.unpack_from("<" + "f" * SENSOR_COUNT, frame, 6)
            yield timestamp_us, forces


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=False, default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--window", type=float, default=10.0, help="seconds")
    parser.add_argument("--mode", choices=["plot", "print"], default="print")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.01)

    if args.mode == "print":
        for ts_us, forces in read_frames(ser):
            values = " ".join(f"{f:8.3f}" for f in forces)
            print(f"{ts_us:10d} {values}")
        return

    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(title="Force Sensors")
    win.resize(1200, 600)
    plot = win.addPlot(title="Force (N)")
    plot.addLegend(offset=(10, 10))
    plot.setLabel("bottom", "t (s)")
    plot.setLabel("left", "Force (N)")
    plot.showGrid(x=True, y=True)

    maxlen = int(args.window * 200)  # approx 200 Hz default plotting
    t_buf = deque(maxlen=maxlen)
    y_buf = [deque(maxlen=maxlen) for _ in range(SENSOR_COUNT)]
    curves = [plot.plot(pen=pg.intColor(i), name=f"S{i}") for i in range(SENSOR_COUNT)]

    t0 = None
    buffer = bytearray()

    def process_serial():
        nonlocal t0
        data = ser.read(4096)
        if data:
            buffer.extend(data)

        while True:
            idx = buffer.find(HEADER)
            if idx < 0:
                buffer.clear()
                break
            if len(buffer) < idx + FRAME_SIZE:
                if idx > 0:
                    del buffer[:idx]
                break

            frame = buffer[idx:idx + FRAME_SIZE]
            del buffer[:idx + FRAME_SIZE]

            ts_us = struct.unpack_from("<I", frame, 2)[0]
            forces = struct.unpack_from("<" + "f" * SENSOR_COUNT, frame, 6)

            if t0 is None:
                t0 = ts_us
            t_s = (ts_us - t0) / 1_000_000.0

            t_buf.append(t_s)
            for i in range(SENSOR_COUNT):
                y_buf[i].append(forces[i])

    def update_plot():
        process_serial()
        if not t_buf:
            return
        t_list = list(t_buf)
        for i in range(SENSOR_COUNT):
            curves[i].setData(t_list, list(y_buf[i]))
        plot.setXRange(max(0, t_list[-1] - args.window), t_list[-1], padding=0)

    win.show()

    timer = QtCore.QTimer()
    timer.timeout.connect(update_plot)
    timer.start(20)

    app.exec()


if __name__ == "__main__":
    main()
