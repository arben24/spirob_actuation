import argparse
import struct
import time
import math
from collections import deque

from PyQt6 import QtCore, QtWidgets, QtGui
import serial

HEADER = b"\xAA\x55"
SENSOR_COUNT = 8
FRAME_SIZE = 2 + 4 + SENSOR_COUNT * 4

class SensorCircleWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.forces = [0.0] * SENSOR_COUNT
        self.max_force = 10.0 # Initiale Skalierung (wird dynamisch angepasst)
        self.min_force = -10.0
        self.setGeometry(0, 0, 800, 800)
        self.setWindowTitle("Force Sensors - Circular Layout")

    def update_forces(self, forces):
        self.forces = forces
        # Dynamisches Anpassen der Min/Max-Skalierung, falls Kräfte stark abweichen
        for f in forces:
            if f > self.max_force:
                self.max_force = f
            if f < self.min_force:
                self.min_force = f
        self.update()

    def get_color(self, value):
        # Mappt den Wert auf eine Farbe. Min_force (Blau) -> 0 -> Max_force (Rot)
        range_val = max(1e-3, self.max_force - self.min_force)
        norm = (value - self.min_force) / range_val
        norm = max(0.0, min(1.0, norm))
        
        # Farbverlauf von Blau (Hue: 240) nach Rot (Hue: 0)
        hue = 240 - norm * 240
        color = QtGui.QColor()
        color.setHsv(int(hue), 200, 200)
        return color

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        # Hintergrund zeichnen
        painter.fillRect(self.rect(), QtGui.QColor("#2b2b2b"))

        rect = self.rect()
        center = rect.center()
        
        # Bestimme den maximalen Radius so, dass alles in Fenster passt
        radius = min(rect.width(), rect.height()) / 2.5
        sensor_radius = radius / 4.0

        font = painter.font()
        font.setPointSize(max(10, int(sensor_radius / 2.5)))
        font.setBold(True)
        painter.setFont(font)

        for i in range(SENSOR_COUNT):
            # 0 ist oben (12 Uhr entspricht -90 Grad)
            # Danach im Uhrzeigersinn (+i * 360/8)
            angle_deg = -90 + i * (360 / SENSOR_COUNT)
            angle_rad = math.radians(angle_deg)
            
            cx = center.x() + radius * math.cos(angle_rad)
            cy = center.y() + radius * math.sin(angle_rad)

            val = self.forces[i]
            color = self.get_color(val)
            
            # Zeichne den Sensor-Kreis
            painter.setBrush(QtGui.QBrush(color))
            painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.black, 3))
            
            painter.drawEllipse(QtCore.QPointF(cx, cy), sensor_radius, sensor_radius)

            # Zeichne den Kraft-Wert in die Mitte des Sensors
            text = f"{val:.1f} N"
            text_rect = QtCore.QRectF(cx - sensor_radius, cy - sensor_radius, sensor_radius * 2, sensor_radius * 2)
            
            # Setze Textfarbe auf Weiß für besseren Kontrast
            painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white))
            painter.drawText(text_rect, QtCore.Qt.AlignmentFlag.AlignCenter, text)

            # Zeichne die Sensor-Nummer (0-7) außerhalb des Kreises
            id_cx = center.x() + (radius + sensor_radius * 1.5) * math.cos(angle_rad)
            id_cy = center.y() + (radius + sensor_radius * 1.5) * math.sin(angle_rad)
            id_rect = QtCore.QRectF(id_cx - sensor_radius, id_cy - sensor_radius, sensor_radius * 2, sensor_radius * 2)
            
            # Helle Schriftfarbe für dunklen Hintergrund
            painter.setPen(QtGui.QPen(QtGui.QColor("#e0e0e0")))
            painter.drawText(id_rect, QtCore.Qt.AlignmentFlag.AlignCenter, f"S{i}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=False, default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--fps", type=int, default=30, help="Updates per second")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.01)

    # Tare all sensors at startup
    print("Sende Tare-Befehl 't a' an die Sensoren...")
    ser.write(b"t a\n")
    ser.flush()
    time.sleep(0.1) # Kurze Pause, damit der Befehl sicher verarbeitet wird

    app = QtWidgets.QApplication([])
    widget = SensorCircleWidget()
    widget.show()

    buffer = bytearray()
    
    # Letzte bekannte Kraftwerte
    latest_forces = [0.0] * SENSOR_COUNT

    def process_serial():
        nonlocal latest_forces
        
        # Serial auslesen, um Buffer nicht vollaufen zu lassen
        data = ser.read(4096)
        if data:
            buffer.extend(data)

        frames_found = False
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

            forces = struct.unpack_from("<" + "f" * SENSOR_COUNT, frame, 6)
            latest_forces = list(forces)
            frames_found = True
            
        if frames_found:
            widget.update_forces(latest_forces)

    # Ein Timer sorgt regelmäßig für das Auslesen der seriellen Daten und löst den Redraw aus.
    # Da reines Auslesen sehr schnell ist, können wir den Timer im GUI Thread laufen lassen.
    timer = QtCore.QTimer()
    timer.timeout.connect(process_serial)
    timer.start(int(1000 / args.fps))

    app.exec()

if __name__ == "__main__":
    main()
