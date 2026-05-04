"""
╔══════════════════════════════════════════════════════════════════╗
║          UAV GROUND STATION — Professional Edition               ║
║          MAVLink-based | PyQt5 | Real-time Telemetry             ║
╚══════════════════════════════════════════════════════════════════╝

Requirements:
    pip install PyQt5 pymavlink pyserial pyqtgraph numpy

Run:
    python uav_ground_station.py
"""

import sys
import math
import time
import threading
import random
from datetime import datetime
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QTabWidget, QFrame, QSplitter, QTextEdit, QGroupBox,
    QSlider, QCheckBox, QSpinBox, QDoubleSpinBox, QProgressBar,
    QStatusBar, QAction, QMenu, QToolBar, QDockWidget, QTableWidget,
    QTableWidgetItem, QHeaderView, QSizePolicy, QScrollArea,
    QDialog, QDialogButtonBox, QFormLayout, QMessageBox
)
from PyQt5.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QPointF, QRectF, QSize,
    QPropertyAnimation, QEasingCurve, pyqtProperty
)
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QLinearGradient,
    QRadialGradient, QPainterPath, QPolygonF, QPixmap, QIcon,
    QFontDatabase, QPalette, QConicalGradient, QTransform
)

import numpy as np

# ─────────────────────────────────────────────────────────────────
#  COLOUR PALETTE
# ─────────────────────────────────────────────────────────────────
class Theme:
    BG_DARK      = QColor("#0A0E14")
    BG_PANEL     = QColor("#0D1117")
    BG_CARD      = QColor("#161B22")
    BG_CARD2     = QColor("#1C2333")
    BORDER       = QColor("#21262D")
    BORDER_ACCENT= QColor("#30363D")

    TEXT_PRIMARY = QColor("#E6EDF3")
    TEXT_SECOND  = QColor("#8B949E")
    TEXT_MUTED   = QColor("#484F58")

    ACCENT_CYAN  = QColor("#58A6FF")
    ACCENT_GREEN = QColor("#3FB950")
    ACCENT_ORANGE= QColor("#D29922")
    ACCENT_RED   = QColor("#F85149")
    ACCENT_PURPLE= QColor("#BC8CFF")
    ACCENT_BLUE  = QColor("#1F6FEB")

    WARNING      = QColor("#D29922")
    DANGER       = QColor("#F85149")
    SUCCESS      = QColor("#3FB950")
    INFO         = QColor("#58A6FF")

    HUD_SKY      = QColor("#0D2B55")
    HUD_GROUND   = QColor("#3D2B1F")
    HUD_LINE     = QColor("#00FF88")
    HUD_TEXT     = QColor("#00FF88")

# ─────────────────────────────────────────────────────────────────
#  TELEMETRY DATA MODEL
# ─────────────────────────────────────────────────────────────────
class TelemetryData:
    def __init__(self):
        # Attitude
        self.roll    = 0.0   # deg
        self.pitch   = 0.0   # deg
        self.yaw     = 0.0   # deg
        # GPS
        self.lat     = 44.4949   # Bologna
        self.lon     = 11.3426
        self.alt     = 0.0       # m
        self.rel_alt = 0.0
        self.fix_type= 3
        self.sats    = 14
        self.hdop    = 0.9
        # Velocity
        self.vx      = 0.0   # m/s
        self.vy      = 0.0
        self.vz      = 0.0
        self.airspeed= 0.0
        self.groundspeed = 0.0
        # Power
        self.battery_voltage  = 16.8  # V
        self.battery_current  = 0.0   # A
        self.battery_remaining= 100   # %
        self.battery_consumed = 0.0   # mAh
        # System
        self.flight_mode = "STABILIZE"
        self.armed       = False
        self.autopilot   = "ArduPilot"
        self.mav_type    = "Quadcopter"
        self.system_id   = 1
        # RC
        self.rssi        = 100
        self.channels    = [1500]*8
        # Sensors
        self.baro_alt    = 0.0
        self.baro_press  = 101325.0
        self.temp        = 25.0
        self.accel_x     = 0.0
        self.accel_y     = 0.0
        self.accel_z     = -9.81
        # IMU
        self.gyro_x      = 0.0
        self.gyro_y      = 0.0
        self.gyro_z      = 0.0
        # Mag
        self.mag_x       = 0.0
        self.mag_y       = 0.0
        self.mag_z       = 0.0
        # Vibration
        self.vibe_x      = 0.0
        self.vibe_y      = 0.0
        self.vibe_z      = 0.0
        # EKF
        self.ekf_flags   = 0x1F
        # Mission
        self.wp_num      = 0
        self.wp_dist     = 0.0
        self.mission_total = 0
        # Timing
        self.timestamp   = time.time()
        self.uptime      = 0
        self.message_rate= 0
        # Connection
        self.connected   = False
        self.packet_loss = 0.0

    def history_fields(self):
        return ['roll','pitch','yaw','alt','airspeed',
                'groundspeed','battery_voltage','battery_current',
                'battery_remaining','baro_press','temp','rssi',
                'vx','vy','vz','vibe_x','vibe_y','vibe_z']

# ─────────────────────────────────────────────────────────────────
#  MAVLINK WORKER THREAD
# ─────────────────────────────────────────────────────────────────
class MAVLinkWorker(QThread):
    telemetry_updated = pyqtSignal(object)
    message_received  = pyqtSignal(str, str)   # (level, text)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.running  = False
        self.connected= False
        self.mavconn  = None
        self.telem    = TelemetryData()
        self._sim_mode= True   # fallback when no hw connected
        self._sim_t   = 0.0

    # ── Public API ─────────────────────────────────────────
    def connect(self, connection_string: str, baud: int = 57600):
        try:
            from pymavlink import mavutil
            self.mavconn = mavutil.mavlink_connection(
                connection_string, baud=baud, autoreconnect=True)
            self.mavconn.wait_heartbeat(timeout=5)
            self.connected = True
            self._sim_mode = False
            self.connection_status.emit(True, f"Connected to {connection_string}")
            self.message_received.emit("INFO",
                f"Heartbeat received — system {self.mavconn.target_system}")
            self._request_streams()
        except ImportError:
            self._sim_mode = True
            self.connected = True
            self.connection_status.emit(True, "SIMULATION MODE (pymavlink not installed)")
            self.message_received.emit("WARN",
                "pymavlink not found — running in simulation mode")
        except Exception as e:
            self.connected = False
            self.connection_status.emit(False, str(e))
            self.message_received.emit("ERROR", str(e))

    def disconnect(self):
        self.connected = False
        self._sim_mode = True
        if self.mavconn:
            try: self.mavconn.close()
            except: pass
            self.mavconn = None
        self.connection_status.emit(False, "Disconnected")

    def send_command(self, cmd: int, p1=0,p2=0,p3=0,p4=0,p5=0,p6=0,p7=0):
        if self._sim_mode: return
        try:
            from pymavlink import mavutil
            self.mavconn.mav.command_long_send(
                self.mavconn.target_system,
                self.mavconn.target_component,
                cmd, 0, p1,p2,p3,p4,p5,p6,p7)
        except Exception as e:
            self.message_received.emit("ERROR", f"Command failed: {e}")

    def arm(self, arm: bool):
        from pymavlink import mavutil
        val = 1.0 if arm else 0.0
        self.send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, val)

    # ── Internal ────────────────────────────────────────────
    def _request_streams(self):
        if self.mavconn is None: return
        try:
            from pymavlink import mavutil
            for stream_id, rate in [
                (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),
                (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 5),
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 20),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 5),
            ]:
                self.mavconn.mav.request_data_stream_send(
                    self.mavconn.target_system,
                    self.mavconn.target_component,
                    stream_id, rate, 1)
        except: pass

    def _simulate(self):
        t = self._sim_t
        self._sim_t += 0.05
        d = self.telem
        d.connected = True
        d.roll    = 15 * math.sin(t * 0.3)
        d.pitch   = 8  * math.sin(t * 0.2 + 1)
        d.yaw     = (t * 20) % 360
        d.rel_alt = 50 + 20 * math.sin(t * 0.1)
        d.alt     = d.rel_alt + 100
        d.airspeed= 12 + 3 * math.sin(t * 0.15)
        d.groundspeed = d.airspeed * 0.95
        d.vx = d.groundspeed * math.cos(math.radians(d.yaw))
        d.vy = d.groundspeed * math.sin(math.radians(d.yaw))
        d.vz = -d.vz * 0.9 + random.gauss(0, 0.05)
        d.battery_voltage   = 16.8 - t * 0.002
        d.battery_current   = 15 + 5 * math.sin(t * 0.4)
        d.battery_remaining = max(0, int(100 - t * 0.1))
        d.battery_consumed += d.battery_current * (1/20) / 3.6
        d.baro_alt  = d.rel_alt + random.gauss(0, 0.1)
        d.baro_press= 101325 - d.alt * 12
        d.temp      = 25 + random.gauss(0, 0.05)
        d.rssi      = int(85 + 10 * math.sin(t * 0.07))
        d.accel_x   = random.gauss(0, 0.05)
        d.accel_y   = random.gauss(0, 0.05)
        d.accel_z   = -9.81 + random.gauss(0, 0.02)
        d.gyro_x    = math.radians(d.roll  * 0.1) + random.gauss(0,0.001)
        d.gyro_y    = math.radians(d.pitch * 0.1) + random.gauss(0,0.001)
        d.gyro_z    = math.radians(20) + random.gauss(0,0.001)
        d.vibe_x    = abs(random.gauss(0, 2))
        d.vibe_y    = abs(random.gauss(0, 2))
        d.vibe_z    = abs(random.gauss(0, 2))
        d.lat      += 0.000001 * math.cos(math.radians(d.yaw))
        d.lon      += 0.000001 * math.sin(math.radians(d.yaw))
        d.sats      = 14 + random.randint(-1, 1)
        d.uptime    = int(t * 20)
        d.flight_mode = "AUTO" if t > 30 else "STABILIZE"
        d.armed     = True
        d.timestamp = time.time()

    def _parse_mavlink(self):
        try:
            msg = self.mavconn.recv_match(blocking=False)
            if msg is None: return
            t = msg.get_type()
            d = self.telem
            if t == 'HEARTBEAT':
                d.armed = bool(msg.base_mode & 128)
                modes = {0:"STABILIZE",2:"ALT_HOLD",3:"AUTO",4:"GUIDED",
                         5:"LOITER",6:"RTL",7:"CIRCLE",9:"LAND",16:"POSHOLD"}
                d.flight_mode = modes.get(msg.custom_mode, str(msg.custom_mode))
                d.connected = True
            elif t == 'ATTITUDE':
                d.roll  = math.degrees(msg.roll)
                d.pitch = math.degrees(msg.pitch)
                d.yaw   = math.degrees(msg.yaw) % 360
            elif t == 'GLOBAL_POSITION_INT':
                d.lat     = msg.lat / 1e7
                d.lon     = msg.lon / 1e7
                d.alt     = msg.alt / 1000.0
                d.rel_alt = msg.relative_alt / 1000.0
                d.vx      = msg.vx / 100.0
                d.vy      = msg.vy / 100.0
                d.vz      = msg.vz / 100.0
                d.groundspeed = math.hypot(d.vx, d.vy)
            elif t == 'SYS_STATUS':
                d.battery_voltage   = msg.voltage_battery / 1000.0
                d.battery_current   = msg.current_battery / 100.0
                d.battery_remaining = msg.battery_remaining
                d.packet_loss       = msg.drop_rate_comm / 100.0
            elif t == 'GPS_RAW_INT':
                d.fix_type = msg.fix_type
                d.sats     = msg.satellites_visible
                d.hdop     = msg.eph / 100.0
            elif t == 'VFR_HUD':
                d.airspeed    = msg.airspeed
                d.groundspeed = msg.groundspeed
                d.baro_alt    = msg.alt
            elif t == 'SCALED_PRESSURE':
                d.baro_press = msg.press_abs * 100
                d.temp       = msg.temperature / 100.0
            elif t == 'RAW_IMU':
                d.accel_x = msg.xacc / 1000.0 * 9.81
                d.accel_y = msg.yacc / 1000.0 * 9.81
                d.accel_z = msg.zacc / 1000.0 * 9.81
                d.gyro_x  = msg.xgyro / 1000.0
                d.gyro_y  = msg.ygyro / 1000.0
                d.gyro_z  = msg.zgyro / 1000.0
                d.mag_x   = msg.xmag
                d.mag_y   = msg.ymag
                d.mag_z   = msg.zmag
            elif t == 'VIBRATION':
                d.vibe_x = msg.vibration_x
                d.vibe_y = msg.vibration_y
                d.vibe_z = msg.vibration_z
            elif t == 'RC_CHANNELS':
                d.rssi = msg.rssi
                d.channels = [msg.chan1_raw,msg.chan2_raw,msg.chan3_raw,msg.chan4_raw,
                               msg.chan5_raw,msg.chan6_raw,msg.chan7_raw,msg.chan8_raw]
            elif t == 'MISSION_CURRENT':
                d.wp_num = msg.seq
            elif t == 'NAV_CONTROLLER_OUTPUT':
                d.wp_dist = msg.wp_dist
            d.timestamp = time.time()
        except Exception as e:
            self.message_received.emit("WARN", f"Parse error: {e}")

    def run(self):
        self.running = True
        _last_emit = 0
        _pkt_count = 0
        _rate_t    = time.time()
        while self.running:
            if self.connected:
                if self._sim_mode:
                    self._simulate()
                    time.sleep(0.05)
                else:
                    self._parse_mavlink()
                _pkt_count += 1
                now = time.time()
                if now - _rate_t >= 1.0:
                    self.telem.message_rate = _pkt_count
                    _pkt_count = 0
                    _rate_t = now
                if now - _last_emit >= 0.05:  # 20 Hz UI update
                    self.telemetry_updated.emit(self.telem)
                    _last_emit = now
            else:
                time.sleep(0.1)

    def stop(self):
        self.running = False
        self.wait()

# ─────────────────────────────────────────────────────────────────
#  ARTIFICIAL HORIZON (HUD)
# ─────────────────────────────────────────────────────────────────
class ArtificialHorizon(QWidget):
    def __init__(self):
        super().__init__()
        self.roll  = 0.0
        self.pitch = 0.0
        self.setMinimumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_attitude(self, roll, pitch):
        self.roll  = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        sz   = min(w, h)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        cx, cy = w / 2, h / 2
        r  = sz / 2 - 4

        # Clip to circle
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), r, r)
        painter.setClipPath(clip)

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(-self.roll)

        pitch_px = self.pitch * (sz / 90.0)

        # Sky
        sky_rect = QRectF(-r, -r * 2 + pitch_px, r * 2, r * 2)
        sky_grad = QLinearGradient(0, -r, 0, pitch_px)
        sky_grad.setColorAt(0.0, QColor("#0A1929"))
        sky_grad.setColorAt(1.0, QColor("#1565C0"))
        painter.fillRect(sky_rect, sky_grad)

        # Ground
        gnd_rect = QRectF(-r, pitch_px, r * 2, r * 2)
        gnd_grad = QLinearGradient(0, pitch_px, 0, r)
        gnd_grad.setColorAt(0.0, QColor("#5D4037"))
        gnd_grad.setColorAt(1.0, QColor("#3E2723"))
        painter.fillRect(gnd_rect, gnd_grad)

        # Horizon line
        pen = QPen(QColor("#00E5FF"), 2)
        painter.setPen(pen)
        painter.drawLine(QPointF(-r, pitch_px), QPointF(r, pitch_px))

        # Pitch ladders
        painter.setPen(QPen(Qt.white, 1))
        painter.setFont(QFont("Courier New", 7))
        for deg in range(-30, 31, 5):
            if deg == 0: continue
            y = pitch_px - deg * (sz / 90.0)
            l = r * 0.25 if deg % 10 == 0 else r * 0.12
            painter.drawLine(QPointF(-l, y), QPointF(l, y))
            if deg % 10 == 0:
                painter.drawText(QPointF(l + 4, y + 4), str(abs(deg)))
                painter.drawText(QPointF(-l - 20, y + 4), str(abs(deg)))

        painter.restore()
        painter.setClipping(False)

        # Roll arc & ticks
        painter.translate(cx, cy)
        painter.setPen(QPen(QColor("#00E5FF"), 1))
        from PyQt5.QtCore import QRect
        arc_rect = QRect(int(-r), int(-r), int(r*2), int(r*2))
        painter.drawArc(arc_rect, 30 * 16, 120 * 16)

        for angle in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            rad = math.radians(angle - 90)
            tick = 10 if angle % 30 == 0 else 5
            x1 = (r - tick) * math.cos(rad)
            y1 = (r - tick) * math.sin(rad)
            x2 = r * math.cos(rad)
            y2 = r * math.sin(rad)
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Roll pointer
        painter.save()
        painter.rotate(-self.roll)
        painter.setPen(QPen(QColor("#00E5FF"), 2))
        poly = QPolygonF([
            QPointF(0, -r + 2),
            QPointF(-8, -r + 18),
            QPointF(8,  -r + 18)
        ])
        painter.setBrush(QBrush(QColor("#00E5FF")))
        painter.drawPolygon(poly)
        painter.restore()

        # Fixed aircraft symbol
        painter.setPen(QPen(QColor("#FFD600"), 2.5))
        painter.drawLine(QPointF(-40, 0), QPointF(-15, 0))
        painter.drawLine(QPointF(-15, 0), QPointF(0, 8))
        painter.drawLine(QPointF(0, 8), QPointF(15, 0))
        painter.drawLine(QPointF(15, 0), QPointF(40, 0))
        painter.drawLine(QPointF(0, 8), QPointF(0, -10))

        # Circle border
        painter.setPen(QPen(Theme.BORDER_ACCENT, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(0, 0), r, r)

        # Labels
        painter.setFont(QFont("Courier New", 8, QFont.Bold))
        painter.setPen(QPen(QColor("#00E5FF")))
        painter.drawText(QPointF(-r - 0, -r + 14),
                         f"R {self.roll:+.1f}°  P {self.pitch:+.1f}°")

# ─────────────────────────────────────────────────────────────────
#  COMPASS ROSE
# ─────────────────────────────────────────────────────────────────
class CompassRose(QWidget):
    def __init__(self):
        super().__init__()
        self.heading = 0.0
        self.setMinimumSize(180, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_heading(self, yaw):
        self.heading = yaw
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        sz   = min(w, h)
        r    = sz / 2 - 6
        cx, cy = w/2, h/2
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(cx, cy)

        # Background
        bg_grad = QRadialGradient(0, 0, r)
        bg_grad.setColorAt(0.0, QColor("#1C2333"))
        bg_grad.setColorAt(1.0, QColor("#0D1117"))
        painter.setBrush(QBrush(bg_grad))
        painter.setPen(QPen(Theme.BORDER_ACCENT, 1))
        painter.drawEllipse(QPointF(0,0), r, r)

        painter.rotate(-self.heading)

        # Degree ticks
        for deg in range(0, 360, 5):
            rad = math.radians(deg - 90)
            big = (deg % 45 == 0)
            med = (deg % 10 == 0)
            tick = 14 if big else (9 if med else 5)
            col  = QColor("#58A6FF") if big else QColor("#484F58")
            wid  = 2 if big else 1
            x1 = (r - tick) * math.cos(rad)
            y1 = (r - tick) * math.sin(rad)
            x2 = r * math.cos(rad)
            y2 = r * math.sin(rad)
            painter.setPen(QPen(col, wid))
            painter.drawLine(QPointF(x1,y1), QPointF(x2,y2))

        # Cardinal labels
        labels = {0: 'N', 90: 'E', 180: 'S', 270: 'W',
                  45:'NE', 135:'SE', 225:'SW', 315:'NW'}
        for deg, label in labels.items():
            rad = math.radians(deg - 90)
            lx = (r - 26) * math.cos(rad)
            ly = (r - 26) * math.sin(rad)
            big = label in ('N','S','E','W')
            color = QColor("#F85149") if label == 'N' else QColor("#E6EDF3" if big else "#8B949E")
            font = QFont("Courier New", 9 if big else 7, QFont.Bold if big else QFont.Normal)
            painter.setPen(QPen(color))
            painter.setFont(font)
            painter.save()
            painter.translate(lx, ly)
            painter.rotate(self.heading)
            painter.drawText(QRectF(-12,-12,24,24), Qt.AlignCenter, label)
            painter.restore()

        painter.rotate(self.heading)  # restore for pointer

        # North pointer
        poly_n = QPolygonF([QPointF(0,-r+22), QPointF(-7,-r+40), QPointF(7,-r+40)])
        painter.setBrush(QBrush(QColor("#F85149")))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(poly_n)
        poly_s = QPolygonF([QPointF(0,r-22), QPointF(-7,r-40), QPointF(7,r-40)])
        painter.setBrush(QBrush(QColor("#8B949E")))
        painter.drawPolygon(poly_s)

        # Heading text (rotate back)
        painter.rotate(self.heading)
        painter.setPen(QPen(QColor("#58A6FF")))
        painter.setFont(QFont("Courier New", 12, QFont.Bold))
        painter.drawText(QRectF(-30,-14,60,28), Qt.AlignCenter,
                         f"{int(self.heading):03d}°")

# ─────────────────────────────────────────────────────────────────
#  METRIC CARD
# ─────────────────────────────────────────────────────────────────
class MetricCard(QFrame):
    def __init__(self, title: str, unit: str = "", color: QColor = None, icon: str = ""):
        super().__init__()
        self.title = title
        self.unit  = unit
        self._color= color or Theme.ACCENT_CYAN
        self._value= "—"
        self._warn = False

        self.setStyleSheet(f"""
            MetricCard {{
                background: {Theme.BG_CARD.name()};
                border: 1px solid {Theme.BORDER.name()};
                border-radius: 8px;
            }}
            MetricCard:hover {{
                border-color: {self._color.name()};
            }}
        """)
        self.setMinimumHeight(88)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(2)

        self._title_lbl = QLabel(f"{icon}  {title}" if icon else title)
        self._title_lbl.setStyleSheet(f"color:{Theme.TEXT_SECOND.name()}; font:10px 'Courier New';")

        self._val_lbl = QLabel("—")
        self._val_lbl.setStyleSheet(
            f"color:{self._color.name()}; font:bold 22px 'Courier New';")

        self._sub_lbl = QLabel(unit)
        self._sub_lbl.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()}; font:9px 'Courier New';")

        lay.addWidget(self._title_lbl)
        lay.addWidget(self._val_lbl)
        lay.addWidget(self._sub_lbl)

    def set_value(self, v, warn=False, sub=""):
        self._val_lbl.setText(str(v))
        if sub:
            self._sub_lbl.setText(sub)
        color = Theme.DANGER if warn else self._color
        self._val_lbl.setStyleSheet(
            f"color:{color.name()}; font:bold 22px 'Courier New';")
        self._warn = warn

# ─────────────────────────────────────────────────────────────────
#  SPARKLINE WIDGET
# ─────────────────────────────────────────────────────────────────
class Sparkline(QWidget):
    def __init__(self, color: QColor = None, max_points: int = 200):
        super().__init__()
        self._data  = deque(maxlen=max_points)
        self._color = color or Theme.ACCENT_CYAN
        self.setMinimumHeight(50)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def push(self, v: float):
        self._data.append(v)
        self.update()

    def paintEvent(self, event):
        if len(self._data) < 2: return
        w, h = self.width(), self.height()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        data = list(self._data)
        mn, mx = min(data), max(data)
        rng = mx - mn or 1

        def to_pt(i, v):
            x = i / (len(data) - 1) * w
            y = h - (v - mn) / rng * (h - 8) - 4
            return QPointF(x, y)

        path = QPainterPath()
        path.moveTo(to_pt(0, data[0]))
        for i in range(1, len(data)):
            path.lineTo(to_pt(i, data[i]))

        # Fill gradient
        fill = QPainterPath(path)
        fill.lineTo(QPointF(w, h))
        fill.lineTo(QPointF(0, h))
        fill.closeSubpath()
        grad = QLinearGradient(0, 0, 0, h)
        c = QColor(self._color)
        c.setAlpha(80)
        grad.setColorAt(0, c)
        c.setAlpha(0)
        grad.setColorAt(1, c)
        painter.fillPath(fill, grad)

        pen = QPen(self._color, 1.5)
        painter.setPen(pen)
        painter.drawPath(path)

        # Last value dot
        pt = to_pt(len(data)-1, data[-1])
        painter.setBrush(QBrush(self._color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(pt, 3, 3)

# ─────────────────────────────────────────────────────────────────
#  BATTERY INDICATOR
# ─────────────────────────────────────────────────────────────────
class BatteryWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.percent  = 100
        self.voltage  = 0.0
        self.current  = 0.0
        self.setMinimumSize(120, 52)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def update_battery(self, pct, v, a):
        self.percent = pct
        self.voltage = v
        self.current = a
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        bw, bh = w - 16, 22
        by = (h - bh) // 2
        bx = 2

        # Border
        painter.setPen(QPen(Theme.BORDER_ACCENT, 1.5))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(bx, by, bw - 10, bh, 3, 3)
        # Tip
        painter.fillRect(bx + bw - 10, by + 6, 8, bh - 12, Theme.BORDER_ACCENT)

        # Fill
        p = max(0, min(100, self.percent)) / 100
        fill_w = int((bw - 14) * p)
        if p > 0.4:   col = Theme.SUCCESS
        elif p > 0.2: col = Theme.WARNING
        else:         col = Theme.DANGER
        painter.fillRect(bx + 2, by + 2, fill_w, bh - 4, col)

        # Text
        painter.setPen(QPen(Qt.white))
        painter.setFont(QFont("Courier New", 8, QFont.Bold))
        painter.drawText(QRectF(bx, by, bw - 10, bh), Qt.AlignCenter,
                         f"{self.percent}%")

        # Sub-labels
        painter.setPen(QPen(Theme.TEXT_SECOND))
        painter.setFont(QFont("Courier New", 7))
        painter.drawText(QRectF(0, by + bh + 2, w//2, 12),
                         Qt.AlignLeft, f"{self.voltage:.2f}V")
        painter.drawText(QRectF(w//2, by + bh + 2, w//2, 12),
                         Qt.AlignRight, f"{self.current:.1f}A")

# ─────────────────────────────────────────────────────────────────
#  GPS STATUS WIDGET
# ─────────────────────────────────────────────────────────────────
class GpsWidget(QWidget):
    FIX_NAMES = {0:"No Fix", 1:"No Fix", 2:"2D Fix", 3:"3D Fix",
                 4:"DGPS", 5:"RTK Float", 6:"RTK Fixed"}

    def __init__(self):
        super().__init__()
        self.fix_type = 0
        self.sats     = 0
        self.hdop     = 99.9
        self.lat      = 0.0
        self.lon      = 0.0
        self.alt      = 0.0
        self.setMinimumHeight(100)

    def update_gps(self, fix, sats, hdop, lat, lon, alt):
        self.fix_type = fix
        self.sats     = sats
        self.hdop     = hdop
        self.lat      = lat
        self.lon      = lon
        self.alt      = alt
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        ok = self.fix_type >= 3
        color = Theme.SUCCESS if self.fix_type >= 3 else (
                Theme.WARNING if self.fix_type == 2 else Theme.DANGER)
        # GPS icon (circle)
        r = 10
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(r + 4, h/2), r, r)

        painter.setPen(QPen(Theme.TEXT_PRIMARY))
        painter.setFont(QFont("Courier New", 9, QFont.Bold))
        lbl = self.FIX_NAMES.get(self.fix_type, "?")
        painter.drawText(QRectF(28, 0, w - 28, h/2),
                         Qt.AlignVCenter | Qt.AlignLeft,
                         f"{lbl}  {self.sats}sat  HDOP:{self.hdop:.1f}")
        painter.setFont(QFont("Courier New", 8))
        painter.setPen(QPen(Theme.TEXT_SECOND))
        painter.drawText(QRectF(28, h/2, w - 28, h/2),
                         Qt.AlignVCenter | Qt.AlignLeft,
                         f"Lat:{self.lat:.6f}  Lon:{self.lon:.6f}  Alt:{self.alt:.1f}m")

# ─────────────────────────────────────────────────────────────────
#  RC CHANNEL DISPLAY
# ─────────────────────────────────────────────────────────────────
class RCChannelWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.channels = [1500] * 8
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_channels(self, ch):
        self.channels = list(ch)
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        n = len(self.channels)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        bar_w  = (w - (n - 1) * 4) / n
        labels = ['THR','ROL','PCH','YAW','CH5','CH6','CH7','CH8']
        colors = [Theme.SUCCESS, Theme.INFO, Theme.ACCENT_ORANGE,
                  Theme.ACCENT_PURPLE] + [Theme.TEXT_SECOND]*4
        for i, val in enumerate(self.channels):
            x = i * (bar_w + 4)
            pct = (val - 1000) / 1000.0
            bh  = (h - 28) * pct
            by  = h - 28 - bh
            # Background track
            painter.fillRect(int(x), 0, int(bar_w), h - 28,
                             QColor(Theme.BORDER.red(), Theme.BORDER.green(),
                                    Theme.BORDER.blue(), 80))
            # Fill
            col = colors[i] if i < len(colors) else Theme.TEXT_SECOND
            painter.fillRect(int(x), int(by), int(bar_w), int(bh), col)
            # Label
            painter.setPen(QPen(Theme.TEXT_SECOND))
            painter.setFont(QFont("Courier New", 7))
            painter.drawText(QRectF(x, h - 28, bar_w, 14),
                             Qt.AlignCenter, labels[i])
            painter.setPen(QPen(Theme.TEXT_PRIMARY))
            painter.setFont(QFont("Courier New", 7, QFont.Bold))
            painter.drawText(QRectF(x, h - 14, bar_w, 14),
                             Qt.AlignCenter, str(val))

# ─────────────────────────────────────────────────────────────────
#  MINI CHART PANEL
# ─────────────────────────────────────────────────────────────────
class ChartPanel(QWidget):
    def __init__(self, title: str, n_traces: int, colors, labels, y_min=None, y_max=None):
        super().__init__()
        self.title  = title
        self.colors = colors
        self.labels = labels
        self.y_min  = y_min
        self.y_max  = y_max
        self._data  = [deque(maxlen=300) for _ in range(n_traces)]
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(100)

    def push(self, *values):
        for i, v in enumerate(values):
            if i < len(self._data):
                self._data[i].append(v)
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        if w < 10 or h < 10: return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        pad = 28

        # Background
        painter.fillRect(0, 0, w, h, Theme.BG_CARD)

        # Title
        painter.setPen(QPen(Theme.TEXT_SECOND))
        painter.setFont(QFont("Courier New", 8, QFont.Bold))
        painter.drawText(4, 12, self.title)

        # Grid lines
        painter.setPen(QPen(QColor(Theme.BORDER.red(),Theme.BORDER.green(),
                                   Theme.BORDER.blue(), 120), 0.5))
        for i in range(1, 4):
            y = pad + (h - pad * 2) * i / 4
            painter.drawLine(QPointF(pad, y), QPointF(w - 4, y))

        all_vals = [v for d in self._data for v in d]
        if not all_vals: return
        mn = self.y_min if self.y_min is not None else min(all_vals)
        mx = self.y_max if self.y_max is not None else max(all_vals)
        rng = mx - mn or 1

        def to_pt(idx, v, n):
            x = pad + idx / max(n - 1, 1) * (w - pad - 4)
            y = h - pad - (v - mn) / rng * (h - pad * 2)
            return QPointF(x, y)

        # Draw traces
        for ti, data in enumerate(self._data):
            lst = list(data)
            if len(lst) < 2: continue
            col = self.colors[ti] if ti < len(self.colors) else Qt.white
            path = QPainterPath()
            path.moveTo(to_pt(0, lst[0], len(lst)))
            for i in range(1, len(lst)):
                path.lineTo(to_pt(i, lst[i], len(lst)))
            painter.setPen(QPen(col, 1.5))
            painter.drawPath(path)

        # Legend
        x_leg = pad
        for ti, lbl in enumerate(self.labels):
            if ti >= len(self._data): break
            col = self.colors[ti] if ti < len(self.colors) else Qt.white
            painter.setPen(QPen(col))
            painter.setFont(QFont("Courier New", 7))
            data = list(self._data[ti])
            val = f"{data[-1]:.2f}" if data else "—"
            painter.fillRect(int(x_leg), h - 14, 8, 8, col)
            painter.drawText(int(x_leg + 10), h - 6, f"{lbl}:{val}")
            x_leg += 80

        # Y axis labels
        painter.setPen(QPen(Theme.TEXT_MUTED))
        painter.setFont(QFont("Courier New", 6))
        painter.drawText(2, pad + 4, f"{mx:.1f}")
        painter.drawText(2, h - pad, f"{mn:.1f}")

# ─────────────────────────────────────────────────────────────────
#  FLIGHT MODE SELECTOR
# ─────────────────────────────────────────────────────────────────
FLIGHT_MODES = [
    "STABILIZE","ACRO","ALT_HOLD","AUTO","GUIDED",
    "LOITER","RTL","CIRCLE","LAND","DRIFT",
    "SPORT","FLIP","AUTOTUNE","POSHOLD","BRAKE",
    "THROW","AVOID_ADSB","GUIDED_NOGPS","SMART_RTL","FLOWHOLD"
]

# ─────────────────────────────────────────────────────────────────
#  CONSOLE WIDGET
# ─────────────────────────────────────────────────────────────────
class ConsoleWidget(QTextEdit):
    LEVEL_COLORS = {
        "INFO":  "#58A6FF",
        "WARN":  "#D29922",
        "ERROR": "#F85149",
        "MAV":   "#3FB950",
        "SYS":   "#BC8CFF",
    }

    MAX_LINES = 1000

    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setFont(QFont("Courier New", 8))
        self.setStyleSheet(f"""
            QTextEdit {{
                background:{Theme.BG_DARK.name()};
                color:{Theme.TEXT_PRIMARY.name()};
                border:none;
            }}
        """)
        self._line_count = 0

    def log(self, level: str, text: str):
        ts    = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = self.LEVEL_COLORS.get(level, "#FFFFFF")
        self.append(
            f'<span style="color:#484F58">[{ts}]</span> '
            f'<span style="color:{color};font-weight:bold">[{level:5s}]</span> '
            f'<span style="color:#E6EDF3">{text}</span>'
        )
        self._line_count += 1
        # Trim old lines to keep memory bounded
        if self._line_count > self.MAX_LINES:
            cursor = self.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.movePosition(cursor.Down, cursor.KeepAnchor, 50)
            cursor.removeSelectedText()
            self._line_count -= 50
        self.verticalScrollBar().setValue(
            self.verticalScrollBar().maximum())

# ─────────────────────────────────────────────────────────────────
#  CONNECTION DIALOG
# ─────────────────────────────────────────────────────────────────
class ConnectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to UAV")
        self.setFixedSize(420, 280)
        self.setStyleSheet(f"""
            QDialog {{ background:{Theme.BG_CARD.name()}; }}
            QLabel  {{ color:{Theme.TEXT_PRIMARY.name()}; font-family:'Courier New'; }}
            QLineEdit, QComboBox, QSpinBox {{
                background:{Theme.BG_DARK.name()};
                color:{Theme.TEXT_PRIMARY.name()};
                border:1px solid {Theme.BORDER_ACCENT.name()};
                border-radius:4px;
                padding:4px 8px;
                font-family:'Courier New';
            }}
            QPushButton {{
                background:{Theme.ACCENT_BLUE.name()};
                color:white;
                border:none;
                border-radius:5px;
                padding:8px 20px;
                font-family:'Courier New';
                font-weight:bold;
            }}
            QPushButton:hover {{ background:{Theme.ACCENT_CYAN.name()}; color:black; }}
        """)

        lay = QVBoxLayout(self)
        lay.setSpacing(12)
        lay.setContentsMargins(20, 20, 20, 20)

        title = QLabel("⚡  UAV CONNECTION SETUP")
        title.setStyleSheet("font-size:13px; font-weight:bold; color:#58A6FF;")
        lay.addWidget(title)

        form = QFormLayout()
        form.setSpacing(8)

        self.type_combo = QComboBox()
        self.type_combo.addItems(["UDP","TCP","Serial","UDP Server","Simulation"])
        self.type_combo.currentTextChanged.connect(self._update_hint)
        form.addRow("Type:", self.type_combo)

        self.addr_edit = QLineEdit("udp:127.0.0.1:14550")
        form.addRow("Connection:", self.addr_edit)

        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 921600)
        self.baud_spin.setValue(57600)
        self.baud_spin.setSingleStep(9600)
        form.addRow("Baud Rate:", self.baud_spin)

        self.hint_lbl = QLabel()
        self.hint_lbl.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()}; font-size:9px;")
        form.addRow("", self.hint_lbl)

        lay.addLayout(form)
        lay.addStretch()

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.button(QDialogButtonBox.Ok).setText("Connect")
        btns.button(QDialogButtonBox.Cancel).setText("Cancel")
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        lay.addWidget(btns)

        self._update_hint("UDP")

    def _update_hint(self, t):
        hints = {
            "UDP":       "Format: udp:host:port  (e.g. udp:127.0.0.1:14550)",
            "TCP":       "Format: tcp:host:port  (e.g. tcp:192.168.1.1:5760)",
            "Serial":    "Format: /dev/ttyACM0   or   COM3",
            "UDP Server":"Format: udpin:0.0.0.0:14550",
            "Simulation":"Auto-starts simulation — no hardware needed",
        }
        self.hint_lbl.setText(hints.get(t, ""))
        defaults = {
            "UDP":       "udp:127.0.0.1:14550",
            "TCP":       "tcp:192.168.1.1:5760",
            "Serial":    "/dev/ttyACM0",
            "UDP Server":"udpin:0.0.0.0:14550",
            "Simulation":"sim:",
        }
        self.addr_edit.setText(defaults.get(t, ""))

    @property
    def connection_string(self): return self.addr_edit.text()
    @property
    def baud(self):              return self.baud_spin.value()

# ─────────────────────────────────────────────────────────────────
#  MAIN WINDOW
# ─────────────────────────────────────────────────────────────────
class GroundStation(QMainWindow):
    STYLESHEET = f"""
        QMainWindow {{
            background:{Theme.BG_DARK.name()};
        }}
        QWidget {{
            background:{Theme.BG_DARK.name()};
            color:{Theme.TEXT_PRIMARY.name()};
            font-family:'Courier New';
        }}
        QTabWidget::pane {{
            border:1px solid {Theme.BORDER.name()};
            background:{Theme.BG_PANEL.name()};
        }}
        QTabBar::tab {{
            background:{Theme.BG_CARD.name()};
            color:{Theme.TEXT_SECOND.name()};
            padding:6px 18px;
            border:1px solid {Theme.BORDER.name()};
            border-bottom:none;
            font-family:'Courier New';
            font-size:10px;
        }}
        QTabBar::tab:selected {{
            background:{Theme.BG_PANEL.name()};
            color:{Theme.ACCENT_CYAN.name()};
            border-top:2px solid {Theme.ACCENT_CYAN.name()};
        }}
        QPushButton {{
            background:{Theme.BG_CARD.name()};
            color:{Theme.TEXT_PRIMARY.name()};
            border:1px solid {Theme.BORDER_ACCENT.name()};
            border-radius:5px;
            padding:6px 14px;
            font-family:'Courier New';
            font-size:10px;
        }}
        QPushButton:hover {{
            border-color:{Theme.ACCENT_CYAN.name()};
            color:{Theme.ACCENT_CYAN.name()};
        }}
        QPushButton:pressed {{
            background:{Theme.ACCENT_CYAN.name()};
            color:black;
        }}
        QComboBox {{
            background:{Theme.BG_CARD.name()};
            color:{Theme.TEXT_PRIMARY.name()};
            border:1px solid {Theme.BORDER_ACCENT.name()};
            border-radius:4px;
            padding:4px 8px;
        }}
        QComboBox::drop-down {{ border:none; }}
        QComboBox QAbstractItemView {{
            background:{Theme.BG_CARD2.name()};
            color:{Theme.TEXT_PRIMARY.name()};
            selection-background-color:{Theme.ACCENT_BLUE.name()};
        }}
        QScrollBar:vertical {{
            background:{Theme.BG_DARK.name()};
            width:8px;
        }}
        QScrollBar::handle:vertical {{
            background:{Theme.BORDER_ACCENT.name()};
            border-radius:4px;
        }}
        QGroupBox {{
            border:1px solid {Theme.BORDER.name()};
            border-radius:6px;
            margin-top:10px;
            font-size:10px;
            color:{Theme.TEXT_SECOND.name()};
        }}
        QGroupBox::title {{
            subcontrol-origin:margin;
            padding:0 6px;
        }}
        QTableWidget {{
            background:{Theme.BG_CARD.name()};
            gridline-color:{Theme.BORDER.name()};
            border:none;
            font-size:10px;
        }}
        QTableWidget::item {{ padding:4px; }}
        QTableWidget::item:selected {{
            background:{Theme.ACCENT_BLUE.name()};
        }}
        QHeaderView::section {{
            background:{Theme.BG_CARD2.name()};
            color:{Theme.TEXT_SECOND.name()};
            border:none;
            padding:4px 8px;
            font-size:9px;
        }}
        QStatusBar {{
            background:{Theme.BG_CARD.name()};
            color:{Theme.TEXT_SECOND.name()};
            font-size:9px;
        }}
        QToolBar {{
            background:{Theme.BG_CARD.name()};
            border-bottom:1px solid {Theme.BORDER.name()};
            spacing:4px;
            padding:4px;
        }}
        QSplitter::handle {{
            background:{Theme.BORDER.name()};
        }}
        QLineEdit {{
            background:{Theme.BG_DARK.name()};
            color:{Theme.TEXT_PRIMARY.name()};
            border:1px solid {Theme.BORDER_ACCENT.name()};
            border-radius:4px;
            padding:4px 8px;
        }}
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV Ground Station  —  Professional Edition")
        self.resize(1600, 920)
        self.setMinimumSize(1200, 700)
        self.setStyleSheet(self.STYLESHEET)

        self._worker = MAVLinkWorker()
        self._worker.telemetry_updated.connect(self._on_telemetry)
        self._worker.message_received.connect(self._on_message)
        self._worker.connection_status.connect(self._on_connection_status)
        self._worker.start()

        self._history = {f: deque(maxlen=500)
                         for f in TelemetryData().history_fields()}
        self._last_telem = TelemetryData()
        self._start_time = time.time()

        self._build_ui()
        self._build_status_bar()
        self._build_toolbar()

        # Refresh timer for charts (they need more frequent updates)
        self._chart_timer = QTimer()
        self._chart_timer.timeout.connect(self._refresh_charts)
        self._chart_timer.start(100)

    # ── UI construction ─────────────────────────────────────
    def _build_toolbar(self):
        tb = QToolBar("Main", self)
        tb.setIconSize(QSize(16,16))
        tb.setMovable(False)
        self.addToolBar(tb)

        self._btn_connect = QPushButton("⚡ CONNECT")
        self._btn_connect.setStyleSheet(
            f"background:{Theme.ACCENT_BLUE.name()};color:white;font-weight:bold;"
            f"border:none;border-radius:4px;padding:6px 16px;")
        self._btn_connect.clicked.connect(self._show_connect_dialog)
        tb.addWidget(self._btn_connect)

        tb.addSeparator()

        self._btn_arm = QPushButton("⚠  ARM")
        self._btn_arm.setStyleSheet(
            f"background:{Theme.DANGER.name()};color:white;font-weight:bold;"
            f"border:none;border-radius:4px;padding:6px 16px;")
        self._btn_arm.clicked.connect(self._arm_disarm)
        self._btn_arm.setEnabled(False)
        tb.addWidget(self._btn_arm)

        tb.addSeparator()

        lbl_mode = QLabel("Mode: ")
        lbl_mode.setStyleSheet("color:#8B949E; padding:0 4px;")
        tb.addWidget(lbl_mode)
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(FLIGHT_MODES)
        self._mode_combo.setFixedWidth(140)
        self._mode_combo.currentTextChanged.connect(self._change_mode)
        tb.addWidget(self._mode_combo)

        tb.addSeparator()

        self._btn_rtl = QPushButton("🏠 RTL")
        self._btn_rtl.setStyleSheet(
            f"background:{Theme.ACCENT_ORANGE.name()};color:white;font-weight:bold;"
            f"border:none;border-radius:4px;padding:6px 16px;")
        self._btn_rtl.clicked.connect(lambda: self._set_mode("RTL"))
        tb.addWidget(self._btn_rtl)

        self._btn_land = QPushButton("▼ LAND")
        self._btn_land.clicked.connect(lambda: self._set_mode("LAND"))
        tb.addWidget(self._btn_land)

        tb.addSeparator()

        # Spacer
        spacer = QWidget(); spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        tb.addWidget(spacer)

        self._conn_indicator = QLabel("● OFFLINE")
        self._conn_indicator.setStyleSheet(f"color:{Theme.DANGER.name()};font-weight:bold;padding:0 12px;")
        tb.addWidget(self._conn_indicator)

        self._rate_lbl = QLabel("0 msg/s")
        self._rate_lbl.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()};padding:0 8px;")
        tb.addWidget(self._rate_lbl)

    def _build_status_bar(self):
        sb = QStatusBar()
        self.setStatusBar(sb)
        self._sb_time  = QLabel("T+ 00:00:00")
        self._sb_mode  = QLabel("MODE: —")
        self._sb_armed = QLabel("DISARMED")
        self._sb_armed.setStyleSheet(f"color:{Theme.DANGER.name()};font-weight:bold;")
        self._sb_lat   = QLabel("LAT: —")
        self._sb_lon   = QLabel("LON: —")
        self._sb_alt   = QLabel("ALT: —")
        sb.addWidget(self._sb_time)
        sb.addWidget(QLabel(" | "))
        sb.addWidget(self._sb_mode)
        sb.addWidget(QLabel(" | "))
        sb.addWidget(self._sb_armed)
        sb.addWidget(QLabel(" | "))
        sb.addWidget(self._sb_lat)
        sb.addWidget(self._sb_lon)
        sb.addWidget(self._sb_alt)

        # Uptime timer
        self._sb_timer = QTimer()
        self._sb_timer.timeout.connect(self._tick_uptime)
        self._sb_timer.start(1000)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        tabs = QTabWidget()
        root.addWidget(tabs)

        tabs.addTab(self._build_dashboard_tab(), "📊  DASHBOARD")
        tabs.addTab(self._build_sensors_tab(),   "📡  SENSORS")
        tabs.addTab(self._build_charts_tab(),    "📈  CHARTS")
        tabs.addTab(self._build_rc_tab(),        "🎮  RC / CHANNELS")
        tabs.addTab(self._build_params_tab(),    "⚙️  PARAMETERS")
        tabs.addTab(self._build_console_tab(),   "💬  CONSOLE")

    # ── TAB: DASHBOARD ──────────────────────────────────────
    def _build_dashboard_tab(self):
        w = QWidget()
        main = QHBoxLayout(w)
        main.setContentsMargins(6, 6, 6, 6)
        main.setSpacing(6)

        # ── Left column: HUD + Compass
        left = QVBoxLayout()
        left.setSpacing(6)

        hud_grp = QGroupBox("ATTITUDE — ARTIFICIAL HORIZON")
        hud_lay = QVBoxLayout(hud_grp)
        self._horizon = ArtificialHorizon()
        hud_lay.addWidget(self._horizon)
        left.addWidget(hud_grp, 3)

        cmp_grp = QGroupBox("HEADING — COMPASS ROSE")
        cmp_lay = QVBoxLayout(cmp_grp)
        self._compass = CompassRose()
        cmp_lay.addWidget(self._compass)
        left.addWidget(cmp_grp, 2)

        main.addLayout(left, 2)

        # ── Centre: metric cards
        centre = QVBoxLayout()
        centre.setSpacing(6)

        # Row 1 — key metrics
        r1 = QHBoxLayout(); r1.setSpacing(6)
        self._card_alt     = MetricCard("ALTITUDE",     "m",   Theme.ACCENT_CYAN,   "▲")
        self._card_spd     = MetricCard("AIRSPEED",     "m/s", Theme.SUCCESS,        "➤")
        self._card_gspd    = MetricCard("GROUNDSPEED",  "m/s", Theme.ACCENT_ORANGE,  "⬤")
        self._card_vspd    = MetricCard("VSPEED",       "m/s", Theme.INFO,           "↕")
        for c in [self._card_alt, self._card_spd, self._card_gspd, self._card_vspd]:
            r1.addWidget(c)
        centre.addLayout(r1)

        # Row 2 — battery
        r2 = QHBoxLayout(); r2.setSpacing(6)
        batt_grp = QGroupBox("BATTERY")
        batt_lay = QVBoxLayout(batt_grp)
        self._battery = BatteryWidget()
        batt_lay.addWidget(self._battery)
        r2.addWidget(batt_grp, 2)

        self._card_volt  = MetricCard("VOLTAGE", "V", Theme.ACCENT_ORANGE)
        self._card_curr  = MetricCard("CURRENT", "A", Theme.WARNING)
        self._card_mah   = MetricCard("CONSUMED","mAh", Theme.TEXT_SECOND)
        for c in [self._card_volt, self._card_curr, self._card_mah]:
            r2.addWidget(c, 1)
        centre.addLayout(r2)

        # Row 3 — GPS
        gps_grp = QGroupBox("GPS")
        gps_lay = QVBoxLayout(gps_grp)
        self._gps_widget = GpsWidget()
        gps_lay.addWidget(self._gps_widget)
        centre.addWidget(gps_grp)

        # Row 4 — more metrics
        r4 = QHBoxLayout(); r4.setSpacing(6)
        self._card_roll  = MetricCard("ROLL",  "°", Theme.ACCENT_CYAN)
        self._card_pitch = MetricCard("PITCH", "°", Theme.INFO)
        self._card_yaw   = MetricCard("YAW",   "°", Theme.ACCENT_PURPLE)
        self._card_rssi  = MetricCard("RSSI",  "%", Theme.SUCCESS)
        for c in [self._card_roll, self._card_pitch, self._card_yaw, self._card_rssi]:
            r4.addWidget(c)
        centre.addLayout(r4)

        # Sparklines
        spk_grp = QGroupBox("ALTITUDE TREND")
        spk_lay = QVBoxLayout(spk_grp)
        self._spk_alt = Sparkline(Theme.ACCENT_CYAN)
        spk_lay.addWidget(self._spk_alt)
        centre.addWidget(spk_grp, 1)

        main.addLayout(centre, 3)

        # ── Right column
        right = QVBoxLayout()
        right.setSpacing(6)

        veh_grp = QGroupBox("VEHICLE STATUS")
        veh_lay = QGridLayout(veh_grp)
        veh_lay.setSpacing(4)
        labels_r = ["Flight Mode","Armed","Autopilot","Type","Uptime","Sys ID","Msg Rate","Pkt Loss"]
        self._status_vals = {}
        for i, lbl in enumerate(labels_r):
            l = QLabel(lbl + ":")
            l.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()}; font-size:9px;")
            v = QLabel("—")
            v.setStyleSheet(f"color:{Theme.TEXT_PRIMARY.name()}; font-size:9px; font-weight:bold;")
            veh_lay.addWidget(l, i, 0)
            veh_lay.addWidget(v, i, 1)
            self._status_vals[lbl] = v
        right.addWidget(veh_grp)

        wp_grp = QGroupBox("MISSION / WAYPOINT")
        wp_lay = QGridLayout(wp_grp)
        wp_labels = ["Current WP","WP Distance","Total WPs"]
        self._wp_vals = {}
        for i, lbl in enumerate(wp_labels):
            l = QLabel(lbl + ":")
            l.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()}; font-size:9px;")
            v = QLabel("—")
            v.setStyleSheet(f"color:{Theme.TEXT_PRIMARY.name()}; font-size:10px; font-weight:bold;")
            wp_lay.addWidget(l, i, 0)
            wp_lay.addWidget(v, i, 1)
            self._wp_vals[lbl] = v
        right.addWidget(wp_grp)

        ekf_grp = QGroupBox("EKF STATUS")
        ekf_lay = QVBoxLayout(ekf_grp)
        self._ekf_flags_lbl = QLabel("—")
        self._ekf_flags_lbl.setStyleSheet(f"color:{Theme.SUCCESS.name()}; font-size:9px;")
        ekf_lay.addWidget(self._ekf_flags_lbl)
        self._ekf_bars = {}
        for name in ['Attitude','Vel Horiz','Vel Vert','Pos Horiz','Pos Vert','Terrain','Const Pos']:
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(100)
            bar.setFixedHeight(12)
            bar.setTextVisible(False)
            bar.setStyleSheet("""
                QProgressBar { background:#161B22; border:none; border-radius:2px; }
                QProgressBar::chunk { background:#3FB950; border-radius:2px; }
            """)
            row = QHBoxLayout()
            l = QLabel(name)
            l.setStyleSheet("font-size:8px; color:#8B949E;")
            l.setFixedWidth(70)
            row.addWidget(l); row.addWidget(bar)
            ekf_lay.addLayout(row)
            self._ekf_bars[name] = bar
        right.addWidget(ekf_grp)

        right.addStretch()
        main.addLayout(right, 1)
        return w

    # ── TAB: SENSORS ────────────────────────────────────────
    def _build_sensors_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        lay.setContentsMargins(6,6,6,6)
        lay.setSpacing(6)

        # IMU
        imu_grp = QGroupBox("IMU — ACCELEROMETER / GYROSCOPE")
        imu_lay = QGridLayout(imu_grp)
        self._imu_cards = {}
        imu_fields = [
            ('Accel X','m/s²',Theme.ACCENT_RED),('Accel Y','m/s²',Theme.SUCCESS),
            ('Accel Z','m/s²',Theme.ACCENT_CYAN),('Gyro X','rad/s',Theme.ACCENT_RED),
            ('Gyro Y','rad/s',Theme.SUCCESS),('Gyro Z','rad/s',Theme.ACCENT_CYAN),
        ]
        for i,(lbl,unit,col) in enumerate(imu_fields):
            c = MetricCard(lbl, unit, col)
            imu_lay.addWidget(c, i//3, i%3)
            self._imu_cards[lbl] = c
        lay.addWidget(imu_grp, 0, 0)

        # Magnetometer
        mag_grp = QGroupBox("MAGNETOMETER")
        mag_lay = QGridLayout(mag_grp)
        self._mag_cards = {}
        for i,(lbl,col) in enumerate([('Mag X',Theme.ACCENT_RED),
                                       ('Mag Y',Theme.SUCCESS),
                                       ('Mag Z',Theme.ACCENT_CYAN)]):
            c = MetricCard(lbl, "μT", col)
            mag_lay.addWidget(c, 0, i)
            self._mag_cards[lbl] = c
        lay.addWidget(mag_grp, 0, 1)

        # Barometer
        baro_grp = QGroupBox("BAROMETER")
        baro_lay = QGridLayout(baro_grp)
        self._baro_cards = {}
        for i,(lbl,unit,col) in enumerate([
            ('Baro Alt','m',Theme.ACCENT_CYAN),
            ('Pressure','Pa',Theme.INFO),
            ('Temperature','°C',Theme.ACCENT_ORANGE)
        ]):
            c = MetricCard(lbl, unit, col)
            baro_lay.addWidget(c, 0, i)
            self._baro_cards[lbl] = c
        lay.addWidget(baro_grp, 1, 0)

        # Vibration
        vib_grp = QGroupBox("VIBRATION")
        vib_lay = QGridLayout(vib_grp)
        self._vib_cards = {}
        for i,(lbl,col) in enumerate([('Vibe X',Theme.ACCENT_RED),
                                       ('Vibe Y',Theme.SUCCESS),
                                       ('Vibe Z',Theme.ACCENT_CYAN)]):
            c = MetricCard(lbl, "m/s²", col)
            vib_lay.addWidget(c, 0, i)
            self._vib_cards[lbl] = c
        lay.addWidget(vib_grp, 1, 1)

        # Charts
        imu_chart = ChartPanel(
            "ACCELEROMETER", 3,
            [Theme.ACCENT_RED, Theme.SUCCESS, Theme.ACCENT_CYAN],
            ['AccX','AccY','AccZ'], -20, 20)
        self._imu_chart = imu_chart
        lay.addWidget(imu_chart, 2, 0)

        gyr_chart = ChartPanel(
            "GYROSCOPE", 3,
            [Theme.ACCENT_RED, Theme.SUCCESS, Theme.ACCENT_CYAN],
            ['GyrX','GyrY','GyrZ'], -5, 5)
        self._gyr_chart = gyr_chart
        lay.addWidget(gyr_chart, 2, 1)

        vib_chart = ChartPanel(
            "VIBRATION", 3,
            [Theme.ACCENT_RED, Theme.SUCCESS, Theme.ACCENT_CYAN],
            ['VibX','VibY','VibZ'], 0, 30)
        self._vib_chart = vib_chart
        lay.addWidget(vib_chart, 3, 0, 1, 2)

        lay.setRowStretch(2, 1)
        lay.setRowStretch(3, 1)
        return w

    # ── TAB: CHARTS ─────────────────────────────────────────
    def _build_charts_tab(self):
        w = QWidget()
        lay = QGridLayout(w)
        lay.setContentsMargins(6,6,6,6)
        lay.setSpacing(6)

        self._ch_attitude = ChartPanel(
            "ATTITUDE", 3,
            [Theme.ACCENT_CYAN, Theme.SUCCESS, Theme.ACCENT_PURPLE],
            ['Roll','Pitch','Yaw'], -180, 180)
        lay.addWidget(self._ch_attitude, 0, 0)

        self._ch_altitude = ChartPanel(
            "ALTITUDE & VSPEED", 2,
            [Theme.ACCENT_CYAN, Theme.ACCENT_ORANGE],
            ['Altitude','VSpeed'])
        lay.addWidget(self._ch_altitude, 0, 1)

        self._ch_speed = ChartPanel(
            "SPEED", 2,
            [Theme.SUCCESS, Theme.ACCENT_ORANGE],
            ['Airspeed','GndSpeed'], 0, 30)
        lay.addWidget(self._ch_speed, 1, 0)

        self._ch_battery = ChartPanel(
            "BATTERY", 2,
            [Theme.ACCENT_ORANGE, Theme.DANGER],
            ['Voltage','Current'])
        lay.addWidget(self._ch_battery, 1, 1)

        self._ch_baro = ChartPanel(
            "BAROMETER", 2,
            [Theme.ACCENT_CYAN, Theme.ACCENT_ORANGE],
            ['Pressure','Temperature'])
        lay.addWidget(self._ch_baro, 2, 0)

        self._ch_rssi = ChartPanel(
            "RSSI / SIGNAL", 1,
            [Theme.SUCCESS],
            ['RSSI'], 0, 100)
        lay.addWidget(self._ch_rssi, 2, 1)

        for r in range(3): lay.setRowStretch(r, 1)
        lay.setColumnStretch(0, 1)
        lay.setColumnStretch(1, 1)
        return w

    # ── TAB: RC ─────────────────────────────────────────────
    def _build_rc_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(6,6,6,6)

        lbl = QLabel("RC CHANNEL MONITOR")
        lbl.setStyleSheet(f"color:{Theme.ACCENT_CYAN.name()};font-size:12px;font-weight:bold;")
        lay.addWidget(lbl)

        rc_grp = QGroupBox("CHANNEL INPUT (PWM μs)")
        rc_lay = QVBoxLayout(rc_grp)
        self._rc_widget = RCChannelWidget()
        self._rc_widget.setMinimumHeight(180)
        rc_lay.addWidget(self._rc_widget)
        lay.addWidget(rc_grp, 2)

        # Channel detail table
        tbl_grp = QGroupBox("CHANNEL DETAILS")
        tbl_lay = QVBoxLayout(tbl_grp)
        self._rc_table = QTableWidget(8, 4)
        self._rc_table.setHorizontalHeaderLabels(["Channel","Name","Value μs","%"])
        self._rc_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._rc_table.setSelectionBehavior(QTableWidget.SelectRows)
        self._rc_table.setEditTriggers(QTableWidget.NoEditTriggers)
        names = ['Throttle','Roll','Pitch','Yaw','AUX1','AUX2','AUX3','AUX4']
        for i in range(8):
            self._rc_table.setItem(i, 0, QTableWidgetItem(f"CH{i+1}"))
            self._rc_table.setItem(i, 1, QTableWidgetItem(names[i]))
            self._rc_table.setItem(i, 2, QTableWidgetItem("1500"))
            self._rc_table.setItem(i, 3, QTableWidgetItem("50%"))
        tbl_lay.addWidget(self._rc_table)
        lay.addWidget(tbl_grp, 2)

        # RSSI
        rssi_grp = QGroupBox("RSSI")
        rssi_lay = QHBoxLayout(rssi_grp)
        self._rssi_bar = QProgressBar()
        self._rssi_bar.setRange(0, 100)
        self._rssi_bar.setValue(100)
        self._rssi_bar.setStyleSheet("""
            QProgressBar { background:#161B22; border:none; border-radius:4px; height:24px; }
            QProgressBar::chunk { background:#3FB950; border-radius:4px; }
        """)
        self._rssi_lbl = QLabel("100 %")
        self._rssi_lbl.setStyleSheet(f"color:{Theme.SUCCESS.name()};font-weight:bold;width:50px;")
        rssi_lay.addWidget(QLabel("Signal:"))
        rssi_lay.addWidget(self._rssi_bar)
        rssi_lay.addWidget(self._rssi_lbl)
        lay.addWidget(rssi_grp)
        return w

    # ── TAB: PARAMETERS ─────────────────────────────────────
    def _build_params_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(6,6,6,6)

        hdr = QHBoxLayout()
        self._param_search = QLineEdit()
        self._param_search.setPlaceholderText("Search parameter…")
        self._param_search.textChanged.connect(self._filter_params)
        self._btn_refresh_params = QPushButton("🔄 Refresh")
        self._btn_refresh_params.clicked.connect(self._refresh_params)
        hdr.addWidget(self._param_search)
        hdr.addWidget(self._btn_refresh_params)
        lay.addLayout(hdr)

        self._param_table = QTableWidget(0, 3)
        self._param_table.setHorizontalHeaderLabels(["Parameter","Value","Description"])
        self._param_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._param_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self._param_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self._param_table.setSelectionBehavior(QTableWidget.SelectRows)
        lay.addWidget(self._param_table)

        self._populate_default_params()
        return w

    def _populate_default_params(self):
        defaults = [
            ("ARMING_CHECK", "1", "Arm safety checks bitmask"),
            ("ATC_RAT_PIT_P", "0.135", "Pitch rate P gain"),
            ("ATC_RAT_PIT_I", "0.135", "Pitch rate I gain"),
            ("ATC_RAT_PIT_D", "0.0036","Pitch rate D gain"),
            ("ATC_RAT_RLL_P", "0.135", "Roll rate P gain"),
            ("ATC_RAT_RLL_I", "0.135", "Roll rate I gain"),
            ("ATC_RAT_RLL_D", "0.0036","Roll rate D gain"),
            ("ATC_RAT_YAW_P", "0.18",  "Yaw rate P gain"),
            ("ATC_RAT_YAW_I", "0.018", "Yaw rate I gain"),
            ("ATC_RAT_YAW_D", "0.0",   "Yaw rate D gain"),
            ("BATT_MONITOR",  "4",     "Battery monitoring type"),
            ("BATT_VOLT_PIN", "2",     "Battery voltage ADC pin"),
            ("BATT_CURR_PIN", "3",     "Battery current ADC pin"),
            ("BATT_LOW_VOLT", "14.4",  "Low battery voltage threshold"),
            ("BATT_CRT_VOLT", "13.8",  "Critical battery voltage"),
            ("FS_BATT_ENABLE","2",     "Battery failsafe action"),
            ("FS_GCS_ENABLE", "1",     "GCS failsafe enable"),
            ("FS_THR_ENABLE", "1",     "Throttle failsafe enable"),
            ("FS_THR_VALUE",  "975",   "Throttle failsafe PWM value"),
            ("GPS_TYPE",      "1",     "GPS type (1=Auto)"),
            ("GPS_NAVFILTER", "8",     "GPS navigation filter"),
            ("INS_GYRO_FILTER","20",   "Gyro noise filter cutoff"),
            ("INS_ACCEL_FILTER","20",  "Accel noise filter cutoff"),
            ("PILOT_SPEED_UP","250",   "Max climb speed cm/s"),
            ("PILOT_SPEED_DN","150",   "Max descent speed cm/s"),
            ("PILOT_ACCEL_Z", "250",   "Vertical accel limit cm/s/s"),
            ("WPNAV_SPEED",   "500",   "Waypoint cruise speed cm/s"),
            ("WPNAV_RADIUS",  "200",   "Waypoint arrival radius cm"),
            ("LOIT_SPEED",    "500",   "Loiter max horizontal speed"),
            ("RTL_ALT",       "1500",  "RTL altitude cm"),
            ("RTL_SPEED",     "0",     "RTL speed (0=WPNAV_SPEED)"),
        ]
        self._all_params = defaults
        self._param_table.setRowCount(len(defaults))
        for i,(name,val,desc) in enumerate(defaults):
            self._param_table.setItem(i,0,QTableWidgetItem(name))
            self._param_table.setItem(i,1,QTableWidgetItem(val))
            self._param_table.setItem(i,2,QTableWidgetItem(desc))

    def _filter_params(self, text):
        for i in range(self._param_table.rowCount()):
            match = (text.lower() in (self._param_table.item(i,0) or QTableWidgetItem()).text().lower() or
                     text.lower() in (self._param_table.item(i,2) or QTableWidgetItem()).text().lower())
            self._param_table.setRowHidden(i, not match)

    def _refresh_params(self):
        self._console.log("SYS","Parameter refresh requested (requires MAVLink connection)")

    # ── TAB: CONSOLE ────────────────────────────────────────
    def _build_console_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(6,6,6,6)

        top = QHBoxLayout()
        self._btn_clear_log = QPushButton("🗑 Clear")
        self._btn_clear_log.clicked.connect(lambda: self._console.clear())
        top.addWidget(self._btn_clear_log)
        top.addStretch()
        lay.addLayout(top)

        self._console = ConsoleWidget()
        lay.addWidget(self._console)

        cmd_row = QHBoxLayout()
        self._cmd_input = QLineEdit()
        self._cmd_input.setPlaceholderText("Send MAVLink command (e.g. ARM, RTL, LAND)…")
        self._cmd_input.returnPressed.connect(self._send_console_command)
        self._btn_send_cmd = QPushButton("Send ▶")
        self._btn_send_cmd.clicked.connect(self._send_console_command)
        cmd_row.addWidget(self._cmd_input)
        cmd_row.addWidget(self._btn_send_cmd)
        lay.addLayout(cmd_row)
        return w

    # ── Slots & Update Logic ─────────────────────────────────
    def _on_telemetry(self, d: TelemetryData):
        self._last_telem = d

        # HUD
        self._horizon.update_attitude(d.roll, d.pitch)
        self._compass.update_heading(d.yaw)

        # Metric cards
        self._card_alt.set_value(f"{d.rel_alt:.1f}", d.rel_alt < 0)
        self._card_spd.set_value(f"{d.airspeed:.1f}")
        self._card_gspd.set_value(f"{d.groundspeed:.1f}")
        self._card_vspd.set_value(f"{d.vz:.2f}", abs(d.vz) > 5)
        self._card_volt.set_value(f"{d.battery_voltage:.2f}",
                                   d.battery_voltage < 14.4)
        self._card_curr.set_value(f"{d.battery_current:.1f}",
                                   d.battery_current > 40)
        self._card_mah.set_value(f"{d.battery_consumed:.0f}")
        self._card_roll.set_value(f"{d.roll:+.1f}", abs(d.roll) > 45)
        self._card_pitch.set_value(f"{d.pitch:+.1f}", abs(d.pitch) > 45)
        self._card_yaw.set_value(f"{d.yaw:.1f}")
        self._card_rssi.set_value(f"{d.rssi}",  d.rssi < 30)

        self._battery.update_battery(
            d.battery_remaining, d.battery_voltage, d.battery_current)

        self._gps_widget.update_gps(
            d.fix_type, d.sats, d.hdop, d.lat, d.lon, d.alt)

        # Vehicle status
        arm_color = Theme.SUCCESS.name() if d.armed else Theme.DANGER.name()
        self._status_vals["Flight Mode"].setText(d.flight_mode)
        self._status_vals["Armed"].setText("ARMED" if d.armed else "DISARMED")
        self._status_vals["Armed"].setStyleSheet(f"color:{arm_color};font-weight:bold;font-size:9px;")
        self._status_vals["Autopilot"].setText(d.autopilot)
        self._status_vals["Type"].setText(d.mav_type)
        uptime = int(d.uptime / 20) if d.uptime else 0
        self._status_vals["Uptime"].setText(
            f"{uptime//3600:02d}:{(uptime%3600)//60:02d}:{uptime%60:02d}")
        self._status_vals["Sys ID"].setText(str(d.system_id))
        self._status_vals["Msg Rate"].setText(f"{d.message_rate} msg/s")
        self._status_vals["Pkt Loss"].setText(f"{d.packet_loss:.1f}%")

        # Waypoint
        self._wp_vals["Current WP"].setText(str(d.wp_num))
        self._wp_vals["WP Distance"].setText(f"{d.wp_dist:.1f} m")
        self._wp_vals["Total WPs"].setText(str(d.mission_total))

        # Sensor cards
        self._imu_cards["Accel X"].set_value(f"{d.accel_x:+.3f}")
        self._imu_cards["Accel Y"].set_value(f"{d.accel_y:+.3f}")
        self._imu_cards["Accel Z"].set_value(f"{d.accel_z:+.3f}")
        self._imu_cards["Gyro X"].set_value(f"{d.gyro_x:+.4f}")
        self._imu_cards["Gyro Y"].set_value(f"{d.gyro_y:+.4f}")
        self._imu_cards["Gyro Z"].set_value(f"{d.gyro_z:+.4f}")
        self._mag_cards["Mag X"].set_value(f"{d.mag_x:.1f}")
        self._mag_cards["Mag Y"].set_value(f"{d.mag_y:.1f}")
        self._mag_cards["Mag Z"].set_value(f"{d.mag_z:.1f}")
        self._baro_cards["Baro Alt"].set_value(f"{d.baro_alt:.2f}")
        self._baro_cards["Pressure"].set_value(f"{d.baro_press:.0f}")
        self._baro_cards["Temperature"].set_value(f"{d.temp:.1f}")
        self._vib_cards["Vibe X"].set_value(f"{d.vibe_x:.2f}", d.vibe_x > 15)
        self._vib_cards["Vibe Y"].set_value(f"{d.vibe_y:.2f}", d.vibe_y > 15)
        self._vib_cards["Vibe Z"].set_value(f"{d.vibe_z:.2f}", d.vibe_z > 15)

        # RC
        self._rc_widget.update_channels(d.channels)
        for i, val in enumerate(d.channels):
            self._rc_table.setItem(i, 2, QTableWidgetItem(str(val)))
            pct = int((val - 1000) / 10)
            self._rc_table.setItem(i, 3, QTableWidgetItem(f"{pct}%"))
        self._rssi_bar.setValue(d.rssi)
        self._rssi_lbl.setText(f"{d.rssi} %")

        # Sparkline
        self._spk_alt.push(d.rel_alt)

        # Toolbar / statusbar
        self._rate_lbl.setText(f"{d.message_rate} msg/s")
        self._sb_mode.setText(f"MODE: {d.flight_mode}")
        self._sb_armed.setText("ARMED" if d.armed else "DISARMED")
        self._sb_armed.setStyleSheet(f"color:{arm_color};font-weight:bold;font-size:9px;")
        self._sb_lat.setText(f"LAT: {d.lat:.6f}")
        self._sb_lon.setText(f"LON: {d.lon:.6f}")
        self._sb_alt.setText(f"ALT: {d.rel_alt:.1f}m")

        # Button state
        self._btn_arm.setEnabled(True)
        arm_txt = "🔓 DISARM" if d.armed else "⚠  ARM"
        self._btn_arm.setText(arm_txt)

        # History update
        for k in self._history:
            self._history[k].append(getattr(d, k, 0))

    def _refresh_charts(self):
        d = self._last_telem
        if not d.connected: return
        self._ch_attitude.push(d.roll, d.pitch, d.yaw)
        self._ch_altitude.push(d.rel_alt, d.vz)
        self._ch_speed.push(d.airspeed, d.groundspeed)
        self._ch_battery.push(d.battery_voltage, d.battery_current)
        self._ch_baro.push(d.baro_press/1000, d.temp)
        self._ch_rssi.push(d.rssi)
        self._imu_chart.push(d.accel_x, d.accel_y, d.accel_z)
        self._gyr_chart.push(d.gyro_x, d.gyro_y, d.gyro_z)
        self._vib_chart.push(d.vibe_x, d.vibe_y, d.vibe_z)

    def _on_message(self, level: str, text: str):
        self._console.log(level, text)

    def _on_connection_status(self, ok: bool, msg: str):
        self._console.log("SYS", msg)
        if ok:
            self._conn_indicator.setText("● ONLINE")
            self._conn_indicator.setStyleSheet(
                f"color:{Theme.SUCCESS.name()};font-weight:bold;padding:0 12px;")
            self._btn_connect.setText("⚡ DISCONNECT")
            self._btn_connect.setStyleSheet(
                f"background:{Theme.DANGER.name()};color:white;font-weight:bold;"
                f"border:none;border-radius:4px;padding:6px 16px;")
        else:
            self._conn_indicator.setText("● OFFLINE")
            self._conn_indicator.setStyleSheet(
                f"color:{Theme.DANGER.name()};font-weight:bold;padding:0 12px;")
            self._btn_connect.setText("⚡ CONNECT")
            self._btn_connect.setStyleSheet(
                f"background:{Theme.ACCENT_BLUE.name()};color:white;font-weight:bold;"
                f"border:none;border-radius:4px;padding:6px 16px;")

    def _show_connect_dialog(self):
        if self._worker.connected:
            self._worker.disconnect()
            return
        dlg = ConnectionDialog(self)
        if dlg.exec_() == QDialog.Accepted:
            cs = dlg.connection_string
            if cs.startswith("sim:") or not cs:
                self._worker._sim_mode = True
                self._worker.connected = True
                self._worker.connection_status.emit(True, "Simulation started")
                self._worker.message_received.emit("INFO",
                    "Simulation mode — all telemetry is synthetic")
            else:
                threading.Thread(
                    target=self._worker.connect,
                    args=(cs, dlg.baud), daemon=True).start()

    def _arm_disarm(self):
        if not self._worker.connected: return
        new_state = not self._last_telem.armed
        if new_state:
            r = QMessageBox.question(self, "Confirm ARM",
                "⚠  Are you sure you want to ARM the vehicle?",
                QMessageBox.Yes | QMessageBox.No)
            if r != QMessageBox.Yes: return
        self._worker.arm(new_state)
        self._console.log("SYS", f"{'ARM' if new_state else 'DISARM'} command sent")

    def _change_mode(self, mode: str):
        if self._worker.connected and not self._worker._sim_mode:
            self._set_mode(mode)

    def _set_mode(self, mode: str):
        self._console.log("SYS", f"Mode change requested: {mode}")
        idx = FLIGHT_MODES.index(mode) if mode in FLIGHT_MODES else -1
        if idx >= 0:
            self._mode_combo.setCurrentIndex(idx)

    def _send_console_command(self):
        text = self._cmd_input.text().strip()
        if not text: return
        self._console.log("SYS", f"Command > {text}")
        cmd = text.upper()
        if cmd == "ARM":    self._arm_disarm()
        elif cmd == "RTL":  self._set_mode("RTL")
        elif cmd == "LAND": self._set_mode("LAND")
        else:
            self._console.log("WARN", f"Unknown command: {text}")
        self._cmd_input.clear()

    def _tick_uptime(self):
        t = int(time.time() - self._start_time)
        self._sb_time.setText(
            f"T+ {t//3600:02d}:{(t%3600)//60:02d}:{t%60:02d}")

    def closeEvent(self, event):
        self._worker.stop()
        event.accept()


# ─────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────
def main():
    # High-DPI flags MUST be set before QApplication is created
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    except Exception:
        pass

    app = QApplication(sys.argv)
    app.setApplicationName("UAV Ground Station")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("ProGS")

    # Dark palette
    pal = QPalette()
    pal.setColor(QPalette.Window,       Theme.BG_DARK)
    pal.setColor(QPalette.WindowText,   Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Base,         Theme.BG_CARD)
    pal.setColor(QPalette.AlternateBase,Theme.BG_CARD2)
    pal.setColor(QPalette.ToolTipBase,  Theme.BG_CARD2)
    pal.setColor(QPalette.ToolTipText,  Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Text,         Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Button,       Theme.BG_CARD)
    pal.setColor(QPalette.ButtonText,   Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Link,         Theme.ACCENT_CYAN)
    pal.setColor(QPalette.Highlight,    Theme.ACCENT_BLUE)
    pal.setColor(QPalette.HighlightedText, QColor("white"))
    app.setPalette(pal)

    win = GroundStation()
    win.show()

    # Auto-start simulation for demo
    win._worker._sim_mode  = True
    win._worker.connected  = True
    win._worker.connection_status.emit(True, "Simulation mode active — click CONNECT to use real hardware")
    win._console.log("INFO", "UAV Ground Station v1.0 — Professional Edition")
    win._console.log("INFO", "Simulation mode running. Connect to real UAV via CONNECT button.")
    win._console.log("INFO", "Supported: UDP, TCP, Serial (USB), UDP Server")

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()