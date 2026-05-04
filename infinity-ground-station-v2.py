"""
╔══════════════════════════════════════════════════════════════════╗
║        UAV GROUND STATION — Professional Edition  v2.0           ║
║        MAVLink-based | PyQt5 | Dual-Window Layout                ║
╠══════════════════════════════════════════════════════════════════╣
║  Window 1 — FLIGHT VIEW  : HUD, Compass, Map, primary metrics    ║
║  Window 2 — TELEMETRY    : Sensors, EKF, Charts, Params, Console ║
╚══════════════════════════════════════════════════════════════════╝

Requirements:
    pip install PyQt5 pymavlink pyserial numpy

Run:
    python uav_ground_station.py
"""

import sys, math, time, threading, random
from datetime import datetime
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QTabWidget, QTextEdit, QGroupBox, QSpinBox,
    QProgressBar, QStatusBar, QToolBar, QTableWidget, QTableWidgetItem,
    QHeaderView, QSizePolicy, QDialog, QDialogButtonBox,
    QFormLayout, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QPointF, QRectF, QSize
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QFont, QLinearGradient,
    QRadialGradient, QPainterPath, QPolygonF, QPalette
)

import numpy as np

# ─────────────────────────────────────────────────────────────────
#  THEME
# ─────────────────────────────────────────────────────────────────
class Theme:
    BG_DARK       = QColor("#0A0E14")
    BG_PANEL      = QColor("#0D1117")
    BG_CARD       = QColor("#161B22")
    BG_CARD2      = QColor("#1C2333")
    BORDER        = QColor("#21262D")
    BORDER_ACCENT = QColor("#30363D")
    TEXT_PRIMARY  = QColor("#E6EDF3")
    TEXT_SECOND   = QColor("#8B949E")
    TEXT_MUTED    = QColor("#484F58")
    ACCENT_CYAN   = QColor("#58A6FF")
    ACCENT_GREEN  = QColor("#3FB950")
    ACCENT_ORANGE = QColor("#D29922")
    ACCENT_RED    = QColor("#F85149")
    ACCENT_PURPLE = QColor("#BC8CFF")
    ACCENT_BLUE   = QColor("#1F6FEB")
    WARNING       = QColor("#D29922")
    DANGER        = QColor("#F85149")
    SUCCESS       = QColor("#3FB950")
    INFO          = QColor("#58A6FF")


def shared_stylesheet():
    t = Theme
    return f"""
        QWidget {{
            background:{t.BG_DARK.name()}; color:{t.TEXT_PRIMARY.name()};
            font-family:'Courier New';
        }}
        QTabWidget::pane {{ border:1px solid {t.BORDER.name()}; background:{t.BG_PANEL.name()}; }}
        QTabBar::tab {{
            background:{t.BG_CARD.name()}; color:{t.TEXT_SECOND.name()};
            padding:6px 18px; border:1px solid {t.BORDER.name()};
            border-bottom:none; font-size:10px;
        }}
        QTabBar::tab:selected {{
            background:{t.BG_PANEL.name()}; color:{t.ACCENT_CYAN.name()};
            border-top:2px solid {t.ACCENT_CYAN.name()};
        }}
        QPushButton {{
            background:{t.BG_CARD.name()}; color:{t.TEXT_PRIMARY.name()};
            border:1px solid {t.BORDER_ACCENT.name()}; border-radius:5px;
            padding:6px 14px; font-size:10px;
        }}
        QPushButton:hover {{ border-color:{t.ACCENT_CYAN.name()}; color:{t.ACCENT_CYAN.name()}; }}
        QPushButton:pressed {{ background:{t.ACCENT_CYAN.name()}; color:black; }}
        QComboBox {{
            background:{t.BG_CARD.name()}; color:{t.TEXT_PRIMARY.name()};
            border:1px solid {t.BORDER_ACCENT.name()}; border-radius:4px; padding:4px 8px;
        }}
        QComboBox::drop-down {{ border:none; }}
        QComboBox QAbstractItemView {{
            background:{t.BG_CARD2.name()}; color:{t.TEXT_PRIMARY.name()};
            selection-background-color:{t.ACCENT_BLUE.name()};
        }}
        QScrollBar:vertical {{ background:{t.BG_DARK.name()}; width:8px; }}
        QScrollBar::handle:vertical {{ background:{t.BORDER_ACCENT.name()}; border-radius:4px; }}
        QGroupBox {{
            border:1px solid {t.BORDER.name()}; border-radius:6px; margin-top:10px;
            font-size:10px; color:{t.TEXT_SECOND.name()};
        }}
        QGroupBox::title {{ subcontrol-origin:margin; padding:0 6px; }}
        QTableWidget {{
            background:{t.BG_CARD.name()}; gridline-color:{t.BORDER.name()};
            border:none; font-size:10px;
        }}
        QTableWidget::item {{ padding:4px; }}
        QTableWidget::item:selected {{ background:{t.ACCENT_BLUE.name()}; }}
        QHeaderView::section {{
            background:{t.BG_CARD2.name()}; color:{t.TEXT_SECOND.name()};
            border:none; padding:4px 8px; font-size:9px;
        }}
        QStatusBar {{ background:{t.BG_CARD.name()}; color:{t.TEXT_SECOND.name()}; font-size:9px; }}
        QToolBar {{
            background:{t.BG_CARD.name()}; border-bottom:1px solid {t.BORDER.name()};
            spacing:4px; padding:4px;
        }}
        QLineEdit {{
            background:{t.BG_DARK.name()}; color:{t.TEXT_PRIMARY.name()};
            border:1px solid {t.BORDER_ACCENT.name()}; border-radius:4px; padding:4px 8px;
        }}
    """


# ─────────────────────────────────────────────────────────────────
#  TELEMETRY DATA MODEL
# ─────────────────────────────────────────────────────────────────
class TelemetryData:
    def __init__(self):
        self.roll = self.pitch = self.yaw = 0.0
        self.lat = 44.4949; self.lon = 11.3426
        self.alt = self.rel_alt = 0.0
        self.fix_type = 3; self.sats = 14; self.hdop = 0.9
        self.vx = self.vy = self.vz = 0.0
        self.airspeed = self.groundspeed = 0.0
        self.battery_voltage = 16.8; self.battery_current = 0.0
        self.battery_remaining = 100; self.battery_consumed = 0.0
        self.flight_mode = "STABILIZE"; self.armed = False
        self.autopilot = "ArduPilot"; self.mav_type = "Quadcopter"
        self.system_id = 1
        self.rssi = 100; self.channels = [1500] * 8
        self.baro_alt = 0.0; self.baro_press = 101325.0; self.temp = 25.0
        self.accel_x = self.accel_y = 0.0; self.accel_z = -9.81
        self.gyro_x = self.gyro_y = self.gyro_z = 0.0
        self.mag_x = self.mag_y = self.mag_z = 0.0
        self.vibe_x = self.vibe_y = self.vibe_z = 0.0
        self.ekf_flags = 0x1F
        self.wp_num = 0; self.wp_dist = 0.0; self.mission_total = 0
        self.timestamp = time.time(); self.uptime = 0
        self.message_rate = 0; self.connected = False; self.packet_loss = 0.0


# ─────────────────────────────────────────────────────────────────
#  MAVLINK WORKER THREAD
# ─────────────────────────────────────────────────────────────────
class MAVLinkWorker(QThread):
    telemetry_updated = pyqtSignal(object)
    message_received  = pyqtSignal(str, str)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.running = False; self.connected = False
        self.mavconn = None; self.telem = TelemetryData()
        self._sim_mode = True; self._sim_t = 0.0

    def connect_vehicle(self, cs, baud=57600):
        try:
            from pymavlink import mavutil
            self.mavconn = mavutil.mavlink_connection(cs, baud=baud, autoreconnect=True)
            self.mavconn.wait_heartbeat(timeout=5)
            self.connected = True; self._sim_mode = False
            self.connection_status.emit(True, f"Connected → {cs}")
            self._request_streams()
        except ImportError:
            self._sim_mode = True; self.connected = True
            self.connection_status.emit(True, "SIMULATION MODE (pymavlink not installed)")
            self.message_received.emit("WARN", "pymavlink not found — running simulation")
        except Exception as e:
            self.connected = False
            self.connection_status.emit(False, str(e))
            self.message_received.emit("ERROR", str(e))

    def disconnect(self):
        self.connected = False; self._sim_mode = True
        if self.mavconn:
            try: self.mavconn.close()
            except: pass
            self.mavconn = None
        self.connection_status.emit(False, "Disconnected")

    def send_command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        if self._sim_mode: return
        try:
            from pymavlink import mavutil
            self.mavconn.mav.command_long_send(
                self.mavconn.target_system, self.mavconn.target_component,
                cmd, 0, p1, p2, p3, p4, p5, p6, p7)
        except Exception as e:
            self.message_received.emit("ERROR", f"Command failed: {e}")

    def arm(self, state):
        from pymavlink import mavutil
        self.send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if state else 0.0)

    def _request_streams(self):
        if not self.mavconn: return
        try:
            from pymavlink import mavutil
            for sid, rate in [
                (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,     10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,  5),
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION,        10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,          20),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,          10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,           5),
            ]:
                self.mavconn.mav.request_data_stream_send(
                    self.mavconn.target_system, self.mavconn.target_component, sid, rate, 1)
        except: pass

    def _simulate(self):
        t = self._sim_t; self._sim_t += 0.05
        d = self.telem; d.connected = True
        d.roll        = 15 * math.sin(t * 0.3)
        d.pitch       = 8  * math.sin(t * 0.2 + 1)
        d.yaw         = (t * 20) % 360
        d.rel_alt     = 50 + 20 * math.sin(t * 0.1)
        d.alt         = d.rel_alt + 100
        d.airspeed    = 12 + 3 * math.sin(t * 0.15)
        d.groundspeed = d.airspeed * 0.95
        d.vx          = d.groundspeed * math.cos(math.radians(d.yaw))
        d.vy          = d.groundspeed * math.sin(math.radians(d.yaw))
        d.vz          = d.vz * 0.9 + random.gauss(0, 0.05)
        d.battery_voltage   = max(12.0, 16.8 - t * 0.002)
        d.battery_current   = 15 + 5 * math.sin(t * 0.4)
        d.battery_remaining = max(0, int(100 - t * 0.1))
        d.battery_consumed += d.battery_current * (1/20) / 3.6
        d.baro_alt    = d.rel_alt + random.gauss(0, 0.1)
        d.baro_press  = 101325 - d.alt * 12
        d.temp        = 25 + random.gauss(0, 0.05)
        d.rssi        = int(85 + 10 * math.sin(t * 0.07))
        d.accel_x     = random.gauss(0, 0.05)
        d.accel_y     = random.gauss(0, 0.05)
        d.accel_z     = -9.81 + random.gauss(0, 0.02)
        d.gyro_x      = math.radians(d.roll  * 0.1) + random.gauss(0, 0.001)
        d.gyro_y      = math.radians(d.pitch * 0.1) + random.gauss(0, 0.001)
        d.gyro_z      = math.radians(20) + random.gauss(0, 0.001)
        d.vibe_x      = abs(random.gauss(0, 2))
        d.vibe_y      = abs(random.gauss(0, 2))
        d.vibe_z      = abs(random.gauss(0, 2))
        d.lat        += 0.000001 * math.cos(math.radians(d.yaw))
        d.lon        += 0.000001 * math.sin(math.radians(d.yaw))
        d.sats        = 14; d.uptime = int(t * 20)
        d.flight_mode = "AUTO" if t > 30 else "STABILIZE"
        d.armed = True; d.timestamp = time.time()

    def _parse_mavlink(self):
        try:
            msg = self.mavconn.recv_match(blocking=False)
            if msg is None: return
            t = msg.get_type(); d = self.telem
            if t == 'HEARTBEAT':
                d.armed = bool(msg.base_mode & 128)
                modes = {0:"STABILIZE",2:"ALT_HOLD",3:"AUTO",4:"GUIDED",5:"LOITER",
                         6:"RTL",7:"CIRCLE",9:"LAND",16:"POSHOLD"}
                d.flight_mode = modes.get(msg.custom_mode, str(msg.custom_mode))
                d.connected = True
            elif t == 'ATTITUDE':
                d.roll = math.degrees(msg.roll); d.pitch = math.degrees(msg.pitch)
                d.yaw  = math.degrees(msg.yaw) % 360
            elif t == 'GLOBAL_POSITION_INT':
                d.lat = msg.lat/1e7; d.lon = msg.lon/1e7
                d.alt = msg.alt/1000; d.rel_alt = msg.relative_alt/1000
                d.vx = msg.vx/100; d.vy = msg.vy/100; d.vz = msg.vz/100
                d.groundspeed = math.hypot(d.vx, d.vy)
            elif t == 'SYS_STATUS':
                d.battery_voltage   = msg.voltage_battery / 1000
                d.battery_current   = msg.current_battery / 100
                d.battery_remaining = msg.battery_remaining
                d.packet_loss       = msg.drop_rate_comm / 100
            elif t == 'GPS_RAW_INT':
                d.fix_type = msg.fix_type; d.sats = msg.satellites_visible
                d.hdop = msg.eph / 100
            elif t == 'VFR_HUD':
                d.airspeed = msg.airspeed; d.groundspeed = msg.groundspeed
                d.baro_alt = msg.alt
            elif t == 'SCALED_PRESSURE':
                d.baro_press = msg.press_abs * 100; d.temp = msg.temperature / 100
            elif t == 'RAW_IMU':
                d.accel_x = msg.xacc/1000*9.81; d.accel_y = msg.yacc/1000*9.81
                d.accel_z = msg.zacc/1000*9.81
                d.gyro_x  = msg.xgyro/1000; d.gyro_y = msg.ygyro/1000; d.gyro_z = msg.zgyro/1000
                d.mag_x   = msg.xmag; d.mag_y = msg.ymag; d.mag_z = msg.zmag
            elif t == 'VIBRATION':
                d.vibe_x = msg.vibration_x; d.vibe_y = msg.vibration_y; d.vibe_z = msg.vibration_z
            elif t == 'RC_CHANNELS':
                d.rssi = msg.rssi
                d.channels = [msg.chan1_raw,msg.chan2_raw,msg.chan3_raw,msg.chan4_raw,
                               msg.chan5_raw,msg.chan6_raw,msg.chan7_raw,msg.chan8_raw]
            elif t == 'MISSION_CURRENT': d.wp_num = msg.seq
            elif t == 'NAV_CONTROLLER_OUTPUT': d.wp_dist = msg.wp_dist
            d.timestamp = time.time()
        except Exception as e:
            self.message_received.emit("WARN", f"Parse: {e}")

    def run(self):
        self.running = True; _last_emit = 0; _pkts = 0; _rate_t = time.time()
        while self.running:
            if self.connected:
                if self._sim_mode:
                    self._simulate(); time.sleep(0.05)
                else:
                    self._parse_mavlink()
                _pkts += 1
                now = time.time()
                if now - _rate_t >= 1.0:
                    self.telem.message_rate = _pkts; _pkts = 0; _rate_t = now
                if now - _last_emit >= 0.05:
                    self.telemetry_updated.emit(self.telem); _last_emit = now
            else:
                time.sleep(0.1)

    def stop(self):
        self.running = False; self.wait()


# ─────────────────────────────────────────────────────────────────
#  CUSTOM WIDGETS
# ─────────────────────────────────────────────────────────────────

class ArtificialHorizon(QWidget):
    def __init__(self):
        super().__init__()
        self.roll = self.pitch = 0.0
        self.setMinimumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_attitude(self, roll, pitch):
        self.roll = roll; self.pitch = pitch; self.update()

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        sz = min(w, h); r = sz / 2 - 4; cx, cy = w / 2, h / 2
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        clip = QPainterPath(); clip.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(clip)
        p.save(); p.translate(cx, cy); p.rotate(-self.roll)
        pp = self.pitch * (sz / 90.0)
        g = QLinearGradient(0, -r, 0, pp)
        g.setColorAt(0, QColor("#0A1929")); g.setColorAt(1, QColor("#1565C0"))
        p.fillRect(QRectF(-r, -r*2+pp, r*2, r*2), g)
        g2 = QLinearGradient(0, pp, 0, r)
        g2.setColorAt(0, QColor("#5D4037")); g2.setColorAt(1, QColor("#3E2723"))
        p.fillRect(QRectF(-r, pp, r*2, r*2), g2)
        p.setPen(QPen(QColor("#00E5FF"), 2))
        p.drawLine(QPointF(-r, pp), QPointF(r, pp))
        p.setPen(QPen(Qt.white, 1)); p.setFont(QFont("Courier New", 7))
        for deg in range(-30, 31, 5):
            if deg == 0: continue
            y = pp - deg * (sz / 90.0); lw = r*0.25 if deg%10==0 else r*0.12
            p.drawLine(QPointF(-lw, y), QPointF(lw, y))
            if deg % 10 == 0:
                p.drawText(QPointF(lw+4, y+4), str(abs(deg)))
                p.drawText(QPointF(-lw-20, y+4), str(abs(deg)))
        p.restore(); p.setClipping(False); p.translate(cx, cy)
        from PyQt5.QtCore import QRect
        p.setPen(QPen(QColor("#00E5FF"), 1))
        p.drawArc(QRect(int(-r), int(-r), int(r*2), int(r*2)), 30*16, 120*16)
        for a in [-60,-45,-30,-20,-10,0,10,20,30,45,60]:
            rad = math.radians(a-90); tk = 10 if a%30==0 else 5
            p.drawLine(QPointF((r-tk)*math.cos(rad),(r-tk)*math.sin(rad)),
                       QPointF(r*math.cos(rad), r*math.sin(rad)))
        p.save(); p.rotate(-self.roll); p.setPen(QPen(QColor("#00E5FF"), 2))
        poly = QPolygonF([QPointF(0,-r+2),QPointF(-8,-r+18),QPointF(8,-r+18)])
        p.setBrush(QBrush(QColor("#00E5FF"))); p.drawPolygon(poly); p.restore()
        p.setPen(QPen(QColor("#FFD600"), 2.5))
        p.drawLine(QPointF(-40,0),QPointF(-15,0)); p.drawLine(QPointF(-15,0),QPointF(0,8))
        p.drawLine(QPointF(0,8),QPointF(15,0));   p.drawLine(QPointF(15,0),QPointF(40,0))
        p.drawLine(QPointF(0,8),QPointF(0,-10))
        p.setPen(QPen(Theme.BORDER_ACCENT, 2)); p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(0,0), r, r)
        p.setFont(QFont("Courier New", 8, QFont.Bold)); p.setPen(QPen(QColor("#00E5FF")))
        p.drawText(QPointF(-r, -r+14), f"R {self.roll:+.1f}°  P {self.pitch:+.1f}°")


class CompassRose(QWidget):
    def __init__(self):
        super().__init__()
        self.heading = 0.0
        self.setMinimumSize(180, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def update_heading(self, yaw):
        self.heading = yaw; self.update()

    def paintEvent(self, _):
        w, h = self.width(), self.height()
        sz = min(w,h); r = sz/2-6; cx, cy = w/2, h/2
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing); p.translate(cx, cy)
        bg = QRadialGradient(0, 0, r)
        bg.setColorAt(0, QColor("#1C2333")); bg.setColorAt(1, QColor("#0D1117"))
        p.setBrush(QBrush(bg)); p.setPen(QPen(Theme.BORDER_ACCENT, 1))
        p.drawEllipse(QPointF(0,0), r, r)
        p.rotate(-self.heading)
        for deg in range(0, 360, 5):
            rad = math.radians(deg-90)
            big = deg%45==0; med = deg%10==0
            tk = 14 if big else (9 if med else 5)
            p.setPen(QPen(QColor("#58A6FF") if big else QColor("#484F58"), 2 if big else 1))
            p.drawLine(QPointF((r-tk)*math.cos(rad),(r-tk)*math.sin(rad)),
                       QPointF(r*math.cos(rad), r*math.sin(rad)))
        for deg, lbl in {0:'N',90:'E',180:'S',270:'W',45:'NE',135:'SE',225:'SW',315:'NW'}.items():
            rad = math.radians(deg-90); lx=(r-26)*math.cos(rad); ly=(r-26)*math.sin(rad)
            big = lbl in ('N','S','E','W')
            p.setPen(QPen(QColor("#F85149") if lbl=='N' else QColor("#E6EDF3" if big else "#8B949E")))
            p.setFont(QFont("Courier New", 9 if big else 7, QFont.Bold if big else QFont.Normal))
            p.save(); p.translate(lx,ly); p.rotate(self.heading)
            p.drawText(QRectF(-12,-12,24,24), Qt.AlignCenter, lbl); p.restore()
        p.rotate(self.heading)
        p.setBrush(QBrush(QColor("#F85149"))); p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([QPointF(0,-r+22),QPointF(-7,-r+40),QPointF(7,-r+40)]))
        p.setBrush(QBrush(QColor("#8B949E")))
        p.drawPolygon(QPolygonF([QPointF(0,r-22),QPointF(-7,r-40),QPointF(7,r-40)]))
        p.rotate(self.heading)
        p.setPen(QPen(QColor("#58A6FF"))); p.setFont(QFont("Courier New", 12, QFont.Bold))
        p.drawText(QRectF(-30,-14,60,28), Qt.AlignCenter, f"{int(self.heading):03d}°")


class MetricCard(QWidget):
    def __init__(self, title, unit="", color=None, icon=""):
        super().__init__()
        self._color = color or Theme.ACCENT_CYAN
        self.setAutoFillBackground(True)
        pal = self.palette(); pal.setColor(self.backgroundRole(), Theme.BG_CARD)
        self.setPalette(pal)
        self.setMinimumHeight(84)
        lay = QVBoxLayout(self); lay.setContentsMargins(12,8,12,8); lay.setSpacing(2)
        t_lbl = QLabel(f"{icon}  {title}" if icon else title)
        t_lbl.setStyleSheet(f"color:{Theme.TEXT_SECOND.name()};font:10px 'Courier New';background:transparent;")
        self._val = QLabel("—")
        self._val.setStyleSheet(f"color:{self._color.name()};font:bold 22px 'Courier New';background:transparent;")
        self._sub = QLabel(unit)
        self._sub.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()};font:9px 'Courier New';background:transparent;")
        lay.addWidget(t_lbl); lay.addWidget(self._val); lay.addWidget(self._sub)

    def set_value(self, v, warn=False, sub=""):
        self._val.setText(str(v))
        if sub: self._sub.setText(sub)
        c = Theme.DANGER if warn else self._color
        self._val.setStyleSheet(f"color:{c.name()};font:bold 22px 'Courier New';background:transparent;")

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QBrush(Theme.BG_CARD)); p.setPen(QPen(Theme.BORDER, 1))
        p.drawRoundedRect(self.rect().adjusted(0,0,-1,-1), 8, 8)


class Sparkline(QWidget):
    def __init__(self, color=None, max_pts=200):
        super().__init__()
        self._data = deque(maxlen=max_pts); self._color = color or Theme.ACCENT_CYAN
        self.setMinimumHeight(50); self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def push(self, v):
        self._data.append(v); self.update()

    def paintEvent(self, _):
        if len(self._data) < 2: return
        w, h = self.width(), self.height()
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        data = list(self._data); mn,mx = min(data),max(data); rng = mx-mn or 1
        def pt(i,v): return QPointF(i/(len(data)-1)*w, h-(v-mn)/rng*(h-8)-4)
        path = QPainterPath(); path.moveTo(pt(0,data[0]))
        for i in range(1,len(data)): path.lineTo(pt(i,data[i]))
        fill = QPainterPath(path); fill.lineTo(QPointF(w,h)); fill.lineTo(QPointF(0,h)); fill.closeSubpath()
        g = QLinearGradient(0,0,0,h); c=QColor(self._color); c.setAlpha(80)
        g.setColorAt(0,c); c.setAlpha(0); g.setColorAt(1,c)
        p.fillPath(fill,g); p.setPen(QPen(self._color,1.5)); p.drawPath(path)
        lp = pt(len(data)-1,data[-1]); p.setBrush(QBrush(self._color)); p.setPen(Qt.NoPen)
        p.drawEllipse(lp, 3, 3)


class BatteryWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.percent=100; self.voltage=0.0; self.current=0.0
        self.setMinimumSize(120,52); self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def update_battery(self, pct, v, a):
        self.percent=pct; self.voltage=v; self.current=a; self.update()

    def paintEvent(self, _):
        w,h = self.width(),self.height()
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        bw,bh = w-16,22; by=(h-bh)//2; bx=2
        p.setPen(QPen(Theme.BORDER_ACCENT,1.5)); p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(bx,by,bw-10,bh,3,3)
        p.fillRect(bx+bw-10,by+6,8,bh-12,Theme.BORDER_ACCENT)
        pct = max(0,min(100,self.percent))/100; fw = int((bw-14)*pct)
        col = Theme.SUCCESS if pct>0.4 else (Theme.WARNING if pct>0.2 else Theme.DANGER)
        if fw>0: p.fillRect(bx+2,by+2,fw,bh-4,col)
        p.setPen(QPen(Qt.white)); p.setFont(QFont("Courier New",8,QFont.Bold))
        p.drawText(QRectF(bx,by,bw-10,bh), Qt.AlignCenter, f"{self.percent}%")
        p.setPen(QPen(Theme.TEXT_SECOND)); p.setFont(QFont("Courier New",7))
        p.drawText(QRectF(0,by+bh+2,w//2,12), Qt.AlignLeft, f"{self.voltage:.2f}V")
        p.drawText(QRectF(w//2,by+bh+2,w//2,12), Qt.AlignRight, f"{self.current:.1f}A")


class GpsWidget(QWidget):
    FIX = {0:"No Fix",1:"No Fix",2:"2D Fix",3:"3D Fix",4:"DGPS",5:"RTK Float",6:"RTK Fixed"}

    def __init__(self):
        super().__init__()
        self.fix_type=0; self.sats=0; self.hdop=99.9
        self.lat=self.lon=self.alt=0.0; self.setMinimumHeight(60)

    def update_gps(self, fix, sats, hdop, lat, lon, alt):
        self.fix_type=fix; self.sats=sats; self.hdop=hdop
        self.lat=lat; self.lon=lon; self.alt=alt; self.update()

    def paintEvent(self, _):
        w,h = self.width(),self.height()
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        color = Theme.SUCCESS if self.fix_type>=3 else (Theme.WARNING if self.fix_type==2 else Theme.DANGER)
        r=10; p.setBrush(QBrush(color)); p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(r+4,h/2), r, r)
        p.setPen(QPen(Theme.TEXT_PRIMARY)); p.setFont(QFont("Courier New",9,QFont.Bold))
        p.drawText(QRectF(28,0,w-28,h/2), Qt.AlignVCenter|Qt.AlignLeft,
                   f"{self.FIX.get(self.fix_type,'?')}  {self.sats}sat  HDOP:{self.hdop:.1f}")
        p.setPen(QPen(Theme.TEXT_SECOND)); p.setFont(QFont("Courier New",8))
        p.drawText(QRectF(28,h/2,w-28,h/2), Qt.AlignVCenter|Qt.AlignLeft,
                   f"Lat:{self.lat:.6f}   Lon:{self.lon:.6f}   Alt:{self.alt:.1f}m")


class MapWidget(QWidget):
    """Built-in vector map with trail, zoom and UAV icon."""
    def __init__(self):
        super().__init__()
        self.lat=44.4949; self.lon=11.3426; self.heading=0.0
        self.trail = deque(maxlen=600); self._zoom = 1.0
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(200)

    def update_position(self, lat, lon, heading):
        self.trail.append((lat,lon)); self.lat=lat; self.lon=lon; self.heading=heading; self.update()

    def wheelEvent(self, e):
        self._zoom = max(0.1, min(50.0, self._zoom * (1.12 if e.angleDelta().y()>0 else 0.89)))
        self.update()

    def paintEvent(self, _):
        w,h = self.width(),self.height(); cx,cy = w//2,h//2
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(0,0,w,h, QColor("#0D1117"))
        p.setPen(QPen(QColor("#1C2333"), 1))
        step = max(15, int(40 * self._zoom))
        for x in range(cx%step, w, step): p.drawLine(x,0,x,h)
        for y in range(cy%step, h, step): p.drawLine(0,y,w,y)
        scale = 0.0001 / self._zoom
        def w2s(lat,lon):
            return QPointF(cx+(lon-self.lon)/scale, cy-(lat-self.lat)/scale)
        trail = list(self.trail)
        if len(trail) > 1:
            p.setPen(QPen(QColor(88,166,255,100), 2))
            for i in range(1,len(trail)): p.drawLine(w2s(*trail[i-1]), w2s(*trail[i]))
        sp = w2s(self.lat, self.lon)
        p.save(); p.translate(sp); p.rotate(self.heading)
        body = QPolygonF([QPointF(0,-14),QPointF(-8,8),QPointF(0,4),QPointF(8,8)])
        p.setBrush(QBrush(QColor("#58A6FF"))); p.setPen(QPen(Qt.white,1)); p.drawPolygon(body)
        p.restore()
        p.setPen(QPen(Theme.TEXT_SECOND)); p.setFont(QFont("Courier New",8))
        p.drawText(6, h-22, f"Lat: {self.lat:.6f}   Lon: {self.lon:.6f}")
        p.drawText(6, h-8,  f"Zoom: {self._zoom:.1f}×   (scroll to zoom)")
        r2=18; ox,oy=w-28,28
        p.setPen(QPen(QColor("#484F58"),1)); p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(ox,oy), r2, r2)
        rad = math.radians(self.heading-90)
        p.setPen(QPen(QColor("#F85149"),2))
        p.drawLine(QPointF(ox,oy), QPointF(ox+r2*math.cos(rad), oy+r2*math.sin(rad)))


class ChartPanel(QWidget):
    def __init__(self, title, n_traces, colors, labels, y_min=None, y_max=None):
        super().__init__()
        self.title=title; self.colors=colors; self.labels=labels
        self.y_min=y_min; self.y_max=y_max
        self._data=[deque(maxlen=300) for _ in range(n_traces)]
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(100)

    def push(self, *values):
        for i,v in enumerate(values):
            if i<len(self._data): self._data[i].append(v)
        self.update()

    def paintEvent(self, _):
        w,h = self.width(),self.height()
        if w<10 or h<10: return
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        pad=28; p.fillRect(0,0,w,h, Theme.BG_CARD)
        p.setPen(QPen(Theme.TEXT_SECOND)); p.setFont(QFont("Courier New",8,QFont.Bold))
        p.drawText(4,12,self.title)
        p.setPen(QPen(QColor(Theme.BORDER.red(),Theme.BORDER.green(),Theme.BORDER.blue(),120), 0.5))
        for i in range(1,4):
            y=pad+(h-pad*2)*i/4; p.drawLine(QPointF(pad,y),QPointF(w-4,y))
        all_v=[v for d in self._data for v in d]
        if not all_v: return
        mn=self.y_min if self.y_min is not None else min(all_v)
        mx=self.y_max if self.y_max is not None else max(all_v)
        rng=mx-mn or 1
        def pt(i,v,n): return QPointF(pad+i/max(n-1,1)*(w-pad-4), h-pad-(v-mn)/rng*(h-pad*2))
        for ti,data in enumerate(self._data):
            lst=list(data)
            if len(lst)<2: continue
            col=self.colors[ti] if ti<len(self.colors) else Qt.white
            path=QPainterPath(); path.moveTo(pt(0,lst[0],len(lst)))
            for i in range(1,len(lst)): path.lineTo(pt(i,lst[i],len(lst)))
            p.setPen(QPen(col,1.5)); p.drawPath(path)
        x_leg=pad
        for ti,lbl in enumerate(self.labels):
            if ti>=len(self._data): break
            col=self.colors[ti] if ti<len(self.colors) else Qt.white
            p.setPen(QPen(col)); p.setFont(QFont("Courier New",7))
            data=list(self._data[ti]); val=f"{data[-1]:.2f}" if data else "—"
            p.fillRect(int(x_leg),h-14,8,8,col)
            p.drawText(int(x_leg+10),h-6,f"{lbl}:{val}"); x_leg+=80
        p.setPen(QPen(Theme.TEXT_MUTED)); p.setFont(QFont("Courier New",6))
        p.drawText(2,pad+4,f"{mx:.1f}"); p.drawText(2,h-pad,f"{mn:.1f}")


class ConsoleWidget(QTextEdit):
    LEVEL_COLORS = {"INFO":"#58A6FF","WARN":"#D29922","ERROR":"#F85149","MAV":"#3FB950","SYS":"#BC8CFF"}
    MAX_LINES = 1000

    def __init__(self):
        super().__init__()
        self.setReadOnly(True); self.setFont(QFont("Courier New", 8))
        self.setStyleSheet(f"QTextEdit {{ background:{Theme.BG_DARK.name()}; color:{Theme.TEXT_PRIMARY.name()}; border:none; }}")
        self._line_count = 0

    def log(self, level, text):
        ts    = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = self.LEVEL_COLORS.get(level, "#FFFFFF")
        self.append(
            f'<span style="color:#484F58">[{ts}]</span> '
            f'<span style="color:{color};font-weight:bold">[{level:5s}]</span> '
            f'<span style="color:#E6EDF3">{text}</span>')
        self._line_count += 1
        if self._line_count > self.MAX_LINES:
            cur = self.textCursor()
            cur.movePosition(cur.Start)
            cur.movePosition(cur.Down, cur.KeepAnchor, 50)
            cur.removeSelectedText(); self._line_count -= 50
        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())


# ─────────────────────────────────────────────────────────────────
#  SHARED TOOLBAR / CONTROLS MIXIN
# ─────────────────────────────────────────────────────────────────
FLIGHT_MODES = [
    "STABILIZE","ACRO","ALT_HOLD","AUTO","GUIDED","LOITER","RTL",
    "CIRCLE","LAND","DRIFT","SPORT","POSHOLD","BRAKE","SMART_RTL"
]


class ControlsMixin:
    """Mixed into both QMainWindow subclasses to share control logic."""

    def _init_controls(self, worker, console):
        self._worker  = worker
        self._console = console
        self._last_d  = TelemetryData()

    def _make_toolbar(self):
        tb = QToolBar("Controls", self); tb.setMovable(False)
        self.addToolBar(tb)

        self._btn_connect = QPushButton("⚡ CONNECT")
        self._btn_connect.setStyleSheet(
            f"background:{Theme.ACCENT_BLUE.name()};color:white;font-weight:bold;"
            f"border:none;border-radius:4px;padding:6px 16px;")
        self._btn_connect.clicked.connect(self._show_connect_dialog)
        tb.addWidget(self._btn_connect); tb.addSeparator()

        self._btn_arm = QPushButton("⚠  ARM")
        self._btn_arm.setStyleSheet(
            f"background:{Theme.DANGER.name()};color:white;font-weight:bold;"
            f"border:none;border-radius:4px;padding:6px 16px;")
        self._btn_arm.clicked.connect(self._arm_disarm)
        self._btn_arm.setEnabled(False)
        tb.addWidget(self._btn_arm); tb.addSeparator()

        lbl = QLabel("Mode: "); lbl.setStyleSheet("color:#8B949E;padding:0 4px;")
        tb.addWidget(lbl)
        self._mode_combo = QComboBox(); self._mode_combo.addItems(FLIGHT_MODES)
        self._mode_combo.setFixedWidth(140)
        self._mode_combo.currentTextChanged.connect(self._change_mode)
        tb.addWidget(self._mode_combo); tb.addSeparator()

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

        sp = QWidget(); sp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        tb.addWidget(sp)

        self._conn_lbl = QLabel("● OFFLINE")
        self._conn_lbl.setStyleSheet(f"color:{Theme.DANGER.name()};font-weight:bold;padding:0 12px;")
        tb.addWidget(self._conn_lbl)

        self._rate_lbl = QLabel("0 msg/s")
        self._rate_lbl.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()};padding:0 8px;")
        tb.addWidget(self._rate_lbl)

    def _update_controls(self, d):
        self._last_d = d
        self._btn_arm.setEnabled(True)
        self._btn_arm.setText("🔓 DISARM" if d.armed else "⚠  ARM")
        self._rate_lbl.setText(f"{d.message_rate} msg/s")

    def on_connection_status(self, ok, msg):
        self._console.log("SYS", msg)
        if ok:
            self._conn_lbl.setText("● ONLINE")
            self._conn_lbl.setStyleSheet(f"color:{Theme.SUCCESS.name()};font-weight:bold;padding:0 12px;")
            self._btn_connect.setText("⚡ DISCONNECT")
            self._btn_connect.setStyleSheet(
                f"background:{Theme.DANGER.name()};color:white;font-weight:bold;"
                f"border:none;border-radius:4px;padding:6px 16px;")
        else:
            self._conn_lbl.setText("● OFFLINE")
            self._conn_lbl.setStyleSheet(f"color:{Theme.DANGER.name()};font-weight:bold;padding:0 12px;")
            self._btn_connect.setText("⚡ CONNECT")
            self._btn_connect.setStyleSheet(
                f"background:{Theme.ACCENT_BLUE.name()};color:white;font-weight:bold;"
                f"border:none;border-radius:4px;padding:6px 16px;")

    def _show_connect_dialog(self):
        if self._worker.connected:
            self._worker.disconnect(); return
        dlg = ConnectionDialog(self)
        if dlg.exec_() == QDialog.Accepted:
            cs = dlg.connection_string
            if cs.startswith("sim:") or not cs:
                self._worker._sim_mode = True; self._worker.connected = True
                self._worker.connection_status.emit(True, "Simulation started")
            else:
                threading.Thread(target=self._worker.connect_vehicle,
                                 args=(cs, dlg.baud), daemon=True).start()

    def _arm_disarm(self):
        if not self._worker.connected: return
        new_state = not self._last_d.armed
        if new_state:
            r = QMessageBox.question(self, "Confirm ARM",
                "⚠  Arm the vehicle?", QMessageBox.Yes | QMessageBox.No)
            if r != QMessageBox.Yes: return
        self._worker.arm(new_state)
        self._console.log("SYS", f"{'ARM' if new_state else 'DISARM'} sent")

    def _change_mode(self, mode):
        if self._worker.connected and not self._worker._sim_mode:
            self._set_mode(mode)

    def _set_mode(self, mode):
        self._console.log("SYS", f"Mode → {mode}")
        if mode in FLIGHT_MODES:
            self._mode_combo.setCurrentIndex(FLIGHT_MODES.index(mode))


# ─────────────────────────────────────────────────────────────────
#  CONNECTION DIALOG
# ─────────────────────────────────────────────────────────────────
class ConnectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to UAV"); self.setFixedSize(420, 280)
        self.setStyleSheet(f"""
            QDialog {{ background:{Theme.BG_CARD.name()}; }}
            QLabel  {{ color:{Theme.TEXT_PRIMARY.name()}; font-family:'Courier New'; }}
            QLineEdit, QComboBox, QSpinBox {{
                background:{Theme.BG_DARK.name()}; color:{Theme.TEXT_PRIMARY.name()};
                border:1px solid {Theme.BORDER_ACCENT.name()};
                border-radius:4px; padding:4px 8px; font-family:'Courier New'; }}
            QPushButton {{
                background:{Theme.ACCENT_BLUE.name()}; color:white; border:none;
                border-radius:5px; padding:8px 20px; font-family:'Courier New'; font-weight:bold; }}
            QPushButton:hover {{ background:{Theme.ACCENT_CYAN.name()}; color:black; }}
        """)
        lay = QVBoxLayout(self); lay.setSpacing(12); lay.setContentsMargins(20,20,20,20)
        title = QLabel("⚡  UAV CONNECTION SETUP")
        title.setStyleSheet("font-size:13px;font-weight:bold;color:#58A6FF;")
        lay.addWidget(title)
        form = QFormLayout(); form.setSpacing(8)
        self.type_combo = QComboBox()
        self.type_combo.addItems(["UDP","TCP","Serial","UDP Server","Simulation"])
        self.type_combo.currentTextChanged.connect(self._update)
        form.addRow("Type:", self.type_combo)
        self.addr_edit = QLineEdit("udp:127.0.0.1:14550")
        form.addRow("Connection:", self.addr_edit)
        self.baud_spin = QSpinBox(); self.baud_spin.setRange(9600,921600)
        self.baud_spin.setValue(57600); self.baud_spin.setSingleStep(9600)
        form.addRow("Baud Rate:", self.baud_spin)
        self.hint_lbl = QLabel()
        self.hint_lbl.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()};font-size:9px;")
        form.addRow("", self.hint_lbl)
        lay.addLayout(form); lay.addStretch()
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.button(QDialogButtonBox.Ok).setText("Connect")
        btns.button(QDialogButtonBox.Cancel).setText("Cancel")
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject)
        lay.addWidget(btns); self._update("UDP")

    def _update(self, t):
        hints   = {"UDP":"udp:host:port","TCP":"tcp:host:port","Serial":"/dev/ttyACM0 or COM3",
                   "UDP Server":"udpin:0.0.0.0:14550","Simulation":"No hardware needed"}
        defaults= {"UDP":"udp:127.0.0.1:14550","TCP":"tcp:192.168.1.1:5760",
                   "Serial":"/dev/ttyACM0","UDP Server":"udpin:0.0.0.0:14550","Simulation":"sim:"}
        self.hint_lbl.setText(hints.get(t,"")); self.addr_edit.setText(defaults.get(t,""))

    @property
    def connection_string(self): return self.addr_edit.text()
    @property
    def baud(self): return self.baud_spin.value()


# ─────────────────────────────────────────────────────────────────
#  WINDOW 1 — FLIGHT VIEW
# ─────────────────────────────────────────────────────────────────
class FlightWindow(ControlsMixin, QMainWindow):
    """HUD · Compass · Map · Primary metrics · GPS · Status · Battery overview."""

    def __init__(self, worker, console):
        QMainWindow.__init__(self)
        self._init_controls(worker, console)
        self.setWindowTitle("✈   FLIGHT VIEW — UAV Ground Station")
        self.resize(1280, 860); self.setMinimumSize(900, 620)
        self.setStyleSheet(shared_stylesheet())
        self._start_time = time.time()
        self._make_toolbar()
        self._build_statusbar()
        self._build_central()
        QTimer.singleShot(0, lambda: None)   # force repaint queue

    # ── status bar ──────────────────────────────────────────────
    def _build_statusbar(self):
        sb = QStatusBar(); self.setStatusBar(sb)
        self._sb_time  = QLabel("T+ 00:00:00")
        self._sb_mode  = QLabel("MODE: —")
        self._sb_armed = QLabel("DISARMED")
        self._sb_armed.setStyleSheet(f"color:{Theme.DANGER.name()};font-weight:bold;")
        self._sb_pos   = QLabel("—")
        for w in [self._sb_time, QLabel(" | "), self._sb_mode, QLabel(" | "),
                  self._sb_armed, QLabel(" | "), self._sb_pos]:
            sb.addWidget(w)
        t = QTimer(self); t.timeout.connect(self._tick); t.start(1000)

    def _tick(self):
        s = int(time.time()-self._start_time)
        self._sb_time.setText(f"T+ {s//3600:02d}:{(s%3600)//60:02d}:{s%60:02d}")

    # ── central ─────────────────────────────────────────────────
    def _build_central(self):
        root = QWidget(); self.setCentralWidget(root)
        main = QHBoxLayout(root); main.setContentsMargins(6,6,6,6); main.setSpacing(6)

        
        ## LEFT — HUD + Compass
        #left = QVBoxLayout(); 
        #left.setSpacing(6)
        #hud_g = QGroupBox("ATTITUDE — ARTIFICIAL HORIZON")
        #QVBoxLayout(hud_g).addWidget(ArtificialHorizon().__class__.__new__(ArtificialHorizon))
        ## (properly instantiate)
        #hud_g2 = QGroupBox("ATTITUDE — ARTIFICIAL HORIZON"); 
        #hl = QVBoxLayout(hud_g2)
        #self._horizon = ArtificialHorizon(); 
        #hl.addWidget(self._horizon)
        #left.addWidget(hud_g2, 3)
        #cmp_g = QGroupBox("HEADING — COMPASS ROSE"); cl = QVBoxLayout(cmp_g)
        #self._compass = CompassRose(); cl.addWidget(self._compass)
        #left.addWidget(cmp_g, 2)
        #main.addLayout(left, 2)

        # LEFT — HUD + Compass
        left = QVBoxLayout()
        left.setSpacing(6)

        # Orizzonte artificiale
        hud_g2 = QGroupBox("ATTITUDE — ARTIFICIAL HORIZON")
        hl = QVBoxLayout(hud_g2)
        self._horizon = ArtificialHorizon()
        hl.addWidget(self._horizon)
        left.addWidget(hud_g2, 3)

        # Bussola
        cmp_g = QGroupBox("HEADING — COMPASS ROSE")
        cl = QVBoxLayout(cmp_g)
        self._compass = CompassRose()
        cl.addWidget(self._compass)
        left.addWidget(cmp_g, 2)

        main.addLayout(left, 2)

        # CENTRE — Map + metrics
        centre = QVBoxLayout(); centre.setSpacing(6)
        map_g = QGroupBox("MAP  (scroll to zoom)"); ml = QVBoxLayout(map_g)
        self._map = MapWidget(); ml.addWidget(self._map)
        centre.addWidget(map_g, 4)

        r1 = QHBoxLayout(); r1.setSpacing(6)
        self._c_alt  = MetricCard("ALTITUDE",    "m",   Theme.ACCENT_CYAN,   "▲")
        self._c_spd  = MetricCard("AIRSPEED",    "m/s", Theme.SUCCESS,        "➤")
        self._c_gspd = MetricCard("GROUNDSPEED", "m/s", Theme.ACCENT_ORANGE,  "⬤")
        self._c_vspd = MetricCard("VSPEED",      "m/s", Theme.INFO,           "↕")
        for c in [self._c_alt, self._c_spd, self._c_gspd, self._c_vspd]: r1.addWidget(c)
        centre.addLayout(r1)

        r2 = QHBoxLayout(); r2.setSpacing(6)
        self._c_roll  = MetricCard("ROLL",  "°", Theme.ACCENT_CYAN)
        self._c_pitch = MetricCard("PITCH", "°", Theme.INFO)
        self._c_yaw   = MetricCard("YAW",   "°", Theme.ACCENT_PURPLE)
        self._c_rssi  = MetricCard("RSSI",  "%", Theme.SUCCESS)
        for c in [self._c_roll, self._c_pitch, self._c_yaw, self._c_rssi]: r2.addWidget(c)
        centre.addLayout(r2)

        gps_g = QGroupBox("GPS"); gl2 = QVBoxLayout(gps_g)
        self._gps = GpsWidget(); gl2.addWidget(self._gps)
        centre.addWidget(gps_g)

        spk_g = QGroupBox("ALTITUDE TREND"); sl = QVBoxLayout(spk_g)
        self._spk_alt = Sparkline(Theme.ACCENT_CYAN); sl.addWidget(self._spk_alt)
        centre.addWidget(spk_g, 1)
        main.addLayout(centre, 3)

        # RIGHT — Status + Mission + Battery
        right = QVBoxLayout(); right.setSpacing(6)

        veh_g = QGroupBox("VEHICLE STATUS"); vl = QGridLayout(veh_g); vl.setSpacing(4)
        self._sv = {}
        for i,lbl in enumerate(["Flight Mode","Armed","Autopilot","Type",
                                  "Uptime","Sys ID","Msg Rate","Pkt Loss"]):
            l=QLabel(lbl+":"); l.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()};font-size:9px;")
            v=QLabel("—");     v.setStyleSheet(f"color:{Theme.TEXT_PRIMARY.name()};font-size:9px;font-weight:bold;")
            vl.addWidget(l,i,0); vl.addWidget(v,i,1); self._sv[lbl]=v
        right.addWidget(veh_g)

        wp_g = QGroupBox("MISSION / WAYPOINT"); wl = QGridLayout(wp_g); self._wp = {}
        for i,lbl in enumerate(["Current WP","WP Distance","Total WPs"]):
            l=QLabel(lbl+":"); l.setStyleSheet(f"color:{Theme.TEXT_MUTED.name()};font-size:9px;")
            v=QLabel("—");     v.setStyleSheet(f"color:{Theme.TEXT_PRIMARY.name()};font-size:10px;font-weight:bold;")
            wl.addWidget(l,i,0); wl.addWidget(v,i,1); self._wp[lbl]=v
        right.addWidget(wp_g)

        batt_g = QGroupBox("BATTERY"); bl = QVBoxLayout(batt_g)
        self._batt_w = BatteryWidget(); bl.addWidget(self._batt_w)
        rb = QHBoxLayout(); rb.setSpacing(6)
        self._c_volt = MetricCard("VOLTAGE","V",   Theme.ACCENT_ORANGE)
        self._c_curr = MetricCard("CURRENT","A",   Theme.WARNING)
        self._c_mah  = MetricCard("CONSUMED","mAh",Theme.TEXT_SECOND)
        for c in [self._c_volt,self._c_curr,self._c_mah]: rb.addWidget(c)
        bl.addLayout(rb)
        right.addWidget(batt_g)
        right.addStretch()
        main.addLayout(right, 1)

    # ── telemetry slot ───────────────────────────────────────────
    def on_telemetry(self, d: TelemetryData):
        self._update_controls(d)
        self._horizon.update_attitude(d.roll, d.pitch)
        self._compass.update_heading(d.yaw)
        self._map.update_position(d.lat, d.lon, d.yaw)
        self._c_alt.set_value(f"{d.rel_alt:.1f}",  d.rel_alt < 0)
        self._c_spd.set_value(f"{d.airspeed:.1f}")
        self._c_gspd.set_value(f"{d.groundspeed:.1f}")
        self._c_vspd.set_value(f"{d.vz:.2f}", abs(d.vz)>5)
        self._c_roll.set_value(f"{d.roll:+.1f}",  abs(d.roll)>45)
        self._c_pitch.set_value(f"{d.pitch:+.1f}", abs(d.pitch)>45)
        self._c_yaw.set_value(f"{d.yaw:.1f}")
        self._c_rssi.set_value(f"{d.rssi}", d.rssi<30)
        self._batt_w.update_battery(d.battery_remaining, d.battery_voltage, d.battery_current)
        self._c_volt.set_value(f"{d.battery_voltage:.2f}", d.battery_voltage<14.4)
        self._c_curr.set_value(f"{d.battery_current:.1f}", d.battery_current>40)
        self._c_mah.set_value(f"{d.battery_consumed:.0f}")
        self._gps.update_gps(d.fix_type, d.sats, d.hdop, d.lat, d.lon, d.alt)
        self._spk_alt.push(d.rel_alt)
        arm_col = Theme.SUCCESS.name() if d.armed else Theme.DANGER.name()
        self._sv["Flight Mode"].setText(d.flight_mode)
        self._sv["Armed"].setText("ARMED" if d.armed else "DISARMED")
        self._sv["Armed"].setStyleSheet(f"color:{arm_col};font-weight:bold;font-size:9px;")
        self._sv["Autopilot"].setText(d.autopilot); self._sv["Type"].setText(d.mav_type)
        up=int(d.uptime/20) if d.uptime else 0
        self._sv["Uptime"].setText(f"{up//3600:02d}:{(up%3600)//60:02d}:{up%60:02d}")
        self._sv["Sys ID"].setText(str(d.system_id))
        self._sv["Msg Rate"].setText(f"{d.message_rate} msg/s")
        self._sv["Pkt Loss"].setText(f"{d.packet_loss:.1f}%")
        self._wp["Current WP"].setText(str(d.wp_num))
        self._wp["WP Distance"].setText(f"{d.wp_dist:.1f} m")
        self._wp["Total WPs"].setText(str(d.mission_total))
        self._sb_mode.setText(f"MODE: {d.flight_mode}")
        self._sb_armed.setText("ARMED" if d.armed else "DISARMED")
        self._sb_armed.setStyleSheet(f"color:{arm_col};font-weight:bold;font-size:9px;")
        self._sb_pos.setText(f"Lat:{d.lat:.6f}  Lon:{d.lon:.6f}  Alt:{d.rel_alt:.1f}m")


# ─────────────────────────────────────────────────────────────────
#  WINDOW 2 — TELEMETRY & DIAGNOSTICS
# ─────────────────────────────────────────────────────────────────
class TelemetryWindow(ControlsMixin, QMainWindow):
    """Sensors · EKF · Charts · Parameters · Console."""

    def __init__(self, worker, console):
        QMainWindow.__init__(self)
        self._init_controls(worker, console)
        self.setWindowTitle("📡   TELEMETRY & DIAGNOSTICS — UAV Ground Station")
        self.resize(1300, 860); self.setMinimumSize(900, 620)
        self.setStyleSheet(shared_stylesheet())
        self._make_toolbar()
        self._build_statusbar()
        self._build_central()
        self._chart_timer = QTimer(self)
        self._chart_timer.timeout.connect(self._push_charts)
        self._chart_timer.start(100)

    def _build_statusbar(self):
        sb = QStatusBar(); self.setStatusBar(sb)
        self._sb_mode  = QLabel("MODE: —")
        self._sb_armed = QLabel("DISARMED")
        self._sb_armed.setStyleSheet(f"color:{Theme.DANGER.name()};font-weight:bold;")
        for w in [self._sb_mode, QLabel(" | "), self._sb_armed]: sb.addWidget(w)

    def _build_central(self):
        root = QWidget(); self.setCentralWidget(root)
        lay  = QVBoxLayout(root); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)
        tabs = QTabWidget(); lay.addWidget(tabs)
        tabs.addTab(self._tab_sensors(),    "📡  SENSORS")
        tabs.addTab(self._tab_battery(),    "🔋  BATTERY")
        tabs.addTab(self._tab_ekf(),        "📐  EKF / HEALTH")
        tabs.addTab(self._tab_charts(),     "📈  CHARTS")
        tabs.addTab(self._tab_params(),     "⚙️  PARAMETERS")
        tabs.addTab(self._tab_console(),    "💬  CONSOLE")

    # ── Tab: Sensors ─────────────────────────────────────────────
    def _tab_sensors(self):
        w = QWidget(); lay = QGridLayout(w); lay.setContentsMargins(6,6,6,6); lay.setSpacing(6)
        # IMU
        ig = QGroupBox("IMU — ACCELEROMETER & GYROSCOPE"); il = QGridLayout(ig)
        self._imu = {}
        for i,(lbl,unit,col) in enumerate([
            ('Accel X','m/s²',Theme.ACCENT_RED),('Accel Y','m/s²',Theme.SUCCESS),
            ('Accel Z','m/s²',Theme.ACCENT_CYAN),('Gyro X','rad/s',Theme.ACCENT_RED),
            ('Gyro Y','rad/s',Theme.SUCCESS),('Gyro Z','rad/s',Theme.ACCENT_CYAN)]):
            c=MetricCard(lbl,unit,col); il.addWidget(c,i//3,i%3); self._imu[lbl]=c
        lay.addWidget(ig,0,0)
        # Mag
        mg = QGroupBox("MAGNETOMETER"); ml = QGridLayout(mg); self._mag = {}
        for i,(lbl,col) in enumerate([('Mag X',Theme.ACCENT_RED),('Mag Y',Theme.SUCCESS),('Mag Z',Theme.ACCENT_CYAN)]):
            c=MetricCard(lbl,"μT",col); ml.addWidget(c,0,i); self._mag[lbl]=c
        lay.addWidget(mg,0,1)
        # Baro
        bg = QGroupBox("BAROMETER"); bl2 = QGridLayout(bg); self._baro = {}
        for i,(lbl,unit,col) in enumerate([
            ('Baro Alt','m',Theme.ACCENT_CYAN),('Pressure','Pa',Theme.INFO),
            ('Temperature','°C',Theme.ACCENT_ORANGE)]):
            c=MetricCard(lbl,unit,col); bl2.addWidget(c,0,i); self._baro[lbl]=c
        lay.addWidget(bg,1,0)
        # Vibration
        vg = QGroupBox("VIBRATION"); vl2 = QGridLayout(vg); self._vib = {}
        for i,(lbl,col) in enumerate([('Vibe X',Theme.ACCENT_RED),('Vibe Y',Theme.SUCCESS),('Vibe Z',Theme.ACCENT_CYAN)]):
            c=MetricCard(lbl,"m/s²",col); vl2.addWidget(c,0,i); self._vib[lbl]=c
        lay.addWidget(vg,1,1)
        # Charts
        self._imu_ch = ChartPanel("ACCELEROMETER",3,[Theme.ACCENT_RED,Theme.SUCCESS,Theme.ACCENT_CYAN],['AccX','AccY','AccZ'],-20,20)
        lay.addWidget(self._imu_ch,2,0)
        self._gyr_ch = ChartPanel("GYROSCOPE",3,[Theme.ACCENT_RED,Theme.SUCCESS,Theme.ACCENT_CYAN],['GyrX','GyrY','GyrZ'],-5,5)
        lay.addWidget(self._gyr_ch,2,1)
        self._vib_ch = ChartPanel("VIBRATION",3,[Theme.ACCENT_RED,Theme.SUCCESS,Theme.ACCENT_CYAN],['VibX','VibY','VibZ'],0,30)
        lay.addWidget(self._vib_ch,3,0,1,2)
        lay.setRowStretch(2,1); lay.setRowStretch(3,1)
        return w

    # ── Tab: Battery ─────────────────────────────────────────────
    def _tab_battery(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(6,6,6,6); lay.setSpacing(6)
        top = QHBoxLayout(); top.setSpacing(6)
        self._bc_pct   = MetricCard("REMAINING",  "%",   Theme.SUCCESS)
        self._bc_volt  = MetricCard("VOLTAGE",    "V",   Theme.ACCENT_ORANGE)
        self._bc_curr  = MetricCard("CURRENT",    "A",   Theme.WARNING)
        self._bc_mah   = MetricCard("CONSUMED",   "mAh", Theme.TEXT_SECOND)
        self._bc_power = MetricCard("POWER",      "W",   Theme.ACCENT_PURPLE)
        for c in [self._bc_pct,self._bc_volt,self._bc_curr,self._bc_mah,self._bc_power]: top.addWidget(c)
        lay.addLayout(top)
        bg = QGroupBox("BATTERY LEVEL"); bl3 = QVBoxLayout(bg)
        self._batt_big = BatteryWidget(); self._batt_big.setMinimumHeight(80); bl3.addWidget(self._batt_big)
        lay.addWidget(bg)
        self._ch_batt     = ChartPanel("VOLTAGE & CURRENT",2,[Theme.ACCENT_ORANGE,Theme.DANGER],['Voltage','Current'])
        self._ch_batt_pct = ChartPanel("REMAINING %",1,[Theme.SUCCESS],['Remaining'],0,100)
        self._ch_power    = ChartPanel("POWER (W)",1,[Theme.ACCENT_PURPLE],['Power'],0,500)
        lay.addWidget(self._ch_batt,2); lay.addWidget(self._ch_batt_pct,1); lay.addWidget(self._ch_power,1)
        return w

    # ── Tab: EKF / Health ────────────────────────────────────────
    def _tab_ekf(self):
        w = QWidget(); lay = QGridLayout(w); lay.setContentsMargins(6,6,6,6); lay.setSpacing(6)
        ekf_g = QGroupBox("EKF STATUS FLAGS"); ekf_l = QVBoxLayout(ekf_g)
        self._ekf_bars = {}
        for name in ['Attitude','Vel Horiz','Vel Vert','Pos Horiz','Pos Vert','Terrain','Const Pos']:
            bar = QProgressBar(); bar.setRange(0,100); bar.setValue(100)
            bar.setFixedHeight(18); bar.setFormat(f"  {name}  %p%")
            bar.setStyleSheet("""
                QProgressBar { background:#161B22; border:none; border-radius:3px;
                    color:#E6EDF3; font-size:9px; font-family:'Courier New'; }
                QProgressBar::chunk { background:#3FB950; border-radius:3px; }
            """)
            ekf_l.addWidget(bar); self._ekf_bars[name] = bar
        lay.addWidget(ekf_g,0,0)
        # Sensor health table
        hg = QGroupBox("SENSOR HEALTH"); hl3 = QVBoxLayout(hg)
        self._health_tbl = QTableWidget(6,3)
        self._health_tbl.setHorizontalHeaderLabels(["Sensor","Status","Value"])
        self._health_tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._health_tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self._health_tbl.setSelectionBehavior(QTableWidget.SelectRows)
        for i,(n,s,v) in enumerate([("GPS","OK","3D Fix"),("IMU","OK","Calibrated"),
                                     ("Barometer","OK","101.3 kPa"),("Compass","OK","Calibrated"),
                                     ("RC Input","OK","RSSI 85%"),("Battery","OK","16.8V")]):
            self._health_tbl.setItem(i,0,QTableWidgetItem(n))
            si=QTableWidgetItem(s); si.setForeground(Theme.SUCCESS)
            self._health_tbl.setItem(i,1,si); self._health_tbl.setItem(i,2,QTableWidgetItem(v))
        hl3.addWidget(self._health_tbl)
        lay.addWidget(hg,0,1)
        self._vib_ekf_ch = ChartPanel("VIBRATION",3,[Theme.ACCENT_RED,Theme.SUCCESS,Theme.ACCENT_CYAN],['VibX','VibY','VibZ'],0,30)
        lay.addWidget(self._vib_ekf_ch,1,0,1,2)
        lay.setRowStretch(1,2)
        return w

    # ── Tab: Charts ──────────────────────────────────────────────
    def _tab_charts(self):
        w = QWidget(); lay = QGridLayout(w); lay.setContentsMargins(6,6,6,6); lay.setSpacing(6)
        self._ch_att  = ChartPanel("ATTITUDE",3,[Theme.ACCENT_CYAN,Theme.SUCCESS,Theme.ACCENT_PURPLE],['Roll','Pitch','Yaw'],-180,180)
        self._ch_alt  = ChartPanel("ALTITUDE & VSPEED",2,[Theme.ACCENT_CYAN,Theme.ACCENT_ORANGE],['Altitude','VSpeed'])
        self._ch_spd  = ChartPanel("SPEED",2,[Theme.SUCCESS,Theme.ACCENT_ORANGE],['Airspeed','GndSpeed'],0,30)
        self._ch_baro = ChartPanel("BAROMETER",2,[Theme.ACCENT_CYAN,Theme.ACCENT_ORANGE],['Press/kPa','Temp°C'])
        self._ch_rssi = ChartPanel("RSSI",1,[Theme.SUCCESS],['RSSI'],0,100)
        self._ch_vel  = ChartPanel("VELOCITY XYZ",3,[Theme.ACCENT_RED,Theme.SUCCESS,Theme.ACCENT_CYAN],['Vx','Vy','Vz'],-15,15)
        lay.addWidget(self._ch_att,0,0); lay.addWidget(self._ch_alt,0,1)
        lay.addWidget(self._ch_spd,1,0); lay.addWidget(self._ch_baro,1,1)
        lay.addWidget(self._ch_rssi,2,0); lay.addWidget(self._ch_vel,2,1)
        for r in range(3): lay.setRowStretch(r,1)
        lay.setColumnStretch(0,1); lay.setColumnStretch(1,1)
        return w

    # ── Tab: Parameters ──────────────────────────────────────────
    def _tab_params(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(6,6,6,6); lay.setSpacing(6)
        hdr = QHBoxLayout()
        self._psearch = QLineEdit(); self._psearch.setPlaceholderText("Search parameter…")
        self._psearch.textChanged.connect(self._filter_params)
        btn_ref = QPushButton("🔄 Refresh from vehicle")
        btn_ref.clicked.connect(lambda: self._console.log("SYS","Refresh requires MAVLink connection"))
        hdr.addWidget(self._psearch); hdr.addWidget(btn_ref); lay.addLayout(hdr)
        self._ptbl = QTableWidget(0,3)
        self._ptbl.setHorizontalHeaderLabels(["Parameter","Value","Description"])
        self._ptbl.horizontalHeader().setSectionResizeMode(0,QHeaderView.ResizeToContents)
        self._ptbl.horizontalHeader().setSectionResizeMode(1,QHeaderView.ResizeToContents)
        self._ptbl.horizontalHeader().setSectionResizeMode(2,QHeaderView.Stretch)
        self._ptbl.setSelectionBehavior(QTableWidget.SelectRows)
        lay.addWidget(self._ptbl); self._fill_params()
        return w

    def _fill_params(self):
        params = [
            ("ARMING_CHECK","1","Arm safety checks bitmask"),
            ("ATC_RAT_PIT_P","0.135","Pitch rate P gain"),("ATC_RAT_PIT_I","0.135","Pitch rate I gain"),
            ("ATC_RAT_PIT_D","0.0036","Pitch rate D gain"),("ATC_RAT_RLL_P","0.135","Roll rate P gain"),
            ("ATC_RAT_RLL_I","0.135","Roll rate I gain"),("ATC_RAT_RLL_D","0.0036","Roll rate D gain"),
            ("ATC_RAT_YAW_P","0.18","Yaw rate P gain"),("ATC_RAT_YAW_I","0.018","Yaw rate I gain"),
            ("ATC_RAT_YAW_D","0.0","Yaw rate D gain"),
            ("BATT_MONITOR","4","Battery monitoring type"),("BATT_VOLT_PIN","2","Battery voltage ADC pin"),
            ("BATT_CURR_PIN","3","Battery current ADC pin"),("BATT_LOW_VOLT","14.4","Low battery threshold V"),
            ("BATT_CRT_VOLT","13.8","Critical battery voltage V"),("FS_BATT_ENABLE","2","Battery failsafe action"),
            ("FS_GCS_ENABLE","1","GCS failsafe enable"),("FS_THR_ENABLE","1","Throttle failsafe enable"),
            ("FS_THR_VALUE","975","Throttle failsafe PWM value"),
            ("GPS_TYPE","1","GPS type (1=Auto)"),("GPS_NAVFILTER","8","GPS navigation filter"),
            ("INS_GYRO_FILTER","20","Gyro noise filter cutoff Hz"),("INS_ACCEL_FILTER","20","Accel filter cutoff Hz"),
            ("PILOT_SPEED_UP","250","Max climb speed cm/s"),("PILOT_SPEED_DN","150","Max descent speed cm/s"),
            ("PILOT_ACCEL_Z","250","Vertical accel limit cm/s²"),("WPNAV_SPEED","500","Waypoint cruise speed cm/s"),
            ("WPNAV_RADIUS","200","Waypoint arrival radius cm"),("LOIT_SPEED","500","Loiter max speed cm/s"),
            ("RTL_ALT","1500","RTL altitude cm"),("RTL_SPEED","0","RTL speed (0=WPNAV_SPEED)"),
            ("EKF_TYPE","2","EKF type (2=EKF2, 3=EKF3)"),("INS_USE","1","IMU1 enable"),
            ("INS_USE2","1","IMU2 enable"),("COMPASS_USE","1","Compass 1 enable"),
            ("LOG_BITMASK","65535","Dataflash log bitmask"),("SERIAL0_BAUD","115","USB baud ×1000"),
            ("SERIAL1_BAUD","57","Telem1 baud ×1000"),
        ]
        self._ptbl.setRowCount(len(params))
        for i,(n,v,d) in enumerate(params):
            self._ptbl.setItem(i,0,QTableWidgetItem(n))
            self._ptbl.setItem(i,1,QTableWidgetItem(v))
            self._ptbl.setItem(i,2,QTableWidgetItem(d))

    def _filter_params(self, text):
        t = text.lower()
        for i in range(self._ptbl.rowCount()):
            a=(self._ptbl.item(i,0) or QTableWidgetItem()).text().lower()
            b=(self._ptbl.item(i,2) or QTableWidgetItem()).text().lower()
            self._ptbl.setRowHidden(i, t not in a and t not in b)

    # ── Tab: Console ─────────────────────────────────────────────
    def _tab_console(self):
        w = QWidget(); lay = QVBoxLayout(w); lay.setContentsMargins(6,6,6,6)
        top = QHBoxLayout()
        btn_clr = QPushButton("🗑 Clear"); btn_clr.clicked.connect(self._console.clear)
        top.addWidget(btn_clr); top.addStretch(); lay.addLayout(top)
        lay.addWidget(self._console)
        cmd_row = QHBoxLayout()
        self._cmd = QLineEdit(); self._cmd.setPlaceholderText("ARM / RTL / LAND …")
        self._cmd.returnPressed.connect(self._send_cmd)
        btn_send = QPushButton("Send ▶"); btn_send.clicked.connect(self._send_cmd)
        cmd_row.addWidget(self._cmd); cmd_row.addWidget(btn_send); lay.addLayout(cmd_row)
        return w

    def _send_cmd(self):
        text = self._cmd.text().strip()
        if not text: return
        self._console.log("SYS", f"Command > {text}")
        cmd = text.upper()
        if   cmd == "ARM":  self._arm_disarm()
        elif cmd == "RTL":  self._set_mode("RTL")
        elif cmd == "LAND": self._set_mode("LAND")
        else: self._console.log("WARN", f"Unknown: {text}")
        self._cmd.clear()

    # ── Telemetry update ─────────────────────────────────────────
    def on_telemetry(self, d: TelemetryData):
        self._update_controls(d)
        arm_col = Theme.SUCCESS.name() if d.armed else Theme.DANGER.name()
        self._sb_mode.setText(f"MODE: {d.flight_mode}")
        self._sb_armed.setText("ARMED" if d.armed else "DISARMED")
        self._sb_armed.setStyleSheet(f"color:{arm_col};font-weight:bold;font-size:9px;")
        # Sensors
        self._imu["Accel X"].set_value(f"{d.accel_x:+.3f}"); self._imu["Accel Y"].set_value(f"{d.accel_y:+.3f}")
        self._imu["Accel Z"].set_value(f"{d.accel_z:+.3f}"); self._imu["Gyro X"].set_value(f"{d.gyro_x:+.4f}")
        self._imu["Gyro Y"].set_value(f"{d.gyro_y:+.4f}"); self._imu["Gyro Z"].set_value(f"{d.gyro_z:+.4f}")
        self._mag["Mag X"].set_value(f"{d.mag_x:.1f}"); self._mag["Mag Y"].set_value(f"{d.mag_y:.1f}")
        self._mag["Mag Z"].set_value(f"{d.mag_z:.1f}")
        self._baro["Baro Alt"].set_value(f"{d.baro_alt:.2f}"); self._baro["Pressure"].set_value(f"{d.baro_press:.0f}")
        self._baro["Temperature"].set_value(f"{d.temp:.1f}")
        self._vib["Vibe X"].set_value(f"{d.vibe_x:.2f}", d.vibe_x>15)
        self._vib["Vibe Y"].set_value(f"{d.vibe_y:.2f}", d.vibe_y>15)
        self._vib["Vibe Z"].set_value(f"{d.vibe_z:.2f}", d.vibe_z>15)
        # Battery
        self._bc_pct.set_value(f"{d.battery_remaining}", d.battery_remaining<20)
        self._bc_volt.set_value(f"{d.battery_voltage:.2f}", d.battery_voltage<14.4)
        self._bc_curr.set_value(f"{d.battery_current:.1f}", d.battery_current>40)
        self._bc_mah.set_value(f"{d.battery_consumed:.0f}")
        pw = d.battery_voltage * d.battery_current
        self._bc_power.set_value(f"{pw:.1f}")
        self._batt_big.update_battery(d.battery_remaining, d.battery_voltage, d.battery_current)
        # EKF
        ekf_names = ['Attitude','Vel Horiz','Vel Vert','Pos Horiz','Pos Vert','Terrain','Const Pos']
        for i,name in enumerate(ekf_names):
            ok  = bool(d.ekf_flags & (1<<i))
            bar = self._ekf_bars[name]; bar.setValue(100 if ok else 0)
            bar.setStyleSheet(f"""
                QProgressBar {{ background:#161B22; border:none; border-radius:3px;
                    color:#E6EDF3; font-size:9px; font-family:'Courier New'; }}
                QProgressBar::chunk {{ background:{'#3FB950' if ok else '#F85149'}; border-radius:3px; }}
            """)
        # Health table
        rows=[("GPS",d.fix_type>=3,f"{d.sats}sat HDOP:{d.hdop:.1f}"),
              ("IMU",True,f"AccZ:{d.accel_z:.2f} m/s²"),
              ("Baro",True,f"{d.baro_press/100:.1f} hPa"),
              ("Compass",True,f"X:{d.mag_x:.0f} Y:{d.mag_y:.0f}"),
              ("RC",d.rssi>30,f"RSSI:{d.rssi}%"),
              ("Battery",d.battery_remaining>20,f"{d.battery_voltage:.2f}V")]
        for i,(n,ok,v) in enumerate(rows):
            si=QTableWidgetItem("OK" if ok else "WARN")
            si.setForeground(Theme.SUCCESS if ok else Theme.WARNING)
            self._health_tbl.setItem(i,0,QTableWidgetItem(n))
            self._health_tbl.setItem(i,1,si)
            self._health_tbl.setItem(i,2,QTableWidgetItem(v))

    def _push_charts(self):
        d = self._last_d
        if not d.connected: return
        self._imu_ch.push(d.accel_x, d.accel_y, d.accel_z)
        self._gyr_ch.push(d.gyro_x,  d.gyro_y,  d.gyro_z)
        self._vib_ch.push(d.vibe_x,  d.vibe_y,  d.vibe_z)
        self._vib_ekf_ch.push(d.vibe_x, d.vibe_y, d.vibe_z)
        self._ch_batt.push(d.battery_voltage, d.battery_current)
        self._ch_batt_pct.push(d.battery_remaining)
        self._ch_power.push(d.battery_voltage * d.battery_current)
        self._ch_att.push(d.roll, d.pitch, d.yaw)
        self._ch_alt.push(d.rel_alt, d.vz)
        self._ch_spd.push(d.airspeed, d.groundspeed)
        self._ch_baro.push(d.baro_press/1000, d.temp)
        self._ch_rssi.push(d.rssi)
        self._ch_vel.push(d.vx, d.vy, d.vz)


# ─────────────────────────────────────────────────────────────────
#  APP CONTROLLER
# ─────────────────────────────────────────────────────────────────
class AppController:
    def __init__(self):
        self._console = ConsoleWidget()
        self._worker  = MAVLinkWorker()
        self._fw = FlightWindow(self._worker, self._console)
        self._tw = TelemetryWindow(self._worker, self._console)
        # Route signals to both windows
        self._worker.telemetry_updated.connect(self._fw.on_telemetry)
        self._worker.telemetry_updated.connect(self._tw.on_telemetry)
        self._worker.message_received.connect(self._console.log)
        self._worker.connection_status.connect(self._fw.on_connection_status)
        self._worker.connection_status.connect(self._tw.on_connection_status)
        self._worker.start()

    def show(self):
        screen = QApplication.primaryScreen().geometry()
        half   = screen.width() // 2
        self._fw.move(0, 30);    self._fw.resize(min(half, 1300), screen.height()-60)
        self._tw.move(half, 30); self._tw.resize(min(half, 1300), screen.height()-60)
        self._fw.show(); self._tw.show()
        # Auto-start simulation
        self._worker._sim_mode = True; self._worker.connected = True
        self._worker.connection_status.emit(True, "Simulation mode — click CONNECT for real hardware")
        self._console.log("INFO", "UAV Ground Station v2.0  —  Professional Edition")
        self._console.log("INFO", "Flight View (left) + Telemetry & Diagnostics (right)")
        self._console.log("INFO", "Simulation running. CONNECT on either window for real UAV.")

    def quit(self):
        self._worker.stop()


# ─────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────
def main():
    try:
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    except Exception:
        pass

    app = QApplication(sys.argv)
    app.setApplicationName("UAV Ground Station"); app.setApplicationVersion("2.0.0")

    pal = QPalette()
    pal.setColor(QPalette.Window,          Theme.BG_DARK)
    pal.setColor(QPalette.WindowText,      Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Base,            Theme.BG_CARD)
    pal.setColor(QPalette.AlternateBase,   Theme.BG_CARD2)
    pal.setColor(QPalette.ToolTipBase,     Theme.BG_CARD2)
    pal.setColor(QPalette.ToolTipText,     Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Text,            Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Button,          Theme.BG_CARD)
    pal.setColor(QPalette.ButtonText,      Theme.TEXT_PRIMARY)
    pal.setColor(QPalette.Link,            Theme.ACCENT_CYAN)
    pal.setColor(QPalette.Highlight,       Theme.ACCENT_BLUE)
    pal.setColor(QPalette.HighlightedText, QColor("white"))
    app.setPalette(pal)

    ctrl = AppController()
    ctrl.show()
    app.aboutToQuit.connect(ctrl.quit)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()