# monitor_teensy_json_eth_only_time.py
# Ethernet-only monitor + ACK + periodic TIME sync + Multimeter mode (big digits)

import sys, json, time, socket, struct, threading, math
from PyQt6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

# TEENSY_IP   = "192.168.200.177"
TEENSY_IP   = "169.254.153.177"
TEENSY_PORT = 6000

HELLO_INTERVAL   = 0.5    # s, khi chưa có stream
NO_FRAME_TIMEOUT = 1.0    # s, ko thấy frame => có thể FW đang OFFLINE và ghi SD
TIME_SYNC_EVERY  = 30.0   # s, gửi TIME <epoch_s>
UI_HZ            = 10.0   # Multimeter refresh ~10Hz

PARAM_NAMES = [
    "AVRMS","BVRMS","CVRMS",
    "AIRMS","BIRMS","CIRMS",
    "AWATT","BWATT","CWATT",
    "AVAR","BVAR","CVAR",
    "AVA","BVA","CVA",
    "FREQ_Hz"
]

def safe_float(x, default=0.0):
    try: return float(x)
    except: return default

# ------------------ Reader ------------------

class UdpReader(QtCore.QObject):
    # vẫn phát 16 thông số cho Grid
    paramFrame = QtCore.pyqtSignal(list)
    status     = QtCore.pyqtSignal(str)
    transport  = QtCore.pyqtSignal(str)
    i2cLock    = QtCore.pyqtSignal(bool)
    walltime   = QtCore.pyqtSignal(str)   # "YYYY-MM-DD HH:MM:SS"
    frameJson  = QtCore.pyqtSignal(dict)  # phát cả raw JSON cho Multimeter

    def __init__(self, ip, port):
        super().__init__()
        self.ip = ip
        self.port = port
        self._running = False
        self._thread  = None
        self.sock = None
        self.last_frame_time = 0.0
        self.fps_count = 0
        self.fps_last_t = 0.0
        self.last_hello = 0.0
        self.last_time_sync = 0.0

    def start(self):
        if self._running: return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.3)
        self._thread = None
        try:
            if self.sock: self.sock.close()
        except: pass
        self.sock = None

    def _send(self, payload: bytes, peer=None):
        try:
            self.sock.sendto(payload, (self.ip, self.port) if peer is None else peer)
        except Exception as e:
            self.status.emit(f"send error: {e}")

    def _send_hello(self):
        self._send(b"HELLO")

    def _send_time(self):
        epoch = int(time.time())
        msg = f"TIME {epoch}".encode()
        self._send(msg)

    def _send_ack(self, seq, peer):
        pkt = b"ACK" + struct.pack("<I", seq & 0xFFFFFFFF)
        self._send(pkt, peer)

    def _loop(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))  # bind ephemeral port
        self.sock.settimeout(0.2)
        self.transport.emit(f"Ethernet ({self.ip}:{self.port})")
        self.status.emit("Waiting for stream... (sending HELLO/TIME)")

        buf = b""
        peer = (self.ip, self.port)
        self.fps_last_t = time.time()

        while self._running:
            now = time.time()

            # 1) Gửi HELLO khi chưa có frame
            if (now - self.last_frame_time) > NO_FRAME_TIMEOUT and (now - self.last_hello) >= HELLO_INTERVAL:
                self._send_hello()
                self.last_hello = now

            # 2) Gửi TIME định kỳ (giữ đồng bộ trên Teensy)
            if (now - self.last_time_sync) >= TIME_SYNC_EVERY:
                self._send_time()
                self.last_time_sync = now

            # 3) Nhận data
            try:
                data, addr = self.sock.recvfrom(65535)
                if not data: continue
                peer = addr
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="ignore").strip()
                    if not s: continue

                    try:
                        d = json.loads(s)
                    except json.JSONDecodeError:
                        continue

                    self.last_frame_time = time.time()
                    self.fps_count += 1
                    # tính fps mỗi ~1s
                    if (self.last_frame_time - self.fps_last_t) >= 1.0:
                        fps = self.fps_count / (self.last_frame_time - self.fps_last_t)
                        self.status.emit(f"Streaming… fps≈{fps:.1f}, last seq={d.get('n','?')}")
                        self.fps_count = 0
                        self.fps_last_t = self.last_frame_time

                    # ACK mỗi frame
                    seq = int(d.get("n", 0)) if "n" in d else 0
                    self._send_ack(seq, peer)

                    # I2C
                    self.i2cLock.emit(bool(d.get("i2c_lock", False)))

                    # Hiển thị thời gian thực nếu có "ts"
                    ts = d.get("ts", None)
                    if isinstance(ts, (int, float)) and ts > 0:
                        lt = time.localtime(int(ts))
                        self.walltime.emit(time.strftime("%Y-%m-%d %H:%M:%S", lt))

                    # Phát raw JSON cho Multimeter
                    self.frameJson.emit(d)

                    # Map 16 tham số (giữ nguyên Grid)
                    A = d.get("A", {}) or {}
                    B = d.get("B", {}) or {}
                    C = d.get("C", {}) or {}
                    va = safe_float(A.get("V",0)); vb = safe_float(B.get("V",0)); vc = safe_float(C.get("V",0))
                    ia = safe_float(A.get("I",0)); ib = safe_float(B.get("I",0)); ic = safe_float(C.get("I",0))
                    pa = safe_float(A.get("P",0)); pb = safe_float(B.get("P",0)); pc = safe_float(C.get("P",0))
                    sa = safe_float(A.get("S",0)); sb = safe_float(B.get("S",0)); sc = safe_float(C.get("S",0))
                    qa = qb = qc = 0.0
                    # ưu tiên Q trong A/B/C (nếu firmware đã embed), nếu không lấy từ extras
                    if "Q" in A or "Q" in B or "Q" in C:
                        qa = safe_float(A.get("Q",0)); qb = safe_float(B.get("Q",0)); qc = safe_float(C.get("Q",0))
                    elif "extras" in d:
                        ex = d["extras"] or {}
                        qa = safe_float((ex.get("A") or {}).get("Q",0))
                        qb = safe_float((ex.get("B") or {}).get("Q",0))
                        qc = safe_float((ex.get("C") or {}).get("Q",0))
                    freq = safe_float(d.get("f",0))

                    vals16 = [va,vb,vc, ia,ib,ic, pa,pb,pc, qa,qb,qc, sa,sb,sc, freq]
                    self.paramFrame.emit(vals16)

            except socket.timeout:
                if (time.time() - self.last_frame_time) > NO_FRAME_TIMEOUT:
                    self.status.emit("No frames… device may be logging to SD (offline).")
                continue
            except Exception as e:
                self.status.emit(f"UDP error: {e}")
                time.sleep(0.2)

# ------------------ UI Widgets ------------------

class ParamGrid(QtWidgets.QWidget):
    def __init__(self, names, parent=None):
        super().__init__(parent)
        self.names = names
        self.layout = QtWidgets.QGridLayout(self)
        self.value_labels = []
        self.units = ["V","V","V","A","A","A","W","W","W","var","var","var","VA","VA","VA","Hz"]
        for i, nm in enumerate(names):
            title = QtWidgets.QLabel(nm); title.setStyleSheet("font-weight:600;")
            val   = QtWidgets.QLabel("--"); val.setStyleSheet("font-size:20px;")
            row   = i // 4; col = (i % 4)*2
            self.layout.addWidget(title, row, col)
            self.layout.addWidget(val,   row, col+1)
            self.value_labels.append(val)
        # PF
        self.pfA = QtWidgets.QLabel("--"); self.pfB = QtWidgets.QLabel("--"); self.pfC = QtWidgets.QLabel("--")
        base_row = (len(names)+3)//4 + 1
        self.layout.addWidget(QtWidgets.QLabel("<b>PF_A</b>"), base_row, 0)
        self.layout.addWidget(self.pfA, base_row, 1)
        self.layout.addWidget(QtWidgets.QLabel("<b>PF_B</b>"), base_row, 2)
        self.layout.addWidget(self.pfB, base_row, 3)
        self.layout.addWidget(QtWidgets.QLabel("<b>PF_C</b>"), base_row, 4)
        self.layout.addWidget(self.pfC, base_row, 5)

    def update_values(self, vals):
        for i,v in enumerate(vals):
            u = self.units[i]
            if u=="Hz": self.value_labels[i].setText(f"{v:.3f} Hz")
            elif u in ("V","A"): self.value_labels[i].setText(f"{v:.3f} {u}")
            else: self.value_labels[i].setText(f"{v:.2f} {u}")
        P_A,P_B,P_C = vals[6],vals[7],vals[8]
        S_A,S_B,S_C = vals[12],vals[13],vals[14]
        def pf(p,s): return (p/s) if abs(s)>1e-9 else 0.0
        self.pfA.setText(f"{pf(P_A,S_A):.3f}")
        self.pfB.setText(f"{pf(P_B,S_B):.3f}")
        self.pfC.setText(f"{pf(P_C,S_C):.3f}")

# -------- Multimeter helpers --------

class EmaFilter:
    def __init__(self, alpha=0.25):
        self.alpha = alpha
        self.y = None
    def reset(self):
        self.y = None
    def update(self, x):
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha*x + (1.0-self.alpha)*self.y
        return self.y

def fmt_unit(value, unit):
    # auto-scale cho W/VA/var, và mV/mA nếu nhỏ
    sgn = "-" if value < 0 else ""
    v = abs(value)
    if unit in ("W","VA","var"):
        if v >= 1e6: return f"{sgn}{v/1e6:.3f} M{unit}"
        if v >= 1e3: return f"{sgn}{v/1e3:.3f} k{unit}"
        return f"{sgn}{v:.3f} {unit}"
    if unit in ("V","A"):
        if v < 1.0:  return f"{sgn}{v*1e3:.2f} m{unit}"
        return f"{sgn}{v:.3f} {unit}"
    if unit == "Hz":
        return f"{value:.2f} Hz"
    if unit == "PF":
        return f"{value:.3f}"
    return f"{value:.3f} {unit}"

# -------- Multimeter Widget --------

class MultimeterWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.last_frame = None
        self.ema = EmaFilter(alpha=0.25)
        self.hold = False
        self.rel_enabled = False
        self.rel_zero = 0.0
        self.min_v = None
        self.max_v = None
        self.avg_sum = 0.0
        self.avg_n = 0

        # Controls
        lay = QtWidgets.QVBoxLayout(self)

        ctrl = QtWidgets.QHBoxLayout()
        self.cmb_chan = QtWidgets.QComboBox()
        self.cmb_chan.addItems(["A","B","C","Total"])
        self.cmb_qty = QtWidgets.QComboBox()
        self.cmb_qty.addItems(["V","A","W","VA","VAR","PF","Hz"])
        self.sld_alpha = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.sld_alpha.setMinimum(1); self.sld_alpha.setMaximum(95); self.sld_alpha.setValue(25)
        self.lbl_alpha = QtWidgets.QLabel("Smoothing α=0.25")
        self.btn_hold = QtWidgets.QPushButton("Hold")
        self.btn_rel  = QtWidgets.QPushButton("Relative")
        self.btn_reset = QtWidgets.QPushButton("Reset Min/Max/Avg")

        self.btn_hold.setCheckable(True)
        self.btn_rel.setCheckable(True)

        ctrl.addWidget(QtWidgets.QLabel("Channel:")); ctrl.addWidget(self.cmb_chan)
        ctrl.addWidget(QtWidgets.QLabel("Quantity:")); ctrl.addWidget(self.cmb_qty)
        ctrl.addSpacing(12)
        ctrl.addWidget(self.lbl_alpha); ctrl.addWidget(self.sld_alpha)
        ctrl.addSpacing(12)
        ctrl.addWidget(self.btn_hold); ctrl.addWidget(self.btn_rel); ctrl.addWidget(self.btn_reset)
        ctrl.addStretch(1)

        # Big display
        self.lbl_big = QtWidgets.QLabel("--")
        self.lbl_big.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.lbl_big.setStyleSheet("""
            font-size: 68px; font-weight: 700;
            padding: 8px; border-radius: 12px;
            background: #111; color: #0f0;
        """)

        # Secondary info row
        info = QtWidgets.QHBoxLayout()
        self.lbl_units = QtWidgets.QLabel("Unit: —")
        self.lbl_pf_tag = QtWidgets.QLabel("PF tag: —")  # lead/lag nếu có Q
        self.lbl_min = QtWidgets.QLabel("Min: —")
        self.lbl_max = QtWidgets.QLabel("Max: —")
        self.lbl_avg = QtWidgets.QLabel("Avg: —")
        for w in (self.lbl_units,self.lbl_pf_tag,self.lbl_min,self.lbl_max,self.lbl_avg):
            w.setStyleSheet("font-size:14px;")
        info.addWidget(self.lbl_units); info.addSpacing(10)
        info.addWidget(self.lbl_pf_tag); info.addStretch(1)
        info.addWidget(self.lbl_min); info.addSpacing(10)
        info.addWidget(self.lbl_max); info.addSpacing(10)
        info.addWidget(self.lbl_avg)

        lay.addLayout(ctrl)
        lay.addWidget(self.lbl_big, stretch=1)
        lay.addLayout(info)

        # Connections
        self.sld_alpha.valueChanged.connect(self.on_alpha)
        self.btn_hold.toggled.connect(self.on_hold)
        self.btn_rel.toggled.connect(self.on_rel)
        self.btn_reset.clicked.connect(self.on_reset)

        # Timer 10 Hz
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_tick)
        self.timer.start(int(1000.0/UI_HZ))

    def on_alpha(self, val):
        a = max(0.01, min(0.95, val/100.0))
        self.ema.alpha = a
        self.lbl_alpha.setText(f"Smoothing α={a:.2f}")

    def on_hold(self, st):
        self.hold = st
        if st:
            self.lbl_big.setStyleSheet("""
                font-size: 68px; font-weight: 700;
                padding: 8px; border-radius: 12px;
                background: #331; color: #fc0;
            """)
        else:
            self.lbl_big.setStyleSheet("""
                font-size: 68px; font-weight: 700;
                padding: 8px; border-radius: 12px;
                background: #111; color: #0f0;
            """)

    def on_rel(self, st):
        self.rel_enabled = st
        if st and self.last_frame is not None:
            # set zero dựa trên giá trị hiện tại
            self.rel_zero = self._raw_value_from_frame(self.last_frame)
        if not st:
            self.rel_zero = 0.0

    def on_reset(self):
        self.min_v = None
        self.max_v = None
        self.avg_sum = 0.0
        self.avg_n = 0
        self.ema.reset()

    def set_frame(self, d: dict):
        self.last_frame = d

    # ---- helpers for Multimeter math ----
    def _pick_channel_dict(self, d, ch):
        if ch in ("A","B","C"):
            return d.get(ch, {}) or {}
        if ch == "Total":
            # Totals từ A/B/C
            A = d.get("A", {}) or {}; B = d.get("B", {}) or {}; C = d.get("C", {}) or {}
            Pa = safe_float(A.get("P",0)); Pb = safe_float(B.get("P",0)); Pc = safe_float(C.get("P",0))
            Sa = safe_float(A.get("S",0)); Sb = safe_float(B.get("S",0)); Sc = safe_float(C.get("S",0))
            Va = safe_float(A.get("V",0)); Vb = safe_float(B.get("V",0)); Vc = safe_float(C.get("V",0))
            Ia = safe_float(A.get("I",0)); Ib = safe_float(B.get("I",0)); Ic = safe_float(C.get("I",0))

            Ptot = Pa+Pb+Pc
            Stot = Sa+Sb+Sc
            # Q total nếu có:
            Qtot = 0.0
            Stot_true = 0.0
            # ưu tiên extras
            if "extras" in d and isinstance(d["extras"], dict):
                ex = d["extras"]
                Qtot = safe_float((ex.get("A") or {}).get("Q",0)) + safe_float((ex.get("B") or {}).get("Q",0)) + safe_float((ex.get("C") or {}).get("Q",0))
                Stot_true = safe_float((ex.get("A") or {}).get("S_true",0)) + safe_float((ex.get("B") or {}).get("S_true",0)) + safe_float((ex.get("C") or {}).get("S_true",0))
            else:
                # nếu FW có thể embed Q trong A/B/C
                Qtot = safe_float(A.get("Q",0)) + safe_float(B.get("Q",0)) + safe_float(C.get("Q",0))
                Stot_true = 0.0  # không có -> 0

            # V,I tổng (ở chế độ DMM, thường quan tâm P,S,PF; V,I tổng không có nghĩa vật lý trực tiếp)
            # Tuy vậy để completeness, mình để None
            return {
                "P": Ptot,
                "S": Stot,
                "PF": (Ptot/Stot) if abs(Stot) > 1e-9 else 0.0,
                "Q": Qtot,
                "S_true": Stot_true if Stot_true>0 else None,
                "V": None, "I": None
            }
        return {}

    def _raw_value_from_frame(self, d):
        qty = self.cmb_qty.currentText()
        ch  = self.cmb_chan.currentText()

        if qty == "Hz":
            return safe_float(d.get("f",0))

        D = self._pick_channel_dict(d, ch)
        if qty == "V":
            return safe_float(D.get("V",0))
        if qty == "A":
            return safe_float(D.get("I",0))
        if qty == "W":
            return safe_float(D.get("P",0))
        if qty == "VA":
            return safe_float(D.get("S",0))
        if qty == "VAR":
            # ưu tiên Q (nếu kênh A/B/C không có, thử extras)
            Qv = D.get("Q", None)
            if Qv is None:
                # riêng kênh A/B/C, nếu FW chưa embed Q thì extras có Q theo pha
                if "extras" in d and ch in ("A","B","C"):
                    ex = d["extras"] or {}
                    Qv = safe_float((ex.get(ch) or {}).get("Q",0))
                else:
                    Qv = 0.0
            return safe_float(Qv, 0.0)
        if qty == "PF":
            pf = D.get("PF", None)
            if pf is None and ch in ("A","B","C"):
                # PF = P/S fallback
                A = d.get(ch, {}) or {}
                P = safe_float(A.get("P",0)); S = safe_float(A.get("S",0))
                pf = (P/S) if abs(S) > 1e-9 else 0.0
            return safe_float(pf if pf is not None else 0.0)
        return 0.0

    def _unit_for(self, qty):
        return {"V":"V","A":"A","W":"W","VA":"VA","VAR":"var","PF":"PF","Hz":"Hz"}.get(qty,"")

    def on_tick(self):
        if self.last_frame is None: return
        if self.hold: return

        raw = self._raw_value_from_frame(self.last_frame)
        if self.rel_enabled:
            raw = raw - self.rel_zero
        val = self.ema.update(raw)

        # min/max/avg
        if self.min_v is None or val < self.min_v: self.min_v = val
        if self.max_v is None or val > self.max_v: self.max_v = val
        self.avg_sum += val; self.avg_n += 1

        qty  = self.cmb_qty.currentText()
        unit = self._unit_for(qty)

        self.lbl_big.setText(fmt_unit(val, unit))
        self.lbl_units.setText(f"Unit: {unit}")

        # PF tag (lead/lag) nếu có Q
        pf_tag = "—"
        if qty in ("PF","W","VA","VAR") and self.last_frame is not None:
            ch = self.cmb_chan.currentText()
            Qtot = None
            if ch == "Total":
                if "extras" in self.last_frame:
                    ex = self.last_frame["extras"] or {}
                    Qtot = safe_float((ex.get("A") or {}).get("Q",0)) + safe_float((ex.get("B") or {}).get("Q",0)) + safe_float((ex.get("C") or {}).get("Q",0))
                else:
                    # nếu FW embed Q từng pha trong A/B/C
                    A = self.last_frame.get("A",{}) or {}
                    B = self.last_frame.get("B",{}) or {}
                    C = self.last_frame.get("C",{}) or {}
                    Qtot = safe_float(A.get("Q",0)) + safe_float(B.get("Q",0)) + safe_float(C.get("Q",0))
            elif ch in ("A","B","C"):
                # lấy Q theo pha nếu có
                phase = self.last_frame.get(ch, {}) or {}
                if "Q" in phase:
                    Qtot = safe_float(phase.get("Q",0))
                elif "extras" in self.last_frame:
                    Qtot = safe_float((self.last_frame["extras"].get(ch) or {}).get("Q", 0))
            if Qtot is not None:
                if abs(Qtot) < 1e-6: pf_tag = "PF: ± (Q≈0)"
                elif Qtot > 0:       pf_tag = "PF: lag (inductive)"
                else:                 pf_tag = "PF: lead (capacitive)"
        self.lbl_pf_tag.setText(pf_tag)

        # mmavg
        self.lbl_min.setText(f"Min: {fmt_unit(self.min_v, unit)}")
        self.lbl_max.setText(f"Max: {fmt_unit(self.max_v, unit)}")
        avg_v = (self.avg_sum/self.avg_n) if self.avg_n>0 else 0.0
        self.lbl_avg.setText(f"Avg: {fmt_unit(avg_v, unit)}")

class BasePlotTab(QtWidgets.QWidget):
    """Base class for voltage/current plot tabs."""
    def __init__(self, title: str, channels: dict, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)

        # Style
        pg.setConfigOptions(antialias=True, background="#1e1e1e", foreground="w")

        # Create the single plot widget for all channels of this type
        self.plot_widget = pg.PlotWidget(title=title)
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend(offset=(10, 10))
        layout.addWidget(self.plot_widget)

        # Initialize lines and data
        self.lines = {}
        self.history = {k: [] for k in channels}
        self.max_points = 300
        self.colors = channels

        # Create a line per channel
        for key, (label, color) in self.colors.items():
            line = self.plot_widget.plot(pen=color, name=label)
            self.lines[key] = line

    def update_frame(self, frame: dict):
        for key in self.lines:
            if key in frame:
                self.history[key].append(frame[key])
                if len(self.history[key]) > self.max_points:
                    self.history[key].pop(0)
                xdata = range(len(self.history[key]))
                self.lines[key].setData(xdata, self.history[key])

class VoltagePlotTab(BasePlotTab):
    def __init__(self, parent=None):
        channels = {
            "AVRMS": ("Phase A", "r"),
            "BVRMS": ("Phase B", "g"),
            "CVRMS": ("Phase C", "b"),
        }
        super().__init__("Voltage (RMS)", channels, parent)
        self.plot_widget.setLabel("left", "Voltage (V)")
        self.plot_widget.setLabel("bottom", "Samples")

class CurrentPlotTab(BasePlotTab):
    def __init__(self, parent=None):
        channels = {
            "AIRMS": ("Phase A", "r"),
            "BIRMS": ("Phase B", "g"),
            "CIRMS": ("Phase C", "b"),
        }
        super().__init__("Current (RMS)", channels, parent)
        self.plot_widget.setLabel("left", "Current (A)")
        self.plot_widget.setLabel("bottom", "Samples")


# ------------------ Main Window ------------------

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ADE7878 Monitor — Ethernet + Real-time timestamps + Multimeter")
        self.resize(1040, 640)

        self.reader = UdpReader(TEENSY_IP, TEENSY_PORT)
        self.reader.paramFrame.connect(self.on_params)
        self.reader.status.connect(self.on_status)
        self.reader.transport.connect(self.on_transport)
        self.reader.i2cLock.connect(self.on_i2c_lock)
        self.reader.walltime.connect(self.on_walltime)
        self.reader.frameJson.connect(self.on_frame_json)

        # Top bar
        top = QtWidgets.QWidget(); tl = QtWidgets.QHBoxLayout(top)
        self.ip_edit = QtWidgets.QLineEdit(TEENSY_IP)
        self.port_edit = QtWidgets.QLineEdit(str(TEENSY_PORT))
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_disconnect = QtWidgets.QPushButton("Disconnect")
        self.lbl_transport = QtWidgets.QLabel("Transport: —")
        self.lbl_status = QtWidgets.QLabel("Idle")
        self.lbl_time = QtWidgets.QLabel("Time: —")
        self.i2c_lbl = QtWidgets.QLabel("I2C: —")
        self.online_lbl = QtWidgets.QLabel("Link: —")

        for w in (self.lbl_status, self.lbl_time, self.lbl_transport, self.online_lbl):
            w.setStyleSheet("font-size:13px;")

        self.btn_connect.clicked.connect(self.do_connect)
        self.btn_disconnect.clicked.connect(self.do_disconnect)

        self.i2c_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#999; color:#fff;")
        self.online_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#999; color:#fff;")

        tl.addWidget(QtWidgets.QLabel("IP:"));   tl.addWidget(self.ip_edit)
        tl.addWidget(QtWidgets.QLabel("Port:")); tl.addWidget(self.port_edit)
        tl.addStretch(1)
        tl.addWidget(self.lbl_time); tl.addSpacing(12)
        tl.addWidget(self.i2c_lbl);  tl.addSpacing(12)
        tl.addWidget(self.online_lbl); tl.addSpacing(12)
        tl.addWidget(self.lbl_transport); tl.addSpacing(12)
        tl.addWidget(self.btn_connect); tl.addWidget(self.btn_disconnect); tl.addSpacing(12)
        tl.addWidget(self.lbl_status, stretch=1)

        # Center: Tabs (Grid + Multimeter)
        self.tab = QtWidgets.QTabWidget()
        self.grid = ParamGrid(PARAM_NAMES)
        self.mm = MultimeterWidget()

        self.tab.addTab(self.grid, "Grid")
        self.tab.addTab(self.mm,   "Multimeter")

        # Addition: Plotting tabs
        self.voltageTab = VoltagePlotTab()
        self.currentTab = CurrentPlotTab()
        self.tab.addTab(self.voltageTab, "Voltages")
        self.tab.addTab(self.currentTab, "Currents")
        # Connect signal to plots
        # self.reader.paramFrame.connect(self.voltageTab.update_frame)
        # self.reader.paramFrame.connect(self.currentTab.update_frame)

        # Root
        root = QtWidgets.QWidget(); rl = QtWidgets.QVBoxLayout(root)
        rl.addWidget(top); rl.addWidget(self.tab, stretch=1)
        self.setCentralWidget(root)

        # Online/Offline indicator timer
        self._link_timer = QtCore.QTimer(self)
        self._link_timer.timeout.connect(self._tick_link)
        self._link_timer.start(250)

        self._last_frame_ts = 0.0


    def _tick_link(self):
        # coi như ONLINE nếu trong 1s gần nhất có frame
        online = (time.time() - self._last_frame_ts) <= NO_FRAME_TIMEOUT
        if online:
            self.online_lbl.setText("Link: ONLINE")
            self.online_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#5cb85c; color:#fff;")
        else:
            self.online_lbl.setText("Link: OFFLINE (SD logging?)")
            self.online_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#d9534f; color:#fff;")

    def do_connect(self):
        try:
            self.reader.ip   = self.ip_edit.text().strip()
            self.reader.port = int(self.port_edit.text().strip())
            self.reader.start()
        except Exception as e:
            self.on_status(f"Connect error: {e}")

    def do_disconnect(self):
        self.reader.stop()
        self.lbl_transport.setText("Transport: —")
        self.lbl_status.setText("Disconnected")
        self.lbl_time.setText("Time: —")
        self.i2c_lbl.setText("I2C: —")
        self.i2c_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#999; color:#fff;")
        self.online_lbl.setText("Link: —")
        self.online_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#999; color:#fff;")

    # ----- Slots -----
    @QtCore.pyqtSlot(list)
    def on_params(self, vals):
        self.grid.update_values(vals)
        # record last timestamp for online indicator
        self._last_frame_ts = time.time()

        # Map the 16 parameters to names for plots
        frame_dict = dict(zip(PARAM_NAMES, vals))

        # Update both plot tabs
        self.voltageTab.update_frame(frame_dict)
        self.currentTab.update_frame(frame_dict)

    @QtCore.pyqtSlot(dict)
    def on_frame_json(self, d):
        self._last_frame_ts = time.time()
        self.mm.set_frame(d)

    @QtCore.pyqtSlot(str)
    def on_status(self, s):
        self.lbl_status.setText(s)

    @QtCore.pyqtSlot(str)
    def on_transport(self, t):
        self.lbl_transport.setText(f"Transport: {t}")

    @QtCore.pyqtSlot(str)
    def on_walltime(self, s):
        self.lbl_time.setText(f"Time: {s}")

    @QtCore.pyqtSlot(bool)
    def on_i2c_lock(self, locked):
        if locked:
            self.i2c_lbl.setText("I2C: LOCK")
            self.i2c_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#d9534f; color:#fff;")
        else:
            self.i2c_lbl.setText("I2C: OK")
            self.i2c_lbl.setStyleSheet("font-weight:600; padding:2px 6px; border-radius:6px; background:#5cb85c; color:#fff;")

def main():
    app = QtWidgets.QApplication(sys.argv)
    # dark-ish palette cho dễ nhìn
    pal = app.palette()
    pal.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#1e1e1e"))
    pal.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor("#bbbbbb"))
    pal.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#252526"))
    pal.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor("#2d2d30"))
    pal.setColor(QtGui.QPalette.ColorRole.ToolTipBase, QtGui.QColor("#ffffdc"))
    pal.setColor(QtGui.QPalette.ColorRole.ToolTipText, QtGui.QColor("#000000"))
    pal.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor("#000000"))
    pal.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor("#2d2d30"))
    pal.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor("#000000"))
    pal.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor("#007acc"))
    pal.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor("#ffffff"))
    app.setPalette(pal)

    win = MainWindow(); win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
