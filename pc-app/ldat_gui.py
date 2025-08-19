import sys, time, csv
from pathlib import Path
from statistics import mean, median, pstdev
from PySide6 import QtCore, QtGui, QtWidgets
import serial, serial.tools.list_ports

EOL = b"\n"   # your firmware accepts LF or CRLF; we'll send LF

# ---------- Serial worker (runs in its own thread) ----------
class SerialWorker(QtCore.QObject):
    line = QtCore.Signal(str)
    status = QtCore.Signal(str)
    connected = QtCore.Signal(bool)

    def __init__(self):
        super().__init__()
        self._ser = None
        self._running = False
        self._buf = bytearray()

    @QtCore.Slot(str, int)
    def start(self, port: str, baud: int):
        self.stop()
        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
            self._running = True
            self.connected.emit(True)
            self.status.emit(f"Connected {port} @ {baud}")
        except Exception as e:
            self._ser = None
            self.connected.emit(False)
            self.status.emit(f"ERROR opening {port}: {e}")
            return
        QtCore.QTimer.singleShot(0, self._pump)

    @QtCore.Slot()
    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            try: self._ser.close()
            except: pass
        self._ser = None
        self.connected.emit(False)

    def _pump(self):
        if not self._running or not self._ser:
            return
        try:
            data = self._ser.read(1024)
            if data:
                self._buf.extend(data)
                while True:
                    # look for LF first, else CR
                    i_n = self._buf.find(b"\n")
                    i_r = self._buf.find(b"\r")
                    idxs = [i for i in (i_n, i_r) if i != -1]
                    if not idxs:
                        break
                    i = min(idxs)
                    raw = bytes(self._buf[:i])        # bytes up to delimiter
                    # remove the line + delimiter (+ optional CRLF partner)
                    rm = 1
                    if i + 1 < len(self._buf) and self._buf[i] in (13,10) and self._buf[i+1] in (13,10) and self._buf[i+1] != self._buf[i]:
                        rm = 2
                    del self._buf[:i + rm]
                    s = raw.decode("utf-8", errors="replace").strip()
                    if s:
                        self.line.emit(s)
        except Exception as e:
            self.status.emit(f"Serial read error: {e}")
            self.stop()
            return
        QtCore.QTimer.singleShot(5, self._pump)

    @QtCore.Slot(str)
    def write_line(self, s: str):
        if not self._ser or not self._ser.is_open: return
        try:
            self._ser.write(s.encode("utf-8") + EOL)
        except Exception as e:
            self.status.emit(f"Serial write error: {e}")

# ---------- Table model ----------
class DataModel(QtGui.QStandardItemModel):
    def __init__(self):
        super().__init__(0, 3)
        self.setHorizontalHeaderLabels(["trial", "latency_us", "light"])

    def add_row(self, trial:int, latency:int, light:int):
        items = [
            QtGui.QStandardItem(str(trial)),
            QtGui.QStandardItem(str(latency)),
            QtGui.QStandardItem(str(light)),
        ]
        for it in items: it.setEditable(False)
        self.appendRow(items)

    def clear_data(self):
        self.setRowCount(0)

    def latency_values(self):
        vals = []
        for r in range(self.rowCount()):
            try:
                vals.append(int(self.item(r,1).text()))
            except: pass
        return vals

# ---------- Main window ----------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JTL LDAT")
        self.resize(980, 640)

        # state
        self.connected = False

        # serial thread
        self.thread = QtCore.QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)
        self.thread.start()

        # UI
        self._build_ui()

        # signals
        self.worker.line.connect(self.on_line)
        self.worker.status.connect(self.log)
        self.worker.connected.connect(self.on_connected)

        # populate ports
        self.refresh_ports()

    # ---- UI layout ----
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        lay = QtWidgets.QVBoxLayout(central)

        # Top bar: port/baud/connect
        top = QtWidgets.QHBoxLayout()
        self.port_cb = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.baud_cb = QtWidgets.QComboBox()
        self.baud_cb.addItems(["115200","230400","460800","921600"])
        self.baud_cb.setCurrentText("115200")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        top.addWidget(QtWidgets.QLabel("Port:"))
        top.addWidget(self.port_cb, 2)
        top.addWidget(self.refresh_btn)
        top.addSpacing(10)
        top.addWidget(QtWidgets.QLabel("Baud:"))
        top.addWidget(self.baud_cb)
        top.addSpacing(10)
        top.addWidget(self.connect_btn)
        top.addWidget(self.disconnect_btn)
        lay.addLayout(top)

        # Mode + calibration controls
        modebar = QtWidgets.QHBoxLayout()
        self.mode_cb = QtWidgets.QComboBox()
        self.mode_cb.addItems(["MODE1 (Calibrate)","MODE2 (Trigger)","MODE3 (Mic)"])
        self.set_mode_btn = QtWidgets.QPushButton("Set Mode")
        self.cal_read_btn = QtWidgets.QPushButton("CAL READ")
        self.cal_auto_btn = QtWidgets.QPushButton("CAL AUTO")
        self.cal_set_spin = QtWidgets.QSpinBox(); self.cal_set_spin.setRange(0, 4095); self.cal_set_spin.setValue(100)
        self.cal_set_btn = QtWidgets.QPushButton("CAL SET")
        modebar.addWidget(self.mode_cb)
        modebar.addWidget(self.set_mode_btn)
        modebar.addSpacing(10)
        modebar.addWidget(self.cal_read_btn)
        modebar.addWidget(self.cal_auto_btn)
        modebar.addWidget(QtWidgets.QLabel("thr:"))
        modebar.addWidget(self.cal_set_spin)
        modebar.addWidget(self.cal_set_btn)
        lay.addLayout(modebar)

        # Test controls
        testbar = QtWidgets.QHBoxLayout()
        self.n_spin = QtWidgets.QSpinBox(); self.n_spin.setRange(1, 100000); self.n_spin.setValue(5)
        self.start_btn = QtWidgets.QPushButton("TEST START N")
        self.stop_btn = QtWidgets.QPushButton("TEST STOP")
        self.clear_btn = QtWidgets.QPushButton("Clear Table")
        self.export_btn = QtWidgets.QPushButton("Export CSV")
        self.hello_btn = QtWidgets.QPushButton("HELLO")
        testbar.addWidget(QtWidgets.QLabel("N:"))
        testbar.addWidget(self.n_spin)
        testbar.addWidget(self.start_btn)
        testbar.addWidget(self.stop_btn)
        testbar.addSpacing(20)
        testbar.addWidget(self.clear_btn)
        testbar.addWidget(self.export_btn)
        testbar.addSpacing(20)
        testbar.addWidget(self.hello_btn)
        lay.addLayout(testbar)

        # Split: table + right pane (stats + log)
        split = QtWidgets.QSplitter()
        split.setOrientation(QtCore.Qt.Horizontal)

        # Table
        self.model = DataModel()
        self.view = QtWidgets.QTableView()
        self.view.setModel(self.model)
        self.view.horizontalHeader().setStretchLastSection(True)
        self.view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        split.addWidget(self.view)

        # Right pane
        right = QtWidgets.QWidget()
        rlay = QtWidgets.QVBoxLayout(right)

        self.stats_label = QtWidgets.QLabel("Stats: —")
        self.stats_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        rlay.addWidget(self.stats_label)

        self.status_label = QtWidgets.QLabel("Status: disconnected")
        rlay.addWidget(self.status_label)

        self.log_edit = QtWidgets.QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        rlay.addWidget(self.log_edit, 1)

        split.addWidget(right)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        lay.addWidget(split, 1)

        # wire buttons
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.do_connect)
        self.disconnect_btn.clicked.connect(self.do_disconnect)

        self.set_mode_btn.clicked.connect(self.do_set_mode)
        self.cal_read_btn.clicked.connect(lambda: self.send("CAL READ"))
        self.cal_auto_btn.clicked.connect(lambda: self.send("CAL AUTO"))
        self.cal_set_btn.clicked.connect(self.do_cal_set)

        self.start_btn.clicked.connect(self.do_start)
        self.stop_btn.clicked.connect(lambda: self.send("TEST STOP"))
        self.clear_btn.clicked.connect(self.clear_table)
        self.export_btn.clicked.connect(self.export_csv)
        self.hello_btn.clicked.connect(lambda: self.send("HELLO"))

    # ---- Port handling ----
    def refresh_ports(self):
        self.port_cb.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_cb.addItem(p.device)
        if not ports:
            self.port_cb.addItem("— no ports —")

    @QtCore.Slot(bool)
    def on_connected(self, ok: bool):
        self.connected = ok
        self.connect_btn.setEnabled(not ok)
        self.disconnect_btn.setEnabled(ok)
        self.status_label.setText("Status: connected" if ok else "Status: disconnected")
        if ok:
            # optional handshake
            QtCore.QTimer.singleShot(150, lambda: self.send("HELLO"))


    def do_connect(self):
        port = self.port_cb.currentText()
        if not port or "no ports" in port:
            self.log("No port selected.")
            return
        baud = int(self.baud_cb.currentText())
        QtCore.QMetaObject.invokeMethod(self.worker, "start", QtCore.Qt.QueuedConnection,
                                        QtCore.Q_ARG(str, port), QtCore.Q_ARG(int, baud))

    def do_disconnect(self):
        QtCore.QMetaObject.invokeMethod(self.worker, "stop", QtCore.Qt.QueuedConnection)

    # ---- Command helpers ----
    def send(self, s: str):
        if not self.connected:
            self.log("Not connected.")
            return
        self.log(f">>> {s}")
        QtCore.QMetaObject.invokeMethod(self.worker, "write_line", QtCore.Qt.QueuedConnection,
                                        QtCore.Q_ARG(str, s))

    def do_set_mode(self):
        idx = self.mode_cb.currentIndex()
        cmd = {0:"MODE1",1:"MODE2",2:"MODE3"}[idx]
        self.send(cmd)

    def do_cal_set(self):
        thr = self.cal_set_spin.value()
        self.send(f"CAL SET {thr}")

    def do_start(self):
        n = self.n_spin.value()
        # You can force Mode2 here so the scheduler runs even if user forgot:
        # self.send("MODE2")
        self.send(f"TEST START {n}")
        # Optional: clear table on each run
        # self.clear_table()

    # ---- Incoming lines ----
    @QtCore.Slot(str)
    def on_line(self, line: str):
        self.log(line)
        if not line:
            return

        if line.startswith("DATA,"):
            parts = [p.strip() for p in line.split(",")]
            # header "DATA,trial,latency_us,light"
            if len(parts) == 4 and parts[1].lower() == "trial":
                return
            # data row
            if len(parts) >= 4:
                try:
                    trial = int(parts[1]); latency = int(parts[2]); light = int(parts[3])
                except:
                    return
                self.model.add_row(trial, latency, light)
                self.update_stats()

        elif line.startswith("DONE"):
            self.status_label.setText("Status: run complete")

    # ---- Stats & CSV ----
    def update_stats(self):
        vals = self.model.latency_values()
        if not vals:
            self.stats_label.setText("Stats: —")
            return
        s = f"Stats: n={len(vals)}  mean={mean(vals):.1f}  median={median(vals):.1f}  " \
            f"min={min(vals)}  max={max(vals)}  std={(pstdev(vals) if len(vals)>1 else 0):.1f}"
        self.stats_label.setText(s)

    def clear_table(self):
        self.model.clear_data()
        self.update_stats()

    def export_csv(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export CSV", "ldat_data.csv", "CSV Files (*.csv)")
        if not path: return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["trial","latency_us","light"])
            for r in range(self.model.rowCount()):
                w.writerow([self.model.item(r,c).text() for c in range(3)])
        self.log(f"Saved: {path}")

    # ---- Logging ----
    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_edit.appendPlainText(f"[{ts}] {msg}")

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()