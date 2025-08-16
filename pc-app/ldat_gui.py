#!/usr/bin/env python3

"""
LDAT Serial GUI (PyQt6)
-----------------------
A simple GUI to talk to your STM32 over the ST-LINK Virtual COM Port.

Features:
- Select COM port + baud and connect
- Toggle Calibration vs Test mode
- Start a test N times
- Receive "DATA,..." lines from the device, show them in a table
- Export to CSV
- Basic statistics (count, mean, std, min, max, p10/50/90) per numeric column

Protocol (you can tweak easily):
PC -> "MODE CAL\n" or "MODE TEST\n"
PC -> "TEST START <n>\n"
MCU -> "ACK <text>\n"        (optional)
MCU -> "DATA,<trial>,<value>\n"  (CSV-compatible lines; can add more columns)
MCU -> "DONE\n"              (optional; used to know a batch finished)

Install:
    pip install -r requirements.txt
Run:
    python ldat_gui.py
"""

import sys
import csv
import time
import threading
from typing import List, Optional

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt, pyqtSignal, QObject
import serial
import serial.tools.list_ports as list_ports


def list_serial_ports() -> List[str]:
    ports = []
    for p in list_ports.comports():
        # Favor human-readable description if present
        label = f"{p.device} - {p.description}"
        ports.append(label)
    return ports


def extract_port_device(label: str) -> str:
    # label format "<device> - <desc>"
    return label.split(" - ")[0].strip()


class SerialWorker(QtCore.QObject):
    """Background reader for the serial port."""
    line_received = pyqtSignal(str)
    connected = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._running = False

    def connect(self, port: str, baud: int):
        try:
            self._ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self._running = True
            self.connected.emit(True, f"Connected to {port} @ {baud}")
            threading.Thread(target=self._read_loop, daemon=True).start()
        except Exception as e:
            self.connected.emit(False, f"Connect failed: {e}")

    def disconnect(self):
        self._running = False
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self.connected.emit(False, "Disconnected")

    def write_line(self, s: str):
        try:
            if self._ser and self._ser.is_open:
                if not s.endswith("\n"):
                    s += "\n"
                self._ser.write(s.encode("utf-8"))
        except Exception as e:
            self.line_received.emit(f"ERR write: {e}")

    def _read_loop(self):
        buf = bytearray()
        while self._running and self._ser and self._ser.is_open:
            try:
                data = self._ser.read(256)
                if data:
                    buf.extend(data)
                    # split by newline
                    while b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        try:
                            text = line.decode("utf-8", errors="replace").strip("\r")
                        except Exception:
                            text = repr(line)
                        self.line_received.emit(text)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.line_received.emit(f"ERR read: {e}")
                break
        self.connected.emit(False, "Disconnected")


class DataTableModel(QtCore.QAbstractTableModel):
    def __init__(self, headers: List[str] = None, parent=None):
        super().__init__(parent)
        self.headers = headers or []
        self.rows: List[List[str]] = []

    def rowCount(self, parent=None):
        return len(self.rows)

    def columnCount(self, parent=None):
        return len(self.headers)

    def data(self, index, role=Qt.ItemDataRole.DisplayRole):
        if not index.isValid():
            return None
        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            return self.rows[index.row()][index.column()]
        return None

    def headerData(self, section, orientation, role=Qt.ItemDataRole.DisplayRole):
        if role == Qt.ItemDataRole.DisplayRole and orientation == Qt.Orientation.Horizontal:
            if 0 <= section < len(self.headers):
                return self.headers[section]
        return None

    def add_row(self, row: List[str]):
        self.beginInsertRows(QtCore.QModelIndex(), len(self.rows), len(self.rows))
        self.rows.append(row)
        self.endInsertRows()

    def clear(self):
        self.beginResetModel()
        self.rows.clear()
        self.endResetModel()

    def set_headers(self, headers: List[str]):
        self.beginResetModel()
        self.headers = headers
        self.rows.clear()
        self.endResetModel()

    def export_csv(self, path: str):
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if self.headers:
                w.writerow(self.headers)
            for r in self.rows:
                w.writerow(r)

    def numeric_columns(self):
        """Return indices of columns that look numeric in the current data."""
        idxs = []
        for c in range(len(self.headers)):
            # check a few rows
            for r in self.rows[:50]:
                try:
                    float(r[c])
                    idxs.append(c)
                    break
                except Exception:
                    continue
        return sorted(set(idxs))


class MainWindow(QtWidgets.QMainWindow):
    send_line = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LDAT Serial GUI")
        self.resize(1000, 650)

        # --- Top controls ---
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports_btn = QtWidgets.QPushButton("Refresh")
        self.baud_combo = QtWidgets.QComboBox()
        for b in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]:
            self.baud_combo.addItem(str(b))
        self.baud_combo.setCurrentText("115200")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Port:"))
        top_bar.addWidget(self.port_combo, stretch=1)
        top_bar.addWidget(self.refresh_ports_btn)
        top_bar.addSpacing(10)
        top_bar.addWidget(QtWidgets.QLabel("Baud:"))
        top_bar.addWidget(self.baud_combo)
        top_bar.addSpacing(10)
        top_bar.addWidget(self.connect_btn)
        top_bar.addWidget(self.disconnect_btn)

        # --- Mode + Test controls ---
        self.mode_group = QtWidgets.QGroupBox("Mode")
        self.cal_radio = QtWidgets.QRadioButton("Calibration")
        self.test_radio = QtWidgets.QRadioButton("Test")
        self.test_radio.setChecked(True)
        mode_layout = QtWidgets.QHBoxLayout()
        mode_layout.addWidget(self.cal_radio)
        mode_layout.addWidget(self.test_radio)
        self.mode_group.setLayout(mode_layout)

        self.reps_spin = QtWidgets.QSpinBox()
        self.reps_spin.setRange(1, 1000000)
        self.reps_spin.setValue(10)
        self.start_btn = QtWidgets.QPushButton("Start Test")
        self.clear_btn = QtWidgets.QPushButton("Clear Table")

        mid_bar = QtWidgets.QHBoxLayout()
        mid_bar.addWidget(QtWidgets.QLabel("Repetitions:"))
        mid_bar.addWidget(self.reps_spin)
        mid_bar.addSpacing(20)
        mid_bar.addWidget(self.start_btn)
        mid_bar.addWidget(self.clear_btn)
        mid_bar.addStretch(1)

        # --- Table ---
        self.table = QtWidgets.QTableView()
        self.model = DataTableModel(headers=["trial", "value"])
        self.table.setModel(self.model)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)

        # --- Log + actions ---
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.send_edit = QtWidgets.QLineEdit()
        self.send_edit.setPlaceholderText("Type raw command and press Enter (e.g., MODE TEST)")
        self.export_btn = QtWidgets.QPushButton("Export CSV")
        self.stats_btn = QtWidgets.QPushButton("Compute Stats")

        bottom_bar = QtWidgets.QHBoxLayout()
        bottom_bar.addWidget(self.export_btn)
        bottom_bar.addWidget(self.stats_btn)
        bottom_bar.addStretch(1)

        # --- Layout ---
        central = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(central)
        v.addLayout(top_bar)
        v.addWidget(self.mode_group)
        v.addLayout(mid_bar)
        v.addWidget(QtWidgets.QLabel("Results"))
        v.addWidget(self.table, stretch=2)
        v.addWidget(QtWidgets.QLabel("Log"))
        v.addWidget(self.log, stretch=1)
        v.addWidget(self.send_edit)
        v.addLayout(bottom_bar)
        self.setCentralWidget(central)

        # --- Serial worker thread ---
        self.worker = SerialWorker()
        self.worker_thread = QtCore.QThread()
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.start()

        self.worker.line_received.connect(self.on_line)
        self.worker.connected.connect(self.on_connected)
        self.send_line.connect(self.worker.write_line)

        # --- Signals ---
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        self.disconnect_btn.clicked.connect(self.on_disconnect_clicked)
        self.cal_radio.toggled.connect(self.on_mode_changed)
        self.start_btn.clicked.connect(self.on_start_test)
        self.clear_btn.clicked.connect(self.on_clear)
        self.export_btn.clicked.connect(self.on_export_csv)
        self.stats_btn.clicked.connect(self.on_stats)
        self.send_edit.returnPressed.connect(self.on_send_line)

        self.refresh_ports()

    # ---------- UI Callbacks ----------
    def log_msg(self, s: str):
        self.log.appendPlainText(s)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list_serial_ports()
        if not ports:
            self.port_combo.addItem("(no ports found)")
        else:
            self.port_combo.addItems(ports)

    def on_connect_clicked(self):
        label = self.port_combo.currentText()
        if "(no ports" in label:
            self.log_msg("No port to connect.")
            return
        port = extract_port_device(label)
        baud = int(self.baud_combo.currentText())
        QtCore.QMetaObject.invokeMethod(
            self.worker, "connect", Qt.ConnectionType.QueuedConnection,
            QtCore.Q_ARG(str, port), QtCore.Q_ARG(int, baud)
        )

    def on_disconnect_clicked(self):
        QtCore.QMetaObject.invokeMethod(self.worker, "disconnect", Qt.ConnectionType.QueuedConnection)

    def on_connected(self, ok: bool, msg: str):
        self.log_msg(msg)
        self.connect_btn.setEnabled(not ok)
        self.disconnect_btn.setEnabled(ok)

    def on_mode_changed(self, checked: bool):
        # calibration if radio is checked; else test
        if self.cal_radio.isChecked():
            self.send_line.emit("MODE CAL")
            self.log_msg(">> MODE CAL")
        else:
            self.send_line.emit("MODE TEST")
            self.log_msg(">> MODE TEST")

    def on_start_test(self):
        n = self.reps_spin.value()
        self.send_line.emit(f"TEST START {n}")
        self.log_msg(f">> TEST START {n}")

    def on_clear(self):
        self.model.clear()
        self.log_msg("Cleared table.")

    def on_export_csv(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save CSV", "results.csv", "CSV Files (*.csv)")
        if not path:
            return
        try:
            self.model.export_csv(path)
            self.log_msg(f"Saved: {path}")
        except Exception as e:
            self.log_msg(f"Export failed: {e}")

    def on_stats(self):
        if not self.model.rows:
            self.log_msg("No data to analyze.")
            return
        # compute basic stats for numeric columns
        numeric_cols = self.model.numeric_columns()
        if not numeric_cols:
            self.log_msg("No numeric columns detected.")
            return

        def percentile(arr, p):
            if not arr:
                return float("nan")
            arr = sorted(arr)
            k = (len(arr)-1) * (p/100.0)
            f = int(k)
            c = min(f+1, len(arr)-1)
            if f == c:
                return arr[f]
            d0 = arr[f] * (c - k)
            d1 = arr[c] * (k - f)
            return d0 + d1

        report_lines = []
        for c in numeric_cols:
            name = self.model.headers[c] if c < len(self.model.headers) else f"col{c}"
            vals = []
            for r in self.model.rows:
                try:
                    vals.append(float(r[c]))
                except Exception:
                    pass
            if not vals:
                continue
            count = len(vals)
            mean = sum(vals)/count
            var = sum((x-mean)**2 for x in vals)/count if count>1 else 0.0
            std = var**0.5
            mn = min(vals)
            mx = max(vals)
            p10 = percentile(vals, 10)
            p50 = percentile(vals, 50)
            p90 = percentile(vals, 90)
            report_lines.append(
                f"[{name}] n={count}  mean={mean:.6g}  std={std:.6g}  min={mn:.6g}  "
                f"p10={p10:.6g}  median={p50:.6g}  p90={p90:.6g}  max={mx:.6g}"
            )

        if report_lines:
            self.log_msg("Stats:\n" + "\n".join(report_lines))
        else:
            self.log_msg("No numeric data found.")

    def on_send_line(self):
        s = self.send_edit.text().strip()
        if s:
            self.send_line.emit(s)
            self.log_msg(">> " + s)
            self.send_edit.clear()

    # ---------- Serial line handler ----------
    @QtCore.pyqtSlot(str)
    def on_line(self, line: str):
        # Examples:
        #   ACK ok
        #   DATA,1,123.45
        #   DONE
        self.log_msg(line)
        if line.startswith("DATA"):
            parts = [p.strip() for p in line.split(",")]
            # If it's the first DATA line and headers unknown, set default headers
            if self.model.columnCount() == 0:
                # Create generic headers based on number of columns
                self.model.set_headers([f"col{i}" for i in range(len(parts))])
            # If header looks default and length changed, reset headers
            if len(parts) != self.model.columnCount():
                self.model.set_headers([f"col{i}" for i in range(len(parts))])
            self.model.add_row(parts)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
