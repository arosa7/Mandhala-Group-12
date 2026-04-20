"""
mandhala_scan_gui.py

PyQt6 GUI combining Arduino motor control + Pico ODMR measurement.

For each angle theta (0, rot_step, 2*rot_step, ..., < 360):
    1. Arduino: move X-axis (outer ring) to theta
    2. Pico:    sweep frequencies min -> max (step = freq_step)
    3. Record RESULT line at each frequency

Final data: data[angle_idx][freq_idx] = (freq_hz, off1, on, off2,
                                         off_ref, delta, contrast, drift)

Requires:  pip install pyserial PyQt6
"""

import sys
import time
import csv
import re
from typing import Optional, List, Tuple

import serial
import serial.tools.list_ports

from PyQt6.QtCore import QThread, pyqtSignal
from PyQt6.QtGui import QFont, QTextCursor
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QTextEdit, QComboBox, QProgressBar,
    QFileDialog, QMessageBox, QDoubleSpinBox, QSpinBox,
)


RE_RESULT = re.compile(
    r"^\s*RESULT\s*,\s*(\d+)" + r"\s*,\s*([+-]?\d+(?:\.\d+)?)" * 7 + r"\s*$",
    re.IGNORECASE,
)


def parse_result(line: str):
    m = RE_RESULT.match(line)
    return tuple(float(m.group(i)) for i in range(1, 9)) if m else None


# --------------------------------------------------------------------------
#  Background worker
# --------------------------------------------------------------------------
class ScanWorker(QThread):
    log = pyqtSignal(str)
    progress = pyqtSignal(int, int)
    finished_ok = pyqtSignal(list)
    finished_err = pyqtSignal(str)

    def __init__(self, arduino_port, pico_port, baud,
                 rot_step, min_f, max_f, step_f, timeout=25.0):
        super().__init__()
        self.arduino_port, self.pico_port, self.baud = arduino_port, pico_port, baud
        self.rot_step, self.min_f, self.max_f, self.step_f = rot_step, min_f, max_f, step_f
        self.timeout = timeout
        self._stop = False

    def request_stop(self):
        self._stop = True

    def _send(self, ser, line, tag):
        ser.write((line.rstrip() + "\n").encode())
        ser.flush()
        self.log.emit(f"[TX {tag}] {line}")

    def _drain(self, ser, seconds, tag):
        t0 = time.time()
        while time.time() - t0 < seconds:
            raw = ser.readline()
            if raw:
                text = raw.decode("utf-8", errors="replace").strip()
                if text:
                    self.log.emit(f"[{tag}] {text}")

    def _wait_move(self, ser):
        """Arduino has no 'done' flag; wait worst-case (half a revolution)."""
        STEPS_HALF = 300          # 200*3 / 2
        STEP_DELAY_US = 1500
        settle_ms = int(STEPS_HALF * 2 * STEP_DELAY_US / 1000) + 300
        t0 = time.time()
        while (time.time() - t0) * 1000 < settle_ms:
            if self._stop:
                return
            raw = ser.readline()
            if raw:
                txt = raw.decode("utf-8", errors="replace").strip()
                if txt:
                    self.log.emit(f"[arduino] {txt}")

    def _meas(self, ser, freq_hz):
        self._send(ser, f"MEAS {freq_hz}", "pico")
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            if self._stop:
                return None
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            parsed = parse_result(text)
            if parsed is None:
                self.log.emit(f"[pico] {text}")
            else:
                return parsed
        raise TimeoutError(f"No RESULT from Pico for {freq_hz} Hz")

    def _config_pico(self, ser):
        self.log.emit("Configuring Pico...")
        for cmd in ["CFG IT_MS 200", "CFG EXTRA_WAIT_MS 20",
                    "CFG FREQ_SETTLE_MS 100", "CFG RF_ON_SETTLE_MS 0",
                    "CFG RF_OFF_SETTLE_MS 0", "CFG DISCARD 1",
                    "CFG AVG 4", "GETCFG", "MODE ODMR"]:
            self._send(ser, cmd, "pico")
            time.sleep(0.15)
            self._drain(ser, 0.2, "pico")

    def run(self):
        arduino = pico = None
        try:
            if self.rot_step <= 0 or self.step_f <= 0:
                raise ValueError("Step sizes must be > 0")
            if self.max_f < self.min_f:
                raise ValueError("max_freq must be >= min_freq")

            angles = [round(i * self.rot_step, 6)
                      for i in range(int(360.0 / self.rot_step))]
            n_f = int(round((self.max_f - self.min_f) / self.step_f)) + 1
            freqs = [self.min_f + i * self.step_f for i in range(n_f)]
            total = len(angles) * len(freqs)

            self.log.emit("=" * 60)
            self.log.emit(f"Starting scan: 360/{self.rot_step} = {len(angles)} angles")
            self.log.emit(f"Collecting [{len(angles)}] scans, {len(freqs)} freqs each "
                          f"({total} total measurements)")
            self.log.emit("=" * 60)

            self.log.emit(f"Opening Arduino on {self.arduino_port}")
            arduino = serial.Serial(self.arduino_port, self.baud, timeout=0.2)
            time.sleep(2.0)
            self._drain(arduino, 0.5, "arduino")

            self.log.emit(f"Opening Pico on {self.pico_port}")
            pico = serial.Serial(self.pico_port, self.baud, timeout=0.2)
            try:
                pico.setDTR(True)
                pico.setRTS(False)
            except Exception:
                pass
            time.sleep(2.5)
            self._drain(pico, 0.5, "pico")

            self._send(pico, "PING", "pico")
            self._drain(pico, 0.3, "pico")
            self._config_pico(pico)

            self.log.emit("Moving to home position (0 deg)...")
            self._send(arduino, "0,0", "arduino")
            self._wait_move(arduino)

            data_3d: List[List[Tuple[float, ...]]] = []
            done = 0
            for ai, angle in enumerate(angles):
                if self._stop:
                    self.log.emit("Stop requested.")
                    break

                self.log.emit(f"\n>>> Scanning at [{angle:.2f}] deg "
                              f"({ai + 1}/{len(angles)})")
                self._send(arduino, f"{angle:.2f},0", "arduino")
                self._wait_move(arduino)

                angle_rows: List[Tuple[float, ...]] = []
                for f_ghz in freqs:
                    if self._stop:
                        break
                    f_hz = int(round(f_ghz * 1e9))
                    self.log.emit(f"   scanning [{f_ghz:.4f} GHz] at [{angle:.2f}] deg")
                    row = self._meas(pico, f_hz)
                    if row is None:
                        break
                    angle_rows.append(row)
                    self.log.emit(f"      delta={row[5]:+.3e}  contrast={row[6]:+.3e}")
                    done += 1
                    self.progress.emit(done, total)
                data_3d.append(angle_rows)

            self.log.emit("\nReturning home...")
            self._send(arduino, "0,0", "arduino")
            self._wait_move(arduino)
            self.log.emit("Scan finished.")
            self.finished_ok.emit(data_3d)

        except Exception as e:
            self.finished_err.emit(f"{type(e).__name__}: {e}")
        finally:
            for s in (arduino, pico):
                if s:
                    try:
                        s.close()
                    except Exception:
                        pass


# --------------------------------------------------------------------------
#  Main window
# --------------------------------------------------------------------------
class MandhalaGUI(QWidget):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mandhala Scan - Arduino + Pico")
        self.resize(960, 600)
        self.worker: Optional[ScanWorker] = None
        self.data_3d: Optional[list] = None
        self._angles: List[float] = []
        self._build_ui()
        self._refresh_ports()

    @staticmethod
    def _spin(lo, hi, val, decimals, suffix):
        s = QDoubleSpinBox()
        s.setRange(lo, hi); s.setDecimals(decimals)
        s.setSingleStep(10 ** -decimals)
        s.setValue(val); s.setSuffix(suffix)
        return s

    def _build_ui(self):
        root = QHBoxLayout(self)
        left, right = QVBoxLayout(), QVBoxLayout()
        root.addLayout(left, 0); root.addLayout(right, 1)

        # --- ports ---
        pbox = QGroupBox("Serial Ports")
        pg = QGridLayout(pbox)
        self.cb_arduino = QComboBox()
        self.cb_pico = QComboBox()
        btn_refresh = QPushButton("Refresh")
        btn_refresh.clicked.connect(self._refresh_ports)
        pg.addWidget(QLabel("Arduino (motor):"), 0, 0); pg.addWidget(self.cb_arduino, 0, 1)
        pg.addWidget(QLabel("Pico (ODMR):"),     1, 0); pg.addWidget(self.cb_pico,    1, 1)
        pg.addWidget(btn_refresh, 2, 0, 1, 2)
        left.addWidget(pbox)

        # --- motor ---
        mbox = QGroupBox("Motor Controls")
        mg = QGridLayout(mbox)
        self.sb_rot = self._spin(0.1, 180.0, 5.0, 2, " deg")
        mg.addWidget(QLabel("rot_step_size:"), 0, 0); mg.addWidget(self.sb_rot, 0, 1)
        left.addWidget(mbox)

        # --- scanning ---
        sbox = QGroupBox("Scanning Controls")
        sg = QGridLayout(sbox)
        self.sb_min  = self._spin(0.001, 10.0, 2.85, 4, " GHz")
        self.sb_max  = self._spin(0.001, 10.0, 2.99, 4, " GHz")
        self.sb_step = self._spin(0.0001, 1.0, 0.01, 4, " GHz")
        sg.addWidget(QLabel("min_freq:"),       0, 0); sg.addWidget(self.sb_min,  0, 1)
        sg.addWidget(QLabel("max_freq:"),       1, 0); sg.addWidget(self.sb_max,  1, 1)
        sg.addWidget(QLabel("freq_step_size:"), 2, 0); sg.addWidget(self.sb_step, 2, 1)
        left.addWidget(sbox)

        # --- baud ---
        bbox = QGroupBox("Connection")
        bg = QGridLayout(bbox)
        self.sb_baud = QSpinBox()
        self.sb_baud.setRange(9600, 1_000_000); self.sb_baud.setValue(115200)
        bg.addWidget(QLabel("Baud:"), 0, 0); bg.addWidget(self.sb_baud, 0, 1)
        left.addWidget(bbox)

        # --- buttons ---
        row = QHBoxLayout()
        self.btn_start = QPushButton("Start")
        self.btn_start.setStyleSheet("font-weight: bold; padding: 8px;")
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet("padding: 8px;")
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(lambda: self.worker and self.worker.request_stop())
        row.addWidget(self.btn_start); row.addWidget(self.btn_stop)
        left.addLayout(row)

        self.btn_save = QPushButton("Save data to CSV...")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._on_save)
        left.addWidget(self.btn_save)
        left.addStretch(1)

        # --- right: log + progress ---
        right.addWidget(QLabel("Terminal output:"))
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True)
        f = QFont("Courier New"); f.setStyleHint(QFont.StyleHint.Monospace); f.setPointSize(9)
        self.txt_log.setFont(f)
        self.txt_log.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4;")
        right.addWidget(self.txt_log, 1)

        self.progress = QProgressBar()
        right.addWidget(self.progress)
        self.lbl_status = QLabel("Idle.")
        right.addWidget(self.lbl_status)

    # --- slots ---
    def _refresh_ports(self):
        ports = sorted(p.device for p in serial.tools.list_ports.comports())
        for cb in (self.cb_arduino, self.cb_pico):
            cur = cb.currentText()
            cb.clear(); cb.addItems(ports)
            if cur in ports:
                cb.setCurrentText(cur)
        if len(ports) >= 2 and self.cb_arduino.currentText() == self.cb_pico.currentText():
            self.cb_pico.setCurrentIndex(1)
        self._log(f"# Found ports: {ports}")

    def _log(self, text):
        self.txt_log.append(text)
        self.txt_log.moveCursor(QTextCursor.MoveOperation.End)

    def _on_start(self):
        ap, pp = self.cb_arduino.currentText(), self.cb_pico.currentText()
        if not ap or not pp:
            QMessageBox.warning(self, "Ports", "Select both Arduino and Pico ports."); return
        if ap == pp:
            QMessageBox.warning(self, "Ports", "Ports must be different."); return
        if self.sb_max.value() < self.sb_min.value():
            QMessageBox.warning(self, "Frequency", "max_freq must be >= min_freq."); return

        rot, mn, mx, stp = (self.sb_rot.value(), self.sb_min.value(),
                            self.sb_max.value(), self.sb_step.value())
        n_a = int(360.0 / rot)
        n_f = int(round((mx - mn) / stp)) + 1
        total = n_a * n_f

        self._log("-" * 60)
        self._log(f"Preview: {n_a} angles x {n_f} freqs = {total} measurements")
        self.progress.setRange(0, total); self.progress.setValue(0)
        self.lbl_status.setText("Running...")
        self.btn_start.setEnabled(False); self.btn_stop.setEnabled(True)
        self.btn_save.setEnabled(False)
        self._angles = [i * rot for i in range(n_a)]

        self.worker = ScanWorker(ap, pp, self.sb_baud.value(), rot, mn, mx, stp)
        self.worker.log.connect(self._log)
        self.worker.progress.connect(self._on_progress)
        self.worker.finished_ok.connect(self._on_done)
        self.worker.finished_err.connect(self._on_err)
        self.worker.start()

    def _on_progress(self, cur, total):
        if self.progress.maximum() != total:
            self.progress.setRange(0, total)
        self.progress.setValue(cur)
        self.lbl_status.setText(f"Progress: {cur} / {total}")

    def _on_done(self, data_3d):
        self.data_3d = data_3d
        self._log(f"# Done. Collected data for {len(data_3d)} angles.")
        self.lbl_status.setText("Finished.")
        self.btn_start.setEnabled(True); self.btn_stop.setEnabled(False)
        self.btn_save.setEnabled(True)

    def _on_err(self, msg):
        self._log(f"# ERROR: {msg}")
        self.lbl_status.setText(f"Error: {msg}")
        self.btn_start.setEnabled(True); self.btn_stop.setEnabled(False)
        QMessageBox.critical(self, "Scan error", msg)

    def _on_save(self):
        if not self.data_3d:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save scan data", "mandhala_scan.csv", "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["angle_deg", "freq_hz", "off1", "on", "off2",
                            "off_ref", "delta", "contrast", "drift"])
                for ai, rows in enumerate(self.data_3d):
                    if ai >= len(self._angles):
                        break
                    for r in rows:
                        w.writerow([f"{self._angles[ai]:.4f}", f"{r[0]:.0f}", *r[1:]])
            self._log(f"# Saved to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Save error", str(e))

    def closeEvent(self, event):
        if self.worker and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(3000)
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MandhalaGUI()
    w.show()
    sys.exit(app.exec())
