#!/usr/bin/env python3
"""
odmr_or_light_mode_working.py

Combined PC-side script for Pico-based measurement.

Modes:
1) MODE = "odmr"
   - Uses the robust startup that already worked in your debug/light test
   - Sends timing/config commands to Pico
   - Sends: MEAS <freq_hz>
   - Expects: RESULT,freq_hz,off1,on,off2,off_ref,delta,contrast,drift
   - Saves CSV and plots mean delta vs frequency

2) MODE = "light"
   - Sends: MODE STREAM
   - Expects: DATA,<ms_since_boot>,<ch0>,<ch1>,<lux>
   - Live plots CH0, CH1, and lux vs time
   - Optionally logs CSV

Requires:
    pip install pyserial numpy matplotlib
"""

import time
import csv
import re
from collections import deque
from typing import Optional, Tuple, List

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
import serial.tools.list_ports


# ========================== USER SETTINGS ==========================
MODE = "odmr"   # "odmr" or "light"

PORT = "COM6"
BAUD = 115200

READ_TIMEOUT_S = 0.2
WRITE_TIMEOUT_S = 2.0

# Keep this the same as the debug script that worked
ASSERT_DTR = True
ASSERT_RTS = False

BOOT_WAIT_S = 2.5

# ---------- Pico timing config ----------
PICO_IT_MS = 200
PICO_EXTRA_WAIT_MS = 20
PICO_FREQ_SETTLE_MS = 100
PICO_RF_ON_SETTLE_MS = 0
PICO_RF_OFF_SETTLE_MS = 0
PICO_DISCARD_SAMPLES = 1
PICO_AVG_SAMPLES_PER_STATE = 4

# ---------- ODMR settings ----------
F_START_HZ = 2.85e9
F_STOP_HZ = 2.9e9
F_STEP_HZ = 0.01e9
N_REPEATS_PER_FREQ = 1
MEAS_TIMEOUT_S = 25.0

ODMR_CSV_PATH = "noField.csv"
ODMR_PLOT = True
PRINT_EACH_MEASUREMENT = True
PRINT_FREQ_SUMMARY = True

# ---------- Light/stream settings ----------
ROLLING_SECONDS = 60
ASSUMED_SAMPLE_HZ = 10
MAX_PLOT_HZ = 20

LIGHT_CSV_LOG = True
LIGHT_CSV_PATH = "tsl2591_log.csv"

PRINT_NONDATA_LINES = True
PRINT_RAW_DATA_LINES = False
HEARTBEAT_PERIOD_S = 1.0
# ==================================================================


RE_RESULT = re.compile(
    r"^\s*RESULT\s*,\s*"
    r"(\d+)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*,\s*"
    r"([+-]?\d+(?:\.\d+)?)\s*$",
    re.IGNORECASE
)

RE_DATA = re.compile(
    r"^\s*DATA\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*([+-]?\d+(?:\.\d+)?))?\s*$",
    re.IGNORECASE
)


def list_ports_hint() -> None:
    ports = list(serial.tools.list_ports.comports())
    print("Available serial ports:")
    if not ports:
        print("  (none)")
        return
    for p in ports:
        print(f"  {p.device:>8}  {p.description}")


def open_serial_port() -> serial.Serial:
    ser = serial.Serial(
        port=PORT,
        baudrate=BAUD,
        timeout=READ_TIMEOUT_S,
        write_timeout=WRITE_TIMEOUT_S,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )

    try:
        ser.setDTR(ASSERT_DTR)
        ser.setRTS(ASSERT_RTS)
    except Exception as e:
        print(f"Warning: couldn't set DTR/RTS: {e}")

    return ser


def send_line(ser: serial.Serial, line: str) -> None:
    payload = (line.rstrip() + "\n").encode("utf-8")
    print(f"[TX] {line}")
    n = ser.write(payload)
    ser.flush()
    if n != len(payload):
        raise IOError(f"Short write: wrote {n} of {len(payload)} bytes")


def read_lines_for(ser: serial.Serial, seconds: float, prefix: str = "[PICO]") -> list[str]:
    lines: list[str] = []
    t0 = time.time()
    while (time.time() - t0) < seconds:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(f"{prefix} {line}")
        lines.append(line)

    return lines


def startup_probe(ser: serial.Serial) -> None:
    """
    Match the startup style that already worked.
    """
    print(f"Connected. DTR={ASSERT_DTR}, RTS={ASSERT_RTS}")
    print("Waiting briefly for boot messages...")
    time.sleep(BOOT_WAIT_S)

    # Drain boot text if present
    read_lines_for(ser, 0.5)

    # Prove comms are alive
    send_line(ser, "PING")
    time.sleep(0.2)
    read_lines_for(ser, 0.5)

    send_line(ser, "STATUS")
    time.sleep(0.2)
    read_lines_for(ser, 0.5)

    send_line(ser, "GETCFG")
    time.sleep(0.2)
    read_lines_for(ser, 0.5)


def configure_pico_timings(ser: serial.Serial) -> None:
    print("Configuring Pico timings...")

    commands = [
        f"CFG IT_MS {int(PICO_IT_MS)}",
        f"CFG EXTRA_WAIT_MS {int(PICO_EXTRA_WAIT_MS)}",
        f"CFG FREQ_SETTLE_MS {int(PICO_FREQ_SETTLE_MS)}",
        f"CFG RF_ON_SETTLE_MS {int(PICO_RF_ON_SETTLE_MS)}",
        f"CFG RF_OFF_SETTLE_MS {int(PICO_RF_OFF_SETTLE_MS)}",
        f"CFG DISCARD {int(PICO_DISCARD_SAMPLES)}",
        f"CFG AVG {int(PICO_AVG_SAMPLES_PER_STATE)}",
        "GETCFG",
    ]

    for cmd in commands:
        send_line(ser, cmd)
        time.sleep(0.2)
        read_lines_for(ser, 0.4)


def build_frequency_list(start_hz: float, stop_hz: float, step_hz: float) -> np.ndarray:
    n = int(round((stop_hz - start_hz) / step_hz)) + 1
    return start_hz + step_hz * np.arange(n, dtype=np.float64)


def parse_result_line(line: str) -> Optional[Tuple[float, float, float, float, float, float, float, float]]:
    m = RE_RESULT.match(line)
    if not m:
        return None

    freq_hz = float(m.group(1))
    off1 = float(m.group(2))
    on = float(m.group(3))
    off2 = float(m.group(4))
    off_ref = float(m.group(5))
    delta = float(m.group(6))
    contrast = float(m.group(7))
    drift = float(m.group(8))

    return (freq_hz, off1, on, off2, off_ref, delta, contrast, drift)


def parse_data_line(line: str) -> Optional[Tuple[float, int, int, Optional[float]]]:
    m = RE_DATA.match(line)
    if not m:
        return None

    ms = int(m.group(1))
    ch0 = int(m.group(2))
    ch1 = int(m.group(3))
    lux = float(m.group(4)) if m.group(4) is not None else None

    t_rel_s = ms / 1000.0
    return (t_rel_s, ch0, ch1, lux)


def wait_for_result(ser: serial.Serial, expected_freq_hz: float, timeout_s: float = 10.0):
    """
    Wait for a RESULT line from the Pico.
    Ignores unrelated lines, but prints them for debugging.
    """
    t0 = time.time()
    expected = int(round(expected_freq_hz))

    while (time.time() - t0) < timeout_s:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        parsed = parse_result_line(line)
        if parsed is None:
            print(f"[PICO] {line}")
            continue

        freq_hz, off1, on, off2, off_ref, delta, contrast, drift = parsed

        if int(round(freq_hz)) != expected:
            print(f"[PICO] Warning: got result for unexpected frequency {freq_hz:.0f} Hz")
            continue

        return parsed

    raise TimeoutError(f"Timed out waiting for RESULT for {expected} Hz")


def run_odmr_mode(ser: serial.Serial) -> None:
    freqs_hz = build_frequency_list(F_START_HZ, F_STOP_HZ, F_STEP_HZ)

    freq_mean_delta = np.full(len(freqs_hz), np.nan, dtype=np.float64)
    freq_sem_delta = np.full(len(freqs_hz), np.nan, dtype=np.float64)

    n_freqs = len(freqs_hz)
    total_meas = n_freqs * N_REPEATS_PER_FREQ
    est_runtime_s = total_meas * MEAS_TIMEOUT_S / 6.0

    print("Setting Pico to ODMR mode...")
    send_line(ser, "MODE ODMR")
    time.sleep(0.2)
    read_lines_for(ser, 0.5)

    print(
        f"Planned run: {n_freqs} frequencies, "
        f"{N_REPEATS_PER_FREQ} repeats/frequency, "
        f"{total_meas} total measurements"
    )
    print(
        f"Estimated worst-case runtime: "
        f"{est_runtime_s / 60:.1f} min "
        f"({est_runtime_s / 3600:.2f} hr)"
    )

    with open(ODMR_CSV_PATH, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "freq_hz",
            "repeat_idx",
            "off1",
            "on",
            "off2",
            "off_ref",
            "delta_on_minus_offref",
            "contrast_on_minus_offref_over_offref",
            "drift_off2_minus_off1",
        ])

        try:
            for fi, freq_hz in enumerate(freqs_hz):
                print(f"\nRequesting measurements at {freq_hz / 1e9:.6f} GHz ...")

                deltas_this_freq: List[float] = []

                for repeat_idx in range(N_REPEATS_PER_FREQ):
                    send_line(ser, f"MEAS {int(round(freq_hz))}")

                    result = wait_for_result(ser, freq_hz, timeout_s=MEAS_TIMEOUT_S)
                    result_freq_hz, off1, on, off2, off_ref, delta, contrast, drift = result

                    if np.isfinite(delta):
                        deltas_this_freq.append(delta)

                    w.writerow([
                        f"{result_freq_hz:.0f}",
                        repeat_idx,
                        off1,
                        on,
                        off2,
                        off_ref,
                        delta,
                        contrast,
                        drift,
                    ])
                    f.flush()

                    if PRINT_EACH_MEASUREMENT:
                        print(
                            f"  rep {repeat_idx + 1:2d}/{N_REPEATS_PER_FREQ} | "
                            f"OFF1={off1:.6g} ON={on:.6g} OFF2={off2:.6g} "
                            f"OFF_REF={off_ref:.6g} Δ={delta:+.3e} "
                            f"contrast={contrast:+.3e} drift={drift:+.3e}"
                        )

                d = np.asarray(deltas_this_freq, dtype=np.float64)
                if d.size > 0:
                    freq_mean_delta[fi] = float(np.mean(d))
                    if d.size > 1:
                        freq_sem_delta[fi] = float(np.std(d, ddof=1) / np.sqrt(d.size))
                    else:
                        freq_sem_delta[fi] = 0.0

                if PRINT_FREQ_SUMMARY:
                    print(
                        f"Freq {freq_hz / 1e9:.6f} GHz summary: "
                        f"mean Δ = {freq_mean_delta[fi]:+.6e}, "
                        f"SEM = {freq_sem_delta[fi]:.6e}, "
                        f"n = {d.size}"
                    )

        finally:
            print("\nFinished ODMR run.")

    print(f"Saved {ODMR_CSV_PATH}")

    valid = np.isfinite(freq_mean_delta)
    if not np.any(valid):
        print("No valid sweep points collected.")
        return

    print("\nSweep summary:")
    for f_hz, d_mean, d_sem in zip(freqs_hz, freq_mean_delta, freq_sem_delta):
        print(f"  {f_hz / 1e9:.6f} GHz : mean Δ = {d_mean:+.6e}, SEM = {d_sem:.6e}")

    if ODMR_PLOT:
        plt.figure(figsize=(8, 5))
        plt.errorbar(
            freqs_hz[valid] / 1e9,
            freq_mean_delta[valid],
            yerr=freq_sem_delta[valid],
            fmt="o-",
            capsize=4,
        )
        plt.xlabel("Frequency (GHz)")
        plt.ylabel("Mean Δ = ON - OFF_REF")
        plt.title(f"ODMR sweep mean Δ vs frequency (n={N_REPEATS_PER_FREQ})")
        plt.grid(True)
        plt.tight_layout()
        plt.show()


def run_light_mode(ser: serial.Serial) -> None:
    maxlen = max(20, int(ROLLING_SECONDS * ASSUMED_SAMPLE_HZ))
    t = deque(maxlen=maxlen)
    ch0s = deque(maxlen=maxlen)
    ch1s = deque(maxlen=maxlen)
    luxs = deque(maxlen=maxlen)

    csv_file = None
    csv_writer = None
    if LIGHT_CSV_LOG:
        csv_file = open(LIGHT_CSV_PATH, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["t_pico_s", "ch0", "ch1", "lux", "raw"])

    print("Switching Pico to stream mode...")
    send_line(ser, "MODE STREAM")
    time.sleep(0.25)
    read_lines_for(ser, 0.75)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.set_title("TSL2591 live (Pico serial)")
    ax.set_xlabel("Pico time (s)")
    ax.set_ylabel("Counts / Lux")

    (ln_ch0,) = ax.plot([], [], label="CH0 (vis+IR)")
    (ln_ch1,) = ax.plot([], [], label="CH1 (IR)")
    (ln_lux,) = ax.plot([], [], label="Lux")
    ax.legend(loc="upper right")
    ax.grid(True)

    total_lines = 0
    total_samples = 0
    last_hb = time.time()
    plot_interval_ms = int(1000 / max(1, MAX_PLOT_HZ))

    def update(_frame):
        nonlocal total_lines, total_samples, last_hb

        read_any = False
        while True:
            try:
                waiting = ser.in_waiting
            except Exception:
                waiting = 0

            if waiting == 0 and read_any:
                break

            raw = ser.readline()
            if not raw:
                break

            read_any = True
            total_lines += 1

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            parsed = parse_data_line(line)
            if parsed is None:
                if PRINT_NONDATA_LINES:
                    print(f"[MSG] {line}")
                continue

            if PRINT_RAW_DATA_LINES:
                print(f"[DATA] {line}")

            t_s, ch0, ch1, lux = parsed
            t.append(t_s)
            ch0s.append(ch0)
            ch1s.append(ch1)
            luxs.append(float("nan") if lux is None else lux)
            total_samples += 1

            if csv_writer is not None:
                csv_writer.writerow([
                    f"{t_s:.3f}",
                    ch0,
                    ch1,
                    "" if lux is None else f"{lux:.4f}",
                    line,
                ])
                csv_file.flush()

        now = time.time()
        if (now - last_hb) >= HEARTBEAT_PERIOD_S:
            last_hb = now
            print(f"[HB] lines={total_lines} samples={total_samples} window_n={len(t)}")

        if len(t) >= 2:
            ln_ch0.set_data(t, ch0s)
            ln_ch1.set_data(t, ch1s)
            ln_lux.set_data(t, luxs)
            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"TSL2591 live (samples={total_samples})")

        return ln_ch0, ln_ch1, ln_lux

    ani = FuncAnimation(
        fig,
        update,
        interval=plot_interval_ms,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        print("Stopping stream...")
        try:
            send_line(ser, "STREAM OFF")
            time.sleep(0.2)
            read_lines_for(ser, 0.5)
        except Exception as e:
            print(f"Warning while stopping stream: {e}")

        if csv_file is not None:
            try:
                csv_file.flush()
                csv_file.close()
            except Exception:
                pass


def main():
    mode = MODE.strip().lower()
    if mode not in ("odmr", "light"):
        raise ValueError("MODE must be 'odmr' or 'light'")

    list_ports_hint()
    print(f"\nOpening {PORT} @ {BAUD} ...")

    try:
        ser = open_serial_port()
    except serial.SerialException as e:
        print(f"ERROR opening {PORT}: {e}")
        return

    try:
        startup_probe(ser)
        configure_pico_timings(ser)

        if mode == "odmr":
            print("Running in ODMR mode.")
            run_odmr_mode(ser)
        else:
            print("Running in light mode.")
            run_light_mode(ser)

    finally:
        print("Closing serial...")
        try:
            ser.close()
        except Exception:
            pass
        print("Closed.")


if __name__ == "__main__":
    main()