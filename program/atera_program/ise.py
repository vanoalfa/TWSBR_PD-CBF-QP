"""ISE (Integral Square Error) evaluation for ATERA.

    error_psi         = setpoint_psi         - angle_psi          (body, MPU6050)
    error_theta_left  = setpoint_theta_left  - angle_theta_left   (left wheel, DDSM115)
    error_theta_right = setpoint_theta_right - angle_theta_right  (right wheel, DDSM115)
    ISE = integral of error(t)^2 dt   (trapezoidal rule with the measured time stamps)

- The plot (3 panels: body, left wheel, right wheel; Y = error, X = time) appears
  when MULAI is pressed and is saved automatically as PLOT_EVALUASI/YYYYMMDD-URUTAN.png
  (plus .csv with the raw samples) when balancing stops.
- Plotting runs in a SEPARATE PROCESS so matplotlib never delays the control loop.
  The control loop only appends a tuple to a list and, a few times per second,
  puts the batch on a queue.
"""

from __future__ import annotations

import csv
import logging
import multiprocessing as mp
import os
import queue
import re
import time
from typing import List, Optional, Tuple

import config

LOGGER = logging.getLogger(__name__)

CSV_COLUMNS = (
    "time_s",
    "setpoint_psi_deg", "angle_psi_deg", "error_psi_deg",
    "setpoint_theta_left_deg", "angle_theta_left_deg", "error_theta_left_deg",
    "setpoint_theta_right_deg", "angle_theta_right_deg", "error_theta_right_deg",
    "u_PD", "u_cmd", "cbf_active", "qp_time_ms",
)
_COL_T, _COL_E_PSI, _COL_E_LEFT, _COL_E_RIGHT = 0, 3, 6, 9
_MAX_LIVE_POINTS = 4000


def plot_dir_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), config.ISE_PLOT_DIR)


def next_run_name(plot_dir: str, previous: str = "") -> str:
    """YYYYMMDD-URUTAN, URUTAN = next free number for today (001, 002, ...).

    `previous` is the last name handed out by this program; its file may still be
    being written by the plot process, so it must not be reused.
    """
    date = time.strftime("%Y%m%d")
    pattern = re.compile(rf"^{date}-(\d+)\.")
    last = 0
    if previous.startswith(date + "-"):
        last = int(previous.split("-")[1])
    if os.path.isdir(plot_dir):
        for name in os.listdir(plot_dir):
            match = pattern.match(name)
            if match:
                last = max(last, int(match.group(1)))
    return f"{date}-{last + 1:03d}"


def write_csv(path: str, rows: List[tuple]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_COLUMNS)
        writer.writerows(rows)


class ISEEvaluator:
    """Lives in the control process. All methods are cheap and never block."""

    def __init__(self) -> None:
        self.plot_dir = plot_dir_path()
        self.active = False
        self.run_name = ""
        self.last_saved = ""
        self.ise_psi = 0.0
        self.ise_theta_left = 0.0
        self.ise_theta_right = 0.0
        self.samples = 0
        self._prev: Optional[Tuple[float, float, float, float]] = None
        self._rows: List[tuple] = []
        self._sent = 0
        self._send_every = max(1, int(float(config.CONTROL_HZ) / float(config.ISE_PLOT_REFRESH_HZ)))
        self._ctx = mp.get_context("spawn")
        self._queue = None
        self._process = None

    # ---- plot process management -------------------------------------------------
    def open(self) -> None:
        os.makedirs(self.plot_dir, exist_ok=True)
        self._ensure_process()

    def _ensure_process(self) -> None:
        if self._process is not None and self._process.is_alive():
            return
        self._queue = self._ctx.Queue()
        settings = {
            "live": bool(config.ISE_LIVE_PLOT),
            "refresh_s": 1.0 / float(config.ISE_PLOT_REFRESH_HZ),
            "save_csv": bool(config.ISE_SAVE_CSV),
        }
        self._process = self._ctx.Process(
            target=_plot_worker, args=(self._queue, settings), name="atera_ise_plot", daemon=True
        )
        self._process.start()
        self._queue.put_nowait(("hello",))   # starts the queue feeder thread now, not at the first sample

    def _put(self, message: tuple) -> bool:
        try:
            if self._process is None or not self._process.is_alive():
                return False
            self._queue.put_nowait(message)
            return True
        except Exception as exc:  # the control loop must never die because of plotting
            LOGGER.error("ISE plot queue error: %s", exc)
            return False

    # ---- one evaluation run ------------------------------------------------------
    def start(self, title: str = "") -> str:
        """Call when MULAI is pressed. Opens the plot and returns the run name."""
        try:
            self._ensure_process()
        except Exception as exc:
            LOGGER.error("ISE plot process could not start: %s", exc)
        self.run_name = next_run_name(self.plot_dir, self.run_name)
        self.ise_psi = self.ise_theta_left = self.ise_theta_right = 0.0
        self.samples = 0
        self._prev = None
        self._rows = []
        self._sent = 0
        self.active = True
        self._put(("start", self.run_name, title))
        return self.run_name

    def add_sample(
        self,
        time_s: float,
        setpoint_psi: float,
        angle_psi: float,
        setpoint_theta_left: float,
        angle_theta_left: float,
        setpoint_theta_right: float,
        angle_theta_right: float,
        u_PD: float = 0.0,
        u_cmd: float = 0.0,
        cbf_active: bool = False,
        qp_time_ms: float = 0.0,
    ) -> None:
        if not self.active:
            return
        error_psi = setpoint_psi - angle_psi
        error_left = setpoint_theta_left - angle_theta_left
        error_right = setpoint_theta_right - angle_theta_right

        if self._prev is not None:
            dt = time_s - self._prev[0]
            if dt > 0.0:
                self.ise_psi += 0.5 * (self._prev[1] ** 2 + error_psi ** 2) * dt
                self.ise_theta_left += 0.5 * (self._prev[2] ** 2 + error_left ** 2) * dt
                self.ise_theta_right += 0.5 * (self._prev[3] ** 2 + error_right ** 2) * dt
        self._prev = (time_s, error_psi, error_left, error_right)

        self._rows.append((
            round(time_s, 5),
            setpoint_psi, angle_psi, error_psi,
            setpoint_theta_left, angle_theta_left, error_left,
            setpoint_theta_right, angle_theta_right, error_right,
            u_PD, u_cmd, int(bool(cbf_active)), qp_time_ms,
        ))
        self.samples += 1
        if (self.samples - self._sent) >= self._send_every:
            self._flush()

    def _ise(self) -> Tuple[float, float, float]:
        return (self.ise_psi, self.ise_theta_left, self.ise_theta_right)

    def _flush(self) -> None:
        if self._sent < len(self._rows):
            self._put(("data", self._rows[self._sent:], self._ise()))
            self._sent = len(self._rows)

    def stop(self) -> str:
        """Call when balancing ends (stop, fault or quit). Saves PNG (+CSV). Returns the base path."""
        if not self.active:
            return ""
        self.active = False
        self._flush()
        base = os.path.join(self.plot_dir, self.run_name)
        if not self._put(("stop", base, self._ise())):
            # Plot process is gone: keep at least the raw data.
            try:
                write_csv(base + ".csv", self._rows)
                LOGGER.warning("ISE plot process not running, saved CSV only: %s.csv", base)
            except Exception as exc:
                LOGGER.error("ISE CSV save failed: %s", exc)
        self.last_saved = base
        return base

    def close(self) -> None:
        self.stop()
        if self._process is None:
            return
        if self._put(("quit",)):
            self._process.join(timeout=15.0)
        if self._process.is_alive():
            self._process.terminate()
        self._process = None


# ==================================================================================
# Plot process
# ==================================================================================
_PANELS = (
    ("Body (MPU6050): error_psi = setpoint_psi - angle_psi", _COL_E_PSI),
    ("Left wheel (DDSM115): error_theta = setpoint_theta - angle_theta", _COL_E_LEFT),
    ("Right wheel (DDSM115): error_theta = setpoint_theta - angle_theta", _COL_E_RIGHT),
)
_INK = "#1f2933"
_MUTED = "#6b7785"
_LINE = "#2563a8"


def _plot_worker(q, settings: dict) -> None:
    import matplotlib

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if not (settings["live"] and has_display):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    live = settings.get("assume_gui", False) or (
        settings["live"] and has_display and matplotlib.get_backend().lower() != "agg"
    )
    if live:
        plt.ion()

    fig = None
    axes = []
    lines = []
    rows: List[tuple] = []
    ise = (0.0, 0.0, 0.0)
    run_name = ""
    title = ""
    running = False
    dirty = False
    last_draw = 0.0

    def new_figure():
        nonlocal fig, axes, lines
        plt.close("all")
        fig, axes_arr = plt.subplots(3, 1, sharex=True, figsize=(10.0, 8.0))
        axes = list(axes_arr)
        lines = []
        for ax in axes:
            ax.axhline(0.0, color=_MUTED, linewidth=0.8)
            (line,) = ax.plot([], [], color=_LINE, linewidth=1.3)
            lines.append(line)
            ax.set_ylabel("Error [deg]", color=_INK)
            ax.grid(True, color="#d9dee4", linewidth=0.6)
            ax.tick_params(colors=_MUTED)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color("#c3cad2")
        axes[-1].set_xlabel("Time since MULAI [s]", color=_INK)
        if live:
            fig.show()

    def redraw(full: bool):
        stride = 1 if full else max(1, len(rows) // _MAX_LIVE_POINTS)
        t = [r[_COL_T] for r in rows[::stride]]
        for ax, line, (label, col), value in zip(axes, lines, _PANELS, ise):
            line.set_data(t, [r[col] for r in rows[::stride]])
            ax.relim()
            ax.autoscale_view()
            ax.set_title(f"{label}   |   ISE = {value:.4f} deg²·s", loc="left", fontsize=10, color=_INK)
        fig.suptitle(f"ATERA ISE evaluation  {run_name}   {title}", fontsize=12, color=_INK)
        fig.tight_layout(rect=(0, 0, 1, 0.97))

    def save(base: str):
        redraw(full=True)
        fig.savefig(base + ".png", dpi=150)
        if settings["save_csv"]:
            write_csv(base + ".csv", rows)

    while True:
        try:
            message = q.get(timeout=0.05)
        except queue.Empty:
            message = None
        except (EOFError, OSError):
            break

        if message is not None:
            kind = message[0]
            try:
                if kind == "start":
                    run_name, title = message[1], message[2]
                    rows = []
                    ise = (0.0, 0.0, 0.0)
                    new_figure()
                    running = True
                    dirty = True
                elif kind == "data" and running:
                    rows.extend(message[1])
                    ise = message[2]
                    dirty = True
                elif kind == "stop" and running:
                    ise = message[2]
                    save(message[1])
                    running = False
                    dirty = False
                    if live:
                        fig.canvas.draw_idle()
                elif kind == "quit":
                    break
            except Exception as exc:  # keep the worker alive for the next run
                print(f"[ise] plot worker error: {exc!r}", flush=True)

        if live and fig is not None:
            try:
                now = time.monotonic()
                if dirty and (now - last_draw) >= settings["refresh_s"]:
                    redraw(full=False)
                    fig.canvas.draw_idle()
                    last_draw = now
                    dirty = False
                fig.canvas.flush_events()
            except Exception:
                pass  # e.g. the user closed the window; saving still works

    plt.close("all")