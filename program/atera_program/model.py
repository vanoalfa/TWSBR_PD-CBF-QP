"""atera_main.py - program utama ATERA (robot fisik) + GUI terminal.

Menjalankan: python3 atera_main.py     (dari folder atera_program, di Raspberry Pi 5)

Alur tiap siklus kontrol (config.CONTROL_HZ):
    joystick -> IMU (psi, dpsi) -> state x = [theta, dtheta, psi, dpsi]
    -> u_PD = pd.compute(x - x_ref)
    -> mode PD     : u = u_PD (dijenuhkan ke batas aktuator)
       mode PD+CBF : u, feasible = cbf_qp.filter(x, u_PD)
    -> DDSM115 (u/2 per roda) -> umpan balik encoder (theta, dtheta) untuk siklus berikutnya
    -> ise.py mencatat error

State program:  IDLE -> (Tombol Y) CALIBRATING -> READY -> (MULAI) BALANCING -> (Tombol X) READY
                FAULT bila |psi| > SAFE_TILT_DEG atau komunikasi hardware gagal (motor dimatikan).

GUI: 6 halaman, pindah dengan LT / RT. Log ditulis ke config.LOG_FILE (bukan ke layar).
"""

from __future__ import annotations

import logging
import math
import os
import shutil
import signal
import sys
import time

import numpy as np

import config
import ise
import joystick_mapping as jm
import model
from cbf_qp import CBFQP, H_NAMES, barrier_values
from ddsm115 import DDSM115Error, DualDDSM115, decode_error
from mpu6050 import MPU6050
from pd_control import PDController

LOGGER = logging.getLogger("atera")
DEG = math.degrees(1.0)

IDLE, CALIBRATING, READY, BALANCING, FAULT = "IDLE", "CALIBRATING", "READY", "BALANCING", "FAULT"
PAGES = ("STATUS", "PD", "CBF-QP", "SENSOR & MOTOR", "ISE", "BANTUAN")
STATE_NAMES = ("theta", "dtheta", "psi", "dpsi")
STATE_UNITS = ("rad", "rad/s", "rad", "rad/s")


def gauge(value: float, limit: float, width: int = 41) -> str:
    """Batang teks: posisi value di antara -limit..+limit (batas ditandai '|')."""
    span = 1.25 * limit
    cells = [" "] * width
    mid = width // 2
    edge = int(round(mid * limit / span))
    cells[mid] = ":"
    cells[mid - edge] = cells[mid + edge] = "|"
    pos = mid + int(round(mid * max(-span, min(span, value)) / span))
    cells[max(0, min(width - 1, pos))] = "#"
    return "[" + "".join(cells) + "]"


class Atera:
    def __init__(self) -> None:
        self.motors = DualDDSM115()
        self.imu = MPU6050()
        self.pd = PDController()
        self.cbf = CBFQP()
        self.joystick = jm.Joystick()
        self.recorder = ise.ISERecorder()

        self.state = IDLE
        self.control_mode = ise.MODE_PD
        self.drive_enabled = False          # Tombol A = True, Tombol B = False
        self.calibrated = False
        self.calibration = {}
        self.exit_requested = False
        self.page = 0
        self.message = "Tekan Tombol Y untuk kalibrasi, lalu MULAI."
        self.fault_reason = ""

        self.x = np.zeros(4)
        self.x_ref = np.zeros(4)
        self.theta_offset = 0.0
        self.sample = None                  # ImuSample terakhir
        self.u_pd = 0.0
        self.u = 0.0
        self.pd_terms = (0.0,) * 5
        self.drive = 0.0
        self.turn = 0.0
        self.balance_t0 = 0.0
        self.cbf_active_count = 0
        self.cbf_infeasible_count = 0
        self.cbf_step_count = 0
        self.saturated_s = 0.0
        self.infeasible_s = 0.0

        self.loop_dt = 1.0 / config.CONTROL_HZ
        self.loop_hz = 0.0
        self.work_ms = 0.0
        self.overruns = 0

    # ------------------------------------------------------------- setup --
    def setup(self) -> None:
        self.imu.open()
        self.motors.initialize()
        self.joystick.open()
        self.set_message("Menyiapkan CBF-QP ...")
        self.render()
        self.cbf.setup()
        self.set_message("Hardware siap. Tekan Tombol Y untuk kalibrasi.")

    def cleanup(self) -> None:
        self.motors.stop_all()
        self.recorder.stop("program keluar")
        self.motors.close()
        self.joystick.close()
        try:
            self.imu.close()
        except Exception as exc:
            LOGGER.error("Gagal menutup IMU: %s", exc)
        self.recorder.close()

    def set_message(self, text: str) -> None:
        self.message = text
        LOGGER.info(text)

    # ------------------------------------------------------ state machine --
    def calibrate(self) -> None:
        if self.state == BALANCING:
            self.set_message("Kalibrasi ditolak: matikan roda dulu (Tombol X).")
            return
        self.motors.stop_all()
        self.state = CALIBRATING
        self.fault_reason = ""
        self.set_message("Kalibrasi IMU ... jaga robot DIAM dan TEGAK di titik seimbang.")
        self.render()
        try:
            self.calibration = self.imu.calibrate()
        except Exception as exc:
            self.set_fault(f"Kalibrasi gagal: {exc}")
            return
        self.calibrated = True
        self.state = READY
        self.set_message(f"Kalibrasi selesai (zero offset {self.calibration['zero_offset_deg']:+.2f} deg). "
                         "Tekan MULAI.")

    def start_balancing(self) -> None:
        if self.state == BALANCING:
            return
        if not self.calibrated:
            self.set_message("Belum dikalibrasi. Tekan Tombol Y dulu.")
            return
        if self.sample is None or abs(self.sample.psi_deg) > config.START_TILT_DEG:
            self.set_message(f"MULAI ditolak: tegakkan robot dulu (|psi| harus < {config.START_TILT_DEG:g} deg).")
            return
        self.pd.reload()
        self.motors.reset_wheel_angles()
        self.theta_offset = self.sample.psi if config.ENCODER_RELATIVE_TO_BODY else 0.0
        self.x_ref = np.zeros(4)
        self.fault_reason = ""
        self.cbf_active_count = self.cbf_infeasible_count = self.cbf_step_count = 0
        self.saturated_s = self.infeasible_s = 0.0
        self.balance_t0 = time.monotonic()
        self.state = BALANCING
        self.recorder.start(self.control_mode)
        self.set_message(f"Balancing aktif, mode {self.control_mode}. Percobaan: {self.recorder.name}")

    def stop_balancing(self, reason: str) -> None:
        self.motors.stop_all()
        self.u = self.u_pd = 0.0
        saved = self.recorder.stop(reason)
        self.state = READY if self.calibrated else IDLE
        self.fault_reason = ""
        self.set_message(reason + (f" Tersimpan: {saved}" if saved else ""))

    def set_fault(self, reason: str) -> None:
        self.motors.stop_all()
        self.u = self.u_pd = 0.0
        saved = self.recorder.stop("FAULT: " + reason)
        self.state = FAULT
        self.fault_reason = reason
        self.set_message("FAULT: " + reason + (f" | tersimpan: {saved}" if saved else ""))

    def set_control_mode(self, mode: str) -> None:
        if mode == ise.MODE_PD_CBF and not self.cbf.available:
            self.set_message("Mode PD+CBF ditolak: " + self.cbf.reason)
            return
        if mode == self.control_mode:
            return
        self.control_mode = mode
        if self.state == BALANCING:
            # satu percobaan = satu mode: tutup rekaman lama, mulai yang baru
            saved = self.recorder.stop("ganti mode")
            self.recorder.start(mode)
            self.set_message(f"Mode {mode}. Percobaan baru: {self.recorder.name}"
                             + (f" (tersimpan: {saved})" if saved else ""))
        else:
            self.set_message(f"Mode kontrol: {mode}")

    def handle_action(self, action: str) -> None:
        if action == jm.ACTION_QUIT:
            self.exit_requested = True
        elif action == jm.ACTION_CALIBRATE:
            self.calibrate()
        elif action == jm.ACTION_START:
            self.start_balancing()
        elif action == jm.ACTION_MOTOR_OFF:
            self.stop_balancing("Roda dimatikan.")
        elif action == jm.ACTION_MODE_PD:
            self.set_control_mode(ise.MODE_PD)
        elif action == jm.ACTION_MODE_PD_CBF:
            self.set_control_mode(ise.MODE_PD_CBF)
        elif action == jm.ACTION_DRIVE_ON:
            self.drive_enabled = True
            self.set_message("Perintah gerak AKTIF (analog + D-Pad).")
        elif action == jm.ACTION_BALANCE_ONLY:
            self.drive_enabled = False
            self.set_message("Mode balancing saja (perintah gerak diabaikan).")
        elif action == jm.ACTION_PAGE_PREV:
            self.page = (self.page - 1) % len(PAGES)
        elif action == jm.ACTION_PAGE_NEXT:
            self.page = (self.page + 1) % len(PAGES)

    # ------------------------------------------------------- loop kontrol --
    def read_state(self) -> None:
        self.sample = self.imu.read()
        psi, dpsi = self.sample.psi, self.sample.dpsi
        theta, dtheta = self.motors.theta, self.motors.dtheta
        if config.ENCODER_RELATIVE_TO_BODY:
            # encoder mengukur roda relatif terhadap badan; model memakai sudut roda absolut
            theta += psi
            dtheta += dpsi
        self.x = np.array([theta - self.theta_offset, dtheta, psi, dpsi])

    def update_reference(self, dt: float) -> None:
        drive, turn = self.joystick.command() if self.drive_enabled else (0.0, 0.0)
        self.drive, self.turn = drive, turn
        dtheta_ref = drive * float(config.JOY_DTHETA_REF)
        theta_ref = self.x_ref[0] + dtheta_ref * dt
        lead = float(config.JOY_THETA_LEAD_MAX)
        theta_ref = max(self.x[0] - lead, min(self.x[0] + lead, theta_ref))
        self.x_ref = np.array([theta_ref, dtheta_ref, drive * math.radians(config.JOY_PSI_REF_DEG), 0.0])

    def control_step(self, dt: float) -> None:
        self.read_state()
        if self.state != BALANCING:
            self.drive = self.turn = 0.0
            if self.state != CALIBRATING:
                self.motors.command_torque(0.0)      # arus nol, tetap membaca encoder
            return

        if abs(self.x[2]) > math.radians(config.SAFE_TILT_DEG):
            self.set_fault(f"|psi| = {abs(self.x[2]) * DEG:.1f} deg melebihi SAFE_TILT_DEG")
            return

        self.update_reference(dt)
        x_err = self.x - self.x_ref
        self.pd_terms = self.pd.terms(x_err)
        self.u_pd = self.pd.compute(x_err)

        if self.control_mode == ise.MODE_PD_CBF:
            u, feasible = self.cbf.filter(self.x, self.u_pd)
            active = self.cbf.active
            self.cbf_step_count += 1
            self.cbf_active_count += int(active)
            self.cbf_infeasible_count += int(not feasible)
        else:
            u = max(-config.u_max, min(config.u_max, self.u_pd))
            feasible, active = True, False
            self.cbf.h = barrier_values(self.x)

        turn = self.turn * float(config.TURN_TORQUE) * float(config.TURN_SIGN)
        self.u = self.motors.command_torque(u, turn)
        self.recorder.add(self.x, self.x_ref, self.u_pd, self.u, active, feasible, self.cbf.h)

        # Pengaman robot fisik: torsi jenuh atau QP infeasible terlalu lama = robot tidak
        # tertolong / melaju tak terkendali -> matikan roda.
        self.saturated_s = self.saturated_s + dt if abs(u) >= 0.999 * config.u_max else 0.0
        self.infeasible_s = self.infeasible_s + dt if not feasible else 0.0
        if self.saturated_s > float(config.SATURATION_FAULT_S):
            self.set_fault(f"torsi jenuh lebih dari {config.SATURATION_FAULT_S:g} s")
        elif self.infeasible_s > float(config.CBF_INFEASIBLE_FAULT_S):
            self.set_fault(f"CBF-QP infeasible lebih dari {config.CBF_INFEASIBLE_FAULT_S:g} s "
                           "(batas psi dan dtheta tidak bisa dipenuhi bersamaan)")

    def run(self) -> None:
        period = 1.0 / float(config.CONTROL_HZ)
        ui_period = 1.0 / float(config.UI_HZ)
        next_tick = time.monotonic()
        last_start = next_tick - period
        last_ui = 0.0
        while not self.exit_requested:
            start = time.monotonic()
            self.loop_dt = max(1e-4, start - last_start)
            last_start = start
            self.loop_hz += 0.05 * (1.0 / self.loop_dt - self.loop_hz)

            for action in self.joystick.poll():
                self.handle_action(action)
            try:
                self.control_step(self.loop_dt)
            except (DDSM115Error, OSError, RuntimeError, ValueError) as exc:
                if self.state != FAULT:
                    self.set_fault(str(exc))

            now = time.monotonic()
            self.work_ms += 0.05 * ((now - start) * 1000.0 - self.work_ms)
            if now - last_ui >= ui_period:
                self.render()
                last_ui = now

            next_tick += period
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                if self.state == BALANCING:
                    self.overruns += 1
                next_tick = time.monotonic()

    # ---------------------------------------------------------------- GUI --
    def render(self) -> None:
        width = max(60, min(100, shutil.get_terminal_size((90, 30)).columns - 1))
        lines = []
        tabs = "  ".join(f"[{i + 1}:{name}]" if i == self.page else f" {i + 1}:{name} "
                         for i, name in enumerate(PAGES))
        joy = self.joystick.name if self.joystick.connected else "TIDAK TERHUBUNG"
        lines.append("ATERA - TWSBR PD + CBF-QP")
        lines.append("=" * width)
        lines.append(f"State: {self.state:<11} | Mode kontrol: {self.control_mode:<6} | "
                     f"Gerak: {'AKTIF (A)' if self.drive_enabled else 'balancing saja (B)'}")
        lines.append(f"Loop : {self.loop_hz:6.1f} Hz (target {config.CONTROL_HZ:g}) | kerja {self.work_ms:4.1f} ms | "
                     f"overrun {self.overruns} | Joystick: {joy}")
        lines.append(f"Pesan: {self.message}")
        if self.fault_reason:
            lines.append(f"FAULT: {self.fault_reason}")
        lines.append("-" * width)
        lines.append(tabs)
        lines.append("-" * width)
        lines.extend((self.page_status, self.page_pd, self.page_cbf,
                      self.page_hardware, self.page_ise, self.page_help)[self.page]())
        lines.append("-" * width)
        lines.append("Y kalibrasi | MULAI | X roda mati | LB PD | RB PD+CBF | A gerak | B diam | LT/RT halaman | QUIT")
        out = "\x1b[H" + "".join(line[:width + 20] + "\x1b[K\n" for line in lines) + "\x1b[J"
        sys.stdout.write(out)
        sys.stdout.flush()

    def page_status(self) -> list:
        x, r = self.x, self.x_ref
        lines = ["state x           terukur                    referensi          error (ref - x)"]
        for i in range(4):
            is_angle = STATE_UNITS[i] == "rad"
            deg_txt = f"({x[i] * DEG:+8.2f} deg{'  ' if is_angle else '/s'})"
            lines.append(f"  {STATE_NAMES[i]:<7} {x[i]:+9.4f} {STATE_UNITS[i]:<5} {deg_txt}   "
                         f"{r[i]:+9.4f}          {r[i] - x[i]:+9.4f}")
        lines.append("")
        lines.append(f"  psi    {gauge(x[2], config.psi_max)}  batas +-{config.psi_max_DEG:g} deg")
        lines.append(f"  dtheta {gauge(x[1], config.dtheta_max)}  batas +-{config.dtheta_max:g} rad/s")
        lines.append(f"  u      {gauge(self.u, config.u_max)}  batas +-{config.u_max:.2f} N m")
        lines.append("")
        lines.append(f"  u_PD = {self.u_pd:+8.4f} N m    u terkirim = {self.u:+8.4f} N m    "
                     f"selisih = {self.u - self.u_pd:+8.4f} N m")
        lines.append(f"  perintah gerak: maju/mundur {self.drive:+.2f} | belok {self.turn:+.2f}")
        if self.state == BALANCING:
            lines.append(f"  lama balancing: {time.monotonic() - self.balance_t0:7.1f} s | "
                         f"ISE psi berjalan: {self.recorder.ise_psi_deg2s:.4f} deg^2 s")
        return lines

    def page_pd(self) -> list:
        pd = self.pd
        e = self.x - self.x_ref
        names = ("Kp_theta  * theta ", "Kd_dtheta * dtheta", "Kp_psi    * psi   ", "Kd_dpsi   * dpsi  ")
        gains = (pd.Kp_theta, pd.Kd_dtheta, pd.Kp_psi, pd.Kd_dpsi)
        lines = ["u_PD = Kp_theta*theta + Kd_dtheta*dtheta + Kp_psi*psi + Kd_dpsi*dpsi + Kvel",
                 "(state diukur relatif terhadap referensi; gain dibaca dari config.py saat MULAI)", ""]
        for i in range(4):
            lines.append(f"  {names[i]} = {gains[i]:9.4f} x {e[i]:+9.4f} = {self.pd_terms[i]:+9.4f} N m")
        lines.append(f"  Kvel                                          = {self.pd_terms[4]:+9.4f} N m")
        lines.append(f"  {'-' * 58}")
        lines.append(f"  u_PD                                          = {self.u_pd:+9.4f} N m")
        sat = "YA" if abs(self.u_pd) > config.u_max else "tidak"
        lines.append(f"  batas aktuator u_max = {config.u_max:.3f} N m ({config.MAX_CURRENT_A:g} A per roda) | jenuh: {sat}")
        lines.append("")
        if not any(gains):
            lines.append("  SEMUA GAIN MASIH 0: robot tidak akan menahan diri. Isi gain di config.py.")
        A, B = model.linearize()
        poles = np.linalg.eigvals(A + np.outer(B, np.array(gains)))
        stable = "stabil" if max(poles.real) < -1e-6 else "TIDAK stabil / marginal"
        lines.append("  pole closed-loop model linear: " + "  ".join(
            f"{p.real:+.1f}{p.imag:+.1f}j" if abs(p.imag) > 1e-6 else f"{p.real:+.1f}" for p in poles))
        lines.append(f"  -> menurut model: {stable}")
        return lines

    def page_cbf(self) -> list:
        cbf = self.cbf
        on = self.control_mode == ise.MODE_PD_CBF
        lines = [f"Filter: {'AKTIF (mode PD+CBF)' if on else 'tidak dipakai (mode PD)'} | "
                 f"solver: {cbf.backend or '-'} | N = {config.CBF_N_HORIZON} | {cbf.reason}",
                 f"Alpha_1 = {config.Alpha_1:g}   Alpha_2 = {config.Alpha_2:g}   "
                 f"psi_max = {config.psi_max_DEG:g} deg   dtheta_max = {config.dtheta_max:g} rad/s", ""]
        units = ("rad", "rad", "rad/s", "rad/s")
        for i in range(4):
            flag = "aman" if cbf.h[i] >= 0.0 else "MELANGGAR"
            lines.append(f"  {H_NAMES[i]:<26} = {cbf.h[i]:+9.4f} {units[i]:<5}  {flag}")
        lines.append("")
        if on:
            lo, hi = cbf.u_bound
            lines.append(f"  selang u yang diizinkan CBF : [{lo:+8.3f}, {hi:+8.3f}] N m")
            lines.append(f"  u_PD = {self.u_pd:+8.4f}  ->  u_QP = {self.u:+8.4f} N m   "
                         f"(koreksi {self.u - max(-config.u_max, min(config.u_max, self.u_pd)):+.4f})")
            lines.append(f"  filter mengoreksi: {'YA' if cbf.active else 'tidak'} | "
                         f"feasible: {'ya' if cbf.feasible else 'TIDAK'} | waktu QP: {cbf.solve_ms:.3f} ms")
            n = max(1, self.cbf_step_count)
            lines.append(f"  sejak MULAI: koreksi {self.cbf_active_count} siklus ({100.0 * self.cbf_active_count / n:.1f}%) | "
                         f"infeasible {self.cbf_infeasible_count} | solver gagal {cbf.fail_count}")
        else:
            lines.append("  Tekan RB untuk mengaktifkan filter CBF-QP.")
        return lines

    def page_hardware(self) -> list:
        lines = ["IMU MPU6050"]
        s = self.sample
        if s is None:
            lines.append("  belum ada data")
        else:
            lines.append(f"  psi = {s.psi_deg:+8.3f} deg | dpsi = {s.dpsi_deg:+8.2f} deg/s | "
                         f"sudut accel = {s.acc_angle_deg:+8.3f} deg | dt = {s.dt * 1000.0:5.2f} ms")
            lines.append(f"  kalman X/Y = {s.kalman_x_deg:+8.3f} / {s.kalman_y_deg:+8.3f} deg | "
                         f"gyro X/Y = {s.gyro_x_deg_s:+7.2f} / {s.gyro_y_deg_s:+7.2f} deg/s")
        c = self.calibration
        if c:
            lines.append(f"  kalibrasi: zero offset {c['zero_offset_deg']:+.3f} deg | bias gyro X/Y "
                         f"{c['gyro_bias_x']:+.3f} / {c['gyro_bias_y']:+.3f} deg/s")
        else:
            lines.append("  kalibrasi: BELUM")
        lines.append("")
        lines.append("MOTOR DDSM115      perintah    arus fb    rpm fb    posisi    theta      dtheta    error")
        for motor in self.motors.motors:
            fb = motor.feedback
            if fb is None:
                lines.append(f"  {motor.name:<12} belum ada umpan balik ({motor.last_error})")
                continue
            lines.append(f"  {motor.name:<12} {motor.command_a:+7.3f} A  {fb.current_a:+7.3f} A  {fb.speed_rpm:+6.0f}   "
                         f"{fb.position_raw:6d}  {motor.theta:+8.3f}  {motor.dtheta:+8.3f}   {decode_error(fb.error_code)}")
        lines.append(f"  gagal komunikasi (total): kiri {self.motors.left.comm_errors_total} | "
                     f"kanan {self.motors.right.comm_errors_total}")
        lines.append("")
        lines.append("Cek arah sebelum tuning (roda mati, robot dipegang):")
        lines.append("  miringkan badan ke DEPAN  -> psi harus POSITIF        (salah: balik IMU_SIGN)")
        lines.append("  putar roda ke arah DEPAN  -> theta harus NAIK         (salah: balik ENCODER_DIRECTION)")
        return lines

    def page_ise(self) -> list:
        rec = self.recorder
        lines = []
        if rec.recording:
            lines.append(f"Merekam: {rec.name}")
            lines.append(f"  durasi {rec.duration:7.1f} s | {len(rec.rows)} sampel")
            lines.append(f"  ISE psi    = {rec.ise_psi_deg2s:12.4f} deg^2 s      ({rec.ise[2]:.6f} rad^2 s)")
            lines.append(f"  ISE dpsi   = {rec.ise_dpsi_deg2s:12.2f} (deg/s)^2 s  ({rec.ise[3]:.6f} (rad/s)^2 s)")
            lines.append(f"  ISE theta  = {rec.ise[0]:12.4f} rad^2 s")
            lines.append(f"  ISE dtheta = {rec.ise[1]:12.4f} (rad/s)^2 s")
        else:
            lines.append("Tidak sedang merekam. Rekaman dimulai saat MULAI dan disimpan saat roda dimatikan.")
        lines.append("")
        lines.append(f"Terakhir: {rec.last_summary or '-'}")
        lines.append(f"Folder  : {ise.plot_dir()}")
        try:
            files = sorted(f for f in os.listdir(ise.plot_dir()) if f.endswith(".png"))[-6:]
        except OSError:
            files = []
        lines.extend("  " + name for name in files)
        return lines

    def page_help(self) -> list:
        lines = ["Joystick:"] + ["  " + line for line in jm.HELP_LINES]
        if not self.joystick.connected:
            lines.append("")
            lines.append(f"Joystick tidak ditemukan di {config.JOYSTICK_PORT}. Perangkat input yang ada:")
            lines.extend("  " + name for name in self.joystick.available_devices()[:6])
        return lines


def _on_sigterm(*_args) -> None:
    raise KeyboardInterrupt


def main() -> int:
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config.LOG_FILE)
    logging.basicConfig(filename=log_path, level=logging.INFO,
                        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")
    signal.signal(signal.SIGTERM, _on_sigterm)
    app = Atera()
    sys.stdout.write("\x1b[?1049h\x1b[?25l\x1b[2J")      # layar alternatif, kursor disembunyikan
    code, error = 0, ""
    try:
        app.setup()
        app.run()
    except KeyboardInterrupt:
        LOGGER.info("Dihentikan dari keyboard")
    except Exception as exc:
        LOGGER.exception("Error fatal")
        code, error = 1, f"{type(exc).__name__}: {exc}"
    finally:
        try:
            app.cleanup()
        finally:
            sys.stdout.write("\x1b[?25h\x1b[?1049l")
            sys.stdout.flush()
    if error:
        print(f"ATERA berhenti karena error: {error}\nDetail: {log_path}")
    elif app.recorder.last_summary:
        print("Percobaan terakhir: " + app.recorder.last_summary)
    return code


if __name__ == "__main__":
    raise SystemExit(main())