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
                FAULT bila |psi| > SAFE_TILT_DEG atau hardware gagal saat balancing (motor dimatikan).

Diagnosa:
- Kegagalan hardware (IMU, motor, joystick) TIDAK menghentikan program. GUI tetap jalan,
  masalahnya ditampilkan beserta saran perbaikan, dan hardware dicoba ulang otomatis.
- Baris "Roda" di kepala GUI selalu menjelaskan kenapa roda bergerak atau diam.
- Halaman DIAGNOSA: status tiap komponen, rantai sinyal dari sensor sampai motor, dan
  hasil uji roda langsung (L3 = kiri, R3 = kanan; robot diangkat, roda mati).

GUI: 7 halaman, pindah dengan LT / RT. Log ditulis ke config.LOG_FILE (bukan ke layar).
"""

from __future__ import annotations

import logging
import math
import os
import shutil
import signal
import sys
import textwrap
import time

import numpy as np

import config
import ise
import joystick_mapping as jm
import model
from cbf_qp import CBFQP, H_NAMES, barrier_values
from ddsm115 import MODE_NAMES, DualDDSM115, decode_error
from mpu6050 import MPU6050
from pd_control import PDController

LOGGER = logging.getLogger("atera")
DEG = math.degrees(1.0)

IDLE, CALIBRATING, READY, BALANCING, FAULT = "IDLE", "CALIBRATING", "READY", "BALANCING", "FAULT"
PAGES = ("STATUS", "DIAGNOSA", "PD", "CBF-QP", "SENSOR & MOTOR", "ISE", "BANTUAN")
STATE_NAMES = ("theta", "dtheta", "psi", "dpsi")
STATE_UNITS = ("rad", "rad/s", "rad", "rad/s")
ERROR, WARN, INFO = "ERROR", "AWAS", "INFO"


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
        self.message = "Menyiapkan hardware ..."
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

        # diagnosa
        self.next_imu_retry = 0.0
        self.next_motor_retry = 0.0
        self.stall_s = [0.0, 0.0]           # roda diperintah tetapi diam [kiri, kanan]
        self.no_current_s = [0.0, 0.0]      # diperintah tetapi arus umpan balik ~0
        self.test = None                    # uji roda yang sedang berjalan
        self.test_result = ["belum diuji (tekan L3)", "belum diuji (tekan R3)"]
        self.render_errors = 0

        self.loop_dt = 1.0 / config.CONTROL_HZ
        self.loop_hz = float(config.CONTROL_HZ)
        self.work_ms = 0.0
        self.overruns = 0

    # ------------------------------------------------------------- setup --
    def setup(self) -> None:
        """Siapkan semua komponen. Kegagalan dicatat untuk GUI, tidak menghentikan program."""
        self.render()
        self.imu.open()
        self.motors.initialize()
        self.joystick.open()
        self.set_message("Menyiapkan CBF-QP ...")
        self.render()
        self.cbf.setup()
        errors = [d for d in self.diagnose() if d[0] == ERROR]
        if errors:
            self.set_message(f"Ada {len(errors)} masalah yang harus dibereskan - lihat baris Masalah / halaman DIAGNOSA.")
        else:
            self.set_message("Hardware siap. Tekan Tombol Y untuk kalibrasi.")
        for level, source, text in self.diagnose():
            LOGGER.info("diagnosa awal [%s] %s: %s", level, source, text)

    def cleanup(self) -> None:
        self.motors.stop_all()
        self.recorder.stop("program keluar")
        self.motors.close()
        self.joystick.close()
        self.imu.close()
        self.recorder.close()

    def set_message(self, text: str) -> None:
        self.message = text
        LOGGER.info(text)

    def maintain_hardware(self) -> None:
        """Coba ulang hardware yang bermasalah (hanya saat tidak balancing)."""
        now = time.monotonic()
        if not self.imu.ready and now >= self.next_imu_retry:
            self.next_imu_retry = now + float(config.IMU_RETRY_S)
            if self.imu.open():
                self.set_message("IMU terhubung kembali.")
        if not self.motors.ready and now >= self.next_motor_retry:
            self.next_motor_retry = now + float(config.MOTOR_RETRY_S)
            if self.motors.initialize():
                self.set_message("Kedua motor siap.")

    # ------------------------------------------------------ state machine --
    def calibrate(self) -> None:
        if self.state == BALANCING:
            self.set_message("Kalibrasi ditolak: matikan roda dulu (Tombol X).")
            return
        imu_errors = [text for level, text in self.imu.problems() if level == ERROR]
        if imu_errors:
            self.set_message("Kalibrasi ditolak, IMU bermasalah: " + imu_errors[0])
            return
        self.test = None
        self.motors.stop_all()
        self.state = CALIBRATING
        self.fault_reason = ""
        self.set_message("Kalibrasi IMU ... jaga robot DIAM dan TEGAK di titik seimbang.")
        self.render()
        try:
            self.calibration = self.imu.calibrate()
        except Exception as exc:
            self.calibrated = False
            self.state = IDLE
            self.set_message(f"Kalibrasi GAGAL: {exc}")
            return
        self.calibrated = True
        self.state = READY
        text = f"Kalibrasi selesai (zero offset {self.calibration['zero_offset_deg']:+.2f} deg). Tekan MULAI."
        if self.calibration["gyro_std"] > float(config.IMU_CAL_MAX_GYRO_STD):
            text = (f"Kalibrasi selesai TETAPI robot bergerak saat kalibrasi (simpangan gyro "
                    f"{self.calibration['gyro_std']:.1f} deg/s). Ulangi Tombol Y dengan robot diam.")
        self.set_message(text)

    def start_balancing(self) -> None:
        if self.state == BALANCING:
            return
        reason = ""
        if not self.imu.ready:
            reason = "IMU bermasalah: " + self.imu.problem
        elif any(level == ERROR for level, _ in self.imu.problems()):
            reason = "IMU bermasalah: " + self.imu.problems()[0][1]
        elif not self.motors.ready:
            reason = "; ".join(self.motors.problems())
        elif not self.calibrated:
            reason = "belum dikalibrasi, tekan Tombol Y dulu"
        elif self.sample is None or abs(self.sample.psi_deg) > config.START_TILT_DEG:
            psi = self.sample.psi_deg if self.sample else float("nan")
            reason = (f"psi = {psi:+.1f} deg, tegakkan robot dulu "
                      f"(|psi| harus < {config.START_TILT_DEG:g} deg)")
        if reason:
            self.set_message("MULAI ditolak: " + reason)
            return
        self.test = None
        self.pd.reload()
        self.motors.reset_wheel_angles()
        self.theta_offset = self.sample.psi if config.ENCODER_RELATIVE_TO_BODY else 0.0
        self.x_ref = np.zeros(4)
        self.fault_reason = ""
        self.cbf_active_count = self.cbf_infeasible_count = self.cbf_step_count = 0
        self.saturated_s = self.infeasible_s = 0.0
        self.stall_s = [0.0, 0.0]
        self.no_current_s = [0.0, 0.0]
        self.balance_t0 = time.monotonic()
        self.state = BALANCING
        self.recorder.start(self.control_mode)
        text = f"Balancing aktif, mode {self.control_mode}. Percobaan: {self.recorder.name}"
        if not any(self.gains()):
            text = "Balancing aktif TETAPI semua gain 0 -> u = 0, roda tidak akan bergerak. Isi gain di config.py."
        self.set_message(text)

    def stop_balancing(self, reason: str) -> None:
        self.test = None
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
        elif action == jm.ACTION_TEST_LEFT:
            self.start_wheel_test(0)
        elif action == jm.ACTION_TEST_RIGHT:
            self.start_wheel_test(1)

    # ----------------------------------------------------------- uji roda --
    def start_wheel_test(self, index: int) -> None:
        """Putar satu roda dengan arus kecil, tanpa lewat kontrol, lalu nilai hasilnya."""
        motor = self.motors.motors[index]
        if self.state in (BALANCING, CALIBRATING):
            self.set_message("Uji roda ditolak: hanya bisa saat roda mati (Tombol X).")
            return
        if not motor.ready:
            self.test_result[index] = "GAGAL - " + motor.problem
            self.set_message(f"Uji {motor.name} ditolak: {motor.problem}")
            return
        self.test = {"index": index, "t_end": time.monotonic() + float(config.MOTOR_TEST_DURATION_S),
                     "theta0": motor.theta, "ok0": motor.count_ok, "fb_sum": 0.0, "n": 0, "rpm_max": 0.0}
        self.set_message(f"Uji {motor.name}: {config.MOTOR_TEST_CURRENT_A:g} A selama "
                         f"{config.MOTOR_TEST_DURATION_S:g} s. Robot harus DIANGKAT.")

    def run_wheel_test(self) -> None:
        test = self.test
        index = test["index"]
        motor = self.motors.motors[index]
        current = float(config.MOTOR_TEST_CURRENT_A)
        self.motors.command_currents(current if index == 0 else 0.0, current if index == 1 else 0.0)
        if motor.feedback is not None:
            test["fb_sum"] += motor.feedback.current_a
            test["n"] += 1
            test["rpm_max"] = max(test["rpm_max"], abs(motor.feedback.speed_rpm))
        if time.monotonic() < test["t_end"] and motor.ready:
            return
        self.motors.stop_all()
        self.test = None
        moved = motor.theta - test["theta0"]
        replies = motor.count_ok - test["ok0"]
        fb_mean = test["fb_sum"] / max(1, test["n"])
        detail = (f"theta berubah {moved:+.2f} rad, arus umpan balik rata-rata {fb_mean:+.3f} A, "
                  f"rpm maks {test['rpm_max']:.0f}, {replies} balasan")
        min_angle = float(config.MOTOR_TEST_MIN_ANGLE)
        if not motor.ready or replies == 0:
            verdict = "GAGAL - motor tidak membalas: " + (motor.problem or motor.last_error)
        elif moved > min_angle:
            verdict = "OK - roda berputar dan theta naik"
        elif moved < -min_angle:
            verdict = ("TERBALIK - theta turun. Bila roda fisik berputar ke DEPAN: balik ENCODER_DIRECTION. "
                       "Bila roda fisik berputar ke BELAKANG: balik tanda MOTOR_SIGN roda ini")
        elif abs(fb_mean) < 0.1 * current:
            verdict = ("GAGAL - roda diam dan arus umpan balik ~0: motor menerima perintah tetapi tidak "
                       "mengalirkan arus (catu daya motor lemah/mati, atau bukan mode current loop)")
        else:
            verdict = ("GAGAL - arus mengalir tetapi roda diam: roda tertahan/terkunci, "
                       "atau MOTOR_TEST_CURRENT_A terlalu kecil")
        self.test_result[index] = f"{verdict} ({detail})"
        self.set_message(f"Uji {motor.name}: {verdict}")

    # ------------------------------------------------------- loop kontrol --
    def read_imu(self) -> bool:
        if not self.imu.ready:
            return False
        try:
            self.sample = self.imu.read()
        except OSError as exc:
            LOGGER.error("%s", exc)
            return False
        return True

    def update_state_vector(self) -> None:
        psi = self.sample.psi if self.sample else 0.0
        dpsi = self.sample.dpsi if self.sample else 0.0
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

    def watch_motors(self, dt: float) -> None:
        """Deteksi roda yang diperintah tetapi tidak berputar / tidak mengalirkan arus."""
        for i, motor in enumerate(self.motors.motors):
            commanded = abs(motor.command_a) >= float(config.STALL_CURRENT_A)
            still = abs(motor.dtheta) < float(config.STALL_MIN_SPEED)
            self.stall_s[i] = self.stall_s[i] + dt if commanded and still else 0.0
            fb = motor.feedback
            dead = fb is not None and abs(fb.current_a) < 0.1 * abs(motor.command_a)
            self.no_current_s[i] = self.no_current_s[i] + dt if commanded and dead else 0.0

    def control_step(self, dt: float) -> None:
        imu_ok = self.read_imu()
        if self.state != BALANCING:
            self.drive = self.turn = 0.0
            if self.state != CALIBRATING:
                self.maintain_hardware()
                if self.test is not None:
                    self.run_wheel_test()
                else:
                    self.motors.command_currents(0.0, 0.0)   # arus nol, tetap membaca encoder
            self.update_state_vector()
            return

        if not imu_ok:
            self.set_fault("IMU gagal dibaca saat balancing: " + self.imu.problem)
            return
        self.update_state_vector()
        if abs(self.x[2]) > math.radians(config.SAFE_TILT_DEG):
            self.set_fault(f"|psi| = {abs(self.x[2]) * DEG:.1f} deg melebihi SAFE_TILT_DEG "
                           f"({config.SAFE_TILT_DEG:g} deg): robot jatuh")
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
        self.u = self.motors.command_torque(u, turn, strict=True)
        self.recorder.add(self.x, self.x_ref, self.u_pd, self.u, active, feasible, self.cbf.h)
        self.watch_motors(dt)

        # Pengaman robot fisik: torsi jenuh atau QP infeasible terlalu lama = robot tidak
        # tertolong / melaju tak terkendali -> matikan roda.
        self.saturated_s = self.saturated_s + dt if abs(u) >= 0.999 * config.u_max else 0.0
        self.infeasible_s = self.infeasible_s + dt if not feasible else 0.0
        if self.saturated_s > float(config.SATURATION_FAULT_S):
            self.set_fault(f"torsi jenuh (|u| = u_max) lebih dari {config.SATURATION_FAULT_S:g} s: "
                           "robot tidak tertolong, atau gain terlalu besar")
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
            except Exception as exc:
                LOGGER.exception("control_step")
                if self.state == BALANCING:
                    self.set_fault(f"{exc}")
                elif self.state != FAULT:
                    self.set_message(f"Error: {type(exc).__name__}: {exc}")

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

    # ----------------------------------------------------------- diagnosa --
    def gains(self) -> tuple:
        return (self.pd.Kp_theta, self.pd.Kd_dtheta, self.pd.Kp_psi, self.pd.Kd_dpsi)

    def current_at_5deg(self) -> float:
        """Arus per roda [A] yang dihasilkan Kp_psi pada psi = 5 deg."""
        return abs(self.pd.Kp_psi) * math.radians(5.0) / 2.0 / config.MOTOR_KT

    def diagnose(self) -> list:
        """Daftar (tingkat, sumber, pesan) dari semua masalah yang terdeteksi saat ini."""
        found = []
        for level, text in self.imu.problems():
            found.append((level, "IMU", text))
        for i, motor in enumerate(self.motors.motors):
            name = motor.name.capitalize()
            if not motor.ready:
                found.append((ERROR, name, motor.problem))
                continue
            fb = motor.feedback
            if fb is not None and fb.error_code:
                found.append((WARN, name, f"motor melaporkan error: {decode_error(fb.error_code)} "
                                          f"(kode 0x{fb.error_code:02X}) -> matikan-nyalakan catu daya motor"))
            if fb is not None and fb.mode != 1:
                found.append((ERROR, name, f"mode motor {fb.mode} ({MODE_NAMES.get(fb.mode, '?')}), bukan current loop"))
            if motor.comm_errors > 0:
                found.append((WARN, name, f"{motor.comm_errors} balasan terakhir gagal: {motor.last_error}"))
            if self.no_current_s[i] > float(config.STALL_TIME_S):
                found.append((WARN, name, f"diperintah {motor.command_a:+.2f} A tetapi arus umpan balik "
                                          f"{fb.current_a:+.2f} A -> catu daya motor lemah/mati"))
            elif self.stall_s[i] > float(config.STALL_TIME_S):
                found.append((WARN, name, f"diperintah {motor.command_a:+.2f} A tetapi roda tidak berputar "
                                          f"-> roda tertahan, atau robot sedang dipegang"))
        if not self.joystick.connected:
            found.append((WARN, "Joystick", self.joystick.problem))
        if not any(self.gains()) and self.pd.Kvel == 0.0:
            found.append((ERROR, "Kontrol", "semua gain PD = 0 di config.py -> u_PD selalu 0, RODA TIDAK AKAN "
                                            "BERGERAK. Isi Kp_psi dan Kd_dpsi (titik awal: 30.9 dan 2.17)"))
        elif self.current_at_5deg() < float(config.DIAG_MIN_CURRENT_AT_5DEG_A):
            found.append((WARN, "Kontrol", f"Kp_psi = {self.pd.Kp_psi:g} terlalu kecil: pada psi = 5 deg arus hanya "
                                           f"{self.current_at_5deg():.3f} A per roda. Satuannya N m/rad (bukan per "
                                           f"derajat); gain lama 0.200 setara dengan sekitar 30.9"))
        if self.imu.ready and not self.calibrated:
            found.append((INFO, "Kalibrasi", "IMU belum dikalibrasi -> tekan Tombol Y (wajib sebelum MULAI)"))
        if not self.cbf.available:
            found.append((INFO, "CBF-QP", "mode PD+CBF belum bisa dipakai: " + self.cbf.reason))
        elif self.cbf.fail_count:
            found.append((WARN, "CBF-QP", f"solver acados gagal {self.cbf.fail_count} kali (memakai solusi analitik)"))
        slow = self.loop_hz < float(config.DIAG_MIN_LOOP_RATIO) * float(config.CONTROL_HZ)
        if slow and self.state != CALIBRATING and self.imu.ready and self.motors.ready:
            found.append((WARN, "Loop", f"loop hanya {self.loop_hz:.0f} Hz dari target {config.CONTROL_HZ:g} Hz "
                                        f"(kerja {self.work_ms:.1f} ms) -> turunkan CONTROL_HZ atau cek "
                                        f"timeout motor/I2C"))
        if self.render_errors:
            found.append((WARN, "GUI", f"{self.render_errors} error saat menggambar GUI (lihat {config.LOG_FILE})"))
        order = {ERROR: 0, WARN: 1, INFO: 2}
        return sorted(found, key=lambda item: order[item[0]])

    def wheel_line(self) -> str:
        """Satu kalimat: kenapa roda bergerak atau diam sekarang."""
        left, right = self.motors.motors
        if self.test is not None:
            return f"UJI {self.motors.motors[self.test['index']].name} sedang berjalan"
        if not self.motors.ready:
            return "DIAM - motor bermasalah: " + "; ".join(self.motors.problems())
        if self.state == FAULT:
            return "DIAM - FAULT: " + self.fault_reason + " (tekan Tombol X lalu MULAI)"
        if self.state != BALANCING:
            return f"DIAM - state {self.state}, roda baru diperintah setelah MULAI"
        if not any(self.gains()) and self.pd.Kvel == 0.0:
            return "DIAM - semua gain 0 di config.py sehingga u_PD = 0"
        currents = f"kiri {left.command_a:+.3f} A, kanan {right.command_a:+.3f} A"
        if max(self.stall_s) > float(config.STALL_TIME_S):
            return f"DIPERINTAH ({currents}) TETAPI TIDAK BERPUTAR - lihat Masalah"
        if max(abs(left.command_a), abs(right.command_a)) < 0.02:
            return (f"hampir diam - perintah arus sangat kecil ({currents}); "
                    f"u = {self.u:+.3f} N m dari psi = {self.x[2] * DEG:+.2f} deg")
        return f"diperintah: {currents} | berputar {self.motors.dtheta:+.2f} rad/s"

    # ---------------------------------------------------------------- GUI --
    def render(self) -> None:
        try:
            self._render()
        except Exception:
            self.render_errors += 1
            if self.render_errors <= 3:
                LOGGER.exception("render")

    def _render(self) -> None:
        width = max(70, min(120, shutil.get_terminal_size((100, 40)).columns - 1))
        self.width = width
        lines = []
        tabs = " ".join(f"[{i + 1}:{name}]" if i == self.page else f" {i + 1}:{name} "
                        for i, name in enumerate(PAGES))
        joy = self.joystick.name if self.joystick.connected else "TIDAK TERHUBUNG"
        lines.append("ATERA - TWSBR PD + CBF-QP")
        lines.append("=" * width)
        lines.append(f"State: {self.state:<11} | Mode kontrol: {self.control_mode:<6} | "
                     f"Gerak: {'AKTIF (A)' if self.drive_enabled else 'balancing saja (B)'}")
        lines.append(f"Loop : {self.loop_hz:6.1f} Hz (target {config.CONTROL_HZ:g}) | kerja {self.work_ms:4.1f} ms | "
                     f"overrun {self.overruns} | Joystick: {joy}")
        lines.extend(self.wrap("Pesan: ", self.message)[:3])
        lines.extend(self.wrap("Roda : ", self.wheel_line())[:3])
        problems = [d for d in self.diagnose() if d[0] != INFO]
        if problems:
            lines.append(f"Masalah ({len(problems)}) - penjelasan lengkap di halaman DIAGNOSA:")
            for level, source, text in problems[:3]:
                lines.extend(self.wrap(f"  [{level}] {source}: ", text)[:3])
        else:
            lines.append("Masalah: tidak ada (IMU, motor, joystick, dan gain terbaca baik)")
        lines.append("-" * width)
        lines.append(tabs)
        lines.append("-" * width)
        lines.extend((self.page_status, self.page_diagnosis, self.page_pd, self.page_cbf,
                      self.page_hardware, self.page_ise, self.page_help)[self.page]())
        lines.append("-" * width)
        lines.append("Y kalibrasi | MULAI | X mati | LB PD | RB PD+CBF | A gerak | B diam | LT/RT hal. | L3/R3 uji roda")
        out = "\x1b[H" + "".join(line[:width] + "\x1b[K\n" for line in lines) + "\x1b[J"
        sys.stdout.write(out)
        sys.stdout.flush()

    def wrap(self, prefix: str, text: str) -> list:
        return textwrap.wrap(text, width=getattr(self, "width", 100), initial_indent=prefix,
                             subsequent_indent=" " * len(prefix)) or [prefix]

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

    def page_diagnosis(self) -> list:
        def tag(ok: bool, warn: bool = False) -> str:
            return "[ OK  ]" if ok else ("[AWAS ]" if warn else "[ERROR]")

        imu, joy = self.imu, self.joystick
        lines = ["KOMPONEN"]
        imu_errors = [t for level, t in imu.problems() if level == ERROR]
        if imu.ready and not imu_errors:
            lines.append(f"  {tag(True)} IMU MPU6050   : {imu.chip_name} (WHO_AM_I 0x{imu.who_am_i & 0xFF:02X}) | "
                         f"{imu.read_count} sampel, {imu.error_count} gagal baca | |a| = {imu.acc_norm:.2f} g")
        else:
            lines.extend(self.wrap(f"  {tag(False)} IMU MPU6050   : ", imu_errors[0] if imu_errors else imu.problem))
        for motor in self.motors.motors:
            head = f"  {tag(motor.ready)} {motor.name.capitalize():<13} : "
            if motor.ready:
                mode = MODE_NAMES.get(motor.feedback.mode, "?") if motor.feedback else "?"
                lines.append(f"{head}{motor.port} | {mode} | balasan baik {motor.count_ok}, tanpa balasan "
                             f"{motor.count_timeout}, rusak {motor.count_bad_frame}")
            else:
                lines.extend(self.wrap(head, motor.problem))
        if joy.connected:
            lines.extend(self.wrap(f"  {tag(True)} Joystick      : ",
                                   f"{joy.name} | {joy.event_count} event | terakhir: {joy.last_event}"))
        else:
            lines.extend(self.wrap(f"  {tag(False, True)} Joystick      : ", joy.problem))
        lines.append(f"  {tag(self.calibrated, True)} Kalibrasi IMU : "
                     + ("sudah" if self.calibrated else "BELUM -> tekan Tombol Y"))
        lines.append(f"  {tag(any(self.gains()))} Gain PD       : Kp_theta={self.pd.Kp_theta:g} Kd_dtheta={self.pd.Kd_dtheta:g} "
                     f"Kp_psi={self.pd.Kp_psi:g} Kd_dpsi={self.pd.Kd_dpsi:g} Kvel={self.pd.Kvel:g}")
        lines.extend(self.wrap(f"  {tag(self.cbf.available, True)} CBF-QP        : ", self.cbf.reason))

        left, right = self.motors.motors
        fb_l, fb_r = left.feedback, right.feedback
        lines.append("")
        lines.append("RANTAI SINYAL (dari sensor sampai roda)")
        lines.append(f"  1 state program        : {self.state}" + ("" if self.state == BALANCING else "  <- roda hanya diperintah saat BALANCING"))
        lines.append(f"  2 psi, dpsi (IMU)      : {self.x[2] * DEG:+8.2f} deg, {self.x[3] * DEG:+8.2f} deg/s")
        lines.append(f"  3 u_PD (pd_control)    : {self.u_pd:+8.4f} N m")
        lines.append(f"  4 u terkirim           : {self.u:+8.4f} N m   (batas +-{config.u_max:.2f})")
        lines.append(f"  5 arus perintah  L / R : {left.command_a:+8.3f} / {right.command_a:+8.3f} A   "
                     f"(mentah {left.command_raw:+d} / {right.command_raw:+d})")
        if fb_l and fb_r:
            lines.append(f"  6 arus umpan balik L/R : {fb_l.current_a:+8.3f} / {fb_r.current_a:+8.3f} A")
            lines.append(f"  7 kecepatan roda  L/R  : {left.dtheta:+8.2f} / {right.dtheta:+8.2f} rad/s   "
                         f"(rpm motor {fb_l.speed_rpm:+.0f} / {fb_r.speed_rpm:+.0f})")
        else:
            lines.append("  6-7 umpan balik motor  : belum ada")
        lines.extend(self.wrap("  => Roda: ", self.wheel_line()))

        lines.append("")
        lines.append("UJI RODA LANGSUNG (robot diangkat, roda mati; L3 = kiri, R3 = kanan)")
        lines.extend(self.wrap("  kiri  : ", self.test_result[0]))
        lines.extend(self.wrap("  kanan : ", self.test_result[1]))

        found = self.diagnose()
        lines.append("")
        lines.append(f"MASALAH TERDETEKSI ({len(found)})" if found else "MASALAH TERDETEKSI: tidak ada")
        for level, source, text in found:
            lines.extend(self.wrap(f"  [{level}] {source}: ", text))
        return lines

    def page_pd(self) -> list:
        pd = self.pd
        e = self.x - self.x_ref
        names = ("Kp_theta  * theta ", "Kd_dtheta * dtheta", "Kp_psi    * psi   ", "Kd_dpsi   * dpsi  ")
        gains = self.gains()
        lines = ["u_PD = Kp_theta*theta + Kd_dtheta*dtheta + Kp_psi*psi + Kd_dpsi*dpsi + Kvel",
                 "(state diukur relatif terhadap referensi; gain dibaca dari config.py saat MULAI)", ""]
        for i in range(4):
            lines.append(f"  {names[i]} = {gains[i]:9.4f} x {e[i]:+9.4f} = {self.pd_terms[i]:+9.4f} N m")
        lines.append(f"  Kvel                                          = {self.pd_terms[4]:+9.4f} N m")
        lines.append(f"  {'-' * 58}")
        lines.append(f"  u_PD                                          = {self.u_pd:+9.4f} N m")
        sat = "YA" if abs(self.u_pd) > config.u_max else "tidak"
        lines.append(f"  batas aktuator u_max = {config.u_max:.3f} N m ({config.MAX_CURRENT_A:g} A per roda) | jenuh: {sat}")
        lines.append(f"  arti Kp_psi: pada psi = 5 deg -> {self.current_at_5deg():.3f} A per roda "
                     f"(satuan gain: N m/rad, torsi total dua roda)")
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
        if not self.imu.ready:
            lines.extend(self.wrap("  TIDAK TERBACA: ", self.imu.problem))
        elif s is None:
            lines.append("  belum ada data")
        else:
            lines.append(f"  psi = {s.psi_deg:+8.3f} deg | dpsi = {s.dpsi_deg:+8.2f} deg/s | "
                         f"sudut accel = {s.acc_angle_deg:+8.3f} deg | dt = {s.dt * 1000.0:5.2f} ms")
            lines.append(f"  kalman X/Y = {s.kalman_x_deg:+8.3f} / {s.kalman_y_deg:+8.3f} deg | "
                         f"gyro X/Y = {s.gyro_x_deg_s:+7.2f} / {s.gyro_y_deg_s:+7.2f} deg/s | |a| = {self.imu.acc_norm:.2f} g")
        c = self.calibration
        if c:
            lines.append(f"  kalibrasi: zero offset {c['zero_offset_deg']:+.3f} deg | bias gyro X/Y "
                         f"{c['gyro_bias_x']:+.3f} / {c['gyro_bias_y']:+.3f} deg/s | simpangan {c['gyro_std']:.2f} deg/s")
        else:
            lines.append("  kalibrasi: BELUM")
        lines.append("")
        lines.append("MOTOR DDSM115      perintah    arus fb    rpm fb    posisi    theta      dtheta    error")
        for motor in self.motors.motors:
            fb = motor.feedback
            if not motor.ready or fb is None:
                lines.extend(self.wrap(f"  {motor.name:<12} TIDAK TERBACA: ", motor.problem or "belum ada umpan balik"))
                continue
            lines.append(f"  {motor.name:<12} {motor.command_a:+7.3f} A  {fb.current_a:+7.3f} A  {fb.speed_rpm:+6.0f}   "
                         f"{fb.position_raw:6d}  {motor.theta:+8.3f}  {motor.dtheta:+8.3f}   {decode_error(fb.error_code)}")
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
        lines.append("")
        lines.append(f"Event joystick terakhir: {self.joystick.last_event}")
        if not self.joystick.connected:
            lines.extend(self.wrap("Joystick: ", self.joystick.problem))
            lines.append("Perangkat input yang ada (isi JOYSTICK_PORT di config.py dengan yang benar):")
            lines.extend("  " + name for name in self.joystick.available_devices()[:8])
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