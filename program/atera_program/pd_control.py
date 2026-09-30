"""Kontroler PD untuk self-balancing robot ATERA.

Output kendali sekarang dibagi menjadi dua bagian yang DIJUMLAHKAN (tanpa
clamp sama sekali di modul ini):

- Loop **psi** : sudut badan robot dari MPU6050.
    Input : angle_psi (sudut badan), angular_dot_psi (kecepatan sudut badan)
    Target: setpoint_psi
    Gain  : Kp_psi, Kd_psi
    -> Ini persis loop PD tunggal yang SEBELUMNYA sudah terbukti stabil.
       Logika & tanda (sign) di loop ini TIDAK diubah sama sekali.

- Loop **theta**: sudut roda kiri & kanan dari encoder DDSM115.
    Input : angle_theta_left/right (sudut roda), angular_dot_theta_left/right
             (kecepatan sudut roda)
    Target: setpoint_theta (default sama untuk roda kiri & kanan)
    Gain  : Kp_theta, Kd_theta
    -> Loop tambahan (opsional). Jika Kp_theta = Kd_theta = 0 (nilai default
       di config.py), loop ini tidak berkontribusi apa pun terhadap output,
       sehingga perilaku robot kembali semirip mungkin dengan versi
       single-loop sebelumnya.

Catatan penting terkait permintaan perubahan:
1. Clamp/pembatasan output PD (yang sebelumnya membatasi base_output &
   left/right_output ke rentang [-OUTPUT_LIMIT, OUTPUT_LIMIT]) sudah
   DIHILANGKAN dari modul ini.
2. Kp_psi, Kd_psi, Kp_theta, Kd_theta sekarang berupa bilangan bulat (int),
   bukan lagi float berskala 0.0 - 1.0. Karena itu keluaran PD di sini TIDAK
   lagi otomatis berada di rentang -1..1.
   PENTING: hasil (Kp*error + Kd*rate) dibagi dulu dengan config.GAIN_SCALE
   sebelum dipakai. Ini WAJIB ada supaya gain integer tetap bisa dituning
   halus (lihat catatan GAIN_SCALE di config.py) -- tanpa pembagi ini,
   gain integer sekecil apa pun (Kp_psi=1) akan langsung membuat motor
   full-power hanya pada error ~1 derajat, menyebabkan osilasi kasar
   (perilaku relay/bang-bang, bukan PD halus).
3. Batas keamanan hardware (arus & kecepatan motor) TETAP ada, tetapi
   levelnya dipindahkan sepenuhnya ke driver motor (lihat ddsm115.py:
   DDSM115Motor.command_normalized(), yang tetap meng-clamp nilai masukan ke
   -1.0..1.0 sebelum dikonversi ke arus/kecepatan fisik). Jadi keluaran PD
   yang besar akan otomatis "mentok" (saturasi) di level motor, bukan
   dibatasi di sini.
4. Alpha_1 dan Alpha_2 pada config.py TIDAK dipakai di modul ini; keduanya
   sengaja disimpan untuk pengembangan PD + CBF-QP berikutnya.
"""
from __future__ import annotations

from dataclasses import dataclass

import config as tunning


@dataclass
class PDControlState:
    # ---------------- Loop psi (sudut badan, dari MPU6050) ----------------
    angle_psi: float               # sudut badan aktual (deg)
    angular_dot_psi: float         # kecepatan sudut badan (deg/s)
    setpoint_psi: float            # target sudut badan (deg)
    error_psi: float               # error = setpoint_psi - angle_psi (deg)
    error_rate_psi: float          # = -angular_dot_psi (deg/s)
    base_output_psi: float         # kontribusi output dari loop psi saja

    # ---------------- Loop theta (sudut roda, dari DDSM115) ----------------
    angle_theta_left: float            # sudut roda kiri, dari posisi encoder (deg)
    angle_theta_right: float           # sudut roda kanan, dari posisi encoder (deg)
    angular_dot_theta_left: float      # kecepatan sudut roda kiri (deg/s)
    angular_dot_theta_right: float     # kecepatan sudut roda kanan (deg/s)
    setpoint_theta: float              # target sudut roda (deg), sama utk kiri & kanan
    error_theta_left: float            # error sudut roda kiri (deg)
    error_theta_right: float           # error sudut roda kanan (deg)
    base_output_theta_left: float      # kontribusi loop theta utk roda kiri
    base_output_theta_right: float     # kontribusi loop theta utk roda kanan

    # ------------- Alias nama lama, dipakai atera_main.py (UI/log) --------
    target_angle_deg: float        # alias setpoint_psi
    error_deg: float                # alias error_psi
    error_rate_deg_s: float         # alias error_rate_psi
    base_output: float              # alias base_output_psi

    # ---------------- Output akhir yang dikirim ke motor -------------------
    left_output: float
    right_output: float


class BalancePDController:
    def __init__(self) -> None:
        # Loop psi (badan)
        self.kp_psi = int(tunning.Kp_psi)
        self.kd_psi = int(tunning.Kd_psi)
        # Loop theta (roda) - boleh 0 untuk menonaktifkan loop ini
        self.kp_theta = int(tunning.Kp_theta)
        self.kd_theta = int(tunning.Kd_theta)

        self.deadband_deg = float(tunning.CONTROLLER_DEADBAND_DEG)
        self.balance_direction_sign = float(tunning.BALANCE_DIRECTION_SIGN)
        self.turn_fraction = float(tunning.TURN_OUTPUT_FRACTION)
        self.default_setpoint_theta = float(getattr(tunning, "SETPOINT_THETA_DEG", 0.0))
        # Pembagi skala gain integer (lihat catatan GAIN_SCALE di config.py).
        # Default 1 (tidak membagi apa pun) kalau config.py belum punya
        # GAIN_SCALE, supaya tetap kompatibel mundur.
        self.gain_scale = float(getattr(tunning, "GAIN_SCALE", 1))
        if self.gain_scale <= 0:
            self.gain_scale = 1.0

    def compute(
        self,
        angle_deg: float,
        angular_rate_deg_s: float,
        target_angle_deg: float = 0.0,
        turn_command: float = 0.0,
        angle_theta_left: float = 0.0,
        angle_theta_right: float = 0.0,
        angular_dot_theta_left: float = 0.0,
        angular_dot_theta_right: float = 0.0,
        setpoint_theta: float | None = None,
    ) -> PDControlState:
        # ============== Loop psi (badan) - TIDAK diubah logikanya ==============
        setpoint_psi = float(target_angle_deg)
        angle_psi = float(angle_deg)
        angular_dot_psi = float(angular_rate_deg_s)

        error_psi = setpoint_psi - angle_psi
        if abs(error_psi) < self.deadband_deg:
            error_psi = 0.0
        # PD dengan D memakai laju sudut terukur (sama seperti versi sebelumnya).
        error_rate_psi = -angular_dot_psi

        base_output_psi = ((self.kp_psi * error_psi) + (self.kd_psi * error_rate_psi)) / self.gain_scale
        base_output_psi *= self.balance_direction_sign
        # (Sengaja TIDAK ada clamp di sini lagi. GAIN_SCALE di atas yang
        # menjaga resolusi tuning tetap halus meski Kp_psi/Kd_psi integer.)

        turn_term = max(-1.0, min(1.0, float(turn_command))) * self.turn_fraction

        # ============== Loop theta (roda kiri & kanan) - BARU ==============
        theta_sp = float(setpoint_theta) if setpoint_theta is not None else self.default_setpoint_theta

        error_theta_left = theta_sp - float(angle_theta_left)
        error_theta_right = theta_sp - float(angle_theta_right)

        base_output_theta_left = (
            (self.kp_theta * error_theta_left) + (self.kd_theta * -float(angular_dot_theta_left))
        ) / self.gain_scale
        base_output_theta_right = (
            (self.kp_theta * error_theta_right) + (self.kd_theta * -float(angular_dot_theta_right))
        ) / self.gain_scale

        # ============== Gabungan psi + theta + turn (tanpa clamp) ==============
        left_output = (base_output_psi - turn_term) + base_output_theta_left
        right_output = (base_output_psi + turn_term) + base_output_theta_right

        return PDControlState(
            angle_psi=angle_psi,
            angular_dot_psi=angular_dot_psi,
            setpoint_psi=setpoint_psi,
            error_psi=error_psi,
            error_rate_psi=error_rate_psi,
            base_output_psi=base_output_psi,
            angle_theta_left=float(angle_theta_left),
            angle_theta_right=float(angle_theta_right),
            angular_dot_theta_left=float(angular_dot_theta_left),
            angular_dot_theta_right=float(angular_dot_theta_right),
            setpoint_theta=theta_sp,
            error_theta_left=error_theta_left,
            error_theta_right=error_theta_right,
            base_output_theta_left=base_output_theta_left,
            base_output_theta_right=base_output_theta_right,
            target_angle_deg=setpoint_psi,
            error_deg=error_psi,
            error_rate_deg_s=error_rate_psi,
            base_output=base_output_psi,
            left_output=left_output,
            right_output=right_output,
        )