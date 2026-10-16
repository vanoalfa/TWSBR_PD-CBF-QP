"""pd_control.py - Kontrol PD robot ATERA (file ini sama persis di simulasi/ dan program/atera_program/).

Keluaran: u_PD = torsi untuk SATU roda (N m).

Rumus PD:
    u_PD = SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
    error = setpoint - state, semua setpoint = 0 (config.*_setpoint). theta diukur dari SETPOINT theta (titik nol s),
    jadi theta = 0 tepat di setpoint. SIGN = config.BALANCE_DIRECTION_SIGN (= -1), sehingga sama dengan
    u_PD = Kp_psi*psi + Kd_psi*dpsi + Kp_theta*theta + Kd_theta*dtheta + Kvel*arah        (semua gain positif)

Mode gerak OTOMATIS dari perintah maju / mundur (set_command), tanpa tombol mode:
    MODE_DIAM  : arah = 0 (D-Pad atas / bawah tidak ditekan). Kp_theta AKTIF -> robot ditahan di setpoint theta.
                 CBF-QP memakai constraint jarak s (h5, h6) dan KOMET |s| > S_KOMET aktif (atera_main.py).
    MODE_GERAK : arah = +1 / -1 (D-Pad atas / bawah ditekan). Kp_theta DIABAIKAN, Kvel*arah AKTIF.
                 Constraint s dan KOMET s tidak dipakai (robot memang berjalan jauh).
    Saat D-Pad DILEPAS (GERAK -> DIAM), atera_main.py memasang setpoint theta baru tepat di posisi itu
    (titik nol s pindah, theta = 0). Tombol B memasang setpoint yang sama kapan saja.
    Belok (D-Pad kiri / kanan) bukan GERAK: tanpa maju / mundur robot tetap DIAM dan berputar di tempat.

    Kecepatan tunak saat GERAK (lantai datar: psi -> 0 dan u -> 0):   Kd_theta*dtheta + Kvel*arah = 0
        dtheta = -Kvel*arah / Kd_theta,   v = R*dtheta
    jadi Kvel harus NEGATIF supaya arah +1 (D-Pad atas) berjalan ke DEPAN.

PD heading (sinkron roda kiri-kanan, compute_belok):
    yaw  = R*(theta_kanan - theta_kiri)/W_RODA       (+ = robot berputar ke KIRI dilihat dari atas, dari encoder)
    dyaw_ref mengejar -belok*YAW_RATE_BELOK (perubahan maks A_YAW_MAKS per detik), yaw_ref += dyaw_ref*dt
        -> D-Pad kiri / kanan dilepas: yaw_ref tetap, robot tidak berputar-putar sendiri
    u_belok = Kp_yaw*(yaw - yaw_ref) + Kd_yaw*(dyaw - dyaw_ref)
    roda kiri u + u_belok, roda kanan u - u_belok   (u_belok > 0 = robot berputar ke KANAN, yaw turun)
    u_belok dibatasi YAW_TORSI_MAKS dan sisa torsi u_max - |u| (balancing didahulukan, rata-rata kedua roda tetap u).

Kontrak: pd.compute(x) -> float (N m),  x = [theta, dtheta, psi, dpsi].
Urutan pemakaian tiap siklus:
    pd.set_command(arah, belok)                          (mode gerak ikut otomatis)
    u_PD = pd.compute(x)
    u_belok = pd.compute_belok(yaw, dyaw, u, u_max, dt)  (u = torsi sesudah CBF-QP / batas arus)
"""

import config

MODE_DIAM = "DIAM"         # nama mode gerak, dipakai juga oleh cbf_qp.py (kunci C_MODE) dan atera_main.py
MODE_GERAK = "GERAK"


def _jepit(nilai, bawah, atas):
    return max(bawah, min(atas, nilai))


def hitung_yaw(theta_kiri, theta_kanan):
    """Sudut (atau kecepatan) yaw dari encoder kedua roda [rad atau rad/s].
    theta_kiri, theta_kanan: sudut roda relatif badan, positif = roda ke DEPAN (psi saling meniadakan)."""
    return config.R * (theta_kanan - theta_kiri) / config.W_RODA


class PDControl:
    def __init__(self):
        # Gain
        self.Kp_psi = float(config.Kp_psi)
        self.Kd_psi = float(config.Kd_psi)
        self.Kp_theta = float(config.Kp_theta)
        self.Kd_theta = float(config.Kd_theta)
        self.Kvel = float(config.Kvel)
        self.Kp_yaw = float(config.Kp_yaw)
        self.Kd_yaw = float(config.Kd_yaw)
        self.balance_direction_sign = float(config.BALANCE_DIRECTION_SIGN)

        # Setpoint (theta diukur dari setpoint theta, jadi theta_setpoint = 0)
        self.theta_setpoint = float(config.theta_setpoint)
        self.dtheta_setpoint = float(config.dtheta_setpoint)
        self.psi_setpoint = float(config.psi_setpoint)
        self.dpsi_setpoint = float(config.dpsi_setpoint)

        # Perintah dari joystick / keyboard
        self.mode_gerak = MODE_DIAM
        self.arah = 0                  # -1 mundur, 0 lepas, +1 maju
        self.belok = 0                 # -1 kiri, 0 lepas, +1 kanan

        # Referensi heading
        self.yaw_ref = 0.0             # [rad]
        self.dyaw_ref = 0.0            # [rad/s]

        # Nilai terakhir (untuk ditampilkan di GUI dan disimpan di CSV)
        self.error_theta = self.error_dtheta = self.error_psi = self.error_dpsi = 0.0
        self.u_PD = 0.0
        self.yaw = self.dyaw = 0.0
        self.u_belok = 0.0

    def set_command(self, arah, belok=0):
        """arah = -1 / 0 / +1 (+1 = maju), belok = -1 / 0 / +1 (+1 = kanan).
        Mode gerak ikut otomatis: MODE_GERAK bila arah != 0, selain itu MODE_DIAM."""
        self.arah = int(arah)
        self.belok = int(belok)
        self.mode_gerak = MODE_GERAK if self.arah != 0 else MODE_DIAM

    def pasang_yaw_ref(self, yaw):
        """Setpoint heading = arah robot sekarang (MULAI, Tombol B, robot diletakkan ulang)."""
        self.yaw_ref = float(yaw)
        self.dyaw_ref = 0.0

    # ------------------------------------------------------------------- PD
    def compute(self, x):
        """u_PD [N m] (torsi SATU roda) untuk state x = [theta, dtheta, psi, dpsi]."""
        theta = float(x[config.IDX_THETA])
        dtheta = float(x[config.IDX_DTHETA])
        psi = float(x[config.IDX_PSI])
        dpsi = float(x[config.IDX_DPSI])

        # Error = setpoint - state
        error_theta = self.theta_setpoint - theta
        error_dtheta = self.dtheta_setpoint - dtheta
        error_psi = self.psi_setpoint - psi
        error_dpsi = self.dpsi_setpoint - dpsi

        # Kp_theta hanya saat DIAM; saat GERAK robot tidak ditahan di setpoint theta
        Kp_theta = 0.0 if self.mode_gerak == MODE_GERAK else self.Kp_theta

        # Persamaan PD (arah = 0 saat DIAM, jadi Kvel*arah hanya bekerja saat GERAK)
        u_PD = (self.Kp_psi * error_psi) + (self.Kd_psi * error_dpsi) \
            + (Kp_theta * error_theta) + (self.Kd_theta * error_dtheta)
        u_PD = u_PD * self.balance_direction_sign
        u_PD = u_PD + (self.Kvel * self.arah)

        self.error_theta = error_theta
        self.error_dtheta = error_dtheta
        self.error_psi = error_psi
        self.error_dpsi = error_dpsi
        self.u_PD = u_PD
        return u_PD

    # -------------------------------------------------------------- heading
    def compute_belok(self, yaw, dyaw, u, u_maks, dt):
        """u_belok [N m] (lihat docstring modul). yaw, dyaw dari hitung_yaw(). u = torsi bersama kedua roda
        (sesudah CBF-QP / batas arus), u_maks = batas torsi satu roda, dt = periode siklus [s]."""
        self.yaw = float(yaw)
        self.dyaw = float(dyaw)

        # Referensi heading: belok +1 (kanan) = berputar searah jarum jam dilihat dari atas = yaw turun
        langkah = config.A_YAW_MAKS * dt
        self.dyaw_ref += _jepit(-self.belok * config.YAW_RATE_BELOK - self.dyaw_ref, -langkah, langkah)
        self.yaw_ref += self.dyaw_ref * dt

        if not config.YAW_AKTIF:
            self.u_belok = 0.0
            return 0.0
        u_belok = self.Kp_yaw * (self.yaw - self.yaw_ref) + self.Kd_yaw * (self.dyaw - self.dyaw_ref)
        u_belok = _jepit(u_belok, -config.YAW_TORSI_MAKS, config.YAW_TORSI_MAKS)
        sisa = max(0.0, float(u_maks) - abs(float(u)))           # balancing didahulukan
        self.u_belok = _jepit(u_belok, -sisa, sisa)
        return self.u_belok