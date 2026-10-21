"""pd_control.py - Kontrol PD robot ATERA (file ini sama persis di simulasi/ dan program/atera_program/).

Keluaran: u_PD = torsi untuk SATU roda (N m).

Rumus PD:
    u_PD = SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
    error = setpoint - state, semua setpoint = 0 (config.*_setpoint). theta diukur dari SETPOINT theta (titik nol s),
    jadi theta = 0 tepat di setpoint. SIGN = config.BALANCE_DIRECTION_SIGN (= -1), sehingga sama dengan
    u_PD = Kp_psi*psi + Kd_psi*dpsi + Kp_theta*theta + Kd_theta*dtheta + Kvel*arah        (semua gain positif)

Mode gerak (mode_gerak), dipilih atur_mode() (simulasi: B / F, robot fisik: Tombol B / A):
    MODE_DIAM      : default. Tanpa Kp_theta (posisi tidak ditahan): Kp_psi, Kd_psi, Kd_theta, Kvel*arah.
    MODE_BALANCING : B (toggle dengan DIAM). Kp_theta AKTIF -> robot kembali ke setpoint theta (dipasang saat masuk
                     BALANCING dan saat D-Pad atas / bawah dilepas). Selama D-Pad atas / bawah ditekan, Kp_theta
                     diabaikan. KOMET |s| > S_KOMET hanya di mode ini.
    MODE_BEBAS     : A (simulasi: F). Tanpa Kp_theta dan tanpa PD heading (yaw tidak ditahan), belok = selisih torsi
                     tetap TORSI_BELOK. A lagi -> DIAM. B saat BEBAS -> BALANCING.
    D-Pad atas / bawah (Kvel*arah) dan kiri / kanan (belok) berlaku di semua mode.

    Kecepatan tunak saat D-Pad atas / bawah ditekan (lantai datar: psi -> 0 dan u -> 0):  Kd_theta*dtheta + Kvel*arah = 0
        dtheta = -Kvel*arah / Kd_theta,   v = R*dtheta
    jadi Kvel harus NEGATIF supaya arah +1 (D-Pad atas) berjalan ke DEPAN.

PD heading (sinkron roda kiri-kanan, compute_belok; tidak dipakai di MODE_BEBAS):
    yaw  = R*(theta_kanan - theta_kiri)/W_RODA       (+ = robot berputar ke KIRI dilihat dari atas, dari encoder)
    dyaw_ref mengejar -belok*YAW_RATE_BELOK (perubahan maks A_YAW_MAKS per detik), yaw_ref += dyaw_ref*dt
        -> D-Pad kiri / kanan dilepas: yaw_ref tetap, robot tidak berputar-putar sendiri
    u_belok = Kp_yaw*(yaw - yaw_ref) + Kd_yaw*(dyaw - dyaw_ref)
    roda kiri u + u_belok, roda kanan u - u_belok   (u_belok > 0 = robot berputar ke KANAN, yaw turun)
    u_belok dibatasi YAW_TORSI_MAKS dan sisa torsi u_max - |u| (balancing didahulukan, rata-rata kedua roda tetap u).
    MODE_BEBAS: u_belok = belok*TORSI_BELOK (tanpa umpan balik yaw), juga dibatasi sisa torsi u_max - |u|.

Kontrak: pd.compute(x) -> float (N m),  x = [theta, dtheta, psi, dpsi].
Urutan pemakaian tiap siklus:
    pd.set_command(arah, belok)                          (pd.atur_mode() untuk Tombol B / A)
    u_PD = pd.compute(x)
    u_belok = pd.compute_belok(yaw, dyaw, u, u_max, dt)  (u = torsi sesudah CBF-QP / batas arus)
"""

import config

MODE_DIAM = "DIAM"         # nama mode gerak, dipakai juga oleh cbf_qp.py (kunci C_MODE) dan atera_main.py
MODE_BALANCING = "BALANCING"
MODE_BEBAS = "BEBAS"


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

        # Mode gerak dan perintah dari joystick / keyboard
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
        """arah = -1 / 0 / +1 (+1 = maju), belok = -1 / 0 / +1 (+1 = kanan)."""
        self.arah = int(arah)
        self.belok = int(belok)

    def atur_mode(self, mode):
        """MODE_DIAM, MODE_BALANCING, atau MODE_BEBAS."""
        if mode not in (MODE_DIAM, MODE_BALANCING, MODE_BEBAS):
            raise ValueError("mode gerak tidak dikenal: %s" % mode)
        self.mode_gerak = mode

    @property
    def tahan_posisi(self):
        """True = Kp_theta bekerja: mode BALANCING dan D-Pad atas / bawah tidak ditekan."""
        return self.mode_gerak == MODE_BALANCING and self.arah == 0

    def pasang_yaw_ref(self, yaw):
        """Setpoint heading = arah robot sekarang (MULAI, masuk BALANCING, keluar BEBAS, robot diletakkan ulang)."""
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

        # Kp_theta hanya saat BALANCING tanpa perintah maju / mundur
        Kp_theta = self.Kp_theta if self.tahan_posisi else 0.0

        # Persamaan PD (Kvel*arah hanya bekerja saat D-Pad atas / bawah ditekan)
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

        if self.mode_gerak == MODE_BEBAS:
            # Program lama: selisih torsi tetap, tanpa umpan balik yaw. Referensi ikut yaw sekarang supaya tidak
            # melompat saat MODE_BEBAS dimatikan.
            self.yaw_ref = self.yaw
            self.dyaw_ref = 0.0
            u_belok = self.belok * config.TORSI_BELOK
        else:
            # Referensi heading: belok +1 (kanan) = berputar searah jarum jam dilihat dari atas = yaw turun
            langkah = config.A_YAW_MAKS * dt
            self.dyaw_ref += _jepit(-self.belok * config.YAW_RATE_BELOK - self.dyaw_ref, -langkah, langkah)
            self.yaw_ref += self.dyaw_ref * dt
            if config.YAW_AKTIF:
                u_belok = self.Kp_yaw * (self.yaw - self.yaw_ref) + self.Kd_yaw * (self.dyaw - self.dyaw_ref)
                u_belok = _jepit(u_belok, -config.YAW_TORSI_MAKS, config.YAW_TORSI_MAKS)
            else:
                u_belok = 0.0

        sisa = max(0.0, float(u_maks) - abs(float(u)))           # balancing didahulukan
        self.u_belok = _jepit(u_belok, -sisa, sisa)
        return self.u_belok