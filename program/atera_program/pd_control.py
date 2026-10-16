"""pd_control.py - Kontrol PD robot ATERA (file ini sama persis di simulasi/ dan program/atera_program/).

Keluaran: u_PD = torsi untuk SATU roda (N m).

[REVISI 16-10] Rumus PD memakai error terhadap SETPOINT BERGERAK (referensi), bukan setpoint 0 tetap:

    u_PD = SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
    error = setpoint - state:   error_psi    = psi_ref    - psi         psi_ref    = 0 (config.psi_setpoint)
                                error_dpsi   = dpsi_ref   - dpsi        dpsi_ref   = 0 (config.dpsi_setpoint)
                                error_theta  = theta_ref  - theta       theta_ref  = s_ref / R   (BERGERAK)
                                error_dtheta = dtheta_ref - dtheta      dtheta_ref = v_ref / R   (BERGERAK)
    SIGN = config.BALANCE_DIRECTION_SIGN (= -1), jadi sama dengan
    u_PD = Kp_psi*(psi - psi_ref) + Kd_psi*(dpsi - dpsi_ref) + Kp_theta*(theta - theta_ref) + Kd_theta*(dtheta - dtheta_ref) + Kvel*arah
    (semua gain positif). Dengan theta_ref = dtheta_ref = 0 hasilnya PERSIS rumus lama.

Referensi dibuat oleh perbarui_referensi(x, dt) tiap siklus (gaya program dosen segway_cbf.py):
    Navigate  : v_target = arah * V_NAVIGATE                       (W / S, D-Pad, analog; 0 = berhenti)
    Balancing : v_target = profil pulang ke titik nol s (s = 0):
                v_target = sign(e) * min(V_PULANG_MAKS, sqrt(2*A_REM_PULANG*|e|), K_PULANG*|e|),   e = 0 - s
                -> robot yang terdorong menjauh kembali pelan-pelan ke titik nol, dan tidak pernah diminta lebih
                   cepat dari yang masih bisa direm sebelum sampai.
    v_ref mengejar v_target dengan percepatan maksimum A_REF_MAKS, s_ref += v_ref*dt,
    s_ref dijepit ke [s - E_S_MAKS, s + E_S_MAKS]  (anti-windup: setpoint tidak tertinggal jauh dari robot).
    Karena setpoint ikut berjalan, Kp_theta dipakai di KEDUA mode (KP_THETA_SEMUA_MODE). Kvel sekarang hanya
    tambahan opsional (feed-forward); kecepatan jalan diatur V_NAVIGATE.

[REVISI 16-10] PD heading (sinkron roda kiri-kanan, compute_belok):
    yaw  = R*(theta_kanan - theta_kiri)/W_RODA       (+ = robot berputar ke KIRI dilihat dari atas, dari encoder)
    u_belok = Kp_yaw*(yaw - yaw_ref) + Kd_yaw*(dyaw - dyaw_ref)
    roda kiri u + u_belok, roda kanan u - u_belok   (u_belok > 0 = robot berputar ke KANAN, yaw turun)
    Balancing : yaw_ref = arah saat MULAI / Tombol B, dyaw_ref = 0 -> robot TIDAK berputar-putar sendiri
    Navigate  : dyaw_ref = -belok * YAW_RATE_NAVIGATE (A / D, D-Pad kiri / kanan), yaw_ref += dyaw_ref*dt
    u_belok dibatasi YAW_TORSI_MAKS dan sisa torsi u_max - |u| (balancing selalu didahulukan, rata-rata kedua
    roda tetap u).

Kontrak (tidak berubah): pd.compute(x) -> float (N m),  x = [theta, dtheta, psi, dpsi].
compute_PD(x) = compute(x) (nama lama, dipakai tuning_optuna.py).
Urutan pemakaian tiap siklus:
    pd.set_command(arah, mode_gerak, belok)
    pd.perbarui_referensi(x, dt)        (tanpa ini referensi tetap 0 = rumus lama, contoh tuning_optuna.py)
    u_PD = pd.compute(x)
    u_belok = pd.compute_belok(yaw, dyaw, u, u_max)     (u = torsi sesudah CBF-QP / batas arus)
"""

import math

import config

MODE_BALANCING = "BALANCING"       # pengenal mode gerak (dipakai juga oleh cbf_qp.py dan atera_main.py)
MODE_NAVIGATE = "NAVIGATE"


def _jepit(nilai, bawah, atas):
    return max(bawah, min(atas, nilai))


def hitung_yaw(theta_kiri, theta_kanan):
    """[REVISI 16-10] Sudut (atau kecepatan) yaw dari encoder kedua roda [rad atau rad/s].
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
        self.Kp_yaw = float(config.Kp_yaw)                       # [REVISI 16-10]
        self.Kd_yaw = float(config.Kd_yaw)                       # [REVISI 16-10]
        self.balance_direction_sign = float(getattr(config, "BALANCE_DIRECTION_SIGN", -1.0))

        # Setpoint tetap badan
        self.psi_setpoint = float(config.psi_setpoint)
        self.dpsi_setpoint = float(config.dpsi_setpoint)

        # Perintah dari joystick / keyboard
        self.mode_gerak = MODE_BALANCING
        self.arah = 0                  # -1 mundur, 0 lepas, +1 maju
        self.belok = 0                 # -1 kiri, 0 lepas, +1 kanan   [REVISI 16-10]

        # [REVISI 16-10] Referensi bergerak (setpoint theta dan dtheta)
        self.s_ref = 0.0               # [m]
        self.v_ref = 0.0               # [m/s]
        self.theta_ref = float(config.theta_setpoint)
        self.dtheta_ref = float(config.dtheta_setpoint)
        self.yaw_ref = 0.0             # [rad]
        self.dyaw_ref = 0.0            # [rad/s]

        # Nilai terakhir (untuk ditampilkan di GUI dan disimpan di CSV)
        self.error_theta = self.error_dtheta = self.error_psi = self.error_dpsi = 0.0
        self.u_PD = 0.0
        self.yaw = self.dyaw = 0.0
        self.u_belok = 0.0

    def set_command(self, arah, mode_gerak, belok=0):
        """arah = -1 / 0 / +1 (+1 = maju), belok = -1 / 0 / +1 (+1 = kanan), mode_gerak = MODE_BALANCING / MODE_NAVIGATE.
        Di mode Balancing arah dan belok diabaikan."""
        self.mode_gerak = mode_gerak
        self.arah = int(arah)
        self.belok = int(belok)

    # ------------------------------------------------------------ referensi
    def reset_referensi(self, x, yaw=0.0, ikut_kecepatan=False):
        """[REVISI 16-10] Setpoint = keadaan robot sekarang. Dipanggil saat MULAI, robot diletakkan ulang, dan saat
        titik nol s pindah (Tombol B). ikut_kecepatan=True: v_ref = kecepatan robot sekarang (supaya robot yang sedang
        berjalan tidak direm mendadak; v_ref lalu berkurang pelan dengan A_REF_MAKS)."""
        s = config.R * float(x[config.IDX_THETA])
        v = config.R * float(x[config.IDX_DTHETA]) if ikut_kecepatan else 0.0
        v_batas = max(config.V_NAVIGATE, config.V_PULANG_MAKS)
        self.s_ref = s
        self.v_ref = _jepit(v, -v_batas, v_batas)
        self.theta_ref = self.s_ref / config.R
        self.dtheta_ref = self.v_ref / config.R
        self.yaw_ref = float(yaw)
        self.dyaw_ref = 0.0

    def perbarui_referensi(self, x, dt):
        """[REVISI 16-10] Satu langkah generator referensi (lihat docstring modul). dt = periode siklus [s]."""
        s = config.R * float(x[config.IDX_THETA])
        if self.mode_gerak == MODE_NAVIGATE:
            v_target = self.arah * config.V_NAVIGATE
        else:
            e = 0.0 - s                                           # titik nol s = rumah
            v_target = math.copysign(min(config.V_PULANG_MAKS, math.sqrt(2.0 * config.A_REM_PULANG * abs(e)),
                                         config.K_PULANG * abs(e)), e)
        langkah_v = config.A_REF_MAKS * dt
        self.v_ref += _jepit(v_target - self.v_ref, -langkah_v, langkah_v)
        self.s_ref += self.v_ref * dt
        self.s_ref = _jepit(self.s_ref, s - config.E_S_MAKS, s + config.E_S_MAKS)
        self.theta_ref = self.s_ref / config.R
        self.dtheta_ref = self.v_ref / config.R

        # heading: belok +1 (kanan) = berputar searah jarum jam dilihat dari atas = yaw turun
        dyaw_target = -self.belok * config.YAW_RATE_NAVIGATE if self.mode_gerak == MODE_NAVIGATE else 0.0
        langkah_w = config.A_YAW_MAKS * dt
        self.dyaw_ref += _jepit(dyaw_target - self.dyaw_ref, -langkah_w, langkah_w)
        self.yaw_ref += self.dyaw_ref * dt

    # ------------------------------------------------------------------- PD
    def compute(self, x):
        """u_PD [N m] (torsi SATU roda) untuk state x = [theta, dtheta, psi, dpsi]."""
        theta = float(x[config.IDX_THETA])
        dtheta = float(x[config.IDX_DTHETA])
        psi = float(x[config.IDX_PSI])
        dpsi = float(x[config.IDX_DPSI])

        # Error = setpoint - state  ([REVISI 16-10] setpoint theta dan dtheta bergerak)
        error_theta = self.theta_ref - theta
        error_dtheta = self.dtheta_ref - dtheta
        error_psi = self.psi_setpoint - psi
        error_dpsi = self.dpsi_setpoint - dpsi

        if self.mode_gerak == MODE_BALANCING:
            Kp_theta = self.Kp_theta
            arah = 0
        else:
            # [REVISI 16-10] Kp_theta tetap dipakai di Navigate (setpoint ikut berjalan). False = perilaku lama.
            Kp_theta = self.Kp_theta if config.KP_THETA_SEMUA_MODE else 0.0
            arah = self.arah

        # Persamaan PD
        u_PD = (self.Kp_psi * error_psi) + (self.Kd_psi * error_dpsi) \
            + (Kp_theta * error_theta) + (self.Kd_theta * error_dtheta)
        u_PD = u_PD * self.balance_direction_sign
        u_PD = u_PD + (self.Kvel * arah)

        self.error_theta = error_theta
        self.error_dtheta = error_dtheta
        self.error_psi = error_psi
        self.error_dpsi = error_dpsi
        self.u_PD = u_PD
        return u_PD

    compute_PD = compute               # nama lama (tuning_optuna.py)

    # -------------------------------------------------------------- heading
    def compute_belok(self, yaw, dyaw, u, u_maks):
        """[REVISI 16-10] u_belok [N m] (lihat docstring modul). yaw, dyaw dari hitung_yaw(). u = torsi bersama kedua
        roda (sesudah CBF-QP / batas arus), u_maks = batas torsi satu roda."""
        self.yaw = float(yaw)
        self.dyaw = float(dyaw)
        if not config.YAW_AKTIF:
            self.u_belok = 0.0
            return 0.0
        u_belok = self.Kp_yaw * (self.yaw - self.yaw_ref) + self.Kd_yaw * (self.dyaw - self.dyaw_ref)
        u_belok = _jepit(u_belok, -config.YAW_TORSI_MAKS, config.YAW_TORSI_MAKS)
        sisa = max(0.0, float(u_maks) - abs(float(u)))           # balancing didahulukan
        self.u_belok = _jepit(u_belok, -sisa, sisa)
        return self.u_belok

    def error_yaw(self):
        """[REVISI 16-10] yaw - yaw_ref [rad] (dipakai KOMET berputar)."""
        return self.yaw - self.yaw_ref