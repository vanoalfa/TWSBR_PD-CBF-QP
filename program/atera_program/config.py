"""config.py - satu-satunya sumber nilai program ATERA (robot fisik).

Nilai fisik, gain PD, dan Alpha CBF-QP sama dengan simulasi/config.py (hasil tuning Optuna, profil DATASHEET),
supaya robot fisik menjalankan kontrol yang sama dengan simulasi.

Satuan SI (kg, m, s, rad, N m) kecuali nama berakhiran _DEG, _A, _HZ, _S.

Konvensi tanda (sama dengan simulasi):
    state x = [theta, dtheta, psi, dpsi]
    psi   > 0 : badan miring ke DEPAN
    theta > 0 : roda menggelinding ke DEPAN (sudut roda absolut, lihat model.hitung_theta)
    u     > 0 : torsi SATU roda yang mendorong roda ke DEPAN; kedua roda diberi u yang sama
"""

import math

# =============================================================================
#                              1. PARAMETER FISIK  (= simulasi)
# =============================================================================
m_r = 0.745            # [kg]      massa SATU roda
m_b = 2.225            # [kg]      massa badan (timbangan: total 3.715 kg - 2 x 0.745 kg)
R = 0.050              # [m]       jari-jari roda
L = 0.100408           # [m]       jarak poros roda ke CoM badan (Fusion 360 ATERA v40)
j_theta = 9.2e-4       # [kg m^2]  momen inersia SATU roda
j_psi = 0.02337        # [kg m^2]  momen inersia badan terhadap CoM badan
g = 9.81               # [m/s^2]
N_WHEEL = 2

zeta_DEG = 0.0                      # [deg] kemiringan bidang: 0.0 (datar) atau 8.0 (miring), + = menanjak ke depan
zeta = math.radians(zeta_DEG)

# =============================================================================
#                              2. STATE DAN SETPOINT
# =============================================================================
IDX_THETA, IDX_DTHETA, IDX_PSI, IDX_DPSI = 0, 1, 2, 3
NX = 4
NU = 1

theta_setpoint = 0.0                # [rad]   titik nol s / theta dipasang Tombol Y (kalibrasi) dan Tombol B
dtheta_setpoint = 0.0               # [rad/s]
psi_setpoint = 0.0                  # [rad]
dpsi_setpoint = 0.0                 # [rad/s]

# =============================================================================
#                              3. SAFETY SET DAN KOMET
# =============================================================================
psi_max_DEG = 15.0                  # [deg]    batas psi yang dijaga CBF (h1, h2)
psi_max = math.radians(psi_max_DEG) # [rad]    |psi|    <= psi_max
dpsi_max = 2.0 * math.pi            # [rad/s]  |dpsi|   <= dpsi_max
dtheta_max = 13.0                   # [rad/s]  |dtheta| <= dtheta_max
s_max = 1.0                         # [m]      |s| <= s_max, s = R*theta dari titik nol s
psi_min, dpsi_min, dtheta_min, s_min = -psi_max, -dpsi_max, -dtheta_max, -s_max

# KOMET = robot tidak tertolong, roda dimatikan (sama dengan simulasi):
#   jatuh : |psi| > SAFE_TILT_DEG (semua mode gerak)
#   lari  : |s|   > S_KOMET       (hanya mode Balancing)
SAFE_TILT_DEG = 40.0                # [deg]  ASUMSI: dipilih pengguna (20261014)
S_KOMET = 2.0                       # [m]
START_TILT_DEG = 15.0               # [deg] MULAI ditolak bila |psi| lebih besar
# Pengaman tambahan robot fisik (tidak ada di simulasi): |u| terus di batas arus selama ini -> roda dimatikan.
# Contoh: robot diangkat atau tertahan dan roda berputar terus. 0 = tidak dipakai.
SATURASI_MAKS_S = 2.0               # [s]

# =============================================================================
#                              4. PD  (= simulasi)
# =============================================================================
# u_PD = Kp_psi*psi + Kd_psi*dpsi + Kp_theta*theta + Kd_theta*dtheta + Kvel*arah
# arah = -1 / 0 / +1 dari joystick (hanya mode Navigate). Semua gain -> torsi SATU roda.
Kp_psi = 10.91          # [N m/rad]
Kd_psi = 2.31           # [N m s/rad]
Kp_theta = 0.10         # [N m/rad]    hanya aktif di mode Balancing (Tombol B)
Kd_theta = 0.13         # [N m s/rad]
Kvel = -0.05            # [N m]        hanya aktif di mode Navigate (Tombol A)

# =============================================================================
#                              5. CBF-QP  (= simulasi)
# =============================================================================
Alpha_1 = 54.73         # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 44.85         # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 55.32         # [1/s]  h3, h4 (dpsi, CBF orde 1)
Alpha_4 = 33.34         # [1/s]  h5, h6 (jarak s, HOCBF orde 2), hanya mode Balancing
Alpha_5 = 80.97         # [1/s]  h5, h6 (jarak s, HOCBF orde 2), hanya mode Balancing
Alpha_6 = 485.31        # [1/s]  h7, h8 (dtheta, CBF orde 1)

# Semua Alpha_<nomor> berurutan di atas (otomatis dipakai cbf_qp.py dan ise.py).
NAMA_ALPHA = []
while ("Alpha_%d" % (len(NAMA_ALPHA) + 1)) in globals():
    NAMA_ALPHA.append("Alpha_%d" % (len(NAMA_ALPHA) + 1))

CBF_QP_SOLVER = "qpoases"           # QP solver CBF-QP (plugin ca.conic CasADi), sudah ikut di paket pip casadi
CBF_INFTY = 1.0e9                   # batas atas "tak hingga" constraint c di QP
CBF_TOL_CONSTRAINT = 1.0e-6         # pelanggaran constraint yang masih diterima saat memeriksa hasil solver
CBF_TOL_BATAS_U = 1.0e-6            # [N m]
CBF_TOL_AKTIF = 1.0e-4              # [N m] |u* - u_PD| di atas ini -> filter dianggap mengoreksi
CBF_TOL_KOEFISIEN = 1.0e-12

# Torsi roda nyata per A arus perintah. Gain dan Alpha di atas dituning dengan 0.75 (profil DATASHEET).
# ASUMSI: torsi roda fisik = datasheet. Uji delay 20261009 memberi ~0.238; bila itu yang benar, isi 0.238 di sini
# (CBF-QP ikut memakai torsi itu) dan pakai gain hasil tuning profil TERUKUR.
MOTOR_KT_EFEKTIF = 0.75             # [N m/A]  (RASIO_TORSI dihitung di akhir file)

# =============================================================================
#                              6. MOTOR DDSM115  (wiki Waveshare DDSM115)
# =============================================================================
LEFT_MOTOR_PORT = "/dev/ttyACM0"
RIGHT_MOTOR_PORT = "/dev/ttyACM1"
LEFT_MOTOR_ID = 1                   # tiap motor punya USB-RS485 sendiri, ID boleh sama
RIGHT_MOTOR_ID = 1
MOTOR_BAUDRATE = 115200             # 8N1, frame 10 byte, CRC-8/MAXIM, maks 500 Hz
MOTOR_TIMEOUT_S = 0.03
MOTOR_MAX_COMM_ERRORS = 5           # gagal komunikasi berturut-turut sebelum motor dianggap putus
MOTOR_RETRY_S = 2.0                 # jeda coba-ulang motor yang bermasalah (saat roda mati)
LEFT_MOTOR_SIGN = -1.0              # arus positif (kerangka robot) = roda ke DEPAN
RIGHT_MOTOR_SIGN = 1.0
ENCODER_DIRECTION = -1.0            # uji inersia roda: arus positif -> posisi encoder menurun
ENCODER_RELATIVE_TO_BODY = True     # encoder mengukur roda relatif terhadap badan: theta = encoder + psi
DTHETA_SOURCE = "posisi"            # "posisi" (turunan encoder, disarankan) atau "rpm" (tertinggal ~0.1 s)
DTHETA_FILTER_TAU_S = 0.020         # [s] low-pass dtheta

MOTOR_KT = 0.75                     # [N m/A] datasheet
MOTOR_CURRENT_FULL_SCALE_A = 8.0    # current loop: -32767..32767 <-> -8..8 A
MOTOR_RAW_FULL_SCALE = 32767
MOTOR_POSITION_COUNTS = 32768       # posisi 0..32767 <-> 0..360 deg
MAX_CURRENT_A = 2.00                # [A] per roda (rated 1.5 A, locked-rotor <= 2.7 A), sama dengan simulasi
MIN_CURRENT_A = -MAX_CURRENT_A
u_max = MOTOR_KT * MAX_CURRENT_A    # [N m] torsi satu roda
u_min = MOTOR_KT * MIN_CURRENT_A

# ASUMSI: torsi tambahan untuk belok [N m]: roda kiri u + TORSI_BELOK*belok, roda kanan u - TORSI_BELOK*belok.
TORSI_BELOK = 0.02

# =============================================================================
#                              7. IMU MPU6050
# =============================================================================
I2C_BUS = 1
MPU6050_ADDRESS = 0x68
IMU_AXIS = "pitch"                  # "pitch" (sumbu Y sensor) atau "roll" (sumbu X sensor)
IMU_SIGN = 1.0                      # balik ke -1.0 bila psi terbaca negatif saat badan miring ke depan
MPU6050_DLPF_CFG = 0                # register 0x1A: 0 = 260 Hz, 2 = 94 Hz, 3 = 44 Hz
GYRO_CALIBRATION_SAMPLES = 800
GYRO_CALIBRATION_DELAY_S = 0.002
ZERO_CALIBRATION_SAMPLES = 350
ZERO_CALIBRATION_DELAY_S = 0.004
IMU_RETRY_S = 1.0
IMU_FROZEN_SAMPLES = 40             # sampel identik berturut-turut -> IMU dianggap macet
IMU_CAL_MAX_GYRO_STD = 1.5          # [deg/s] di atas ini robot dianggap bergerak saat kalibrasi

# =============================================================================
#                              8. JOYSTICK  (Fantech WGP-13s)
# =============================================================================
JOYSTICK_PORT = "/dev/input/event5"
JOYSTICK_RECONNECT_S = 1.0
JOYSTICK_AMBANG_ANALOG = 0.5        # analog dianggap ditekan bila simpangannya di atas nilai ini (0..1)
# ASUMSI: PAD Atas/Bawah (code 17) = maju/mundur, PAD Kiri/Kanan (code 16) = belok. Tukar bila ingin sebaliknya.
DPAD_MAJU_CODE = 17
DPAD_BELOK_CODE = 16

# =============================================================================
#                              9. RUNTIME, GUI, DAN ISE
# =============================================================================
CONTROL_HZ = 200.0                  # sama dengan simulasi (DDSM115 maks 500 Hz)
DT = 1.0 / CONTROL_HZ               # [s]
UI_HZ = 10.0
LOOP_MIN_RASIO = 0.8                # loop di bawah 80 % CONTROL_HZ -> peringatan
LOG_FILE = "atera.log"
ISE_PLOT_DIR = "PLOT_EVALUASI/ATERA_RASPI"   # hasil robot fisik: <jenis>/<PD atau CBFQP>/ (lihat ise.py)
DURASI_PENGUJIAN = 30.0             # [s] lama satu PENGUJIAN (L3), sama dengan simulasi
# ASUMSI: robot dianggap pulih setelah gangguan bila |psi| < EVAL_PSI_PULIH_DEG terus-menerus EVAL_PULIH_LAMA_S detik
EVAL_PSI_PULIH_DEG = 2.0            # [deg]
EVAL_PULIH_LAMA_S = 1.0             # [s]

# Warna garis batas di gambar ise.py dan plot realtime:
#   merah  = constraint barrier CBF-QP (psi_max, dpsi_max, dtheta_max, s_max)
#   hitam  = batas KOMET (SAFE_TILT_DEG dan S_KOMET)
#   kuning / oranye = penanda |psi| 5 deg / 10 deg (hanya gambar, tidak dipakai kontrol)
PLOT_SAFETY_WARNA = "red"
PLOT_KOMET_WARNA = "black"
PLOT_PENANDA_PSI = [(5.0, "gold"), (10.0, "darkorange")]  # (derajat, warna); "gold" = kuning yang terbaca di latar putih
# Batas sumbu tetap semua gambar: psi [rad], dpsi [rad/s], theta [rad], dtheta [rad/s], s [m]  (theta 60 rad = s 3 m)
PLOT_BATAS = {"psi": 1.0, "dpsi": 10.0, "theta": 60.0, "dtheta": 30.0, "s": 3.0}
PLOT_SAFETY_TEBAL = 2.2             # garis yang membentuk kotak barrier (menyambung)
PLOT_SAFETY_TIPIS = 1.0             # perpanjangan garis di luar kotak (putus-putus)
PLOT_KOMET_TEBAL = 1.6
# Barrier yang digambar di plot: nama -> (nomor h batas atas, nomor h batas bawah) di DAFTAR_C cbf_qp.py.
# Batas yang constraint-nya tidak aktif (tidak dicentang / tidak dipakai mode itu) digambar abu-abu tipis.
CBF_BARRIER_H = {"psi": (1, 2), "dpsi": (3, 4), "s": (5, 6), "dtheta": (7, 8)}
PLOT_NONAKTIF_WARNA = "0.6"
PLOT_WARNA_PD = "tab:purple"        # kurva mode PD (ungu)
PLOT_WARNA_PD_CBF = "tab:blue"      # kurva mode PD + CBF-QP (biru)

# =============================================================================
# Turunan (jangan diubah)
# =============================================================================
RASIO_TORSI = MOTOR_KT_EFEKTIF / MOTOR_KT     # torsi nyata / torsi perintah, dipakai g(x) di CBF-QP