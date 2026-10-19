"""config.py - satu-satunya sumber nilai program ATERA (konstanta fisik, gain, batas, pengaturan simulasi).

Satuan SI (kg, m, s, rad, N m) kecuali nama berakhiran _DEG, _A, _HZ, _RPM.

Konvensi tanda (dipakai model.py dan semua file lain):
    state x = [theta, dtheta, psi, dpsi]
    psi   > 0 : badan miring ke DEPAN, diukur dari garis vertikal (gravitasi)
    theta > 0 : roda menggelinding ke DEPAN, sudut roda absolut (definisinya ada di model.hitung_theta)
    zeta  > 0 : bidang menanjak ke arah DEPAN
    u     > 0 : torsi motor SATU roda, mendorong roda ke DEPAN (reaksinya mendorong badan ke belakang)
            kedua roda diberi u yang sama, jadi torsi total = N_WHEEL*u

Mode gerak (pd_control.py: MODE_DIAM, MODE_GERAK, MODE_BEBAS):
    DIAM  : W / S tidak ditahan. Robot ditahan di setpoint theta (Kp_theta), constraint s dan KOMET s aktif.
    GERAK : W / S ditahan. Kp_theta dan constraint s diabaikan, Kvel*arah menggerakkan robot.
    BEBAS : tombol F (toggle, robot fisik: Tombol A). Tanpa PD heading, Kp_theta dan constraint s diabaikan, belok =
            selisih torsi TORSI_BELOK (seperti program lama).
Setpoint theta (titik nol jarak s, s = R*theta) dipasang saat W / S dilepas, oleh tombol B, dan saat BEBAS dimatikan.
"""

import math

# =============================================================================
#                              1. PARAMETER FISIK
# =============================================================================
m_r = 0.745            # [kg]      massa SATU roda (dua roda = 1.490 kg)
m_b = 2.225            # [kg]      massa badan (timbangan: total 3.715 kg - 2 x 0.745 kg)
R = 0.050              # [m]       jari-jari roda
L = 0.100408           # [m]       jarak CoM kedua roda (poros) ke CoM badan, diukur di Fusion 360 ATERA v40
# Geseran depan-belakang CoM badan terhadap poros = 0.419 mm (Z Delta), diabaikan di model.
j_theta = 9.2e-4       # [kg m^2]  momen inersia roda (bagian berputar, satu roda)
# BUTUH PENGUJIAN BEBAN TAMBAHAN (ddsm115_inersia_testing.py --j-add --g-ref): nilai j_theta dihitung dari
# arus umpan balik x 0.75, skala torsinya belum terverifikasi.
j_psi = 0.02337        # [kg m^2]  momen inersia badan terhadap CoM badan (sumbu sejajar poros roda)
# ASUMSI: j_psi = inersia badan Fusion (0.02090) x 2.225 / 1.990131, lihat catatan di ATERA_URDF.urdf.xacro.
# BUTUH PENGUJIAN BANDUL ULANG: data bandul 20261004 memberi inersia negatif (tidak mungkin), jadi nilai dari
# Fusion ini belum terverifikasi dengan pengukuran.
g = 9.81               # [m/s^2]

# ASUMSI: j_theta adalah inersia SATU roda (cocok dengan 1/2*m_r*R^2 = 9.3e-4), model memakai 2*j_theta.
# ASUMSI: j_psi dihitung terhadap PUSAT MASSA badan, model menambahkan m_b*L^2.
N_WHEEL = 2

zeta_DEG = 0.0                      # [deg] 0.0 (datar) atau 8.0 (miring)
zeta = math.radians(zeta_DEG)       # [rad]

# =============================================================================
#                              2. INDEKS STATE  (<kontrak>)
# =============================================================================
IDX_THETA = 0
IDX_DTHETA = 1
IDX_PSI = 2
IDX_DPSI = 3
NX = 4
NU = 1

# =============================================================================
#                              3. SETPOINT
# =============================================================================
theta_setpoint = 0.0                # [rad]    theta diukur dari setpoint theta (titik nol s): B, W / S dilepas, R
psi_setpoint = 0.0                  # [rad]
dpsi_setpoint = 0.0                 # [rad/s]
dtheta_setpoint = 0.0               # [rad/s]

# =============================================================================
#                              4. SAFETY SET DAN KOMET
# =============================================================================
# Safety set (batas simetris: min = -max)
psi_max_DEG = 10.0                  # [deg]    batas psi yang dijaga CBF (h1, h2); bisa diubah di jendela Kontrol
                                    #          (= robot fisik; 10 deg supaya koreksi CBF-QP terlihat jelas dibanding PD)
psi_max = math.radians(psi_max_DEG) # [rad]    |psi|    <= psi_max
dpsi_max = 2.0 * math.pi            # [rad/s]  |dpsi|   <= dpsi_max
# ASUMSI: sedikit di bawah batas kecepatan motor DDSM115 (no-load 200 rpm = 20.9 rad/s, wiki Waveshare), = robot fisik.
# Dengan 13 rad/s h7/h8 melarang roda mengejar badan setelah dorongan ~10 N (PD butuh ~19 rad/s) -> robot jatuh.
dtheta_max = 20.0                   # [rad/s]  |dtheta| <= dtheta_max
psi_min = -psi_max
dpsi_min = -dpsi_max
dtheta_min = -dtheta_max
s_max = 0.5                         # [m]      jarak tempuh s = R*theta (dari titik nol s), |s| <= s_max
                                    #          (0.5 m = robot fisik; barrier terlihat dan robot tidak lari jauh)
s_min = -s_max

# KOMET = kondisi robot yang sudah tidak tertolong, berlaku di mode PD maupun PD+CBF. Roda dimatikan.
#   Jatuh    : |psi| > SAFE_TILT_DEG                (semua mode gerak)
#   Lari     : |s|   > S_KOMET                      (hanya mode DIAM; saat GERAK / BEBAS robot memang berjalan jauh)
#   Berputar : |yaw - yaw_ref| > YAW_KOMET_DEG      (mode DIAM dan GERAK; di mode BEBAS yaw memang bebas)
SAFE_TILT_DEG = 40.0                # [deg]  ASUMSI: dipilih pengguna (20261014)
S_KOMET = 2.0                       # [m]
# KOMET berputar: s memakai RATA-RATA kedua roda, jadi robot yang berputar di tempat tidak pernah mencapai
# |s| > S_KOMET. ASUMSI: 90 deg (sama dengan config robot fisik).
YAW_KOMET_DEG = 90.0                # [deg]

# =============================================================================
#                              5. PD
# =============================================================================
# u_PD = BALANCE_DIRECTION_SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi
#                                  + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
# error = setpoint - state,   arah = -1 / 0 / +1 dari W / S (robot fisik: D-Pad atas / bawah)
Kp_psi = 09.69       #33.15              # [N m/rad]    semua gain menghasilkan torsi SATU roda
Kd_psi = 01.83       #5.2                # [N m s/rad]
Kp_theta = 0.03      #0.24             # [N m/rad]    hanya mode DIAM (diabaikan saat GERAK / BEBAS)
Kd_theta = 0.25      #0.41             # [N m s/rad]
# Kvel*arah hanya mode GERAK / BEBAS. Kecepatan tunak di lantai datar: v = -R*Kvel*arah/Kd_theta
# -> Kvel NEGATIF supaya W maju (Kd_theta 0.13: Kvel -0.26 -> 0.10 m/s, -0.52 -> 0.20 m/s, -0.78 -> 0.30 m/s).
# ASUMSI: -0.52 (0.20 m/s), sama dengan robot fisik.
Kvel = -0.99                # [N m]

# Dengan error = setpoint - state dan u > 0 = roda ke depan, gain positif menstabilkan bila tandanya -1
# (sama dengan apalah_bisa/tunning.py).
BALANCE_DIRECTION_SIGN = -1.0
# BUTUH PENGUJIAN ARAH KOREKSI: gain kecil, badan dimiringkan ke depan, roda harus berputar ke DEPAN.

# =============================================================================
#                              5b. PD HEADING (roda kiri dan kanan bergerak bersama)  (= robot fisik 4b)
# =============================================================================
# yaw = R*(theta_kanan - theta_kiri)/W_RODA (+ = berputar ke KIRI dilihat dari atas)
# u_belok = Kp_yaw*(yaw - yaw_ref) + Kd_yaw*(dyaw - dyaw_ref); roda kiri u + u_belok, roda kanan u - u_belok
#   A / D dilepas : yaw_ref tetap.   A / D ditahan : dyaw_ref = -belok*YAW_RATE_BELOK (saat DIAM = berputar di tempat).
W_RODA = 0.258                      # [m] tengah ban kiri ke tengah ban kanan (diukur pengguna)
YAW_AKTIF = True
Kp_yaw = 0.30                       # [N m/rad]     ASUMSI (lihat robot fisik config.py 4b)
Kd_yaw = 0.10                       # [N m s/rad]
YAW_RATE_BELOK = 1.0                # [rad/s] kecepatan belok A / D
A_YAW_MAKS = 3.0                    # [rad/s^2]
YAW_TORSI_MAKS = 0.40               # [N m]
# Mode BEBAS (tombol F): PD heading mati, belok A / D = selisih torsi tetap (= robot fisik 4b). ASUMSI: 0.02 N m.
TORSI_BELOK = 0.02                  # [N m]

# =============================================================================
#                              6. CBF-QP
# =============================================================================
# Gain (input manual, default 0; alpha_i(h) = Alpha_i * h)
Alpha_1 = 35.15 #23.95          # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 52.64 #20.09          # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 46.67 #51.74          # [1/s]  h3, h4 (dpsi, CBF orde 1)
Alpha_4 = 36.95 #11.52          # [1/s]  h5, h6 (jarak tempuh s, HOCBF orde 2), hanya mode DIAM
Alpha_5 = 50.18 #35.41          # [1/s]  h5, h6 (jarak tempuh s, HOCBF orde 2), hanya mode DIAM
Alpha_6 = 223.09 #84.67         # [1/s]  h7, h8 (dtheta, CBF orde 1)

# Nama semua Alpha di atas: Alpha_1, Alpha_2, ... (berurutan, berhenti di nomor pertama yang tidak ada).
# Alpha baru cukup ditambahkan di atas, lalu otomatis dipakai cbf_qp.py, simulasi.py (jendela Kontrol), dan ise.py (kolom CSV).
NAMA_ALPHA = []
while ("Alpha_%d" % (len(NAMA_ALPHA) + 1)) in globals():
    NAMA_ALPHA.append("Alpha_%d" % (len(NAMA_ALPHA) + 1))

# QP solver CBF-QP (plugin ca.conic CasADi). "qpoases" = active-set, sudah ikut di paket pip casadi.
CBF_QP_SOLVER = "qpoases"
# pilihan lain: "eksak" (solusi pasti numpy), "osqp", "proxqp", "qrqp", "hpipm", "daqp", "highs", "ipqp".
# Bandingkan:  python3 cbf_qp.py --banding-solver
CBF_QP_SOLVER_BANDING = ["eksak", "qpoases", "osqp", "proxqp", "qrqp", "hpipm", "daqp", "highs", "ipqp"]
# Slack, margin, DOB (= robot fisik, penjelasan lengkap di cbf_qp.py dan program/atera_program/config.py)
CBF_SLACK_AKTIF = True
# bobot slack per constraint (exact penalty, urutan DAFTAR_C h1..h8): yang kecil dilonggarkan dulu bila bertabrakan
CBF_RHO_SLACK = [1.0e6, 1.0e6, 1.0e4, 1.0e4, 1.0e2, 1.0e2, 1.0e2, 1.0e2]    # h1 h2 | h3 h4 | h5 h6 | h7 h8
CBF_SLACK_EPS = 1.0e-4
CBF_TOL_SLACK = 1.0e-6
CBF_MARGIN_DEG = 1.0                # [deg] barrier psi = psi_max - margin (garis merah plot tetap di psi_max)
DOB_AKTIF = True                    # True: d_hat dipakai CBF-QP. False: hanya dicatat
DOB_TAU_S = 0.020                   # [s]
DOB_BATAS = 200.0                   # [rad/s^2]
# False = SAMA DENGAN ROBOT FISIK: CBF-QP TIDAK diberi tahu kemiringan lantai (model CBF memakai zeta = 0, seperti
#         robot dengan zeta_DEG = 0), gangguan kemiringan ditaksir DOB. Dipakai simulasi (atera_main.py) dan
#         tuning_optuna.py, jadi gain dan Alpha hasil tuning langsung berlaku di robot pada kemiringan berapa pun.
# True  = CBF-QP memakai kemiringan lantai simulasi config.zeta (model tepat, tidak bisa dilakukan di robot).
CBF_TAHU_ZETA = False

# Toleransi numerik (bukan besaran fisik)
CBF_INFTY = 1.0e9                   # batas atas "tak hingga" constraint c di QP
CBF_TOL_CONSTRAINT = 1.0e-6         # pelanggaran constraint yang masih diterima saat memeriksa hasil solver
CBF_TOL_BATAS_U = 1.0e-6            # [N m] pelanggaran batas u yang masih diterima
CBF_TOL_AKTIF = 1.0e-4              # [N m] |u* - u_PD| di atas ini -> filter dianggap mengoreksi
CBF_TOL_KOEFISIEN = 1.0e-12         # |koefisien u| di bawah ini dianggap nol

# =============================================================================
#                              7. MOTOR DDSM115  (wiki Waveshare DDSM115)
# =============================================================================
LEFT_MOTOR_PORT = "/dev/ttyACM0"
RIGHT_MOTOR_PORT = "/dev/ttyACM1"
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 1
MOTOR_BAUDRATE = 115200             # 8N1, frame 10 byte, CRC-8/MAXIM
MOTOR_TIMEOUT_S = 0.03
MOTOR_COMM_MAX_HZ = 500.0
LEFT_MOTOR_SIGN = -1.0              # dari apalah_bisa/tunning.py
RIGHT_MOTOR_SIGN = 1.0
# BUTUH PENGUJIAN ARAH RODA: robot diangkat, u > 0 harus memutar KEDUA roda ke DEPAN.

# True: encoder mengukur roda relatif terhadap badan, theta = theta_encoder + psi (lihat model.hitung_theta).
ENCODER_RELATIVE_TO_BODY = True
# BUTUH PENGUJIAN ENCODER: arus 0, tahan roda di lantai, miringkan badan 10 deg dengan tangan;
# posisi encoder berubah ~10 deg = True, tidak berubah = False.

MOTOR_KT = 0.75                     # [N m/A]  torque constant (datasheet)
# BUTUH PENGUJIAN BEBAN TAMBAHAN (ddsm115_inersia_testing.py --j-add --g-ref): uji inersia 20261004 memberi
# G = K/J = 260 rad/s^2 per A perintah, yaitu K = 0.24 N m/A untuk j_theta = 9.2e-4 (bukan 0.75).
MOTOR_RATED_TORQUE = 0.96           # [N m]
MOTOR_RATED_CURRENT_A = 1.5         # [A]
MOTOR_STALL_TORQUE = 2.0            # [N m]    locked-rotor torque
MOTOR_STALL_CURRENT_A = 2.7         # [A]      locked-rotor current
MOTOR_NO_LOAD_SPEED_RPM = 200.0     # [rpm]    +-10
MOTOR_CURRENT_FULL_SCALE_A = 8.0    # current loop: -32767..32767 <-> -8..8 A
MOTOR_RAW_FULL_SCALE = 32767
MOTOR_POSITION_COUNTS = 32767       # posisi 0..32767 <-> 0..360 deg

# ASUMSI: batas arus per roda mengikuti apalah_bisa/tunning.py (di atas rated 1.5 A, di bawah stall 2.7 A).
MAX_CURRENT_A = 2.00                # [A] per roda
# BUTUH PENGUJIAN ARUS MAKSIMUM: jalankan balancing di 1.80 A, pantau error code dan suhu DDSM115.
MIN_CURRENT_A = -MAX_CURRENT_A

# Arus per roda:  i = u / MOTOR_KT
# Batas arus pada CBF-QP:  MIN_CURRENT_A <= u / MOTOR_KT <= MAX_CURRENT_A  <=>  u_min <= u <= u_max
u_max = MOTOR_KT * MAX_CURRENT_A                      # [N m] torsi satu roda
u_min = MOTOR_KT * MIN_CURRENT_A                      # [N m]

# =============================================================================
#                              8. IMU MPU6050  (datasheet InvenSense MPU-6000/6050) DAN KALMAN
# =============================================================================
I2C_BUS = 1
MPU6050_ADDRESS = 0x68
IMU_AXIS = "pitch"
IMU_SIGN = 1.0
L_IMU = 0.312739                    # [m] jarak CoM kedua roda (poros) ke IMU, diukur di Fusion 360
# BUTUH PENGUJIAN ARAH IMU: badan dimiringkan ke DEPAN, psi harus terbaca POSITIF.
# FS_SEL 0/1/2/3 = +-250/500/1000/2000 deg/s. dpsi_max = 2*pi rad/s = 360 deg/s, jadi minimal FS_SEL = 1.
MPU6050_GYRO_FS_SEL = 1
MPU6050_GYRO_LSB_PER_DEG_S = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}[MPU6050_GYRO_FS_SEL]
# AFS_SEL 0/1/2/3 = +-2/4/8/16 g
MPU6050_ACCEL_AFS_SEL = 0
MPU6050_ACCEL_LSB_PER_G = {0: 16384.0, 1: 8192.0, 2: 4096.0, 3: 2048.0}[MPU6050_ACCEL_AFS_SEL]
GYRO_CALIBRATION_SAMPLES = 800
GYRO_CALIBRATION_DELAY_S = 0.002
ZERO_CALIBRATION_SAMPLES = 350
ZERO_CALIBRATION_DELAY_S = 0.004

# =============================================================================
#                              9. JOYSTICK  (Fantech WGP-13s)
# =============================================================================
JOYSTICK_PORT = "/dev/input/event5"
JOYSTICK_AMBANG_ANALOG = 0.5        # analog dianggap ditekan bila simpangannya di atas nilai ini (0..1)

# =============================================================================
#                              10. RUNTIME DAN EVALUASI ISE
# =============================================================================
CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ               # [s]
UI_HZ = 20.0
ISE_PLOT_DIR = "PLOT_EVALUASI"

# ASUMSI: satu pengujian (tombol T) = DURASI_PENGUJIAN detik waktu simulasi sejak dorongan O / P dimulai.
DURASI_PENGUJIAN = 30.0             # [s]
# zeta baru dipakai bila nilainya di jendela Kontrol tidak berubah selama ini (supaya robot tidak diletakkan
# ulang berkali-kali saat angkanya masih diketik).
ZETA_TUNGGU = 0.5                   # [s]

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
# 11. REALISME SIMULASI  (hanya dipakai simulasi.py, tidak dipakai robot fisik)
# =============================================================================
# Hasil testing/DELAY/atera_delay_testing.py, 20261009.
# K efektif = G x j_theta, G = 255 (kiri) dan 264 (kanan) rad/s^2 per A -> 0.234 dan 0.243 N m/A.
# ASUMSI: benar bila j_theta = 9.2e-4 (lihat BUTUH PENGUJIAN BEBAN TAMBAHAN di atas).
MOTOR_KT_EFEKTIF = 0.75 #0.238            # [N m/A] torsi roda nyata per A arus perintah (rata-rata kiri dan kanan)
# torsi nyata / torsi perintah. Dipakai simulasi.py (torsi roda di PyBullet) DAN cbf_qp.py (g(x) di model CBF-QP),
# supaya CBF-QP menghitung dengan tenaga roda yang sama dengan yang dipakai simulasi.
RASIO_TORSI = MOTOR_KT_EFEKTIF / MOTOR_KT
# Uji loop: perintah terkirim 1.8 ms setelah IMU dibaca (+ hitungan PD / CBF-QP), torsi mulai 0..4 ms kemudian.
# ASUMSI: dibulatkan menjadi 1 periode kontrol.
SIM_JEDA_TORSI = 0.005              # [s] jeda dari state dibaca sampai torsi bekerja (kelipatan 1 ms)
# Batas arus bergantung kecepatan motor (back-EMF), digabung dengan batas arus MAX_CURRENT_A:
#   i_maju_maks(w)   = min(MAX_CURRENT_A, MOTOR_STALL_CURRENT_A * (1 - w / w_noload))   (>= 0)
#   i_mundur_maks(w) = min(MAX_CURRENT_A, MOTOR_STALL_CURRENT_A * (1 + w / w_noload))   (>= 0)
#   w = kecepatan roda relatif badan (= putaran motor), w_noload = MOTOR_NO_LOAD_SPEED_RPM.
# Jadi makin cepat roda, makin kecil arus yang bisa masuk, nol di ~20.9 rad/s (data robot 20261015: |dtheta|
# tidak pernah > ~21 rad/s). Tanpa ini (False) roda simulasi bisa berputar secepat apa pun (motor ideal).
SIM_BATAS_KECEPATAN = True
SIM_MODEL_IMU = True                # True: psi dari model MPU6050 + Kalman, False: psi sempurna
# Noise saat robot diam (0.5 s pertama delay_imu_20261009_134728.csv).
SIM_GYRO_NOISE_DEG_S = 0.10         # [deg/s] simpangan baku gyro
SIM_ACC_NOISE_DEG = 0.14            # [deg]   simpangan baku sudut akselerometer

# =============================================================================
#                              12. PYBULLET (simulasi.py)
# =============================================================================
SUBSTEP = 5                 # jumlah langkah fisika dalam satu siklus kontrol (DT)
GAYA_DORONG_AWAL = 5.0      # [N] besar gaya dorongan (tombol O / P dan tuning_optuna.py), bisa diubah di jendela Kontrol

# >>>>>>>>>> DORONGAN = PUKULAN BANDUL (UBAH DI SINI) <<<<<<<<<<
# Simulasi memberi gaya konstan GAYA selama LAMA_DORONG, jadi impuls ke robot J = GAYA x LAMA_DORONG [N s].
# Konversi dari pengujian bandul fisik ke gaya simulasi:
#     dz  = l_bd * (1 - cos(phi_0))              tinggi jatuh titik berat bandul [m]
#     v_1 = sqrt(2 * g * dz)                     kecepatan bandul sesaat sebelum memukul [m/s]
#     v_2 = +/- sqrt(2 * g * l_bd * (1 - cos(phi_1)))   sesaat sesudah memukul (+ terus maju, - memantul) [m/s]
#     J_bd = m_bd * (v_1 - v_2)                  impuls yang diterima robot [N s]
#     F_bd = J_bd / LAMA_DORONG                  gaya simulasi (gaya_dorong) yang setara [N]
#   m_bd  : massa bandul [kg]                    l_bd  : panjang tali, titik gantung ke titik berat bandul [m]
#   phi_0 : sudut lepas dari vertikal [rad]      phi_1 : sudut ayun bandul setelah memukul (dari video) [rad]
# Contoh: m_bd 2 kg, l_bd 0.5125 m, phi_0 30 deg -> v_1 = 1.161 m/s; bila phi_1 = 17 deg (terus maju)
#         v_2 = 0.663 m/s, J_bd = 1.00 N s, F_bd = 1.00 / 0.015 = 66.4 N.
# ASUMSI: lama kontak pukulan bandul ~15 ms (tidak diukur). Boleh dipakai karena jauh lebih singkat dari skala
# waktu jatuh badan sqrt((m_b L^2 + j_psi) / (m_b g L)) ~ 0.145 s, sehingga yang menentukan robot jatuh hanya J.
# Rujukan: OpenStax University Physics Vol. 1 bab 9.2 (J = F_rata-rata x dt); Hessel dkk. 2006 dan Nikolov 2020
# (lama kontak tumbukan dalam orde milidetik). Nilai lama 0.10 s memberi batas ~12.5 N (= 1.25 N s).
LAMA_DORONG = 0.015         # [s] lama gaya dorongan bekerja (= lama kontak pukulan bandul)

# Titik dorong: bidang DORONG_SISI x DORONG_SISI di sisi luar bagian ATAS badan.
#   - tepi atas bidang = permukaan teratas badan (dihitung otomatis dari robot.urdf)
#   - jarak bidang dari garis tengah badan = JARAK_IMU_KE_SISI_LUAR (sisi luar badan)
#   - O (ke depan)    : bidang di sisi BELAKANG badan, gaya ke arah depan badan
#   - P (ke belakang) : bidang di sisi DEPAN badan, gaya ke arah belakang badan
#   - gaya dibagi rata ke DORONG_TITIK x DORONG_TITIK titik di bidang itu, tegak lurus permukaan badan
# ASUMSI: bidang berada di tengah lebar badan (DORONG_GESER_SAMPING = 0), sisi depan dan belakang sama.
DORONG_SISI = 0.040         # [m] 40 mm x 40 mm
DORONG_TITIK = 5            # titik per sisi (5 x 5 = 25 titik)
DORONG_GESER_SAMPING = 0.0  # [m] geser bidang searah poros roda (+ = ke arah roda kiri)
DORONG_WARNA = [0.10, 0.75, 0.25, 0.30]
DORONG_WARNA_AKTIF = [0.10, 0.75, 0.25, 0.90]
GESEKAN_RODA = 1.0          # koefisien gesek roda dengan lantai
GESEKAN_PUTAR = 0.003       # gesekan roda saat robot berputar di tempat (yaw), supaya belok berhenti saat dilepas
PSI_AWAL_DEG = 0.0          # [deg] kemiringan badan saat robot diletakkan
URDF_PILIHAN = ["urdf/robot.urdf", "urdf/urdf/robot.urdf", "../urdf/urdf/robot.urdf"]   # relatif terhadap simulasi.py

# Kamera
KAMERA_JARAK = 0.9          # [m]
KAMERA_YAW = 60.0           # [deg]
KAMERA_PITCH = -15.0        # [deg]
KAMERA_BATAS_GESER = 0.25   # [m] kamera baru digeser bila robot sudah sejauh ini dari titik tengah layar

# Bidang merah safety set (batas psi)
# Bentuk | | : dua bidang (persegi panjang) TEGAK di depan dan di belakang robot, ikut berpindah bersama robot.
# Letaknya dipilih supaya sisi terluar badan di ketinggian IMU tepat menyentuh bidang saat psi = +-psi_max:
#     jarak mendatar dari poros = L_IMU*sin(psi_max) + JARAK_IMU_KE_SISI_LUAR*cos(psi_max)
# ASUMSI: jarak IMU ke sisi terluar badan sama untuk sisi depan dan sisi belakang.
JARAK_IMU_KE_SISI_LUAR = 0.044199   # [m] diukur di Fusion 360
BIDANG_LEBAR = 0.36         # [m] searah poros roda
BIDANG_TINGGI = 0.42        # [m] dari lantai ke atas
BIDANG_WARNA = [1.0, 0.0, 0.0, 0.22]
BIDANG_WARNA_LANGGAR = [1.0, 0.0, 0.0, 0.60]       # lebih gelap saat psi melewati batas

# Kotak biru safety set jarak tempuh s (s_min <= s <= s_max, s = R*theta dari titik nol s).
# Panjang kotak searah lintasan = s_max - s_min, lebar dan tinggi di bawah.
# Kotak diam di dunia (tidak ikut robot), diletakkan di posisi awal robot, mengikuti kemiringan lantai.
KUBUS_LEBAR = 1.0           # [m] searah poros roda
KUBUS_TINGGI = 1.0          # [m] tegak lurus lantai
KUBUS_WARNA = [0.10, 0.40, 1.0, 0.20]
KUBUS_WARNA_LANGGAR = [0.10, 0.40, 1.0, 0.45]      # lebih gelap saat s melewati batas

# Teks di jendela PyBullet
WARNA_BIASA = [0.0, 0.0, 0.0]
WARNA_PERINGATAN = [0.85, 0.30, 0.0]
JARAK_TEKS = 0.10           # [m] jarak bidang teks dari kamera (dekat kamera supaya tidak tertutup lantai / robot)
JARAK_BARIS = 0.055         # jarak antar baris teks parameter (1.0 = setengah tinggi layar)
BARIS_PER_TAMPIL = 1        # jumlah baris parameter yang ditulis ulang tiap tampilkan() dipanggil.
                            # Satu kali menulis teks di PyBullet memakan waktu sekitar satu frame layar,
                            # jadi baris ditulis bergiliran supaya simulasi tidak melambat.
JUMLAH_BARIS_MAKS = 8

# Plot realtime (jendela matplotlib, hanya dengan --plot)
PLOT_RENTANG = 10.0         # [s] lebar jendela waktu yang ditampilkan
PLOT_SETIAP = 2             # plot digambar ulang tiap sekian kali tampilkan() dipanggil
PLOT_BACKEND = ["QtAgg", "Qt5Agg", "TkAgg"]

# Jendela Kontrol (Qt)
WARNA_QT_TEKAN = "#1d9e5a"       # tombol sedang ditekan
WARNA_QT_MODE = "#544bb8"        # mode BEBAS sedang aktif (tombol F)
WARNA_QT_UJI = "#c26a00"         # mode pengujian aktif
WARNA_QT_PD = "#9467bd"          # tombol mode kontrol PD aktif (= tab:purple)
WARNA_QT_PD_CBF = "#1f77b4"      # tombol mode kontrol PD + CBF-QP aktif (= tab:blue)
WARNA_QT_BANDING = "#b3261e"     # perekam perbandingan (tombol 3) aktif
# Jumlah desimal tampilan tiap parameter di jendela Kontrol: nama -> desimal. Semua Alpha_n memakai DESIMAL_ALPHA.
# Angka yang diketik boleh lebih teliti; tampilannya menyesuaikan.
DESIMAL = {"Kp_psi": 2, "Kd_psi": 3, "Kp_theta": 4, "Kd_theta": 4, "Kvel": 4, "Gaya_dorong": 2, "zeta_DEG": 1,
           "psi_max_DEG": 1}
DESIMAL_ALPHA = 2

# =============================================================================
#                              13. TUNING OPTUNA  (hanya dipakai tuning_optuna.py)
# =============================================================================
# Tuning BERTAHAP: satu study Optuna per tahap, dijalankan satu per satu (python3 tuning_optuna.py --tahap <NAMA>).
#   PD            Kp_psi, Kd_psi, Kp_theta, Kd_theta   semua gain PD sekaligus, cari gaya dorong tertinggi
#   CBFQP_PSI     Alpha_1, Alpha_2           barrier h1, h2
#   CBFQP_DPSI    Alpha_3                    barrier h1 .. h4  (Alpha_1, Alpha_2 dari config.py)
#   CBFQP_S       Alpha_4, Alpha_5           barrier h1 .. h6
#   CBFQP_DTHETA  Alpha_6                    barrier h1 .. h8
# Nilai yang tidak dituning di satu tahap SELALU diambil dari config.py: salin hasil tiap tahap ke config.py
# sebelum menjalankan tahap berikutnya.
# Tuning GABUNGAN (tahap SEMUA, terpisah dari tahap bertahap di atas): Kp_psi, Kd_psi, Kp_theta, Kd_theta,
#   Alpha_1 .. Alpha_6 dituning SEKALIGUS. Tiap trial menguji gain yang SAMA dua kali (DUA MODE):
#     1. PD saja (CBF-QP mati)       -> skor PD saja      (S1 dorongan + S2 bidang miring + S3 dorongan di miring)
#     2. PD + CBF-QP (h1 .. h8)      -> skor PD + CBF-QP  (S1 + S2 + S3, lihat blok "SKENARIO S1 S2 S3" di bawah)
#     skor trial = skor PD saja + skor PD + CBF-QP + OPTUNA_BOBOT_CBF_BURUK x max(0, skor PD + CBF-QP - skor PD saja)
#   Jadi gain PD harus bagus SENDIRIAN (dipakai juga di mode PD saja untuk perbandingan), dan trial yang PD + CBF-QP-nya
#   lebih buruk dari PD saja dihukum: Optuna mencari Alpha yang membuat CBF-QP tidak memperburuk PD.
#   Bandingkan hasilnya dengan hasil bertahap pada skenario yang sama:
#       python3 tuning_optuna.py --profil <PROFIL> --banding
# SYARAT LOLOS tiap pengujian (tidak jatuh, tidak chattering, tidak berayun / overshoot): lihat blok
#   ">>>>>>>>>>  SYARAT LOLOS SATU PENGUJIAN  (UBAH DI SINI)  <<<<<<<<<<" di bawah.
# Study sebuah tahap tidak boleh dilanjutkan bila nilai-nilai yang dikunci (atau skenario di bawah, termasuk syarat
# lolos dan fisika simulasi: jeda torsi, batas kecepatan motor, noise IMU, batas KOMET, ...) berubah; tuning_optuna.py
# menghentikan program dan menjelaskan bedanya.
#
# SATU PENGUJIAN = OPTUNA_ULANGAN kali skenario berikut, robot di-reset di awal tiap ulangan (lantai
# OPTUNA_ZETA_DEG, mode DIAM). Di dalam satu ulangan tidak ada reset:
#   1. Tenang awal  : robot diletakkan, ditunggu sampai STABIL (maks OPTUNA_DURASI_STABIL detik, tidak dinilai ISE).
#   2. Dorongan     : didorong sesuai urutan OPTUNA_ARAH_DORONG. Dorongan berikutnya LANGSUNG diberikan
#                     OPTUNA_DURASI_DORONGAN detik setelah dorongan sebelumnya, stabil atau belum (kondisi terburuk).
#   3. Setelah dorongan terakhir: ditunggu sampai STABIL (maks OPTUNA_DURASI_STABIL detik sejak dorongan itu).
#   Titik nol s = posisi awal tiap ulangan. Torsi roda mengikuti profil (OPTUNA_PROFIL di bawah).
# Ulangan ke-k (k = 0, 1, ...) dibuat berbeda supaya hasil tidak bergantung pada keberuntungan:
#   noise IMU memakai seed OPTUNA_SEED + k, dan dorongan pertama ditunda k x OPTUNA_GESER_DORONG detik setelah
#   tenang awal, jarak antar dorongan menjadi OPTUNA_DURASI_DORONGAN + k x OPTUNA_GESER_DORONG.
#   Skor pengujian = JUMLAH skor semua ulangan. Komet di satu ulangan menghentikan pengujian (ulangan sisanya tidak
#   dijalankan).
# STABIL = |psi| < OPTUNA_STABIL_PSI, |dpsi| < OPTUNA_STABIL_DPSI, |dtheta| < OPTUNA_STABIL_DTHETA, ketiganya
#   terus-menerus selama OPTUNA_STABIL_LAMA detik (psi hasil Kalman, sama dengan yang dibaca kontrol).
# Skor satu pengujian (makin kecil makin baik), dari state yang dibaca kontrol (psi hasil Kalman):
#   skor = integral[(psi/psi_max)^2 + (s/s_max)^2] dt                       [sejak dorongan pertama tiap ulangan]
#        + OPTUNA_PENALTI_JARAK x lama |s| > s_max                         [per detik, tahap dengan "penalti_jarak"]
#        + OPTUNA_PENALTI_GAGAL x lama status CBF bukan AMAN               [per detik, tahap CBFQP_* dan SEMUA]
#        + OPTUNA_PENALTI_KOMET x (1 + sisa waktu / waktu total)   bila KOMET (pengujian langsung berhenti; sisa dihitung dari seluruh ulangan)
# Tidak stabil TIDAK diberi penalti: hanya dicatat di detail. ISE sudah lebih besar bila robot lambat pulih,
# dan robot yang cepat stabil setelah dorongan terakhir mengakhiri ulangan (dan ISE-nya) lebih awal.
#
# Gaya dorong:
#   Tahap dengan "cari_gaya" (PD dan SEMUA): satu trial mencari gaya TERTINGGI yang masih lolos satu pengujian
#     (tanpa komet),
#     dengan bisection pada kisi gaya_dorong .. OPTUNA_GAYA_MAKS berjarak OPTUNA_GAYA_LANGKAH (gaya_dorong dari profil).
#     ASUMSI bisection: robot yang lolos pada gaya F juga lolos pada gaya yang lebih kecil.
#       skor trial = -OPTUNA_BOBOT_GAYA x gaya_lolos + skor pengujian pada gaya_lolos
#     Bobot dibuat besar supaya gaya lebih tinggi SELALU lebih baik; ISE hanya membedakan trial dengan gaya sama.
#     Bila gaya terendah pun komet: skor trial = skor pengujian pada gaya terendah (dengan penalti komet).
#   Tahap CBFQP_*: satu pengujian pada TIAP gaya OPTUNA_GAYA_CBF_FRAKSI x gaya_pd_terbaik (profil, diisi dari
#     hasil tahap PD); skor trial = jumlah skor semua gaya. Alpha dinilai dari dorongan kecil sampai besar.
# CHATTERING = sinyal kendali (torsi u) berpindah cepat bolak-balik di antara batas jenuhnya; robot bisa tampak diam
#   tetapi motor bergetar / berdengung dan panas. Diukur sebagai chattering_per_s (CSV) / ch/s (terminal) = berapa
#   kali per detik torsi u berbalik dari >= +a ke <= -a (atau sebaliknya), a = OPTUNA_AMBANG_CHATTERING x u_max.
#   Getaran kecil di sekitar 0 (noise) tidak dihitung. Angka besar (puluhan per detik) = chattering parah.
#   ch/s > OPTUNA_BATAS_CHATTERING = pengujian KOMET (lihat SYARAT LOLOS).
# Tahap dengan "skor_s": False menghapus suku s, penalti jarak, dan komet |s|: yang dinilai hanya psi.
OPTUNA_TRIAL_PD = 163221013                 # jumlah trial tahap PD (bisa diubah per tahap di OPTUNA_TAHAP)
OPTUNA_TRIAL_CBFQP = 163221013              # jumlah trial tahap CBFQP_* (bisa diubah per tahap di OPTUNA_TAHAP)
OPTUNA_TRIAL_SEMUA = 163221013              # jumlah trial tahap SEMUA (tuning gabungan 10 variabel)
OPTUNA_LANGKAH = 0.01                  # 2 angka di belakang koma: gain kecil (Kp_theta ~0.05, Kd_theta ~0.1)
                                      #     butuh ketelitian ini (langkah 0.01 = 10-20 % dari nilainya)

# Batas pencarian PER PROFIL: OPTUNA_BATAS[profil][nama] = (batas bawah, batas atas). Edit sebelum tuning.
# Batas tidak termasuk pemeriksaan skenario: mengubahnya tidak membuat study ditolak (trial lama tetap tersimpan).
# Alpha = 0 membuat CBF-QP tidak aktif (status ALPHA_NOL), jadi batas bawah Alpha harus > 0.
# Profil berbeda butuh batas berbeda: torsi roda DATASHEET (0.75) sekitar 3.15x lebih kuat dari TERUKUR (0.238),
# sehingga gain yang sama bekerja seperti gain 3x lebih besar.
# Dasar pemilihan (model linear model.py, sampling 200 Hz, jeda torsi 5 ms; analisis 20261012-13). Batas stabil
# model ternyata konservatif (gain terbaik DATASHEET Kd_psi 2.97 stabil di simulasi, model memberi batas ~1.9),
# jadi batas atas diberi margin sekitar 1.5-2x di atas batas model.
#                         TERUKUR (0.238)                       DATASHEET (0.75)
#   Kp_psi   : > 1.7 (badan bisa ditegakkan)                > 0.55
#   Kd_psi   : batas stabil model ~5.5-6.4                  ~1.8-2.0  (di atasnya chattering karena jeda)
#   Kp_theta : batas stabil model ~Kp_psi / 30 .. / 15      ~Kp_psi / 12
#   Kd_theta : batas stabil model ~1.0                      ~0.4
#   Alpha_1 x Alpha_2 >= 211                                >= 664   (torsi +-1.5 N m tidak dipotong saat tegak)
#   Alpha_1  <= ~28                                         <= ~55   (badan masih bisa direm sebelum psi_max)
#   Alpha_3  >= 8.8                                         >= 28    (barrier dpsi tidak memotong torsi saat tegak)
#   Alpha_4 x Alpha_5 >= 7.1                                >= 22    (barrier s tidak memotong torsi saat diam)
#   Alpha_6  >= 10.8                                        >= 34    (barrier dtheta, dtheta_max = 13 rad/s)
# Rentang terlalu lebar (contoh Kp_psi 5..100, Kd_psi 0.5..100) membuat sebagian besar trial tidak stabil:
# hanya ~4 % (TERUKUR) dan ~1 % (DATASHEET) kombinasi yang stabil menurut model, sehingga Optuna sulit belajar.
OPTUNA_BATAS = {
    "TERUKUR": {
        "Kp_psi":   (5.0, 80.0),
        "Kd_psi":   (0.5, 9.0),
        "Kp_theta": (0.0, 4.0),
        "Kd_theta": (0.0, 1.5),
        "Alpha_1":  (14.53, 28.28),
        "Alpha_2":  (14.53, 28.28),
        "Alpha_3":  (20.00, 100.0),
        "Alpha_4":  (10.00, 100.0),
        "Alpha_5":  (10.00, 100.0),
        "Alpha_6":  (11.00, 100.0),
    },
    "DATASHEET": {
        # Batas PD DATASHEET (analisis 20261017): trial yang lolos >= 9.5 N di simulasi sekarang dan >= 20 N di
        # simulasi lama ada di Kp_psi 8.4 .. 23.2, Kd_psi 1.63 .. 2.68, Kp_theta 0 .. 0.22, Kd_theta 0.02 .. 0.17.
        # Semua trial dengan Kd_psi > 5 hanya lolos <= 10 N dan chattering (batas lebar 0.3 .. 100 membuat tuning
        # SEMUA 20261017 tidak pernah mencoba daerah ini). Kd_theta sampai 0.3: menaikkan Kd_theta meredam ayunan
        # maju-mundur robot (underdamped). Dibutuhkan tahap PD dan SEMUA.
        "Kp_psi":   (5.0, 25.0),
        "Kd_psi":   (1.5, 3.5),
        "Kp_theta": (0.0, 0.3),
        "Kd_theta": (0.1, 0.6),
        "Alpha_1": (25.77, 54.77),
        "Alpha_2": (25.77, 54.77),
        "Alpha_3": (30.00, 150.0),
        "Alpha_4": (10.00, 100.0),
        "Alpha_5": (10.00, 100.0),
        "Alpha_6": (1.00, 500.0),
    },
}

# Definisi tahap (urutan = urutan pengerjaan):
#   variabel : yang dituning di tahap ini (batasnya di OPTUNA_BATAS)
#   trial    : jumlah trial (python3 tuning_optuna.py --trial N menggantikannya)
#   barrier  : None = mode PD saja. Daftar nomor = hanya constraint nomor itu yang aktif di CBF-QP
#              (nomor = urutan di DAFTAR_C, cbf_qp.py: 1 = h1, 2 = h2, ... 8 = h8), kumulatif dari tahap sebelumnya.
#   paksa    : (opsional) nilai yang dipaksa selama tahap ini, menggantikan config.py
#   skor_s   : (opsional, default True) False = skor hanya dari psi
#   penalti_jarak : (opsional, default True) False = OPTUNA_PENALTI_JARAK tidak dihitung (suku s di ISE tetap)
#   cari_gaya     : (opsional, default False) True = tiap trial mencari gaya dorong tertinggi yang lolos (lihat atas)
#   stagnant      : (opsional, default OPTUNA_STAGNANT) batas stagnan khusus tahap ini (lihat AUTO-EXIT di bawah)
#   dua_mode      : (opsional, default False) True = tiap trial PD saja + PD + CBF-QP dengan gain sama (tahap SEMUA)
#   miring        : (opsional, default False) True = tiap trial (tiap mode) menjalankan S1 -> S2 -> S3 (lihat blok
#                   "SKENARIO S1 S2 S3" di bawah); butuh cari_gaya = True
OPTUNA_TAHAP = {
    "PD":           {"variabel": ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta"], "trial": OPTUNA_TRIAL_PD,
                     "barrier": None, "penalti_jarak": False, "cari_gaya": True},
    "CBFQP_PSI":    {"variabel": ["Alpha_1", "Alpha_2"], "trial": OPTUNA_TRIAL_CBFQP, "barrier": [1, 2]},
    "CBFQP_DPSI":   {"variabel": ["Alpha_3"], "trial": OPTUNA_TRIAL_CBFQP, "barrier": [1, 2, 3, 4]},
    "CBFQP_S":      {"variabel": ["Alpha_4", "Alpha_5"], "trial": OPTUNA_TRIAL_CBFQP, "barrier": [1, 2, 3, 4, 5, 6]},
    "CBFQP_DTHETA": {"variabel": ["Alpha_6"], "trial": OPTUNA_TRIAL_CBFQP, "barrier": [1, 2, 3, 4, 5, 6, 7, 8]},
    # Tuning gabungan (bukan bagian urutan bertahap): DUA MODE, PD saja + PD + CBF-QP h1 .. h8 (lihat atas).
    "SEMUA":        {"variabel": ["Kp_psi", "Kd_psi", "Kp_theta", "Kd_theta",
                                  "Alpha_1", "Alpha_2", "Alpha_3", "Alpha_4", "Alpha_5", "Alpha_6"],
                     "trial": OPTUNA_TRIAL_SEMUA, "barrier": [1, 2, 3, 4, 5, 6, 7, 8], "penalti_jarak": False,
                     "cari_gaya": True, "dua_mode": True, "miring": True},
}

# Profil optimasi. Tiap profil punya study Optuna sendiri (hasilnya tidak tercampur):
#   study tiap tahap: ATERA_<TAHAP>_<PROFIL>, contoh ATERA_PD_TERUKUR
# kt_efektif  : torsi roda nyata per A arus perintah [N m/A], menggantikan MOTOR_KT_EFEKTIF saat tuning
# gaya_dorong : [N] gaya dorong TERENDAH yang dicoba tahap PD (awal pencarian gaya)
# gaya_pd_terbaik : [N] gaya lolos tertinggi dari hasil tahap PD (dicetak tuning_optuna.py, salin ke sini).
#                   HANYA dipakai tahap CBFQP_* (OPTUNA_GAYA_CBF_FRAKSI x gaya_pd_terbaik). Tahap PD, SEMUA, dan
#                   --banding mencari gayanya sendiri, jadi tidak membutuhkannya. None = belum ada.
# Jalankan:  python3 tuning_optuna.py --profil TERUKUR --tahap PD
OPTUNA_PROFIL = {
    "TERUKUR": {
        "kt_efektif": 0.238,        # hasil uji delay 20261009 (G x j_theta)
        "gaya_dorong": 5.0,         # [N] gaya terendah yang dicoba tahap PD
        "gaya_pd_terbaik": 11.5,    # [N] dari simulasi 20261013 (sebelum SIM_BATAS_KECEPATAN): ulangi tahap PD
                                    #     sebelum menjalankan tahap CBFQP_* profil ini
        "terverifikasi": True,
        "catatan": "K diturunkan dari uji inersia/delay, bergantung pada j_theta = 9.2e-4",
    },
    "DATASHEET": {
        "kt_efektif": 0.75,         # datasheet Waveshare DDSM115 (= MOTOR_KT)
        "gaya_dorong": 5.0,        # [N] gaya terendah yang dicoba tahap PD
        "gaya_pd_terbaik": None,    # [N] 25.0 lama berasal dari simulasi tanpa SIM_BATAS_KECEPATAN (20261014),
                                    #     di simulasi sekarang tahap PD hanya lolos 12 N: isi lagi dari tahap PD
        "terverifikasi": False,
        "catatan": "BELUM TERVERIFIKASI: nilai 0.75 belum cocok dengan uji delay (0.24). "
                   "BUTUH PENGUJIAN TORSI RODA (ddsm115_torsi_statis_testing.py)",
    },
}
OPTUNA_ZETA_DEG = 0.0               # [deg] lantai saat optimasi
OPTUNA_ZETA_UJI_DEG = 5.2           # [deg] lantai untuk menguji hasil terbaik (tidak ikut dioptimasi)
OPTUNA_ARAH_DORONG = [1, -1, 1, -1]  # urutan dorongan: 1 = dorong depan, -1 = dorong belakang (tanpa reset di antaranya)
OPTUNA_DURASI_DORONGAN = 15.0       # [s] jarak antar dorongan (terpisah dari DURASI_PENGUJIAN). ASUMSI 20261017:
                                    #     respon 25 N di simulasi 20261014 reda sekitar 15 s setelah dorongan
# Syarat STABIL (lihat penjelasan di atas). Hanya menentukan kapan dorongan pertama diberikan dan kapan ulangan
# selesai, TIDAK diberi penalti.
OPTUNA_STABIL_PSI = 0.15707963     # [rad]   |psi|    < nilai ini (10 deg)
OPTUNA_STABIL_DPSI = 0.17453293     # [rad/s] |dpsi|   < nilai ini (10 deg/s)
OPTUNA_STABIL_DTHETA = 0.17453293   # [rad/s] |dtheta| < nilai ini (10 deg/s)
OPTUNA_STABIL_LAMA = 3.0            # [s] ketiga syarat di atas harus terpenuhi TERUS-MENERUS selama ini (ASUMSI)
OPTUNA_DURASI_STABIL = 20.0         # [s] batas waktu MAKSIMUM menunggu stabil (tenang awal dan setelah dorongan
                                    #     terakhir, dihitung dari awal fase itu); lewat dari ini ulangan lanjut
OPTUNA_SEED = 1                     # seed sampler Optuna dan noise IMU ulangan pertama (ulangan ke-k: OPTUNA_SEED + k)
OPTUNA_ULANGAN = 2                  # jumlah ulangan skenario (dengan reset) dalam satu pengujian
OPTUNA_GESER_DORONG = 0.13          # [s] ASUMSI: pergeseran waktu dorong per ulangan (~setengah periode osilasi
                                    #     3.8 Hz yang teramati di simulasi 20261012)
                                    
OPTUNA_GAYA_MAKS = 100.0            # [N] gaya tertinggi yang dicoba tahap PD dan SEMUA. Dengan LAMA_DORONG 0.015 s
                                    #     100 N = 1.5 N s, di atas batas robot (~1.0 N s, model planar, ~65-75 N).
                                    #     Nilai lama 25 N hanya cocok untuk LAMA_DORONG 0.10 s.

OPTUNA_GAYA_LANGKAH = 0.5           # [N] ketelitian pencarian gaya tahap PD
OPTUNA_BOBOT_GAYA = 1000.0          # [per N] bobot gaya lolos di skor tahap PD (jauh lebih besar dari ISE)
OPTUNA_GAYA_CBF_FRAKSI = [0.50, 0.75, 1.0, 1.01, 1.15, 1.25, 1.5, 1.75, 2]   # tahap CBFQP_*: gaya = fraksi x gaya_pd_terbaik (boleh > 1)
OPTUNA_AMBANG_CHATTERING = 0.5      # fraksi u_max; |u| di atas ini dihitung torsi besar (ch/s)
OPTUNA_MULAI_DARI_CONFIG = True     # study BARU: trial pertama = nilai config.py (titik yang sudah diketahui)
OPTUNA_BOBOT_CBF_BURUK = 1.0        # tahap SEMUA (dua mode): bobot penalti bila skor PD + CBF-QP lebih buruk dari
                                    #     PD saja. 1.0 = keunggulan PD saja tidak dihitung (skor = 2 x skor PD+CBF);
                                    #     lebih besar = CBF-QP yang memperburuk PD makin dihukum. 0 = tanpa penalti

# -----------------------------------------------------------------------------------------------------------------
# >>>>>>>>>>  SKENARIO S1 S2 S3 TIAP TRIAL  (tahap dengan "miring": True, yaitu SEMUA)  <<<<<<<<<<
# Tiap trial, untuk TIAP mode (PD saja dan PD + CBF-QP), dengan gain yang SAMA:
#   S1 dorongan, lantai datar : seperti sebelumnya, gaya dorong dinaikkan (bisection OPTUNA_GAYA_LANGKAH) sampai
#                               gagal -> gaya_lolos [N] (lantai OPTUNA_ZETA_DEG, skenario dorongan di atas).
#   S2 bidang miring          : TANPA dorongan. Kemiringan dinaikkan OPTUNA_MIRING_LANGKAH tiap berhasil
#                               (bisection pada kisi OPTUNA_MIRING_LANGKAH .. OPTUNA_MIRING_MAKS) -> miring_lolos [deg].
#                               Satu pengujian S2 = OPTUNA_ULANGAN ulangan (reset, seed noise sama dengan S1), robot
#                               diletakkan tegak lalu dibiarkan OPTUNA_MIRING_LAMA detik. BERHASIL bila sepanjang waktu:
#                               tidak jatuh (|psi| <= SAFE_TILT_DEG), |s| <= s_max (tidak meluncur melewati barrier s),
#                               dan ch/s <= OPTUNA_BATAS_CHATTERING.
#                               ASUMSI bisection: berhasil pada kemiringan z berarti berhasil pada kemiringan lebih kecil.
#   S3 dorongan, bidang miring: gaya_lolos S1 diberikan di kemiringan miring_lolos S2 (skenario dorongan dan SYARAT
#                               LOLOS sama dengan S1) -> lolos / gagal. Tidak dijalankan (= gagal) bila miring_lolos = 0.
#   S2 dan S3 baru dijalankan setelah S1 punya hasil: bila gaya terendah pun gagal (gaya_lolos = 0), miring_lolos = 0
#   dan S3 gagal.
#   skor mode = skor S1 (-OPTUNA_BOBOT_GAYA x gaya_lolos + skor pengujian)
#             - OPTUNA_BOBOT_MIRING x miring_lolos - (OPTUNA_BOBOT_S3 bila S3 lolos)
#   Dua mode digabung seperti biasa (OPTUNA_BOBOT_CBF_BURUK). Makin kecil makin baik.
# ASUMSI bobot: satu langkah S2 (0.1 deg x 5000 = 500) = satu langkah S1 (0.5 N x 1000 = 500); S3 lolos = 10 langkah.
OPTUNA_MIRING_LANGKAH = 0.1         # [deg] kenaikan kemiringan tiap berhasil (kisi S2)
OPTUNA_MIRING_MAKS = 30.0           # [deg] kemiringan tertinggi yang dicoba S2 (ASUMSI). Untuk diam di kemiringan
                                    #     zeta badan harus condong psi = asin(0.83 x sin zeta) (model.py): 6.6 deg
                                    #     di 8 deg, 12.4 deg di 15 deg. Mode PD + CBF-QP dibatasi barrier psi
                                    #     (psi_max - CBF_MARGIN_DEG = 9 deg -> sekitar 10.8 deg), mode PD saja tidak,
                                    #     jadi batas ini dibuat jauh di atasnya.
OPTUNA_MIRING_LAMA = 15.0           # [s] lama robot dibiarkan di bidang miring tiap ulangan S2 (ASUMSI, = jarak
                                    #     antar dorongan; data robot 20261019: meluncur sampai diam dalam ~5 s)
OPTUNA_BOBOT_MIRING = 5000.0        # [per deg] bobot kemiringan lolos S2 di skor
OPTUNA_BOBOT_S3 = 5000.0            # bonus skor bila S3 lolos
# -----------------------------------------------------------------------------------------------------------------

# -----------------------------------------------------------------------------------------------------------------
# >>>>>>>>>>  SYARAT LOLOS SATU PENGUJIAN  (UBAH DI SINI)  <<<<<<<<<<
# Satu pengujian (satu gaya dorong, OPTUNA_ULANGAN ulangan) dianggap LOLOS hanya bila SEMUA syarat terpenuhi.
# Yang tidak memenuhi dihitung KOMET (penalti OPTUNA_PENALTI_KOMET), jadi gaya itu tidak dianggap lolos.
#   1. tidak jatuh      : |psi| <= SAFE_TILT_DEG dan |s| <= S_KOMET (bagian 2)
#   2. tidak chattering : ch/s <= OPTUNA_BATAS_CHATTERING  (torsi tidak bergetar bolak-balik, lihat CHATTERING)
#   3. tidak berayun    : overshoot psi <= OPTUNA_SYARAT_OVERSHOOT_PSI  DAN  overshoot s <= OPTUNA_SYARAT_OVERSHOOT_S
#                         (target critically damped: kembali ke titik awal tanpa berayun melewatinya)
# Overshoot dihitung untuk TIAP dorongan (fungsi overshoot_persen di tuning_optuna.py), yang terbesar dipakai:
#     e(t)      = sinyal - acuan          acuan = rata-rata OPTUNA_OVERSHOOT_ACUAN_S detik sebelum dorongan
#     puncak    = e dengan |e| terbesar sejak dorongan sampai dorongan berikutnya
#     overshoot = simpangan terbesar ke arah BERLAWANAN dengan puncak (setelah puncak) / |puncak| x 100 %
#     Contoh psi: didorong, psi naik ke +8 deg, lalu sempat -2 deg sebelum tenang -> overshoot 25 % (berayun).
#                 didorong, psi naik ke +8 deg, lalu turun pelan ke 0 tanpa lewat minus -> 0 % (critically damped).
#     s (posisi robot maju-mundur) dihitung dengan cara yang sama.
# Batas psi dibuat lebih longgar dari s: badan memang harus sedikit condong ke arah sebaliknya untuk mengerem roda
# (model linear: overshoot psi ~5-6 % walau gain critically damped; gain config.py 20261017: psi ~20 %, s ~8 %).
# Hubungan overshoot dan rasio redaman zeta (sistem orde 2, zeta = -ln(OS) / sqrt(pi^2 + ln(OS)^2)):
#     overshoot  20 %   10 %   5 %    2 %    1 %    0 %
#     zeta       0.46   0.59   0.69   0.78   0.83   >= 1 (critically damped / overdamped)
# Noise IMU menambah sekitar 1-3 % pada hasil hitungan, jadi batas di bawah ~3 % kurang bisa dipercaya.
# Isi 0 untuk mematikan satu syarat.
OPTUNA_SYARAT_OVERSHOOT_PSI = 30.0  # [%] ASUMSI: overshoot psi maksimum yang masih lolos
OPTUNA_SYARAT_OVERSHOOT_S = 5.0     # [%] ASUMSI: overshoot s maksimum yang masih lolos
OPTUNA_BATAS_CHATTERING = 2.0       # [per s] ASUMSI: ch/s maksimum (data 20261016-17: trial PD halus < 0.1/s,
                                    #     trial SEMUA yang bergetar 4 .. 26/s; 2 berada di celah keduanya)
# Teknis perhitungan overshoot (biasanya tidak perlu diubah):
OPTUNA_OVERSHOOT_ACUAN_S = 1.0      # [s] lama rata-rata sebelum dorongan yang dipakai sebagai acuan
OPTUNA_OVERSHOOT_RATA_S = 0.05      # [s] rata-rata bergerak sinyal sebelum dihitung (meredam noise IMU)
OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG = 0.5   # [deg] puncak psi lebih kecil dari ini tidak dinilai (dorongan terlalu kecil)
OPTUNA_OVERSHOOT_PUNCAK_S_M = 0.01      # [m]   puncak s lebih kecil dari ini tidak dinilai
# -----------------------------------------------------------------------------------------------------------------
OPTUNA_PENALTI_KOMET = 1000.0
OPTUNA_PENALTI_JARAK = 10.0         # [per detik]
OPTUNA_PENALTI_GAGAL = 10.0         # [per detik]
# Pruning (auto-stop trial yang jelek): skor dilaporkan tiap 1 detik; trial dihentikan bila skornya lebih buruk
# dari median trial sebelumnya pada detik yang sama. Baru aktif setelah OPTUNA_PRUNER_STARTUP trial selesai dan
# setelah OPTUNA_PRUNER_WARMUP_S detik pertama. Tahap "cari_gaya" (PD, SEMUA) tidak di-prune: skornya baru
# diketahui setelah pencarian gaya selesai.
OPTUNA_PRUNER_STARTUP = 10
OPTUNA_PRUNER_WARMUP_S = 5
# AUTO-EXIT STAGNAN (early stopping seluruh study, berbeda dengan pruning yang hanya menghentikan satu trial):
# tuning satu tahap berhenti sendiri bila skor terbaik tidak MEMBAIK selama OPTUNA_STAGNANT trial selesai
# berturut-turut, lalu langsung lanjut ke laporan (nilai untuk config.py, CSV, PNG) seperti biasa.
# Membaik = skor turun lebih dari
#   OPTUNA_STAGNANT_MIN x max(1, |skor terbaik acuan|)
#   (relatif; tahap PD dan SEMUA: kenaikan gaya lolos 0.5 N selalu terhitung)
# Trial pruned tidak dihitung. Riwayat study di database ikut dihitung, jadi study yang dilanjutkan tidak mulai dari 0.
# 0 = auto-exit mati. Bisa diganti per tahap (kunci "stagnant" di OPTUNA_TAHAP) atau saat menjalankan: --stagnant N.
# ASUMSI: 300 trial dan 0.1 %. Mengubahnya TIDAK membuat study ditolak (bukan bagian skenario).
OPTUNA_STAGNANT = 300
OPTUNA_STAGNANT_MIN = 1.0e-3