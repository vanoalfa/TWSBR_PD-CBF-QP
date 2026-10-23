"""config.py - satu-satunya sumber nilai program ATERA (robot fisik).

Nilai fisik, gain PD, dan Alpha CBF-QP sama dengan simulasi/config.py (hasil tuning Optuna, profil DATASHEET),
supaya robot fisik menjalankan kontrol yang sama dengan simulasi.

Satuan SI (kg, m, s, rad, N m) kecuali nama berakhiran _DEG, _A, _HZ, _S.

Konvensi tanda (sama dengan simulasi):
    state x = [theta, dtheta, psi, dpsi]
    psi   > 0 : badan miring ke DEPAN
    theta > 0 : roda menggelinding ke DEPAN (sudut roda absolut, lihat model.hitung_theta)
    u     > 0 : torsi SATU roda yang mendorong roda ke DEPAN; roda kiri u + u_belok, roda kanan u - u_belok
                (u_belok = PD heading, bagian 4b; rata-rata kedua roda tetap u)
    yaw   > 0 : robot berputar ke KIRI dilihat dari atas, yaw = R*(theta_kanan - theta_kiri)/W_RODA

Mode gerak (pd_control.py):
    DIAM      : default setelah MULAI. Tanpa Kp_theta (posisi tidak ditahan).
    BALANCING : Tombol B (toggle dengan DIAM; dari BEBAS langsung BALANCING). Kp_theta aktif, KOMET s aktif. Setpoint
                theta (titik nol s) dipasang saat masuk BALANCING dan saat D-Pad atas / bawah dilepas.
    BEBAS     : Tombol A. Tanpa Kp_theta dan PD heading, belok = selisih torsi TORSI_BELOK (bisa ditarik tali).
                A lagi -> DIAM.
    D-Pad (Kvel*arah, belok) berlaku di semua mode; di BALANCING, Kp_theta diabaikan selama D-Pad atas / bawah ditekan.
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

zeta_DEG = 0.0                      # [deg] kemiringan bidang yang dipakai model CBF-QP dan DOB: 0.0 (datar), 8.0
                                    #       (menanjak ke DEPAN robot), -8.0 (robot menghadap turunan). Isi sesuai
                                    #       bidang sebelum menjalankan program (= simulasi CBF_TAHU_ZETA = True).
zeta = math.radians(zeta_DEG)

# =============================================================================
#                              2. STATE DAN SETPOINT
# =============================================================================
IDX_THETA, IDX_DTHETA, IDX_PSI, IDX_DPSI = 0, 1, 2, 3
NX = 4
NU = 1

theta_setpoint = 0.0                # [rad]   theta diukur dari setpoint theta (titik nol s), yang dipasang oleh
                                    #         Tombol Y (kalibrasi), Tombol B, dan saat D-Pad atas / bawah dilepas
dtheta_setpoint = 0.0               # [rad/s]
psi_setpoint = 0.0                  # [rad]
dpsi_setpoint = 0.0                 # [rad/s]

# =============================================================================
#                              3. SAFETY SET DAN KOMET
# =============================================================================
# Safety set CBF-QP (= simulasi): h1, h2 = psi, h3, h4 = dtheta
psi_max_DEG = 10.0                  # [deg]    batas psi yang dijaga CBF (h1, h2)
psi_max = math.radians(psi_max_DEG) # [rad]    |psi|    <= psi_max
# Sedikit di bawah batas motor (no-load DDSM115 200 rpm = 20.9 rad/s; data robot 20261015: |dtheta| tidak pernah
# > ~21 rad/s). Uji model planar 16-10: dengan 13 rad/s barrier dtheta melarang roda mengejar badan setelah dorongan
# ~10 N (PD butuh ~19 rad/s) sehingga robot jatuh; dengan 20 rad/s tidak.
dtheta_max = 20.0                   # [rad/s]  |dtheta| <= dtheta_max
# Bukan constraint CBF-QP (dipantau di GUI dan plot):
dpsi_max = 2.0 * math.pi            # [rad/s]  garis acuan |dpsi|
s_max = 0.5                         # [m]      garis acuan |s|, s = R*theta dari titik nol s

# KOMET = robot tidak tertolong, roda dimatikan (sama dengan simulasi):
#   jatuh    : |psi| > SAFE_TILT_DEG               (semua mode gerak)
#   lari     : |s|   > S_KOMET                     (hanya mode BALANCING tanpa D-Pad atas / bawah)
#   berputar : |yaw - yaw_ref| > YAW_KOMET_DEG     (mode DIAM dan BALANCING; di mode BEBAS yaw memang bebas)
SAFE_TILT_DEG = 40.0                # [deg]  ASUMSI: dipilih pengguna (20261014)
S_KOMET = 2.0                       # [m]
START_TILT_DEG = 15.0               # [deg] MULAI ditolak bila |psi| lebih besar
# Pengaman tambahan robot fisik (tidak ada di simulasi): |u| terus di batas arus selama ini -> roda dimatikan.
# Contoh: robot diangkat atau tertahan dan roda berputar terus. 0 = tidak dipakai.
SATURASI_MAKS_S = 2.0               # [s]
# KOMET berputar: s = R*theta memakai RATA-RATA kedua roda, jadi robot yang berputar di tempat (roda kiri maju,
# kanan mundur) tidak pernah mencapai |s| > S_KOMET.
# ASUMSI: 90 deg (robot sudah menghadap ke samping = PD heading tidak bekerja, contoh tanda roda salah).
YAW_KOMET_DEG = 90.0                # [deg]
# Pengereman aktif saat KOMET. Arus 0 A di DDSM115 (current loop) = roda BEBAS (tidak direm), jadi
# roda yang sedang berputar ~20 rad/s tetap berputar lama setelah KOMET. Selama KOMET_REM_LAMA_S detik setelah
# KOMET, tiap roda diberi arus yang melawan putarannya sendiri:  i = -KOMET_REM_GAIN_A * dtheta_motor  (dijepit
# +-KOMET_REM_ARUS_MAKS), sampai kedua roda < KOMET_REM_SELESAI rad/s, lalu 0 A. Hanya KOMET; Tombol X tetap 0 A.
# (Byte "brake" protokol DDSM115 hanya berlaku di speed loop, sedangkan ATERA memakai current loop.)
KOMET_REM_AKTIF = True
KOMET_REM_GAIN_A = 0.10             # [A per rad/s]
KOMET_REM_ARUS_MAKS = 1.0           # [A]
KOMET_REM_LAMA_S = 2.0              # [s]
KOMET_REM_SELESAI = 0.5             # [rad/s]

# =============================================================================
#                              5. PD
# =============================================================================
# u_PD = BALANCE_DIRECTION_SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi
#                                  + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
# error = setpoint - state,   arah = -1 / 0 / +1 dari W / S (robot fisik: D-Pad atas / bawah)
Kp_psi = 13.94                     # [N m/rad]    semua gain menghasilkan torsi SATU roda
Kd_psi = 01.96                      # [N m s/rad]
Kp_theta = 0.17                     # [N m/rad]    hanya mode BALANCING tanpa W / S
Kd_theta = 0.27                     # [N m s/rad]
# Kvel*arah saat W / S ditahan. Kecepatan tunak di lantai datar: v = -R*Kvel*arah/Kd_theta
# -> Kvel NEGATIF supaya W maju (Kd_theta 0.29: Kvel -1.16 -> 0.20 m/s), sama dengan robot fisik.
Kvel = -1.50                        # [N m]

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
Alpha_1 = 11.24                     # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 18.79                     # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 438.98                    # [1/s]  h3, h4 (dtheta, CBF orde 1)
# Kvel*arah saat D-Pad atas / bawah ditekan (arah +1 = atas, -1 = bawah). Kecepatan tunak di lantai datar:
#     v = -R*Kvel*arah/Kd_theta      -> Kvel NEGATIF supaya D-Pad atas berjalan ke DEPAN
#     (Kd_theta 0.29: Kvel -1.16 -> 0.20 m/s; no-load DDSM115 1.05 m/s)
Kvel = -1.50                        # [N m]
# Dengan error = setpoint - state dan u > 0 = roda ke depan, gain positif menstabilkan bila tandanya -1
# (sama dengan simulasi/config.py). Tanpa ini pd_control.py juga memakai -1.
BALANCE_DIRECTION_SIGN = -1.0

# =============================================================================
#                              4b. PD HEADING (roda kiri dan kanan bergerak bersama)
# =============================================================================
# yaw = R*(theta_kanan - theta_kiri)/W_RODA dari encoder (+ = berputar ke KIRI dilihat dari atas).
# u_belok = Kp_yaw*(yaw - yaw_ref) + Kd_yaw*(dyaw - dyaw_ref); roda kiri u + u_belok, roda kanan u - u_belok.
#   D-Pad kiri / kanan dilepas : yaw_ref tetap -> robot tidak berputar-putar.
#   D-Pad kiri / kanan ditekan : dyaw_ref = -belok*YAW_RATE_BELOK (saat DIAM = berputar di tempat).
# ASUMSI gain: model yaw  I_yaw*ddyaw = (W/R)*u_belok,  I_yaw ~ 0.054 kg m^2 (badan 0.0165 dari robot.urdf + roda
# 2*m_r*(W/2)^2 + inersia putar roda 2*j_theta*(W/(2R))^2)  ->  Kp 0.30, Kd 0.10: omega_n ~5 rad/s, zeta ~0.9
# (dengan torsi nyata 0.238 N m/A: omega_n ~3 rad/s, zeta ~0.5). Naikkan pelan-pelan bila robot masih berputar.
W_RODA = 0.258                      # [m] tengah ban kiri ke tengah ban kanan (diukur; ATERA.step: poros motor +-124 mm)
YAW_AKTIF = True
Kp_yaw = 0.30                       # [N m/rad]
Kd_yaw = 0.10                       # [N m s/rad]
YAW_RATE_BELOK = 1.0                # [rad/s] kecepatan belok D-Pad kiri / kanan, A / D (~57 deg/s)
A_YAW_MAKS = 3.0                    # [rad/s^2] perubahan maksimum dyaw_ref
YAW_TORSI_MAKS = 0.40               # [N m] batas |u_belok| (juga dibatasi sisa torsi u_max - |u|)
# Mode BEBAS (Tombol A): PD heading mati, belok D-Pad = selisih torsi tetap: roda kiri u + TORSI_BELOK*belok, kanan
# u - TORSI_BELOK*belok (dibatasi sisa torsi u_max - |u|). ASUMSI: 0.02 N m = nilai program lama (commit 61784ea).
# Naikkan bila belok di mode BEBAS terlalu lemah (YAW_TORSI_MAKS sebagai batas yang masih wajar).
TORSI_BELOK = 0.02                  # [N m]

# =============================================================================
#                              5. CBF-QP  (= simulasi)
# =============================================================================
# Gain (input manual, default 0; alpha_i(h) = Alpha_i * h)
Alpha_1 = 35.62                     # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 39.32                     # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 245.94                    # [1/s]  h3, h4 (dtheta, CBF orde 1)

# Semua Alpha_<nomor> berurutan di atas (otomatis dipakai cbf_qp.py dan ise.py).
NAMA_ALPHA = []
while ("Alpha_%d" % (len(NAMA_ALPHA) + 1)) in globals():
    NAMA_ALPHA.append("Alpha_%d" % (len(NAMA_ALPHA) + 1))

CBF_QP_SOLVER = "daqp"              # QP solver CBF-QP (plugin ca.conic CasADi), ikut di paket pip casadi.
# Uji Raspi 20261020 (--banding-solver, 3000 kasus): daqp gagal 0 %, 0.20 ms; qpoases gagal 26 %; osqp gagal 28 %, 5.3 ms.
# pilihan lain: "eksak" (solusi pasti numpy, tanpa iterasi), "osqp", "proxqp", "qrqp", "hpipm", "daqp",
# "highs", "ipqp". Bandingkan di Raspberry Pi:  python3 cbf_qp.py --banding-solver
CBF_QP_SOLVER_BANDING = ["eksak", "qpoases", "osqp", "proxqp", "qrqp", "hpipm", "daqp", "highs", "ipqp"]
# Slack per constraint (exact penalty, lihat cbf_qp.py):
#   min (u - u_PD)^2 + sum (rho_k*d_k + CBF_SLACK_EPS*d_k^2),  c_k + d_k >= 0,  d_k >= 0
# Selama semua constraint bisa dipenuhi: d = 0 dan u SAMA dengan QP lama (syarat: rho_k cukup besar, > ~2*|du|/|a_k|,
# untuk ATERA rho >= 10 sudah cukup). Bila constraint bertabrakan, yang bobotnya KECIL dilonggarkan dulu = PRIORITAS.
# Urutan = DAFTAR_C (h1 .. h4). ASUMSI prioritas: psi (h1, h2) > dtheta (h3, h4).
# Beda dengan segway_cbf.py dosen (satu slack bersama, penalti kuadrat): alasannya di docstring cbf_qp.py.
CBF_SLACK_AKTIF = True              # False = tanpa slack (perilaku lama: tidak ada solusi -> GAGAL -> u_PD)
CBF_RHO_SLACK = [1.0e6, 1.0e6, 1.0e2, 1.0e2]    # h1 h2 (psi) | h3 h4 (dtheta)
CBF_SLACK_EPS = 1.0e-4              # suku kuadrat kecil (supaya QP solver punya Hessian positif)
CBF_TOL_SLACK = 1.0e-6              # slack di atas ini -> status SLACK
# Barrier psi memakai psi_max - CBF_MARGIN_DEG (menutup jeda DOB saat dorongan mulai / berhenti).
CBF_MARGIN_DEG = 1.0                # [deg] ASUMSI (dosen 0.5 deg dengan DOB 2 ms; robot nyata DOB 20 ms)
# DOB (Disturbance Observer, class DOB di cbf_qp.py): d_hat = low-pass(percepatan terukur - model).
DOB_AKTIF = True                    # True: d_hat dipakai CBF-QP. False: d_hat hanya dihitung dan dicatat di CSV
# Noise turunan gyro saat robot diam (data 20261015 KE-04): 0.7 rad/s^2 -> 0.36 (tau 10 ms), 0.26 (20 ms), 0.16 (50 ms)
DOB_TAU_S = 0.020                   # [s] konstanta waktu low-pass DOB (dosen 0.002 s di simulasi tanpa noise)
DOB_BATAS = 200.0                   # [rad/s^2] batas |d_hat| (pengaman bila sensor sesaat salah)
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
MAX_CURRENT_A = 2.50                # [A] per roda (rated 1.5 A, locked-rotor <= 2.7 A), sama dengan simulasi
MIN_CURRENT_A = -MAX_CURRENT_A
u_max = MOTOR_KT * MAX_CURRENT_A    # [N m] torsi satu roda
u_min = MOTOR_KT * MIN_CURRENT_A

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
# Arah D-Pad dibaca dari NAMA di joystick_mapping.DPAD_MAP ("Kiri"/"Kanan"/"Atas"/"Bawah"). Bila belok
# lewat analog kiri masih terbalik, ganti ke -1.0 (D-Pad tidak terpengaruh). ASUMSI: 1.0 (belum diuji).
JOYSTICK_ANALOG_BELOK_TANDA = 1.0

# Keyboard terminal (SSH / terminal tempat GUI berjalan), tombolnya lihat joystick_mapping.TOMBOL_KEYBOARD.
# Terminal tidak memberi tahu saat tombol DILEPAS: W A S D dianggap masih ditahan selama pengulangan otomatis
# keyboard terus datang. ASUMSI: jeda pengulangan pertama keyboard +-0.5 s, berikutnya +-30 ms.
KEYBOARD_AKTIF = True
KEYBOARD_TAHAN_AWAL_S = 0.6         # [s] tekanan pertama W A S D dianggap ditahan selama ini
KEYBOARD_TAHAN_ULANG_S = 0.15       # [s] setelah pengulangan otomatis mulai: lepas bila tidak ada huruf baru selama ini

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
#   merah  = barrier CBF-QP yang dipakai (CBF_BARRIER_H: psi_max, dtheta_max)
#   abu-abu putus-putus = garis acuan tanpa CBF (dpsi_max, s_max)
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
# Batas yang constraint-nya tidak aktif atau bukan constraint (dpsi, s) digambar abu-abu tipis.
CBF_BARRIER_H = {"psi": (1, 2), "dtheta": (3, 4)}
PLOT_NONAKTIF_WARNA = "0.6"
PLOT_WARNA_PD = "tab:purple"        # kurva mode PD (ungu)
PLOT_WARNA_PD_CBF = "tab:blue"      # kurva mode PD + CBF-QP (biru)

# Plot realtime DI LAPTOP (laptop dan Raspi satu WiFi / hotspot):
#   Raspi  : python3 atera_main.py --plot              membuka PLOT_PORT dan mengirim data plot (TCP, JSON per baris)
#   laptop : python3 ise.py --plot-realtime <IP Raspi> jendela matplotlib di laptop (bisa dari WSL), IP tampil di GUI
PLOT_PORT = 5005                    # ASUMSI: port TCP bebas di Raspi (ubah bila bentrok; laptop: --port <nomor>)
PLOT_TUNGGU_MAKS = 200000           # [byte] data belum terkirim (~20 s data). Lebih dari ini = jaringan terlalu lambat:
                                    #     laptop diputus, lalu laptop menyambung lagi otomatis
PLOT_RENTANG = 10.0                 # [s] lebar jendela waktu yang ditampilkan
PLOT_KIRIM_SETIAP = 2               # satu titik plot tiap sekian siklus kontrol (200 Hz / 2 = 100 titik per detik)
PLOT_HZ = 10.0                      # [Hz] kecepatan gambar ulang jendela plot (laptop)
PLOT_BACKEND = ["QtAgg", "Qt5Agg", "TkAgg"]   # di LAPTOP, dicoba berurutan (pip install PyQt5, atau sudo apt install python3-tk)

# =============================================================================
# Turunan (jangan diubah)
# =============================================================================
RASIO_TORSI = MOTOR_KT_EFEKTIF / MOTOR_KT     # torsi nyata / torsi perintah, dipakai g(x) di CBF-QP