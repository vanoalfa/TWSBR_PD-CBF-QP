"""config.py - satu-satunya sumber nilai program ATERA (konstanta fisik, gain, batas, pengaturan simulasi).

Satuan SI (kg, m, s, rad, N m) kecuali nama berakhiran _DEG, _A, _HZ, _RPM.

Konvensi tanda (dipakai model.py dan semua file lain):
    state x = [theta, dtheta, psi, dpsi]
    psi   > 0 : badan miring ke DEPAN, diukur dari garis vertikal (gravitasi)
    theta > 0 : roda menggelinding ke DEPAN, sudut roda absolut (definisinya ada di model.hitung_theta)
    zeta  > 0 : bidang menanjak ke arah DEPAN
    u     > 0 : torsi motor SATU roda, mendorong roda ke DEPAN (reaksinya mendorong badan ke belakang)
            kedua roda diberi u yang sama, jadi torsi total = N_WHEEL*u

Dua mode gerak (pd_control.py: MODE_BALANCING, MODE_NAVIGATE):
    Balancing : robot diam di satu titik (Kp_theta aktif), semua barrier CBF aktif
    Navigate  : robot dijalankan joystick / D-Pad (Kp_theta = 0, Kvel x arah), barrier jarak s nonaktif
Titik nol jarak s (s = R*theta) direset ke posisi saat itu setiap robot masuk mode Balancing.
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
theta_setpoint = 0.0                # [rad]    sudut roda dinolkan saat robot masuk mode Balancing
psi_setpoint = 0.0                  # [rad]
dpsi_setpoint = 0.0                 # [rad/s]
dtheta_setpoint = 0.0               # [rad/s]

# =============================================================================
#                              4. SAFETY SET DAN KOMET
# =============================================================================
# Safety set (batas simetris: min = -max)
psi_max = math.pi / 12.0            # [rad]    |psi|    <= psi_max
dpsi_max = 2.0 * math.pi            # [rad/s]  |dpsi|   <= dpsi_max
dtheta_max = 2.0 * math.pi          # [rad/s]  |dtheta| <= dtheta_max
psi_min = -psi_max
dpsi_min = -dpsi_max
dtheta_min = -dtheta_max
s_max = 1.0                         # [m]      jarak tempuh s = R*theta (dari titik nol s), |s| <= s_max
s_min = -s_max

# KOMET = kondisi robot yang sudah tidak tertolong, berlaku di mode PD maupun PD+CBF. Roda dimatikan.
#   Jatuh : |psi| > SAFE_TILT_DEG                      (semua mode gerak)
#   Lari  : |s|   > S_KOMET        (hanya mode Balancing, di mode Navigate robot memang boleh berjalan jauh)
SAFE_TILT_DEG = 30.0                # [deg]
S_KOMET = 2.0                       # [m]

# =============================================================================
#                              5. PD
# =============================================================================
# u_PD = BALANCE_DIRECTION_SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi
#                                  + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
# error = setpoint - state,   arah = -1 / 0 / +1 dari D-Pad joystick
Kp_psi = 15.0           # [N m/rad]    semua gain menghasilkan torsi SATU roda
Kd_psi = 1.5           # [N m s/rad]
Kp_theta = 0.02         # [N m/rad]    hanya aktif di mode Balancing (Tombol B)
Kd_theta = 0.04         # [N m s/rad]
Kvel = -0.05             # [N m]       hanya aktif di mode Navigate (Tombol A / N)

# Dengan error = setpoint - state dan u > 0 = roda ke depan, gain positif menstabilkan bila tandanya -1
# (sama dengan apalah_bisa/tunning.py).
BALANCE_DIRECTION_SIGN = -1.0
# BUTUH PENGUJIAN ARAH KOREKSI: gain kecil, badan dimiringkan ke depan, roda harus berputar ke DEPAN.

# =============================================================================
#                              6. CBF-QP
# =============================================================================
# Gain (input manual, default 0; alpha_i(h) = Alpha_i * h)
Alpha_1 = 40.0          # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 40.0          # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 40.0          # [1/s]  h3, h4 (dpsi, CBF orde 1)
Alpha_4 = 40.0          # [1/s]  h5, h6 (jarak tempuh s, HOCBF orde 2), hanya mode Balancing
Alpha_5 = 40.0          # [1/s]  h5, h6 (jarak tempuh s, HOCBF orde 2), hanya mode Balancing
Alpha_6 = 10.0          # [1/s]  h7, h8 (dtheta, CBF orde 1)

# Nama semua Alpha di atas: Alpha_1, Alpha_2, ... (berurutan, berhenti di nomor pertama yang tidak ada).
# Alpha baru cukup ditambahkan di atas, lalu otomatis dipakai cbf_qp.py, simulasi.py (slider), dan ise.py (kolom CSV).
NAMA_ALPHA = []
while ("Alpha_%d" % (len(NAMA_ALPHA) + 1)) in globals():
    NAMA_ALPHA.append("Alpha_%d" % (len(NAMA_ALPHA) + 1))

CBF_N_HORIZON = 1      # N pada HPIPM (acados)

# Toleransi numerik (bukan besaran fisik)
CBF_INFTY = 1.0e9                   # batas atas "tak hingga" constraint h di acados
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

# Kalman sudut (kalman.py di robot, KalmanIMU di simulasi.py). Harus sama dengan kalman.py di atera_program.
KALMAN_Q_ANGLE = 0.001
KALMAN_Q_BIAS = 0.003
KALMAN_R_MEASURE = 0.03

# =============================================================================
#                              9. JOYSTICK  (Fantech WGP-13s)
# =============================================================================
JOYSTICK_PORT = "/dev/input/event5"
JOYSTICK_AMBANG_ANALOG = 0.5        # analog dianggap ditekan bila simpangannya di atas nilai ini (0..1)

# ASUMSI: torsi tambahan untuk belok (N m). Roda kiri diberi u - TORSI_BELOK*belok, roda kanan u + TORSI_BELOK*belok.
TORSI_BELOK = 0.02

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
# ulang berkali-kali saat slider sedang digeser).
ZETA_TUNGGU = 0.5                   # [s]

# Garis batas safety set dan warna kurva di gambar ise.py dan plot realtime
PLOT_SAFETY_WARNA = "red"
PLOT_SAFETY_TEBAL = 2.2             # garis yang membentuk kotak barrier (menyambung)
PLOT_SAFETY_TIPIS = 1.0             # perpanjangan garis di luar kotak (putus-putus)
PLOT_WARNA_PD = "tab:blue"
PLOT_WARNA_PD_CBF = "tab:orange"

# =============================================================================
#                              11. REALISME SIMULASI  (hanya dipakai simulasi.py, tidak dipakai robot fisik)
# =============================================================================
# Hasil testing/DELAY/atera_delay_testing.py, 20261009.
# K efektif = G x j_theta, G = 255 (kiri) dan 264 (kanan) rad/s^2 per A -> 0.234 dan 0.243 N m/A.
# ASUMSI: benar bila j_theta = 9.2e-4 (lihat BUTUH PENGUJIAN BEBAN TAMBAHAN di atas).
MOTOR_KT_EFEKTIF = 0.238            # [N m/A] torsi roda nyata per A arus perintah (rata-rata kiri dan kanan)
# Uji loop: perintah terkirim 1.8 ms setelah IMU dibaca (+ hitungan PD / CBF-QP), torsi mulai 0..4 ms kemudian.
# ASUMSI: dibulatkan menjadi 1 periode kontrol.
SIM_JEDA_TORSI = 0.005              # [s] jeda dari state dibaca sampai torsi bekerja (kelipatan 1 ms)
SIM_MODEL_IMU = True                # True: psi dari model MPU6050 + Kalman, False: psi sempurna
# Noise saat robot diam (0.5 s pertama delay_imu_20261009_134728.csv).
SIM_GYRO_NOISE_DEG_S = 0.10         # [deg/s] simpangan baku gyro
SIM_ACC_NOISE_DEG = 0.14            # [deg]   simpangan baku sudut akselerometer

# =============================================================================
#                              12. PYBULLET (simulasi.py)
# =============================================================================
SUBSTEP = 5                 # jumlah langkah fisika dalam satu siklus kontrol (DT)
GAYA_DORONG_AWAL = 20.0     # [N] besar gaya dorongan (tombol O / P dan tuning_optuna.py), bisa diubah di jendela Kontrol
LAMA_DORONG = 0.10          # [s] lama gaya dorongan bekerja
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
WARNA_QT_MODE = "#544bb8"        # mode yang sedang aktif
WARNA_QT_ABAI = "#9a9a9a"        # tulisan tombol yang sedang diabaikan
WARNA_QT_UJI = "#c26a00"         # mode pengujian aktif
# Slider tiap parameter: nama -> (batas bawah, batas atas, langkah, jumlah desimal).
# Kolom angka boleh diisi di luar batas slider; batas slider akan ikut melebar. Semua Alpha_n memakai SLIDER_ALPHA.
SLIDER = {
    "Kp_psi": (0.0, 60.0, 0.1, 2),
    "Kd_psi": (0.0, 10.0, 0.01, 3),
    "Kp_theta": (0.0, 0.5, 0.001, 4),
    "Kd_theta": (0.0, 0.5, 0.001, 4),
    "Kvel": (-0.5, 0.5, 0.001, 4),
    "Gaya_dorong": (0.0, 40.0, 0.1, 2),
    "zeta_DEG": (-15.0, 15.0, 0.5, 1),      # kemiringan lantai [deg], + = menanjak ke depan
}
SLIDER_ALPHA = (0.0, 100.0, 0.1, 2)

# =============================================================================
#                              13. TUNING OPTUNA  (hanya dipakai tuning_optuna.py)
# =============================================================================
# Satu trial = OPTUNA_ARAH_DORONG pengujian di lantai OPTUNA_ZETA_DEG, mode Balancing: robot dibiarkan tenang
# OPTUNA_WAKTU_SEBELUM_DORONG detik, didorong, lalu direkam OPTUNA_DURASI_UJI detik. Gaya dorong dan torsi roda
# mengikuti profil (OPTUNA_PROFIL di bawah).
# Skor (makin kecil makin baik), dihitung dari state yang dibaca kontrol (psi hasil Kalman):
#   skor = integral[(psi/psi_max)^2 + (s/s_max)^2] dt
#        + OPTUNA_PENALTI_JARAK x lama |s| > s_max                 [per detik]
#        + OPTUNA_PENALTI_GAGAL x lama status CBF bukan AMAN       [per detik, hanya tahap CBFQP]
#        + OPTUNA_PENALTI_KOMET x (1 + sisa waktu / waktu total)   bila KOMET (trial langsung berhenti)
OPTUNA_TRIAL_PD = 163221013               # jumlah trial tahap PD (gain PD)
OPTUNA_TRIAL_CBFQP = 163221013            # jumlah trial tahap CBFQP (Alpha, gain PD dikunci ke hasil tahap PD)
OPTUNA_BATAS_BAWAH = 0.0            # batas bawah semua gain dan Alpha
OPTUNA_LANGKAH = 0.01             # 2 angka di belakang koma
# Alpha = 0 membuat CBF-QP tidak aktif (status ALPHA_NOL), jadi batas bawah Alpha = OPTUNA_LANGKAH.

# Batas ATAS pencarian tiap variabel. Edit sesuai kebutuhan.
# Variabel yang tidak ada di tabel (misalnya Alpha_7 nanti) memakai OPTUNA_BATAS_ATAS_LAINNYA.
OPTUNA_BATAS_ATAS = {
    "Kp_psi":   500.00,
    "Kd_psi":   250.00,
    "Kp_theta": 500.00,
    "Kd_theta": 50.00,
    "Alpha_1":  500.00,
    "Alpha_2":  500.00,
    "Alpha_3":  500.00,
    "Alpha_4":  500.00,
    "Alpha_5":  500.00,
    "Alpha_6":  500.00,
}
OPTUNA_BATAS_ATAS_LAINNYA = 500.0

# Profil optimasi. Tiap profil punya study Optuna sendiri (hasilnya tidak tercampur):
#   tahap PD  -> ATERA_PD_<PROFIL>        tahap CBFQP -> ATERA_PD+CBF_<PROFIL>
# kt_efektif  : torsi roda nyata per A arus perintah [N m/A], menggantikan MOTOR_KT_EFEKTIF saat tuning
# gaya_dorong : gaya dorong saat tuning, menggantikan GAYA_DORONG_AWAL saat tuning
# Jalankan:  python3 tuning_optuna.py --profil TERUKUR --tahap PD
OPTUNA_PROFIL = {
    "TERUKUR": {
        "kt_efektif": 0.238,        # hasil uji delay 20261009 (G x j_theta)
        "gaya_dorong": 11.0,        # [N] ASUMSI: dorongan terbesar yang masih bisa dipulihkan pada 0.238 (uji simulasi)
        "terverifikasi": True,
        "catatan": "K diturunkan dari uji inersia/delay, bergantung pada j_theta = 9.2e-4",
    },
    "DATASHEET": {
        "kt_efektif": 0.75,         # datasheet Waveshare DDSM115 (= MOTOR_KT)
        "gaya_dorong": 26.0,        # [N]
        "terverifikasi": False,
        "catatan": "BELUM TERVERIFIKASI: nilai 0.75 belum cocok dengan uji delay (0.24). "
                   "BUTUH PENGUJIAN TORSI RODA (ddsm115_torsi_statis_testing.py)",
    },
}
OPTUNA_ZETA_DEG = 0.0               # [deg] lantai saat optimasi
OPTUNA_ZETA_UJI_DEG = 5.2           # [deg] lantai untuk menguji hasil terbaik (tidak ikut dioptimasi)
OPTUNA_ARAH_DORONG = [1]            # 1 = dorong depan, -1 = dorong belakang; satu pengujian per isi daftar
OPTUNA_WAKTU_SEBELUM_DORONG = 3.0   # [s] ASUMSI: robot dibiarkan tenang dulu sebelum didorong (tidak dinilai)
OPTUNA_DURASI_UJI = 30.0            # [s] lama perekaman tiap pengujian sejak dorongan (terpisah dari DURASI_PENGUJIAN)
OPTUNA_SEED = 1                     # seed sampler Optuna dan noise IMU (semua trial dinilai dengan noise yang sama)
OPTUNA_PENALTI_KOMET = 1000.0
OPTUNA_PENALTI_JARAK = 10.0         # [per detik]
OPTUNA_PENALTI_GAGAL = 10.0         # [per detik]
# Pruning (auto-stop trial yang jelek): skor dilaporkan tiap 1 detik; trial dihentikan bila skornya lebih buruk
# dari median trial sebelumnya pada detik yang sama. Baru aktif setelah OPTUNA_PRUNER_STARTUP trial selesai dan
# setelah OPTUNA_PRUNER_WARMUP_S detik pertama.
OPTUNA_PRUNER_STARTUP = 10
OPTUNA_PRUNER_WARMUP_S = 5