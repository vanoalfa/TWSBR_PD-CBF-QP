"""config.py - satu-satunya sumber nilai program ATERA (konstanta fisik, gain, batas).

Satuan SI (kg, m, s, rad, N m) kecuali nama berakhiran _DEG, _A, _HZ, _RPM.

Konvensi tanda (dipakai model.py dan semua file lain):
    state x = [theta, dtheta, psi, dpsi]
    psi   > 0 : badan miring ke DEPAN, diukur dari garis vertikal (gravitasi)
    theta > 0 : roda menggelinding ke DEPAN, sudut roda absolut
    zeta  > 0 : bidang menanjak ke arah DEPAN
    u     > 0 : torsi motor SATU roda, mendorong roda ke DEPAN (reaksinya mendorong badan ke belakang)
                kedua roda diberi u yang sama, jadi torsi total = N_WHEEL*u
"""

import math

# =============================================================================
# 1. PARAMETER FISIK  (<proyek>)
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
# 2. INDEKS STATE  (<kontrak>)
# =============================================================================
IDX_THETA = 0
IDX_DTHETA = 1
IDX_PSI = 2
IDX_DPSI = 3
NX = 4
NU = 1

# =============================================================================
# 3. SETPOINT
# =============================================================================
theta_setpoint = 0.0                # [rad]    sudut roda dinolkan saat balancing dimulai
psi_setpoint = 0.0                  # [rad]
dpsi_setpoint = 0.0                 # [rad/s]
dtheta_setpoint = 0.0               # [rad/s]

# =============================================================================
# 4. SAFETY SET  (batas simetris: min = -max)
# =============================================================================
psi_max = math.pi / 12.0            # [rad]    |psi|    <= psi_max
dpsi_max = 2.0 * math.pi            # [rad/s]  |dpsi|   <= dpsi_max
dtheta_max = 2.0 * math.pi          # [rad/s]  |dtheta| <= dtheta_max
psi_min = -psi_max
dpsi_min = -dpsi_max
dtheta_min = -dtheta_max
s_max = 1.0                         # [m]      jarak tempuh s = R*theta (dari posisi awal), |s| <= s_max
s_min = -s_max

# =============================================================================
# 5. GAIN PD  (input manual, default 0)
# =============================================================================
# u_PD = BALANCE_DIRECTION_SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi
#                                  + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah
# error = setpoint - state,   arah = -1 / 0 / +1 dari D-Pad joystick
Kp_psi = 15.0           # [N m/rad]    semua gain menghasilkan torsi SATU roda
Kd_psi = 1.5           # [N m s/rad]
Kp_theta = 0.02         # [N m/rad]    hanya aktif di Mode Balancing (Tombol B)
Kd_theta = 0.04         # [N m s/rad]
Kvel = -0.05             # [N m]

# Dengan error = setpoint - state dan u > 0 = roda ke depan, gain positif menstabilkan bila tandanya -1
# (sama dengan apalah_bisa/tunning.py).
BALANCE_DIRECTION_SIGN = -1.0
# BUTUH PENGUJIAN ARAH KOREKSI: gain kecil, badan dimiringkan ke depan, roda harus berputar ke DEPAN.

# =============================================================================
# 6. GAIN CBF-QP  (input manual, default 0; alpha_i(h) = Alpha_i * h)
# =============================================================================
Alpha_1 = 40.0          # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 40.0          # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 40.0          # [1/s]  h3, h4 (dpsi, CBF orde 1)
Alpha_4 = 40.0          # [1/s]  h5, h6 (jarak tempuh s, HOCBF orde 2)
Alpha_5 = 40.0          # [1/s]  h5, h6 (jarak tempuh s, HOCBF orde 2)
Alpha_6 = 10.0          # [1/s]  h7, h8 (dtheta, CBF orde 1)

CBF_N_HORIZON = 1      # N pada HPIPM (acados)

# =============================================================================
# 7. MOTOR DDSM115  (wiki Waveshare DDSM115)
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

# True: encoder mengukur roda relatif terhadap badan, theta = theta_encoder + psi.
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
# 8. IMU MPU6050  (datasheet InvenSense MPU-6000/6050)
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
# 9. JOYSTICK  (Fantech WGP-13s)
# =============================================================================
JOYSTICK_PORT = "/dev/input/event5"
JOYSTICK_AMBANG_ANALOG = 0.5        # analog dianggap ditekan bila simpangannya di atas nilai ini (0..1)

# ASUMSI: torsi tambahan untuk belok (N m). Roda kiri diberi u - TORSI_BELOK*belok, roda kanan u + TORSI_BELOK*belok.
TORSI_BELOK = 0.02

# =============================================================================
# 10. RUNTIME / KEAMANAN / EVALUASI
# =============================================================================
CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ               # [s]
UI_HZ = 20.0
SAFE_TILT_DEG = 30.0                # |psi| di atas ini -> roda dimatikan
ISE_PLOT_DIR = "PLOT_EVALUASI"

# =============================================================================
# 11. REALISME SIMULASI  (hanya dipakai simulasi.py, tidak dipakai robot fisik)
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
# 12. TUNING OPTUNA  (hanya dipakai tuning_optuna.py)
# =============================================================================
# Satu trial = dua pengujian di lantai OPTUNA_ZETA_DEG: robot didorong ke depan, lalu (diletakkan ulang)
# didorong ke belakang. Tiap pengujian direkam DURASI_PENGUJIAN detik (atera_main.py) sejak dorongan,
# gaya dorong = GAYA_DORONG_AWAL (simulasi.py).
# Skor (makin kecil makin baik), dihitung dari state SEBENARNYA di PyBullet:
#   skor = integral[(psi/psi_max)^2 + (s/s_max)^2] dt
#        + OPTUNA_PENALTI_JARAK      x lama |s| > s_max          [per detik]
#        + OPTUNA_PENALTI_INFEASIBLE x lama status CBF bukan OK  [per detik, hanya tahap PD+CBF]
#        + OPTUNA_PENALTI_JATUH x (1 + sisa waktu / waktu total)  bila |psi| > SAFE_TILT_DEG atau |s| > OPTUNA_S_HENTI
OPTUNA_TRIAL_PD = 163221013               # jumlah trial tahap 1 (gain PD)
OPTUNA_TRIAL_CBF = 163221013              # jumlah trial tahap 2 (Alpha, gain PD dikunci ke hasil tahap 1)
OPTUNA_BATAS_BAWAH = 0.0            # rentang pencarian semua gain dan Alpha
OPTUNA_BATAS_ATAS = 500.0
OPTUNA_LANGKAH = 0.01             # 2 angka di belakang koma
# Alpha = 0 membuat CBF-QP tidak aktif (status ALPHA_NOL), jadi batas bawah Alpha = OPTUNA_LANGKAH.
OPTUNA_ZETA_DEG = 0.0               # [deg] lantai saat optimasi
OPTUNA_ZETA_UJI_DEG = 5.2           # [deg] lantai untuk menguji hasil terbaik (tidak ikut dioptimasi)
OPTUNA_WAKTU_SEBELUM_DORONG = 1.0   # [s] ASUMSI: robot dibiarkan tenang dulu sebelum didorong (tidak dinilai)
OPTUNA_SEED = 1                     # seed sampler Optuna dan noise IMU (semua trial dinilai dengan noise yang sama)
OPTUNA_PENALTI_JATUH = 1000.0
OPTUNA_PENALTI_JARAK = 10.0         # [per detik]
OPTUNA_PENALTI_INFEASIBLE = 10.0    # [per detik]
OPTUNA_S_HENTI = 3.0                # [m] |s| di atas ini dianggap lari (dihentikan seperti jatuh)
# Pruning (auto-stop trial yang jelek): skor dilaporkan tiap 1 detik; trial dihentikan bila skornya lebih buruk
# dari median trial sebelumnya pada detik yang sama. Baru aktif setelah OPTUNA_PRUNER_STARTUP trial selesai dan
# setelah OPTUNA_PRUNER_WARMUP_S detik pertama.
OPTUNA_PRUNER_STARTUP = 10
OPTUNA_PRUNER_WARMUP_S = 5