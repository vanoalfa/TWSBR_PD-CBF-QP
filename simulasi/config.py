"""config.py - satu-satunya sumber nilai program ATERA (konstanta fisik, gain, batas, pengaturan simulasi).

Satuan SI (kg, m, s, rad, N m) kecuali nama berakhiran _DEG, _A, _HZ, _RPM.

Konvensi tanda (dipakai model.py dan semua file lain):
    state x = [theta, dtheta, psi, dpsi]
    psi   > 0 : badan miring ke DEPAN, diukur dari garis vertikal (gravitasi)
    theta > 0 : roda menggelinding ke DEPAN, sudut roda absolut (definisinya ada di model.hitung_theta)
    zeta  > 0 : bidang menanjak ke arah DEPAN
    u     > 0 : torsi motor SATU roda, mendorong roda ke DEPAN (reaksinya mendorong badan ke belakang)
            kedua roda diberi u yang sama, jadi torsi total = N_WHEEL*u

Mode gerak (pd_control.py: MODE_DIAM, MODE_BALANCING, MODE_BEBAS):
    DIAM      : default. Tanpa Kp_theta (posisi tidak ditahan).
    BALANCING : tombol B (toggle dengan DIAM, robot fisik: Tombol B). Kp_theta aktif, KOMET s aktif. Setpoint theta
                (titik nol s, s = R*theta) dipasang saat masuk BALANCING dan saat W / S dilepas.
    BEBAS     : tombol F (robot fisik: Tombol A). Tanpa Kp_theta dan PD heading, belok = selisih torsi TORSI_BELOK.
                F lagi -> DIAM.
    W / S (Kvel*arah) berlaku di semua mode; di BALANCING, Kp_theta diabaikan selama W / S ditahan.
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
# Safety set CBF-QP (batas simetris: min = -max): h1, h2 = psi, h3, h4 = dtheta
psi_max_DEG = 10.0                  # [deg]    batas psi yang dijaga CBF (h1, h2); bisa diubah di jendela Kontrol
                                    #          (= robot fisik; 10 deg supaya koreksi CBF-QP terlihat jelas dibanding PD)
psi_max = math.radians(psi_max_DEG) # [rad]    |psi|    <= psi_max
# ASUMSI: sedikit di bawah batas kecepatan motor DDSM115 (no-load 200 rpm = 20.9 rad/s, wiki Waveshare), = robot fisik.
# Dengan 13 rad/s barrier dtheta melarang roda mengejar badan setelah dorongan ~10 N (PD butuh ~19 rad/s) -> jatuh.
dtheta_max = 20.0                   # [rad/s]  |dtheta| <= dtheta_max
psi_min = -psi_max
dtheta_min = -dtheta_max
# Bukan constraint CBF-QP (dipantau di plot / GUI):
dpsi_max = 2.0 * math.pi            # [rad/s]  garis acuan |dpsi|
dpsi_min = -dpsi_max
s_max = 0.5                         # [m]      batas |s| (s = R*theta dari titik nol s): syarat lolos tuning mode
                                    #          BALANCING (S2) dan normalisasi ISE s
s_min = -s_max

# KOMET = kondisi robot yang sudah tidak tertolong, berlaku di mode PD maupun PD+CBF. Roda dimatikan.
#   Jatuh    : |psi| > SAFE_TILT_DEG                (semua mode gerak)
#   Lari     : |s|   > S_KOMET                      (hanya mode BALANCING tanpa W / S)
#   Berputar : |yaw - yaw_ref| > YAW_KOMET_DEG      (mode DIAM dan BALANCING; di mode BEBAS yaw memang bebas)
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
Kp_psi = 12.12                    # [N m/rad]    semua gain menghasilkan torsi SATU roda
Kd_psi = 02.27                      # [N m s/rad]
Kp_theta = 0.15                     # [N m/rad]    hanya mode BALANCING tanpa W / S
Kd_theta = 0.32                     # [N m s/rad]
# Kvel*arah saat W / S ditahan. Kecepatan tunak di lantai datar: v = -R*Kvel*arah/Kd_theta
# -> Kvel NEGATIF supaya W maju (Kd_theta 0.29: Kvel -1.16 -> 0.20 m/s), sama dengan robot fisik.
Kvel = -1.16                        # [N m]

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
Alpha_1 = 07.42                     # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_2 = 08.45                     # [1/s]  h1, h2 (psi, HOCBF orde 2)
Alpha_3 = 269.18                    # [1/s]  h3, h4 (dtheta, CBF orde 1)

# Nama semua Alpha di atas: Alpha_1, Alpha_2, ... (berurutan, berhenti di nomor pertama yang tidak ada).
# Alpha baru cukup ditambahkan di atas, lalu otomatis dipakai cbf_qp.py, simulasi.py (jendela Kontrol), dan ise.py (kolom CSV).
NAMA_ALPHA = []
while ("Alpha_%d" % (len(NAMA_ALPHA) + 1)) in globals():
    NAMA_ALPHA.append("Alpha_%d" % (len(NAMA_ALPHA) + 1))

# QP solver CBF-QP (cbf_qp.py):
#   "acados" : acados (HPIPM, N = ACADOS_N_HORIZON), pengaturan di bagian 6b
#   "eksak"  : solusi pasti numpy (tanpa iterasi)
#   plugin ca.conic CasADi: "daqp", "qpoases", "osqp", "proxqp", "qrqp"
# Uji Raspi 20261020 (--banding-solver, 3000 kasus): daqp gagal 0 %, 0.20 ms; qpoases gagal 26 %; osqp gagal 28 %.
# Bandingkan:  python3 cbf_qp.py --banding-solver
CBF_QP_SOLVER = "acados"
# Daftar --banding-solver (acados selalu diuji terakhir). Tidak dimasukkan (uji 20261025): "hpipm" CasADi menolak
# struktur QP ini, "highs" bisa macet (iterasi tanpa akhir), "ipqp" gagal di semua kasus.
CBF_QP_SOLVER_BANDING = ["eksak", "daqp", "qpoases", "osqp", "proxqp", "qrqp", "acados"]
# Slack, margin, DOB (= robot fisik, penjelasan lengkap di cbf_qp.py dan program/atera_program/config.py)
CBF_SLACK_AKTIF = True
# bobot slack per constraint (exact penalty, urutan DAFTAR_C h1..h4): yang kecil dilonggarkan dulu bila bertabrakan
CBF_RHO_SLACK = [1.0e6, 1.0e6, 1.0e2, 1.0e2]    # h1 h2 (psi) | h3 h4 (dtheta)
CBF_SLACK_EPS = 1.0e-4
CBF_TOL_SLACK = 1.0e-6
CBF_MARGIN_DEG = 1.0                # [deg] barrier psi = psi_max - margin (garis merah plot tetap di psi_max)
DOB_AKTIF = True                    # True: d_hat dipakai CBF-QP. False: hanya dicatat
DOB_TAU_S = 0.020                   # [s]
DOB_BATAS = 200.0                   # [rad/s^2]
# True  = CBF-QP (dan DOB) memakai kemiringan lantai config.zeta: ikut berubah saat zeta_DEG diubah di jendela Kontrol
#         dan saat tuning_optuna.py menguji bidang miring. Robot fisik: isi zeta_DEG sesuai bidang (+ = menanjak ke
#         depan robot), program/atera_program/config.py.
# False = model CBF memakai zeta = 0 (kemiringan hanya ditaksir DOB).
CBF_TAHU_ZETA = True

# Toleransi numerik (bukan besaran fisik)
CBF_INFTY = 1.0e9                   # batas atas "tak hingga" constraint c di QP
CBF_TOL_CONSTRAINT = 1.0e-6         # pelanggaran constraint yang masih diterima saat memeriksa hasil solver
CBF_TOL_BATAS_U = 1.0e-6            # [N m] pelanggaran batas u yang masih diterima
CBF_TOL_AKTIF = 1.0e-4              # [N m] |u* - u_PD| di atas ini -> filter dianggap mengoreksi
CBF_TOL_KOEFISIEN = 1.0e-12         # |koefisien u| di bawah ini dianggap nol

# =============================================================================
#                              6b. ACADOS  (CBF_QP_SOLVER = "acados", WORKSPACE ACADOS di cbf_qp.py)
# =============================================================================
# OCP: stage 0 = CBF-QP (x dikunci ke state terukur, variabel u), stage 1..N = prediksi tanpa cost / constraint.
ACADOS_N_HORIZON = 1                # N pada HPIPM (batasan skripsi: N = 1)
ACADOS_NLP_SOLVER = "SQP_RTI"       # satu QP per siklus (constraint affine terhadap u -> hasil tepat). "SQP" juga bisa
ACADOS_QP_SOLVER = "PARTIAL_CONDENSING_HPIPM"   # atau "FULL_CONDENSING_HPIPM"
# Uji 20261025 (12000 kasus acak di sekitar safety set, termasuk yang butuh slack; acados v0.5.1, x86):
#   ROBUST + mu0 1e5 : gagal 0 %, |u - u_eksak| < 1e-13 kecuali 1 kasus ekstrem (0.08 N m)
#   ROBUST + mu0 1e4 : gagal 2 kasus;   SPEED : gagal 7 % (mu0 1e4) sampai 58 % (mu0 bawaan), saat slack dibutuhkan
ACADOS_HPIPM_MODE = "ROBUST"        # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST"
ACADOS_QP_MU0 = 1.0e5               # barrier awal HPIPM (0 = bawaan hpipm_mode)
ACADOS_QP_ITER_MAX = 50             # iterasi maksimum HPIPM per siklus
ACADOS_PRINT_LEVEL = 0              # 0 = diam
# True  = kode C solver dibuat dan di-compile ulang tiap program mulai (wajib setelah WORKPLACE / jumlah Alpha /
#         ACADOS_* berubah).
# False = pakai kode C di ACADOS_FOLDER_KODE yang sudah ada (mulai lebih cepat); dibuat otomatis bila belum ada.
ACADOS_BUILD = True
ACADOS_FOLDER_KODE = "c_generated_code"  # relatif terhadap folder cbf_qp.py
# Folder acados (berisi interfaces/acados_template). Kosong = dicari otomatis: variabel ACADOS_SOURCE_DIR, folder
# "acados" di sebelah / di atas folder program ini, ~/acados, atau lokasi pip install -e acados_template.
ACADOS_SOURCE_DIR = ""

# =============================================================================
#                              7. MOTOR DDSM115  (wiki Waveshare DDSM115)
# =============================================================================
# Port, ID, arah, dan skala komunikasi motor hanya dipakai robot fisik (program/atera_program/config.py bagian 6).
# True: encoder mengukur roda relatif terhadap badan, theta = theta_encoder + psi (lihat model.hitung_theta).
ENCODER_RELATIVE_TO_BODY = True
# BUTUH PENGUJIAN ENCODER: arus 0, tahan roda di lantai, miringkan badan 10 deg dengan tangan;
# posisi encoder berubah ~10 deg = True, tidak berubah = False.

MOTOR_KT = 0.75                     # [N m/A]  torque constant (datasheet)
# BUTUH PENGUJIAN BEBAN TAMBAHAN (ddsm115_inersia_testing.py --j-add --g-ref): uji inersia 20261004 memberi
# G = K/J = 260 rad/s^2 per A perintah, yaitu K = 0.24 N m/A untuk j_theta = 9.2e-4 (bukan 0.75).
MOTOR_STALL_CURRENT_A = 2.7         # [A]      locked-rotor current (datasheet; rated 1.5 A / 0.96 N m)
MOTOR_NO_LOAD_SPEED_RPM = 200.0     # [rpm]    +-10

# ASUMSI: batas arus per roda mengikuti apalah_bisa/tunning.py (di atas rated 1.5 A, di bawah stall 2.7 A).
MAX_CURRENT_A = 2.50                # [A] per roda
# BUTUH PENGUJIAN ARUS MAKSIMUM: jalankan balancing di 1.80 A, pantau error code dan suhu DDSM115.
MIN_CURRENT_A = -MAX_CURRENT_A

# Arus per roda:  i = u / MOTOR_KT
# Batas arus pada CBF-QP:  MIN_CURRENT_A <= u / MOTOR_KT <= MAX_CURRENT_A  <=>  u_min <= u <= u_max
u_max = MOTOR_KT * MAX_CURRENT_A                      # [N m] torsi satu roda
u_min = MOTOR_KT * MIN_CURRENT_A                      # [N m]

# =============================================================================
#                              8. IMU MPU6050  (datasheet InvenSense MPU-6000/6050) DAN KALMAN
# =============================================================================
IMU_AXIS = "pitch"
L_IMU = 0.312739                    # [m] jarak CoM kedua roda (poros) ke IMU, diukur di Fusion 360
# I2C, skala sensor, dan kalibrasi hanya dipakai robot fisik (program/atera_program/config.py bagian 7).

# =============================================================================
#                              9. RUNTIME DAN EVALUASI ISE
# =============================================================================
CONTROL_HZ = 200.0
DT = 1.0 / CONTROL_HZ               # [s]
ISE_PLOT_DIR = "PLOT_EVALUASI"

# ASUMSI: satu pengujian (tombol T) = DURASI_PENGUJIAN detik waktu simulasi sejak dorongan O / P dimulai.
DURASI_PENGUJIAN = 30.0             # [s]
# zeta baru dipakai bila nilainya di jendela Kontrol tidak berubah selama ini (supaya robot tidak diletakkan
# ulang berkali-kali saat angkanya masih diketik).
ZETA_TUNGGU = 0.5                   # [s]

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
# Batas yang constraint-nya tidak aktif (tidak dicentang / tidak dipakai mode itu) atau bukan constraint (dpsi, s)
# digambar abu-abu tipis.
CBF_BARRIER_H = {"psi": (1, 2), "dtheta": (3, 4)}
PLOT_NONAKTIF_WARNA = "0.6"
PLOT_WARNA_PD = "tab:purple"        # kurva mode PD (ungu)
PLOT_WARNA_PD_CBF = "tab:blue"      # kurva mode PD + CBF-QP (biru)

# =============================================================================
#                              10. REALISME SIMULASI  (hanya dipakai simulasi.py, tidak dipakai robot fisik)
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
#                              11. PYBULLET (simulasi.py)
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
WARNA_QT_MODE = "#544bb8"        # mode BALANCING (tombol B) / BEBAS (tombol F) sedang aktif
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
#                              12. TUNING OPTUNA  (hanya dipakai tuning_optuna.py)
# =============================================================================
# SATU TAHAP (SEMUA), satu study:  python3 tuning_optuna.py --profil DATASHEET
# Semua OPTUNA_VARIABEL dicari bersamaan. Satu trial = OPTUNA_BLOK berurutan:
#   PD_DIAM       : mode DIAM (Kp_theta mati) | PD saja       (Kp_psi, Kd_psi, Kd_theta)
#   CBF_DIAM      : mode DIAM                 | PD + CBF-QP   (+ Alpha_1 .. Alpha_3), acuan = PD_DIAM trial itu
#   PD_BALANCING  : mode BALANCING            | PD saja       (+ Kp_theta)
#   CBF_BALANCING : mode BALANCING            | PD + CBF-QP   acuan = PD_BALANCING trial itu
# Tiap blok menjalankan S1 -> S2 -> S3 (blok "SKENARIO S1 S2 S3" di bawah):
#   skor blok  = -OPTUNA_BOBOT_GAYA x gaya_lolos + skor pengujian S1 - OPTUNA_BOBOT_MIRING x miring_lolos
#                - (OPTUNA_BOBOT_S3 bila S3 lolos)
#   skor trial = jumlah skor blok (makin kecil makin baik)
# SYARAT blok PD + CBF-QP, tidak boleh lebih buruk dari acuan PD:
#   S1 langsung diuji pada gaya lolos acuan, S2 langsung pada kemiringan lolos acuan. Bila acuan lolos S3, PD + CBF-QP
#   juga harus lolos S3 (di gaya dan kemiringan lolosnya sendiri, atau di gaya dan kemiringan acuan).
#   Gagal syarat, atau komet pada gaya terendah S1 -> trial BERHENTI, skor = skor yang sudah dihitung
#   + OPTUNA_PENALTI_SYARAT.
# MODE DIAM (blok PD_DIAM dan CBF_DIAM): tanpa syarat posisi (overshoot s, |s| <= s_max di S2, KOMET s).
#   Robot yang meluncur tetap mendapat skor lebih buruk lewat suku s di ISE S1.
# Study tidak boleh dilanjutkan bila skenario, nilai terkunci, atau fisika simulasi berubah (program berhenti dan
# menjelaskan bedanya; hapus study dengan --hapus).
#
# SATU PENGUJIAN = OPTUNA_ULANGAN kali skenario berikut, robot di-reset di awal tiap ulangan. Di dalam satu ulangan:
#   1. Tenang awal  : robot diletakkan, ditunggu sampai STABIL (maks OPTUNA_DURASI_STABIL detik, tidak dinilai ISE).
#   2. Dorongan     : didorong sesuai urutan OPTUNA_ARAH_DORONG, berjarak acak OPTUNA_JARAK_ACAK_MIN .. MAKS detik
#                     (seed OPTUNA_SEED_JARAK + ulangan, sama untuk semua trial), stabil atau belum.
#   3. Setelah dorongan terakhir: ditunggu sampai STABIL (maks OPTUNA_DURASI_STABIL detik).
#   Ulangan ke-k: noise IMU seed OPTUNA_SEED + k, dorongan pertama ditunda k x OPTUNA_GESER_DORONG detik.
# STABIL = |psi| < OPTUNA_STABIL_PSI, |dpsi| < OPTUNA_STABIL_DPSI, |dtheta| < OPTUNA_STABIL_DTHETA terus-menerus selama
#   OPTUNA_STABIL_LAMA detik (hanya menentukan kapan dorongan pertama dan akhir ulangan, tanpa penalti).
# Skor satu pengujian (state yang dibaca kontrol, psi hasil Kalman):
#   skor = integral[(psi/psi_max)^2 + (s/s_max)^2] dt              [sejak dorongan pertama tiap ulangan]
#        + OPTUNA_PENALTI_GAGAL x lama status CBF bukan AMAN        [per detik, hanya PD + CBF-QP]
#        + OPTUNA_PENALTI_KOMET x (1 + sisa waktu / waktu total)    bila KOMET (pengujian langsung berhenti)
# CHATTERING = torsi u bolak-balik di antara batas jenuhnya: ch/s = berapa kali per detik u berbalik dari >= +a ke
#   <= -a (atau sebaliknya), a = OPTUNA_AMBANG_CHATTERING x u_max. ch/s > OPTUNA_BATAS_CHATTERING = KOMET.
OPTUNA_LANGKAH = 0.01                  # 2 angka di belakang koma: gain kecil butuh ketelitian ini

# Batas pencarian PER PROFIL: OPTUNA_BATAS[profil][nama] = (batas bawah, batas atas). Mengubahnya tidak membuat study
# ditolak. Alpha = 0 membuat CBF-QP tidak aktif (status ALPHA_NOL), jadi batas bawah Alpha harus > 0.
# DATASHEET (analisis 20261017-20): Kd_psi > 5 chattering; Kp_theta >= 0.15 supaya galat posisi di bidang miring kecil
# (s_eq = R*(u_eq - Kp_psi*psi_eq)/Kp_theta); Alpha_1, Alpha_2 kecil = CBF mulai mengerem lebih awal (ASUMSI 20261020).
OPTUNA_BATAS = {
    "TERUKUR": {
        "Kp_psi":   (5.0, 80.0),
        "Kd_psi":   (0.5, 9.0),
        "Kd_theta": (0.0, 1.5),
        "Kp_theta": (0.15, 4.0),
        "Alpha_1":  (5.00, 28.28),
        "Alpha_2":  (5.00, 28.28),
        "Alpha_3":  (11.00, 100.0),
    },
    "DATASHEET": {
        "Kp_psi":   (5.0, 25.0),
        "Kd_psi":   (1.5, 5.5),
        "Kd_theta": (0.1, 5.0),
        "Kp_theta": (0.15, 5.0),
        "Alpha_1":  (5.00, 55.00),
        "Alpha_2":  (5.00, 55.00),
        "Alpha_3":  (1.00, 500.0),
    },
}

# Variabel yang dituning (batasnya di OPTUNA_BATAS). Yang tidak ada di sini dikunci dari config.py.
OPTUNA_VARIABEL = ["Kp_psi", "Kd_psi", "Kd_theta", "Alpha_1", "Alpha_2", "Alpha_3", "Kp_theta"]
# Urutan blok satu trial: (nama, mode gerak "DIAM" / "BALANCING", kontrol "PD" / "PD+CBF").
# Blok PD+CBF memakai acuan blok PD dengan mode yang sama sebelumnya.
OPTUNA_BLOK = [
    ("PD_DIAM", "DIAM", "PD"),
    ("CBF_DIAM", "DIAM", "PD+CBF"),
    ("PD_BALANCING", "BALANCING", "PD"),
    ("CBF_BALANCING", "BALANCING", "PD+CBF"),
]
# Jumlah trial maksimum (python3 tuning_optuna.py --trial N menggantikannya); berhenti lebih awal bila stagnan
# (OPTUNA_STAGNANT)
OPTUNA_TRIAL = 163221013

# Profil optimasi (study: ATERA_SEMUA_<PROFIL>):
# kt_efektif  : torsi roda nyata per A arus perintah [N m/A], menggantikan MOTOR_KT_EFEKTIF saat tuning
# gaya_dorong : [N] gaya dorong terendah yang dicoba S1
OPTUNA_PROFIL = {
    "TERUKUR": {
        "kt_efektif": 0.238,        # hasil uji delay 20261009 (G x j_theta)
        "gaya_dorong": 5.0,
        "terverifikasi": True,
        "catatan": "K diturunkan dari uji inersia/delay, bergantung pada j_theta = 9.2e-4",
    },
    "DATASHEET": {
        "kt_efektif": 0.75,         # datasheet Waveshare DDSM115 (= MOTOR_KT)
        "gaya_dorong": 5.0,
        "terverifikasi": False,
        "catatan": "BELUM TERVERIFIKASI: nilai 0.75 belum cocok dengan uji delay (0.24). "
                   "BUTUH PENGUJIAN TORSI RODA (ddsm115_torsi_statis_testing.py)",
    },
}
OPTUNA_ZETA_DEG = 0.0               # [deg] lantai S1
OPTUNA_ZETA_UJI_DEG = 5.2           # [deg] lantai uji ulang hasil terbaik bila kemiringan lolos S2 = 0
OPTUNA_ARAH_DORONG = [1, -1, 1, -1]  # urutan dorongan: 1 = dorong depan, -1 = dorong belakang (tanpa reset di antaranya)
OPTUNA_JARAK_ACAK_MIN = 8.0        # [s] ASUMSI: dorongan bisa datang sebelum robot tenang (kondisi buruk)
OPTUNA_JARAK_ACAK_MAKS = 15.0       # [s] ASUMSI: rata-rata 15 s
OPTUNA_SEED_JARAK = 1001            # seed jarak acak ulangan pertama (terpisah dari seed noise IMU)
OPTUNA_STABIL_PSI = 0.15707963      # [rad]   |psi|    < nilai ini (10 deg)
OPTUNA_STABIL_DPSI = 0.17453293     # [rad/s] |dpsi|   < nilai ini (10 deg/s)
OPTUNA_STABIL_DTHETA = 0.17453293   # [rad/s] |dtheta| < nilai ini (10 deg/s)
OPTUNA_STABIL_LAMA = 3.0            # [s] ketiga syarat di atas harus terpenuhi TERUS-MENERUS selama ini (ASUMSI)
OPTUNA_DURASI_STABIL = 20.0         # [s] batas waktu MAKSIMUM menunggu stabil (tenang awal dan setelah dorongan terakhir)
OPTUNA_SEED = 1                     # seed sampler Optuna dan noise IMU ulangan pertama (ulangan ke-k: OPTUNA_SEED + k)
OPTUNA_ULANGAN = 2                  # jumlah ulangan skenario (dengan reset) dalam satu pengujian
OPTUNA_GESER_DORONG = 0.13          # [s] ASUMSI: pergeseran waktu dorong per ulangan
OPTUNA_GAYA_MAKS = 100.0            # [N] gaya tertinggi yang dicoba S1. Dengan LAMA_DORONG 0.015 s 100 N = 1.5 N s,
                                    #     di atas batas robot (~1.0 N s, model planar, ~65-75 N).
OPTUNA_GAYA_LANGKAH = 0.5           # [N] ketelitian pencarian gaya S1
OPTUNA_BOBOT_GAYA = 1000.0          # [per N] bobot gaya lolos di skor (jauh lebih besar dari ISE)
OPTUNA_AMBANG_CHATTERING = 0.5      # fraksi u_max; |u| di atas ini dihitung torsi besar (ch/s)
OPTUNA_MULAI_DARI_CONFIG = True     # study BARU: trial pertama = nilai config.py (titik yang sudah diketahui)
# Penalti gagal syarat PD + CBF-QP / komet gaya terendah. 1e6 > selisih skor terbesar yang mungkin (4 blok x
# (100 N x 1000 + 10 deg x 5000 + 5000) = 6.2e5), jadi trial yang gagal SELALU lebih buruk dari trial yang lolos.
OPTUNA_PENALTI_SYARAT = 1.0e7
# MULAI DARI TERBAIK: S1 dan S2 blok PD dimulai dari hasil trial terbaik sejauh ini (langkah naik / turun
# dilipatgandakan 1, 2, 4, ... kisi, lalu bisection). Hasil sama dengan bisection biasa, pengujian lebih sedikit.
OPTUNA_MULAI_DARI_TERBAIK = True
# PEMANGKASAN AMAN: setelah S1 tiap blok, skor terbaik yang MASIH MUNGKIN dihitung (blok ini S2 lolos sampai
# OPTUNA_MIRING_MAKS dan S3 lolos, blok berikutnya lolos sampai OPTUNA_GAYA_MAKS, OPTUNA_MIRING_MAKS dan S3). Bila tetap
# tidak lebih baik dari trial terbaik, trial berhenti dengan skor = skor terbaik yang masih mungkin itu.
# Trial yang bisa menang tidak pernah dibuang.
OPTUNA_PANGKAS_AMAN = True

# -----------------------------------------------------------------------------------------------------------------
# >>>>>>>>>>  SKENARIO S1 S2 S3 TIAP BLOK  <<<<<<<<<<
#   S1 dorongan, lantai datar : gaya dorong dinaikkan (bisection kisi OPTUNA_GAYA_LANGKAH) sampai gagal -> gaya_lolos [N]
#   S2 bidang miring          : TANPA dorongan, kemiringan dinaikkan (bisection kisi OPTUNA_MIRING_LANGKAH .. MAKS)
#                               -> miring_lolos [deg]. OPTUNA_ULANGAN ulangan, robot diletakkan tegak lalu dibiarkan
#                               OPTUNA_MIRING_LAMA detik. BERHASIL bila tidak jatuh, ch/s <= OPTUNA_BATAS_CHATTERING,
#                               dan (hanya mode BALANCING) |s| <= s_max.
#   S3 dorongan, bidang miring: gaya_lolos S1 di kemiringan miring_lolos S2 (skenario dan syarat lolos S1).
#   S2 dan S3 baru dijalankan bila S1 punya gaya lolos (> 0); S3 butuh miring_lolos > 0.
# ASUMSI bisection: lolos pada gaya / kemiringan X berarti lolos pada nilai yang lebih kecil.
# ASUMSI bobot: satu langkah S2 (0.1 deg x 5000 = 500) = satu langkah S1 (0.5 N x 1000 = 500).
OPTUNA_MIRING_LANGKAH = 0.1         # [deg] kenaikan kemiringan tiap berhasil (kisi S2)
OPTUNA_MIRING_MAKS = 10.0           # [deg] kemiringan tertinggi S2 (kebutuhan skripsi 8 deg + margin 2 deg)
OPTUNA_MIRING_LAMA = 15.0           # [s] lama robot dibiarkan di bidang miring tiap ulangan S2
OPTUNA_BOBOT_MIRING = 5000.0        # [per deg] bobot kemiringan lolos S2 di skor
OPTUNA_BOBOT_S3 = 5000.0            # bonus skor bila S3 lolos
# -----------------------------------------------------------------------------------------------------------------

# -----------------------------------------------------------------------------------------------------------------
# >>>>>>>>>>  SYARAT LOLOS SATU PENGUJIAN  (UBAH DI SINI)  <<<<<<<<<<
# Satu pengujian dianggap LOLOS hanya bila SEMUA syarat terpenuhi; yang tidak memenuhi dihitung KOMET.
#   1. tidak jatuh      : |psi| <= SAFE_TILT_DEG, dan (hanya mode BALANCING) |s| <= S_KOMET
#   2. tidak chattering : ch/s <= OPTUNA_BATAS_CHATTERING
#   3. tidak berayun    : overshoot psi <= OPTUNA_SYARAT_OVERSHOOT_PSI, dan (hanya mode BALANCING) overshoot s <=
#                         OPTUNA_SYARAT_OVERSHOOT_S
# Overshoot dihitung untuk TIAP dorongan (overshoot_persen di tuning_optuna.py), yang terbesar dipakai:
#     e(t)      = sinyal - acuan          acuan = rata-rata OPTUNA_OVERSHOOT_ACUAN_S detik sebelum dorongan
#     puncak    = e dengan |e| terbesar sejak dorongan sampai dorongan berikutnya
#     overshoot = simpangan terbesar ke arah BERLAWANAN dengan puncak (setelah puncak) / |puncak| x 100 %
#     PD + CBF-QP: overshoot psi hanya dinilai SETELAH CBF-QP selesai mengoreksi dan psi kembali di dalam psi_max
#     (sesudah sampel terakhir dengan u != u_PD, status bukan AMAN, atau |psi| > psi_max). ASUMSI 20261020 (O-b).
# Hubungan overshoot dan rasio redaman zeta (sistem orde 2, zeta = -ln(OS) / sqrt(pi^2 + ln(OS)^2)):
#     overshoot  20 %   10 %   5 %    2 %    1 %    0 %
#     zeta       0.46   0.59   0.69   0.78   0.83   >= 1 (critically damped / overdamped)
# Isi 0 untuk mematikan satu syarat.
OPTUNA_SYARAT_OVERSHOOT_PSI = 30.0  # [%] ASUMSI: overshoot psi maksimum yang masih lolos
OPTUNA_SYARAT_OVERSHOOT_S = 5.0     # [%] ASUMSI: overshoot s maksimum yang masih lolos (mode BALANCING)
# Kp_theta (HANYA blok BALANCING, skor pengujian S1): jarak sekecil mungkin dan critically damped (overshoot s -> 0 %)
#   pen_s = OPTUNA_BOBOT_S_MAKS x |s| maks [m] + OPTUNA_BOBOT_OVERSHOOT_S x overshoot s [%]
OPTUNA_BOBOT_S_MAKS = 10000.0       # [per m] ASUMSI: 0.1 m = 1 N gaya lolos (OPTUNA_BOBOT_GAYA 1000 per N)
OPTUNA_BOBOT_OVERSHOOT_S = 200.0    # [per %] ASUMSI: 5 % overshoot s = 1 N gaya lolos
OPTUNA_BATAS_CHATTERING = 2.0       # [per s] ASUMSI: ch/s maksimum
# Teknis perhitungan overshoot (biasanya tidak perlu diubah):
OPTUNA_OVERSHOOT_ACUAN_S = 1.0      # [s] lama rata-rata sebelum dorongan yang dipakai sebagai acuan
OPTUNA_OVERSHOOT_RATA_S = 0.05      # [s] rata-rata bergerak sinyal sebelum dihitung (meredam noise IMU)
OPTUNA_OVERSHOOT_PUNCAK_PSI_DEG = 0.5   # [deg] puncak psi lebih kecil dari ini tidak dinilai
OPTUNA_OVERSHOOT_PUNCAK_S_M = 0.01      # [m]   puncak s lebih kecil dari ini tidak dinilai
# -----------------------------------------------------------------------------------------------------------------
OPTUNA_PENALTI_KOMET = 1000.0
OPTUNA_PENALTI_GAGAL = 10.0         # [per detik]
# AUTO-EXIT STAGNAN: tuning berhenti sendiri bila skor terbaik tidak MEMBAIK selama OPTUNA_STAGNANT trial
# selesai berturut-turut, lalu langsung lanjut ke laporan. Membaik = skor turun lebih dari
# OPTUNA_STAGNANT_MIN x max(1, |skor terbaik acuan|). Riwayat study ikut dihitung. 0 = mati (atau --stagnant N).
OPTUNA_STAGNANT = 300
OPTUNA_STAGNANT_MIN = 1.0e-3