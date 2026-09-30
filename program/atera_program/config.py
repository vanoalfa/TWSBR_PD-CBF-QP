# =========================
# Tunning Control
# =========================
# CATATAN PERUBAHAN (lihat pd_control.py untuk detail):
# - Kp/Kd lama (skala ternormalisasi 0.0 - 1.0, dipakai untuk loop psi saja)
#   diganti dengan 4 gain baru yang terpisah untuk loop psi (badan) dan loop
#   theta (roda), dan sekarang berupa bilangan bulat (int) karena clamp
#   output PD di pd_control.py sudah DIHILANGKAN sesuai permintaan.
# - Kp_theta = Kd_theta = 0 (default) membuat loop theta tidak berkontribusi
#   sama sekali, sehingga perilaku robot semirip mungkin dengan versi PD
#   single-loop sebelumnya yang sudah terbukti stabil.
#
# --- PERBAIKAN "OSILASI KASAR" (GAIN_SCALE) ---
# Karena command_normalized() di ddsm115.py TETAP membatasi input ke
# rentang +-1.0 (tidak diubah), gain integer TANPA skala ternyata jauh
# lebih "kasar" daripada gain lama yang berupa pecahan: Kp_psi=1 saja
# sudah membuat motor full-power hanya pada error ~1 derajat (dulu perlu
# ~5 derajat dengan Kp=0.200), sehingga PD berperilaku seperti relay/
# bang-bang dan menghasilkan osilasi kasar (limit cycle), bukan PD halus.
#
# GAIN_SCALE membagi hasil (Kp*error + Kd*rate) SEBELUM dikirim ke motor,
# sehingga Kp_psi/Kd_psi/Kp_theta/Kd_theta tetap bilangan BULAT (0 s.d.
# tak terbatas, sesuai permintaan awal) tapi tiap kenaikan 1 satuan hanya
# menggeser gain efektif sebesar 1/GAIN_SCALE -- resolusi tuning jadi
# halus lagi seperti versi float lama.
GAIN_SCALE = 1000

# Nilai di bawah dipilih supaya gain EFEKTIF (Kp_psi/GAIN_SCALE dst)
# MEREPRODUKSI PERSIS gain lama yang sudah terbukti stabil, sebagai titik
# awal yang aman:
#   Kp lama = 0.200  ->  Kp_psi / GAIN_SCALE = 200 / 1000 = 0.200
#   Kd lama = 0.014  ->  Kd_psi / GAIN_SCALE =  14 / 1000 = 0.014
# Untuk fine-tuning, naik/turunkan Kp_psi & Kd_psi dalam langkah kecil
# (mis. +-5 s.d. +-20) dari titik ini -- jauh lebih halus daripada
# menaikkan gain 1 demi 1 tanpa skala seperti sebelumnya.
Kp_psi = 200    # dahulu Kp = 0.200 (loop sudut badan / psi)
Kd_psi = 14     # dahulu Kd = 0.014 (loop sudut badan / psi)
Kp_theta = 100    # BARU: loop sudut roda / theta (0 = nonaktif secara default)
Kd_theta = 0    # BARU: loop kecepatan sudut roda / theta (0 = nonaktif)

# Setpoint sudut roda (theta) default, dalam derajat, dipakai oleh loop theta
# di pd_control.py (dipakai sama untuk roda kiri & kanan). Nilai 0 berarti
# PD theta berusaha menahan roda agar tidak "lari" dari posisi saat balancing
# dimulai (anti-drift), tanpa mengganggu perintah maju/mundur joystick yang
# tetap dikendalikan lewat setpoint_psi (target_angle_deg) seperti biasa.
SETPOINT_THETA_DEG = 0

# Disimpan untuk pengembangan PD + CBF-QP berikutnya (BELUM dipakai di mana
# pun pada kode saat ini). Sengaja TIDAK dihapus/diubah sesuai permintaan.
Alpha_1 = 0.0
Alpha_2 = 0.0

# OUTPUT_LIMIT tidak lagi dipakai untuk clamp di pd_control.py (clamp PD
# sudah dihilangkan). Dibiarkan tersimpan sebagai referensi / kalau-kalau
# masih dipakai modul lain di kemudian hari. Batas keamanan output ke motor
# yang sesungguhnya sekarang murni ditentukan oleh MAX_CURRENT_A /
# MAX_SPEED_RPM di driver motor (ddsm115.py).
OUTPUT_LIMIT = 1.0
CONTROLLER_DEADBAND_DEG = 0.05

# Perintah manual saat balancing aktif
MANUAL_FORWARD_TARGET_DEG = 1.20
MANUAL_BACKWARD_TARGET_DEG = -1.20

# =========================
# Motor / serial parameters
# =========================
LEFT_MOTOR_PORT = "/dev/ttyACM0"
RIGHT_MOTOR_PORT = "/dev/ttyACM1"
# Karena tiap motor memakai USB-RS485 terpisah, ID bisa sama-sama 1.
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 1
MOTOR_BAUDRATE = 115200
MOTOR_TIMEOUT_S = 0.03
# Pilihan: "current" atau "speed"
# Untuk balancing, mode "current" biasanya lebih responsif.
MOTOR_CONTROL_MODE = "current"
# Jika arah putaran bebeda, ubah salah satu sign '-' ini.
LEFT_MOTOR_SIGN = -1.0
RIGHT_MOTOR_SIGN = 1.0
# Jika arah roda robot sama jatuhnya badan sama maka ubah sign '-' dihilangkan/ditambah.
BALANCE_DIRECTION_SIGN = -1.0
# Valid saat speed loop. Satuan mengikuti protokol Waveshare:
# waktu akselerasi per 1 rpm dalam kelipatan 0.1 ms.
MOTOR_ACCEL_TIME = 3
# Batas aman awal saat bring-up
MAX_CURRENT_A = 1.80
MAX_SPEED_RPM = 120.0
TURN_OUTPUT_FRACTION = 0.18

# =========================
# IMU / MPU6050 parameters
# =========================
I2C_BUS = 1
MPU6050_ADDRESS = 0x68
# Ubah ke "roll" jika sumbu balancing Anda adalah roll.
IMU_AXIS = "pitch"
# Koreksi orientasi pemasangan IMU.
IMU_SIGN = 1.0
GYRO_CALIBRATION_SAMPLES = 800
GYRO_CALIBRATION_DELAY_S = 0.002
ZERO_CALIBRATION_SAMPLES = 350
ZERO_CALIBRATION_DELAY_S = 0.004

# =========================
# JOYSTICK
# =========================
JOYSTICK_PATH = '/dev/input/event5'

# =========================
# Runtime / UI / safety
# =========================
CONTROL_HZ = 200.0
UI_HZ = 20.0
KEY_HOLD_TIMEOUT_S = 0.18
SAFE_TILT_DEG = 30.0

# Interval log status ke terminal/file jika dibutuhkan
STATUS_LOG_PERIOD_S = 0.5

# =========================
# Evaluasi error (ise.py)
# =========================
# Nama folder tempat ise.py menyimpan grafik evaluasi (format nama file:
# YYYYMMDD-URUTAN.png). Folder ini diasumsikan sudah ada di direktori yang
# sama dengan program (sesuai catatan user), tapi ise.py tetap akan
# membuatnya otomatis (os.makedirs) kalau ternyata belum ada.
PLOT_EVALUASI_DIR = "PLOT_EVALUASI"