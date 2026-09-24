# kalman.py
class KalmanAngle:
    """1D Kalman Filter untuk mengombinasikan respon frekuensi tinggi dari Gyroscope

    dan respon frekuensi rendah dari Accelerometer.
    """

    def __init__(self) -> None:
        self.Q_angle: float = 0.001
        self.Q_bias: float = 0.003
        self.R_measure: float = 0.03

        self.angle: float = 0.0
        self.bias: float = 0.0
        self.rate: float = 0.0
        self.P: list[list[float]] = [[0.0, 0.0], [0.0, 0.0]]

    def get_angle(self, new_angle: float, new_rate: float, dt: float) -> float:
        """Menghitung estimasi sudut baru menggunakan algoritma Kalman Filter."""
        # Tahap 1: Prediksi Status (State Prediction)
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        # Tahap 2: Matriks Kovarians Error (Error Covariance Update)
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Tahap 3: Perhitungan Inovasi Kunci
        y = new_angle - self.angle
        s = self.P[0][0] + self.R_measure

        # Tahap 4: Gain Kalman Matrix
        K = [self.P[0][0] / s, self.P[1][0] / s]

        # Tahap 5: Update Estimasi Status Sudut & Bias
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Tahap 6: Update Matriks Kovarians
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

    def set_angle(self, angle: float) -> None:
        """Mengatur sudut acuan awal."""
        self.angle = angle

    def get_rate(self) -> float:
        """Mengembalikan laju kecepatan sudut (gyro rate)."""
        return self.rate