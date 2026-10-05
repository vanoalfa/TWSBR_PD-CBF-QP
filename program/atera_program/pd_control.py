"""pd_control.py - kontrol PD nominal ATERA. Keluarannya u_PD.

    u_PD = Kp_theta*theta + Kd_dtheta*dtheta + Kp_psi*psi + Kd_dpsi*dpsi + Kvel

    x    = [theta, dtheta, psi, dpsi]   (SI, rad)
    u_PD = torsi TOTAL kedua roda [N m] (tiap roda menerima u_PD / 2)

Catatan untuk tuning:
- Tidak ada saturasi di sini. Batas aktuator diterapkan di ddsm115.py (mode PD) atau
  sebagai bound u di cbf_qp.py (mode PD+CBF).
- Dengan konvensi tanda di config.py, keempat gain POSITIF. Kp_psi dan Kd_dpsi menegakkan
  badan; Kp_theta dan Kd_dtheta mengembalikan roda ke posisi awal sehingga robot diam.
  Hanya dengan gain psi, model linear punya dua pole di 0 (robot berdiri tapi hanyut).
- Dari model linear: Kd_dtheta harus jauh lebih kecil dari Kd_dpsi (kira-kira
  Kd_dtheta < 0.4 * Kd_dpsi), begitu juga Kp_theta terhadap Kp_psi. Naikkan pelan-pelan.
  Cek pole dengan:  python3 model.py
- Kvel adalah konstanta torsi (bias). Di bidang datar biarkan 0; di bidang miring nilainya
  mendekati model.equilibrium()[1] untuk menahan robot di tanjakan.
- Untuk perintah gerak, atera_main.py memanggil compute(x - x_ref): rumusnya tetap sama,
  hanya titik acuannya yang digeser.
"""

from __future__ import annotations

import config


class PDController:
    def __init__(self) -> None:
        self.reload()

    def reload(self) -> None:
        """Baca ulang gain dari config.py."""
        self.Kp_theta = float(config.Kp_theta)
        self.Kd_dtheta = float(config.Kd_dtheta)
        self.Kp_psi = float(config.Kp_psi)
        self.Kd_dpsi = float(config.Kd_dpsi)
        self.Kvel = float(config.Kvel)

    def terms(self, x) -> tuple:
        """Kelima suku u_PD secara terpisah (untuk GUI dan log)."""
        return (
            self.Kp_theta * float(x[0]),
            self.Kd_dtheta * float(x[1]),
            self.Kp_psi * float(x[2]),
            self.Kd_dpsi * float(x[3]),
            self.Kvel,
        )

    def compute(self, x) -> float:
        """u_PD [N m] dari state x = [theta, dtheta, psi, dpsi]."""
        return float(sum(self.terms(x)))


_default = PDController()


def compute(x) -> float:
    """Kontrak: pd.compute(x) -> float [N m]."""
    return _default.compute(x)