"""pd_control.py - Kontrol PD robot ATERA.

Keluaran: u_PD = torsi untuk SATU roda (N m).

    u_PD = Kp_psi*psi + Kd_psi*dpsi + Kp_theta*theta + Kd_theta*dtheta + Kvel*arah

    state x = [theta, dtheta, psi, dpsi], semua gain positif.
    Hasilnya sama dengan simulasi/pd_control.py (-1 * (Kp*(setpoint - state) + ...) dengan setpoint 0).

Mode Balancing (Tombol B) : Kp_theta dipakai, arah = 0.
Mode Navigate  (Tombol A) : Kp_theta = 0, arah dari joystick (-1 mundur, 0 lepas, +1 maju).
Error (setpoint - state) tetap dihitung untuk ISE (ise.py).
"""

import config

MODE_BALANCING = "BALANCING"
MODE_NAVIGATE = "NAVIGATE"


class PDControl:
    def __init__(self):
        # Gain
        self.Kp_psi = float(config.Kp_psi)
        self.Kd_psi = float(config.Kd_psi)
        self.Kp_theta = float(config.Kp_theta)
        self.Kd_theta = float(config.Kd_theta)
        self.Kvel = float(config.Kvel)

        # Perintah dari joystick
        self.mode_gerak = MODE_BALANCING
        self.arah = 0                  # -1 mundur, 0 lepas, +1 maju

        # Nilai terakhir (GUI dan CSV)
        self.error_theta = self.error_dtheta = self.error_psi = self.error_dpsi = 0.0
        self.u_PD = 0.0

    def set_command(self, arah, mode_gerak):
        """arah = -1 / 0 / +1 (+1 = maju), mode_gerak = MODE_BALANCING / MODE_NAVIGATE."""
        self.mode_gerak = mode_gerak
        self.arah = int(arah)

    def compute(self, x):
        """u_PD [N m] untuk state x = [theta, dtheta, psi, dpsi]."""
        theta = float(x[0])
        dtheta = float(x[1])
        psi = float(x[2])
        dpsi = float(x[3])

        # Mode Balancing: Kp_theta dipakai dan arah diabaikan.
        if self.mode_gerak == MODE_BALANCING:
            Kp_theta = self.Kp_theta
            arah = 0
        else:
            Kp_theta = 0.0
            arah = self.arah

        # Persamaan PD
        u_PD = (self.Kp_psi * psi) + (self.Kd_psi * dpsi) \
            + (Kp_theta * theta) + (self.Kd_theta * dtheta) \
            + (self.Kvel * arah)
        self.u_PD = u_PD

        # Error untuk ISE
        self.error_theta = config.theta_setpoint - theta
        self.error_dtheta = config.dtheta_setpoint - dtheta
        self.error_psi = config.psi_setpoint - psi
        self.error_dpsi = config.dpsi_setpoint - dpsi
        return u_PD