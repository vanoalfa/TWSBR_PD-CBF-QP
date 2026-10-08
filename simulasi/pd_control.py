"""pd_control.py - Kontrol PD robot ATERA.

Keluaran: u_PD = torsi untuk SATU roda (N m).

Rumus:
    u_PD = BALANCE_DIRECTION_SIGN * (Kp_psi*error_psi + Kd_psi*error_dpsi
                                     + Kp_theta*error_theta + Kd_theta*error_dtheta) + Kvel*arah

    error = setpoint - state
    state x = [theta, dtheta, psi, dpsi]

Mode Balancing (Tombol B) : Kp_theta dipakai, D-Pad tidak dipakai (arah = 0).
Mode Jalan (Tombol A)     : Kp_theta = 0, arah dari D-Pad (-1 mundur, 0 lepas, +1 maju).
"""

import config


class PDControl:
    def __init__(self):
        # Gain
        self.Kp_psi = float(config.Kp_psi)
        self.Kd_psi = float(config.Kd_psi)
        self.Kp_theta = float(config.Kp_theta)
        self.Kd_theta = float(config.Kd_theta)
        self.Kvel = float(config.Kvel)
        self.balance_direction_sign = float(config.BALANCE_DIRECTION_SIGN)

        # Setpoint
        self.theta_setpoint = float(config.theta_setpoint)
        self.dtheta_setpoint = float(config.dtheta_setpoint)
        self.psi_setpoint = float(config.psi_setpoint)
        self.dpsi_setpoint = float(config.dpsi_setpoint)

        # Perintah dari joystick
        self.mode_balancing = True     # True = Mode Balancing (Tombol B)
        self.arah = 0                  # -1 mundur, 0 lepas, +1 maju

        # Nilai terakhir (untuk ditampilkan di GUI dan disimpan di CSV)
        self.error_theta = 0.0
        self.error_dtheta = 0.0
        self.error_psi = 0.0
        self.error_dpsi = 0.0
        self.u_PD = 0.0

    def set_command(self, arah, mode_balancing):
        """Dipanggil dari atera_main.py setiap ada perintah joystick."""
        # ASUMSI: arah = +1 berarti maju.
        self.mode_balancing = bool(mode_balancing)
        self.arah = int(arah)

    def compute_PD(self, x):
        # State
        theta = float(x[0])
        dtheta = float(x[1])
        psi = float(x[2])
        dpsi = float(x[3])

        # Error = setpoint - state
        error_theta = self.theta_setpoint - theta
        error_dtheta = self.dtheta_setpoint - dtheta
        error_psi = self.psi_setpoint - psi
        error_dpsi = self.dpsi_setpoint - dpsi

        # Mode Balancing: Kp_theta dipakai dan D-Pad diabaikan.
        if self.mode_balancing:
            Kp_theta = self.Kp_theta
            arah = 0
        else:
            Kp_theta = 0.0
            arah = self.arah

        # Persamaan PD
        u_PD = (self.Kp_psi * error_psi) + (self.Kd_psi * error_dpsi) \
            + (Kp_theta * error_theta) + (self.Kd_theta * error_dtheta)
        u_PD = u_PD * self.balance_direction_sign
        u_PD = u_PD + (self.Kvel * arah)

        self.error_theta = error_theta
        self.error_dtheta = error_dtheta
        self.error_psi = error_psi
        self.error_dpsi = error_dpsi
        self.u_PD = u_PD
        return u_PD