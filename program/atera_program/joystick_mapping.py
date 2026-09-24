# Pemetaan Tombol Digital (Code EV_KEY)
BTN_MAP = {
    304: "Tombol A",        # Press this first and LT or RT for swich from Mode Balancing to Mode Kontrol (Mono or Nugget)
    305: "Tombol B",        # Just balancing (no joystick) (Mode Balancing)
    307: "Tombol X",        # Turn off wheel
    308: "Tombol Y",        # Kalibrasi
    310: "LB",              # Untuk switch ke mode PD
    311: "RB",              # Untuk Switch ke Mode PD + CBF-QP
    312: "LT (Digital)",    # Mode Mono (Forward Backward)
    313: "RT (Digital)",    # Mode Nugget (Forward Backward Left Right)
    314: "QUIT",            # Quit Program
    315: "MULAI",           # Starting Program (Balancing after kalibrasi)
    317: "L3",              
    318: "R3",              
}

# Pemetaan Sumbu Analog Kontinu (Code EV_ABS)
ABS_MAP = {
    0: "Analog Kiri (Y)",
    1: "Analog Kiri (X)",
    2: "Analog Kanan (Y)",
    5: "Analog Kanan (X)",
    9: "Analog RT",
    10: "Analog LT",
}

# Pemetaan D-Pad / PAD (Code EV_ABS dengan Nilai Diskrit)
DPAD_MAP = {
    16: {-1: "PAD Kiri", 1: "PAD Kanan"}, # Untuk maju dan mundur, lepas = berhenti
    17: {-1: "PAD Atas", 1: "PAD Bawah"}, # untuk kiri dan kanan, lepas = berhenti
}