# Pemetaan Tombol Digital (Code EV_KEY)
BTN_MAP = {
    304: "A",
    305: "B",
    307: "X",
    308: "Y",
    310: "LB",
    311: "RB",
    312: "LT (Digital)",
    313: "RT (Digital)",
    314: "QUIT",
    315: "MULAI",
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
    16: {-1: "PAD Kiri", 1: "PAD Kanan"},
    17: {-1: "PAD Atas", 1: "PAD Bawah"},
}