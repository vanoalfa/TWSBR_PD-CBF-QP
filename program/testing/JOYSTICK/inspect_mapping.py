# Pemetaan Tombol Digital (Code EV_KEY)
BTN_MAP = {
    305: "Tombol A",
    306: "Tombol B",
    304: "Tombol X",
    307: "Tombol Y",
    308: "LB",
    309: "RB",
    310: "LT (Digital)",
    311: "RT (Digital)",
    312: "QUIT",
    313: "MULAI",
    314: "L3",
    315: "R3",
}

# Pemetaan Sumbu Analog Kontinu (Code EV_ABS)
ABS_MAP = {
    0: "Analog Kiri (Y)",
    1: "Analog Kiri (X)",
    2: "Analog Kanan (Y)",
    5: "Analog Kanan (X)",
    #9: "Analog RT",
    #10: "Analog LT",
}

# Pemetaan D-Pad / PAD (Code EV_ABS dengan Nilai Diskrit)
DPAD_MAP = {
    16: {-1: "PAD Kiri", 1: "PAD Kanan"},
    17: {-1: "PAD Atas", 1: "PAD Bawah"},
}