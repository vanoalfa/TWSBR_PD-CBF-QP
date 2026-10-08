# Mapping ATERA (sama dengan joystick_mapping.py)
BTN_MAP = {
    306: "Tombol A", 305: "Tombol B", 307: "Tombol X", 304: "Tombol Y",
    308: "LB", 309: "RB", 310: "LT (Digital)", 311: "RT (Digital)",
    312: "QUIT", 313: "MULAI", 314: "L3", 315: "R3",
}
ABS_MAP = {
    1: "Analog Kiri (Y)", 0: "Analog Kiri (X)", 5: "Analog Kanan (Y)", 2: "Analog Kanan (X)",
    9: "Analog RT", 10: "Analog LT",
}
DPAD_MAP = {
    16: {-1: "PAD Kiri", 1: "PAD Kanan", 0: "PAD Kiri/Kanan dilepas"},
    17: {-1: "PAD Atas", 1: "PAD Bawah", 0: "PAD Atas/Bawah dilepas"},
}