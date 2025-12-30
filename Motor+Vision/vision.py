# vision.py 

import cv2 as cv
import numpy as np

# --- Rangos HSV ---
RANGES = {
    "Amarillo": (np.array([15, 120, 140]), np.array([35, 255, 255])),
    "Azul":     (np.array([90, 120,  70]), np.array([130,255, 255])),
    "Rojo1":    (np.array([0,  120, 120]), np.array([10, 255, 255])),
    "Rojo2":    (np.array([170,120, 120]), np.array([180,255, 255]))
}

DRAW = {
    "Amarillo": (0, 255, 255),
    "Azul":     (255, 0, 0),
    "Rojo":     (0, 0, 255)
}

MIN_AREA = 600
KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))


def process_mask(mask):
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, KERNEL, iterations=1)
    return mask


def find_and_draw(mask, frame, label):
    found = False
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        if cv.contourArea(cnt) > MIN_AREA:
            found = True
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(frame, (x, y), (x + w, y + h), DRAW[label], 2)
            cv.putText(frame, label, (x, y - 8),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7,
                       DRAW[label], 2, cv.LINE_AA)
    return found


def detect_colors(frame, draw):
    """
    Retorna un set con los colores detectados.
    Si draw=True, dibuja bounding boxes en el frame.
    """
    detected = set()

    blurred = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    # --- Amarillo ---
    low, high = RANGES["Amarillo"]
    mask = process_mask(cv.inRange(hsv, low, high))
    if find_and_draw(mask, frame, "Amarillo") if draw else has_object(mask):
        detected.add("Amarillo")

    # --- Azul ---
    low, high = RANGES["Azul"]
    mask = process_mask(cv.inRange(hsv, low, high))
    if find_and_draw(mask, frame, "Azul") if draw else has_object(mask):
        detected.add("Azul")

    # --- Rojo ---
    low1, high1 = RANGES["Rojo1"]
    low2, high2 = RANGES["Rojo2"]
    mask = process_mask(
        cv.bitwise_or(
            cv.inRange(hsv, low1, high1),
            cv.inRange(hsv, low2, high2)
        )
    )
    if find_and_draw(mask, frame, "Rojo") if draw else has_object(mask):
        detected.add("Rojo")

    return detected


def has_object(mask):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv.contourArea(cnt) > MIN_AREA:
            return True
    return False