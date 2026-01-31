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


def find_and_draw(mask, frame_draw, label, draw=True):
    
    """
    - Detecta objetos del color Rojo, Amarillo o Azul, (independientemente de cual)
    - Si draw=True, dibuja bounding boxes y centroides de los objetos en frame
    - Devuevle:
        *found (bool)
        *detected_centroids(lista)
    """
    
    found = False
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    detected_centroids=[]

    for cnt in contours:
        area=cv.contourArea(cnt)
        if area > MIN_AREA:
            
            # Color encontrado
            found=True
            
            # Centroide(s)
            M=cv.moments(cnt)
            
            if M["m00"]!=0:
            
                cx=int(M["m10"]/M["m00"])
                cy=int(M["m01"]/M["m00"])
                
                detected_centroids.append((label,cx,cy))
                
            if draw:
                
                x, y, w, h = cv.boundingRect(cnt)
            
                # Dibujar bounding box del color detectado
                cv.rectangle(frame_draw, (x, y), (x + w, y + h), DRAW[label], 2)
            
                # Texto describiendo el color detectado
                cv.putText(frame_draw, label, (x, y - 8), cv.FONT_HERSHEY_SIMPLEX, 0.7, DRAW[label], 2, cv.LINE_AA)
                
                # Círculo del centroide
                cv.circle(frame_draw,(cx,cy), 5, (255,255,255), -1)
               
           
    return found, detected_centroids


def detect_colors(frame, draw=True):
    """
    - Procesa frames, y una vez detectado un color mediante find_and_draw, decide qué color (label) es
    - Devuelve:
        * detected_colors (set)
        * all_centroids (lista)
    - Sintaxsis:
        * colors,centroids=detect_colors(frame) <--- por default dibuja
        * colors,centroids=detect_colors(frame,draw=False) <------ solo detecta, no dibuja
    """
    
    detected_colors = set()
    all_centroids=[]

    blurred = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    # --- Amarillo ---
    low, high = RANGES["Amarillo"]
    mask = process_mask(cv.inRange(hsv, low, high))
    found, centroids=find_and_draw(mask,frame,"Amarillo",draw)
    
    if found:
        detected_colors.add("Amarillo")
        all_centroids.extend(centroids)

    # --- Azul ---
    low, high = RANGES["Azul"]
    mask = process_mask(cv.inRange(hsv, low, high))
    found, centroids=find_and_draw(mask,frame,"Azul",draw)
    
    if found:
        detected_colors.add("Azul")
        all_centroids.extend(centroids)

    
    # --- Rojo ---
    low1, high1 = RANGES["Rojo1"]
    low2, high2 = RANGES["Rojo2"]
    mask = process_mask(cv.bitwise_or(cv.inRange(hsv, low1, high1), cv.inRange(hsv, low2, high2)))
    found, centroids=find_and_draw(mask,frame,"Rojo",draw)
    
    if found:
        detected_colors.add("Rojo")
        all_centroids.extend(centroids)

    return detected_colors, all_centroids

