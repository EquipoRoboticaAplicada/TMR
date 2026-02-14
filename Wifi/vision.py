# vision.py 

import os
import json
import cv2 as cv
import numpy as np


# --- Rangos HSV ---

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(BASE_DIR, "config")
COLOR_FILE = os.path.join(CONFIG_DIR, "colors.json")

if not os.path.exists(COLOR_FILE):
    raise FileNotFoundError(
        f"No se encontró el archivo de calibración en:\n{COLOR_FILE}\n"
        "Ejecuta primero calibrate_colors.py"
    )

with open(COLOR_FILE, "r") as f:
    COLOR_RANGES = json.load(f)


DRAW = {
    "green": (0, 255, 0),
    "blue":     (255, 0, 0),
    "red":     (0, 0, 255)
}

MIN_AREA = 600
KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))

# Máscaras para el procesado

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
    areas=[]

    for cnt in contours:
        area=cv.contourArea(cnt)
        if area > MIN_AREA:
            
            # Color encontrado
            found=True
            areas.append(area)
            
            # Centroide(s)
            M=cv.moments(cnt)
            
            if M["m00"]!=0:
            
                cx=int(M["m10"]/M["m00"])
                cy=int(M["m01"]/M["m00"])
                
                detected_centroids.append((label,cx,cy))
                
            if draw:
                
                x, y, w, h = cv.boundingRect(cnt)
            
                color_bgr = DRAW.get(label, (255, 255, 255))
                cv.rectangle(frame_draw, (x, y), (x + w, y + h), color_bgr, 2)
                cv.putText(frame_draw, label, (x, y - 8), cv.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2, cv.LINE_AA)
                
                # Círculo del centroide
                cv.circle(frame_draw,(cx,cy), 5, (255,255,255), -1)
               
           
    return found, detected_centroids, areas


def detect_colors(frame, draw=True):
    """
    - Procesa frames, y una vez detectado un color mediante find_and_draw, decide qué color (label) es
    - Devuelve:
        * detected_colors (set)
        * all_centroids (lista)
        * areas (lista)
    - Sintaxsis:
        * colors,centroids=detect_colors(frame) <--- por default dibuja
        * colors,centroids=detect_colors(frame,draw=False) <------ solo detecta, no dibuja
    """
    
    colors =[]
    centroids=[]
    areas=[]

    blurred = cv.GaussianBlur(frame, (5, 5), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    for color, ranges in COLOR_RANGES.items():
        lower=np.array(ranges["lower"])
        upper=np.array(ranges["upper"])
        
        mask=process_mask(cv.inRange(hsv,lower,upper))
        
        found,c_list, a_list=find_and_draw(mask,frame,color,draw)
        
        if found:
            colors.extend([color]*len(c_list))
            centroids.extend(c_list)
            areas.extend(a_list)

    return colors, centroids, areas

def crosslines(screen):
    h, w = screen.shape[:2]
    cx,cy=w//2,h//2
    cv.line(screen, (cx-50, 0), (cx-50, h), (255,255,255), 1) # left vertical line
    cv.line(screen, (cx+50, 0), (cx+50, h), (255,255,255), 1) # right vertical line

    cv.line(screen, (0, cy-40), (w, cy-40), (255,255,255), 1) # top horizontal line
    cv.line(screen, (0, cy+70), (w, cy+70), (255,255,255), 1) # bottom horizontal line
    return screen