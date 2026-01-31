from vision import detect_colors
import cv2 as cv

import time

from gpiozero import DigitalOutputDevice

IO1=DigitalOutputDevice(23) # para accionar motor


# ----- FUNCIONES ADICIONALES -----


    
# ----- MAIN -----

def main():
    # --- WEBCAM ---
    webcam = cv.VideoCapture(0, cv.CAP_V4L2)
    webcam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, 320)
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
    
    WINDOW_NAME= "VISION ROVER"
    cv.namedWindow(WINDOW_NAME, cv.WINDOW_NORMAL)

    if not webcam.isOpened():
        print("❌ No se pudo abrir la webcam")
        return
    
    
    
    DETECTION_FRAMES=18
    
    DRAW=True  # control para dibujar (o no, o nada :D)
    
    IO1.on()
    
    
    counter=0
    color_detected=False
    
    while True:
        ret, frame=webcam.read()
        if not ret:
            print("❌ Error leyendo frame")
            break
        
        # Implementación de vision.py
        colors, centroids= detect_colors(frame, draw=DRAW)
        
        if colors:
            counter+=1
        else:
            counter=0
            if color_detected:
                print("Color perdido, reanudando motor")
                IO1.on()
                color_detected=False
        
        if counter >= DETECTION_FRAMES and not color_detected:
            print("Color confirmado: ", colors)
            IO1.off()
            color_detected=True
                
        
        cv.imshow("VISION ROVER", frame)

        # ESC para salir
        if cv.waitKey(1) & 0xFF == 27:
            break
        
        if cv.getWindowProperty(WINDOW_NAME, cv.WND_PROP_VISIBLE)< 1:
            break

    # Cleanup
    IO1.off()
    webcam.release()
    cv.destroyAllWindows()

# ----- ENTRY POINT -----

if __name__ == "__main__":
    main()
        
    
    