from vision import detect_colors
import cv2 as cv

import time

from gpiozero import PWMOutputDevice, DigitalOutputDevice

ENA=PWMOutputDevice(18)
IN1=DigitalOutputDevice(23)
IN2=DigitalOutputDevice(24)


# ----- FUNCIONES DEL MOTOR -----

def motor_on(speed=0.5):
    IN1.on()
    IN2.off()
    ENA.value=speed
    
    
def motor_off():
    ENA.off()
    IN1.off()
    IN2.off()
    
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
    
    
    
    motor_on() # motor se enciende
    
    DETECTION_FRAMES=18
    MOTOR_SPEED=0.5
    
    
    counter=0
    color_detected=False
    
    while True:
        ret, frame=webcam.read()
        if not ret:
            print("❌ Error leyendo frame")
            break
        
        colors= detect_colors(frame, draw=True)
        
        if colors:
            counter+=1
        else:
            counter=0
            if color_detected:
                print("Color perdido, reanudando motor")
                motor_on(MOTOR_SPEED)
                color_detected=False
        
        if counter >= DETECTION_FRAMES and not color_detected:
            print("Color confirmado: ", colors)
            motor_off()
            color_detected=True
                
        
        cv.imshow("VISION ROVER", frame)

        # ESC para salir
        if cv.waitKey(1) & 0xFF == 27:
            break
        
        if cv.getWindowProperty(WINDOW_NAME, cv.WND_PROP_VISIBLE)< 1:
            break

    motor_off()
    webcam.release()
    cv.destroyAllWindows()

# ----- ENTRY POINT -----

if __name__ == "__main__":
    main()
        
    
    