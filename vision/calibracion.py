import cv2 as cv
import numpy as np
import json


# CONFIGURACION


BOX_SIZE=80
TOL_H=40
TOL_S=40
TOL_V=40

COLORS=["red","blue","green"]
WINDOW_NAME="Color_Calibration"

cap=cv.VideoCapture(1)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()
    

cv.namedWindow(WINDOW_NAME,cv.WINDOW_NORMAL)

calibrated={}

color_index=0

while color_index<len(COLORS):
    color_name=COLORS[color_index]
    
    while True:
        ret,frame=cap.read()
        if not ret:
            break
        
        h,w,_=frame.shape
        cx,cy=w//2,h//2
        half=BOX_SIZE//2
        
        roi=frame[cy-half:cy+half,cx-half:cx+half]
        hsv=cv.cvtColor(roi,cv.COLOR_BGR2HSV)
        mean_hsv=np.mean(hsv.reshape(-1,3),axis=0).astype(int)
        
        cv.rectangle(frame,(cx-half,cy-half),(cx+half,cy+half),(0,255,0),2)
        cv.putText(frame,f"Calibrando: {color_name}",(10,30),cv.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
        
        
        cv.imshow(WINDOW_NAME,frame)
        
        key=cv.waitKey(1) & 0xFF
        
        if key==ord("c"):
            
            h,s,v=mean_hsv
            
            lower=[int(max(0,h-TOL_H)),int(max(0,s-TOL_S)),int(max(0,v-TOL_V))]
            
            upper=[int(min(179,h+TOL_H)),int(min(255,s+TOL_S)),int(min(255,v+TOL_V))]
            
            calibrated[color_name]={"lower":lower,"upper":upper}
            
            print(f"{color_name} calibrado")
            print("HSV: ",mean_hsv)
            break
        
        elif key==27:
            cap.release()
            cv.destroyAllWindows()
            exit()
            
    color_index+=1
    
with open("colors.json","w") as f:
    json.dump(calibrated,f,indent=4)

print("Calibración Completada")
print("Archivo .json guardado")

cap.release()
cv.destroyAllWindows()
        