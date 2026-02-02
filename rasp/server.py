from flask import Flask, Response, request, jsonify
from video_stream import gen_frames
# from gpiozero import DigitalOutputDevice
import time
import serial

# ---- UART ESP32 ----
UART_LEFT  = "/dev/ttyACM0"
# UART_RIGHT = "/dev/ttyUSB1"

BAUDRATE = 115200

try:
    ser_left = serial.Serial(UART_LEFT, BAUDRATE, timeout=0.05)
    ser_right = serial.Serial(UART_RIGHT, BAUDRATE, timeout=0.05)
    print("UARTs abiertos correctamente")
except Exception as e:
    print("Error abriendo UART:", e)
    ser_left = None
    ser_right = None

def send_rpm(left_rpm, right_rpm):
    if ser_left:
        ser_left.write(f"S{left_rpm:.1f}\n".encode())
    if ser_right:
        ser_right.write(f"S{right_rpm:.1f}\n".encode())


# ----- GPIO -----
# 
# IN1 = DigitalOutputDevice(23)
# 
# def signal_on():
#     IN1.on()
#     
# 
# def signal_off():
#     IN1.off()
    

# ----- Estado de Visión -----

vision_state = {"colors":[],"centroids":[],"areas":[],"time": 0.0}

# ----- Estado de control -----

control_state = {"colors":[],"centroids":[],"areas":[],"time": 0.0}

# ----- Flask -----
app = Flask(__name__)


@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/command", methods=["POST"])
def command():
    data = request.json
    cmd = data.get("cmd")

    # if cmd == "signal_on":
    #     signal_on()
    # elif cmd == "signal_off":
    #     signal_off()

    return jsonify({"status": "ok", "cmd": cmd})

@app.route("/vision_data",methods=["POST"])
def vision_data():
    global vision_state
    
    data=request.json
    vision_state["colors"]=data.get("colors",[])
    vision_state["centroids"]=data.get("centroids",[])
    vision_state["areas"]=data.get("areas",[])
    vision_state["time"]=data.get("time",time.time())
    
    print("Vision state: ", vision_state)
    return jsonify({"status":"ok"})
    

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000,threaded=True)