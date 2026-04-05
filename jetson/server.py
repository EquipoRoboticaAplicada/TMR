from flask import Flask, Response, jsonify
from video_stream import gen_frames, init_video_stream
from connect import ESP
import socket
from command import Route_Command

esp: ESP = None
vision = None
tracker = None
odo = None

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

def init_app(esp_instance: ESP, zed_instance, vision_instance, tracker_instance, odo_instance, cmd_instance):
    global esp, vision, tracker, odo, cmd
    esp     = esp_instance
    vision  = vision_instance
    tracker = tracker_instance
    odo     = odo_instance
    cmd = cmd_instance
    init_video_stream(zed_instance)

app = Flask(__name__)

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/telemetry", methods=["GET"])
def telemetry():
    return jsonify({"rover_state": esp.get_rover_state()})

@app.route("/pose/reset", methods=["POST"])
def pose_reset():
    odo.reset_pose()
    cmd.reset_path()
    return jsonify({"ok": True})

@app.route("/odometry", methods=["GET"])
def pose():
    x, y, theta = odo.pose
    v, omega    = odo.velocity
    return jsonify({
        "x":     x,
        "y":     y,
        "theta": theta,
        "v":     v,
        "omega": omega,
    })

def run():
    jetson_IP = get_local_ip()
    print(f"Iniciando servidor Flask en http://{jetson_IP}:5000")
    app.run(host=jetson_IP, port=5000, threaded=True, use_reloader=False)