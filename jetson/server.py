from flask import Flask, Response, request, jsonify
from video_stream import gen_frames, init_video_stream
from connect import ESP

esp: ESP = None
vision = None
tracker = None

def init_app(esp_instance: ESP, zed_instance, vision_instance, tracker_instance):
    global esp, vision, tracker
    esp = esp_instance
    vision = vision_instance
    tracker = tracker_instance
    init_video_stream(zed_instance)

app = Flask(__name__)

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/telemetry", methods=["GET"])
def telemetry():
    return jsonify({"rover_state": esp.get_rover_state()})