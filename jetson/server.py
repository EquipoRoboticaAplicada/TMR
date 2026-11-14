import json
import time
import socket
import threading
import paho.mqtt.client as mqtt
from connect import ESP

# Global system instances
esp: ESP = None
vision = None
tracker = None
odo = None
cmd = None
sender = None
_last_teleop_seq = -1

# Topics Configuration
TOPIC_SENSORS   = "rover/sensors"
TOPIC_TELEMETRY = "rover/telemetry"
TOPIC_ODOMETRY  = "rover/odometry"
TOPIC_COMMANDS  = "rover/commands"  # For state actions: reset_pose, set_mode, resume_route, emergency_stop
TOPIC_TELEOP    = "rover/teleop"    # Fast QoS 0 channel for DualSense RPM setpoints
TOPIC_STATUS    = "rover/status"    # For Last Will and Testament

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

def init_app(esp_instance: ESP, zed_instance, vision_instance, tracker_instance, odo_instance, cmd_instance, sender_instance=None):
    global esp, vision, tracker, odo, cmd, sender
    esp     = esp_instance
    vision  = vision_instance
    tracker = tracker_instance
    odo     = odo_instance
    cmd     = cmd_instance
    sender  = sender_instance or getattr(cmd_instance, "sender", None)
    # NOTE: Keep your video stream initialization here. 
    # If using Flask purely for video, don't delete your gen_frames logic.

# MQTT Callbacks
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Successfully connected to MQTT Broker!")
        # Notify network that rover is active
        client.publish(TOPIC_STATUS, "online", qos=1, retain=True)
        # Subscribe to inbound commands (state transitions) and fast teleop stream
        client.subscribe(TOPIC_COMMANDS, qos=1)
        client.subscribe(TOPIC_TELEOP, qos=0)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    """Handles inbound control commands and low-latency DualSense teleop from the Base Station."""
    global odo, cmd, sender, esp, _last_teleop_seq
    try:
        payload = json.loads(msg.payload.decode())

        # 1. Fast path: Low-latency continuous DualSense setpoints (rover/teleop, QoS 0)
        if msg.topic == TOPIC_TELEOP:
            if sender is None:
                return
            seq = payload.get("s")
            if seq is not None:
                # Permitir reinicio de secuencia (seq <= 1) cuando el cliente reconecta
                if seq <= 1 or seq > _last_teleop_seq or (_last_teleop_seq - seq) > 1000:
                    _last_teleop_seq = seq
                else:
                    return  # Descartar paquete viejo fuera de orden
            left_rpm  = payload.get("l", 0)
            right_rpm = payload.get("r", 0)
            max_rpm   = payload.get("max")
            sender.send_manual(left_rpm, right_rpm, max_rpm=max_rpm)
            return

        # 2. Control & state actions (rover/commands, QoS 1)
        action = payload.get("action")

        if action == "reset_pose":
            print("Received remote command: Resetting Pose and Path.")
            if odo is not None:
                odo.reset_pose()
            if cmd is not None:
                cmd.reset_path()
            client.publish("rover/commands/response", json.dumps({"status": "ok", "action": "reset_pose"}), qos=1)

        elif action == "set_mode":
            manual = bool(payload.get("manual", False))
            _last_teleop_seq = -1
            if sender is not None:
                sender.set_manual_mode(manual)
            if cmd is not None and manual:
                cmd.pause_route()
            mode_str = "MANUAL" if manual else "STANDBY"
            print(f"[MQTT] Modo cambiado a: {mode_str}")
            client.publish("rover/commands/response", json.dumps({"status": "ok", "mode": mode_str}), qos=1)

        elif action == "resume_route":
            if cmd is not None:
                cmd.resume_route()
            client.publish("rover/commands/response", json.dumps({"status": "ok", "action": "resume_route"}), qos=1)

        elif action == "pause_route":
            if cmd is not None:
                cmd.pause_route()
            client.publish("rover/commands/response", json.dumps({"status": "ok", "action": "pause_route"}), qos=1)

        elif action == "emergency_stop":
            print("[MQTT] 🛑 PARO DE EMERGENCIA recibido.")
            if sender is not None:
                sender.emergency_stop()
            elif esp is not None:
                esp.send_uart("0", "0")
            if cmd is not None:
                cmd.pause_route()
            client.publish("rover/commands/response", json.dumps({"status": "ok", "action": "emergency_stop"}), qos=1)

        elif action == "act_arm":
            print("[MQTT] Accionando brazo (B0).")
            if esp is not None:
                esp.act_arm()
            client.publish("rover/commands/response", json.dumps({"status": "ok", "action": "act_arm"}), qos=1)

    except Exception as e:
        print(f"Error processing inbound MQTT message: {e}")

def telemetry_publisher_loop(client, rate_hz=10):
    """Thread loop that continuously stream data over MQTT."""
    global esp, odo
    interval = 1.0 / rate_hz
    
    print(f"Starting telemetry streaming thread at {rate_hz}Hz...")
    
    while True:
        try:
            # 1. Fetch data from hardware instances
            x, y, theta = odo.pose
            v, omega    = odo.velocity
            
            odometry_data = {"x": x, "y": y, "theta": theta, "v": v, "omega": omega}
            sensor_data   = {"rover_sensors": esp.get_sensor_state()}
            telemetry_data = {"rover_state": esp.get_rover_state()}

            # 2. Publish to respective MQTT topics
            # QoS=0 is fine for high-frequency telemetry where losing an individual frame doesn't matter
            client.publish(TOPIC_ODOMETRY, json.dumps(odometry_data), qos=0)
            client.publish(TOPIC_SENSORS, json.dumps(sensor_data), qos=0)
            client.publish(TOPIC_TELEMETRY, json.dumps(telemetry_data), qos=0)

        except Exception as e:
            print(f"Error in telemetry loop: {e}")
            
        time.sleep(interval)

def run(broker_ip="172.32.149.2", broker_port=1883):
    """Starts the MQTT loop and internal streaming threads instead of Flask app.run()."""
    
    # Initialize Client (v2 library compatibility)
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="Jetson_Rover")
    client.on_connect = on_connect
    client.on_message = on_message

    # Set Last Will: If Jetson drops out violently, Broker sets topic to offline
    client.will_set(TOPIC_STATUS, "offline", qos=1, retain=True)

    try:    
        print(f"Connecting to MQTT Broker at {broker_ip}:{broker_port}...")
        client.connect(broker_ip, broker_port, keepalive=60)
    except Exception as e:
        print(f"Error connecting to MQTT Broker: {e}")

    # Start network loop in its own background thread
    client.loop_start()

    # Start a dedicated thread to stream the sensor/odometry data loops at 10Hz
    pub_thread = threading.Thread(target=telemetry_publisher_loop, args=(client, 10), daemon=True)
    pub_thread.start()