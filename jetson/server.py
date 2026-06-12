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

# Topics Configuration
TOPIC_SENSORS   = "rover/sensors"
TOPIC_TELEMETRY = "rover/telemetry"
TOPIC_ODOMETRY  = "rover/odometry"
TOPIC_COMMANDS  = "rover/commands"  # For actions like pose reset
TOPIC_STATUS    = "rover/status"    # For Last Will and Testament

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
    cmd     = cmd_instance
    # NOTE: Keep your video stream initialization here. 
    # If using Flask purely for video, don't delete your gen_frames logic.

# MQTT Callbacks
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Successfully connected to MQTT Broker!")
        # Notify network that rover is active
        client.publish(TOPIC_STATUS, "online", qos=1, retain=True)
        # Subscribe to inbound commands (like pose reset)
        client.subscribe(TOPIC_COMMANDS, qos=1)
    else:
        print(f"Connection failed with code {rc}")

def on_message(client, userdata, msg):
    """Handles inbound control commands from the Base Station."""
    global odo, cmd
    try:
        payload = json.loads(msg.payload.decode())
        action = payload.get("action")

        if action == "reset_pose":
            print("Received remote command: Resetting Pose and Path.")
            odo.reset_pose()
            cmd.reset_path()
            # Optional: publish a confirmation back
            client.publish("rover/commands/response", json.dumps({"status": "ok"}), qos=1)

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

def run(broker_ip="localhost", broker_port=1883):
    """Starts the MQTT loop and internal streaming threads instead of Flask app.run()."""
    
    # Initialize Client (v2 library compatibility)
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="Jetson_Rover")
    client.on_connect = on_connect
    client.on_message = on_message

    # Set Last Will: If Jetson drops out violently, Broker sets topic to offline
    client.will_set(TOPIC_STATUS, "offline", qos=1, retain=True)

    print(f"Connecting to MQTT Broker at {broker_ip}:{broker_port}...")
    client.connect(broker_ip, broker_port, keepalive=60)

    # Start network loop in its own background thread
    client.loop_start()

    # Start a dedicated thread to stream the sensor/odometry data loops at 10Hz
    pub_thread = threading.Thread(target=telemetry_publisher_loop, args=(client, 10), daemon=True)
    pub_thread.start()