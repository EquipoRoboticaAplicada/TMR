import threading
import json
import time
import paho.mqtt.client as mqtt

class Receiver:
    def __init__(self, broker_ip="localhost", port=1883, poll_hz=10.0):
        self.broker_ip = broker_ip
        self.port = port
        self._lock = threading.Lock()
        
        # Keep the exact same variable naming structure your HMI expects
        self.pose = (0.0, 0.0, 0.0)      # (x, y, theta)
        self.velocity = (0.0, 0.0)      # (v_lineal, v_angular)
        self.pitch = 0.0
        self.heading = 0.0
        self._terrain_text = "NONE"
        self._peso = 0.0
        
        # Stale/Timeout tracking
        self.is_stale = True
        self._last_msg_time = 0
        self._timeout_threshold = 2.0  # seconds before showing disconnected

        # Initialize MQTT client
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="HMI_Base_Station")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print("HMI Receiver connected to MQTT Broker!")
            # Subscribe to all telemetry coming from the rover
            self.client.subscribe("rover/odometry", qos=0)
            self.client.subscribe("rover/sensors", qos=0)
            self.client.subscribe("rover/telemetry", qos=0)
            self.client.subscribe("rover/status", qos=1)
        else:
            print(f"HMI Receiver connection failed with code {rc}")

    def on_message(self, client, userdata, msg):
        """Processes incoming data safely separating text from JSON streams."""
        with self._lock:
            self._last_msg_time = time.time()
            self.is_stale = False
            
            try:
                # 1. Handle raw text status topics BEFORE parsing JSON
                if msg.topic == "rover/status":
                    status_text = msg.payload.decode().strip()
                    if status_text == "offline":
                        self.is_stale = True
                    return  # Exit early since this isn't JSON

                # 2. Safely parse JSON for all other telemetry topics
                payload = json.loads(msg.payload.decode())
                
                if msg.topic == "rover/odometry":
                    self.pose = (payload.get("x", 0.0), payload.get("y", 0.0), payload.get("theta", 0.0))
                    self.velocity = (payload.get("v", 0.0), payload.get("omega", 0.0))
                    
                elif msg.topic == "rover/sensors":
                    sensor_state = payload.get("rover_sensors", {})
                    self.pitch = sensor_state.get("pitch", 0.0)
                    self.heading = sensor_state.get("heading", 0.0)
                    self._terrain_text = sensor_state.get("terrain", "NONE")
                    
                elif msg.topic == "rover/telemetry":
                    rover_state = payload.get("rover_state", {})
                    self._peso = rover_state.get("peso", 0.0)

            except Exception as e:
                print(f"⚠️ Error parsing topic {msg.topic}: {e}")

    def _monitor_connection(self):
        """Background loop checking if the rover has stopped transmitting."""
        while self._running:
            if time.time() - self._last_msg_time > self._timeout_threshold:
                with self._lock:
                    self.is_stale = True
            time.sleep(0.5)

    def start(self):
        self._running = True
        print(f"Connecting to MQTT Broker at {self.broker_ip}...")
        self.client.connect(self.broker_ip, self.port, keepalive=60)
        
        # Start MQTT thread network loop
        self.client.loop_start()
        
        # Start connection watchdog monitor thread
        self.monitor_thread = threading.Thread(target=self._monitor_connection, daemon=True)
        self.monitor_thread.start()

    def stop(self):
        self._running = False
        self.client.loop_stop()
        self.client.disconnect()
        
    def reset_rover_pose(self):
        """Call this from the UI to publish a command back to the Jetson."""
        command = {"action": "reset_pose"}
        self.client.publish("rover/commands", json.dumps(command), qos=1)