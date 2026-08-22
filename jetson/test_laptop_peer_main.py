import math
import random
import threading
import time
from jetson import server

# Target broker IP (Computer A - your Fedora laptop)
BROKER_IP = "172.32.216.190"
BROKER_PORT = 1883


def dummy_telemetry_loop():
    x, y, theta = 0.0, 0.0, 0.0
    v, omega = 0.5, 0.1
    pitch, roll, heading = 0.0, 0.0, 0.0
    peso = 250.0

    print("Dummy telemetry generator running...")
    while True:
        # Simulate simple circular/forward movement
        theta = (theta + 0.05) % (2 * math.pi)
        x += 0.05 * math.cos(theta)
        y += 0.05 * math.sin(theta)

        pitch = round(5.0 * math.sin(time.time()), 2)
        roll = round(3.0 * math.cos(time.time()), 2)
        heading = round(math.degrees(theta), 2)
        peso += random.uniform(-0.5, 0.5)

        # 1. Update Odometry
        server.update_odometry(x=round(x, 3), y=round(y, 3), theta=round(theta, 3), v=v, omega=omega)

        # 2. Update Sensors
        server.update_sensors(pitch=pitch, roll=roll, heading=heading, terrain="SMOOTH")

        # 3. Update Motors & Weight Telemetry
        server.update_telemetry(
            left_motors=[{"id": 1, "rpm": 120.0}, {"id": 2, "rpm": 120.0}],
            right_motors=[{"id": 1, "rpm": 122.0}, {"id": 2, "rpm": 122.0}],
            peso=round(peso, 2),
        )

        time.sleep(0.1)  # 10 Hz


def main():
    print(f"📡 Connecting to MQTT Broker at {BROKER_IP}:{BROKER_PORT}...")
    
    # Start MQTT server thread pointing to Computer A
    server_thread = threading.Thread(
        target=server.run,
        kwargs={"broker_ip": BROKER_IP, "broker_port": BROKER_PORT},
        daemon=True,
    )
    server_thread.start()

    # Start dummy data publishing loop
    dummy_telemetry_loop()


if __name__ == "__main__":
    main()