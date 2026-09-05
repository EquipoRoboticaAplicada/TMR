import sys
import types
import time
import threading

# 1. Mock 'pyzed' module workspace in memory
pyzed_mock = types.ModuleType("pyzed")
pyzed_mock.sl = types.ModuleType("sl")
pyzed_mock.sl.Camera = lambda: None 
sys.modules["pyzed"] = pyzed_mock
sys.modules["pyzed.sl"] = pyzed_mock.sl

# 2. Mock out the hardware classes
import connect
import vision_zed
import odo
import main  # Import main module frame directly so we can patch its variables

BROKER_IP = "172.32.167.230"
BROKER_PORT = 1883

class MockESP:
    def connect(self): print("[Mock] Serial Connection established.")
    def send_uart(self, *args): pass 
    def get_sensor_state(self):
        return {"pitch": 5.2, "heading": 90.0, "terrain": "SMOOTH"}
    def get_rover_state(self):
        return {"peso": 120.5}
    def close(self): print("[Mock] Serial Closed.")

class MockZEDShared:
    def start(self):
        print("[Mock] ZED Camera Thread started.")
        return self
    def stop(self): pass

class MockVisionZED:
    def __init__(self, zed_shared): pass
    def start(self):
        print("[Mock] Vision Object Tracking Pipeline running.")
        return self
    def stop(self): pass

# Inject dummy configurations
connect.ESP = MockESP
vision_zed.ZEDShared = MockZEDShared
vision_zed.VisionZED = MockVisionZED

# 3. Dynamic Odometry Walk Simulator (Safely overriding properties)
class MockOdometry(odo.RoverOdometry):
    def __init__(self, esp):
        # Initialize internal private variables to store the fake vectors
        self._fake_pose = (0.0, 0.0, 0.0)      # x, y, theta
        self._fake_velocity = (0.1, 0.01)     # v, omega
        threading.Thread(target=self._fake_walk, daemon=True).start()
        
    # Override the read-only properties with custom dynamic getters
    @property
    def pose(self):
        return self._fake_pose

    @property
    def velocity(self):
        return self._fake_velocity

    def _fake_walk(self):
        x, y, theta = 0.0, 0.0, 0.0
        while True:
            x += 0.05
            y += 0.02
            theta = (theta + 0.03) % 6.28
            self._fake_pose = (x, y, theta)
            time.sleep(0.1) # Send state coordinate ticks at 10Hz
            
    def stop(self): pass

odo.RoverOdometry = MockOdometry

# 4. Overwrite main's internal variable routing to point directly to localhost
main.LAPTOP_BROKER_IP = BROKER_IP
main.DRAW_LOCAL = False

if __name__ == "__main__":
    print("Bootstrapping local hardware simulation context on LOCALHOST...")
    
    # Re-run the structural block under our new clean parameters
    import server
    server.init_app(MockESP(), None, None, None, MockOdometry(None), None)
    
    print("Launching client stream to localhost:1883...")
    server.run(broker_ip=BROKER_IP, broker_port=BROKER_PORT)
    
    # Keep the main process string completely awake
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping simulation.")