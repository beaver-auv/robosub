import numpy as np
import socket
import json
import time
from scipy.spatial.transform import Rotation as R
from vnpy import VnSensor
from ms5837 import DENSITY_FRESHWATER, MS5837_30BA


from ezauv.hardware.sensor_interface import Sensor


class VectorNavIMU(Sensor):
    def __init__(self, port, baud):
        self.vectornav = VnSensor()
        self.vectornav.connect(port, baud)
        self.calibrated_heading = 0

    def get_data(self) -> dict:
        rot = self.vectornav.read_yaw_pitch_roll()
        # rot.x == yaw, rot.y == pitch, rot.z == roll
        accel = self.vectornav.read_yaw_pitch_roll_magnetic_acceleration_and_angular_rates().accel
        return {
            "rotation": R.from_euler(rot.x - self.calibrated_heading, rot.y, rot.z, "zyx"),
            "acceleration": np.array([accel.x, accel.y, accel.z])
            }

    def initialize(self) -> None:
        interval = 0.1
        total = int(5 / interval)
        self.log(f"Calibrating IMU heading over 5 seconds ({total} checks)...")
        heading_sum = 0
        for i in np.linspace(0, 5, total):
            heading_sum += self.vectornav.read_yaw_pitch_roll().x
            time.sleep(interval)
        self.calibrated_heading = heading_sum / total

    def overview(self) -> None:
        print(f"VectorNav IMU --- {self.vectornav.read_model_number()}")

    

class DepthSensor(Sensor):
    def __init__(self, bus=1, density=DENSITY_FRESHWATER):
        self.sensor = MS5837_30BA(bus)
        self.initial = 0
        self.density = density
    
    def get_data(self) -> float:
        if self.sensor.read():
            {"depth": self.sensor.depth()}
        else:
            self.log("Depth sensor died!")
            exit(1)  # TODO error handling
    
    def initialize(self) -> None:
        if not self.sensor.init():
            print("Sensor could not be initialized during depth sensor initialization")
            raise ConnectionError()

        if not self.sensor.read():
            print("Sensor read failed during depth sensor initialization")
            raise ConnectionError()

        self.sensor.setFluidDensity(self.density)

    def overview(self) -> None:
        print("remember to make this better later")

class DVLSensor(Sensor):
    def __init__(self, ip="192.168.194.95", port=16171, timeout=1.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

    def initialize(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.ip, self.port))
        self.log(f"DVL connected to {self.ip}:{self.port}")

    def get_data(self) -> dict:
        try:
            data = self.sock.recv(4096).decode("utf-8").strip()

            for line in data.split("\r\n"):
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if msg.get("type") == "velocity":
                    self.velocity = np.array([
                        msg.get("vx", self.velocity[0]),
                        msg.get("vy", self.velocity[1]),
                        msg.get("vz", self.velocity[2])
                    ])
                elif msg.get("type") == "position_local":
                    self.position = np.array([
                        msg.get("x", self.position[0]),
                        msg.get("y", self.position[1]),
                        msg.get("z", self.position[2])
                    ])

        except socket.timeout:
            pass

        return {
            "velocity": self.velocity,
            "position": self.position
        }

    def overview(self) -> None:
        self.log(f"Waterlinked DVL-A50 Sensor connected to {self.ip}:{self.port}")

class Camera(Sensor):
    def __init__(self):
        ...

    def initialize(self) -> None:
        ...

    def get_data(self) -> dict:
        angle = socket.get()
        return {
            "angle_to_buoy": angle
        }

    def overview(self) -> None:
        ...
