import numpy as np
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
        interval = 0.01
        total = int(5 / interval)
        self.log(f"Calibrating IMU heading over 5 seconds ({total} checks)...")
        heading_sum = 0
        for i in np.linspace(0, 5, total):
            heading_sum += self.vectornav.read_yaw_pitch_roll().x
        self.calibrated_heading = heading_sum / total

    def overview(self) -> None:
        self.log(f"VectorNav IMU --- {self.vectornav.read_model_number()}")

    

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
        self.log("remember to make this better later")