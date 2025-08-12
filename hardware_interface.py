import numpy as np

from hardware.motor_serial import MotorSerial
from hardware.sensors import VectorNavIMU, DepthSensor

class SubHardware:
    def __init__(self, *, arduino_port, vectornav_port, vectornav_baud=921600, depth_bus=1):
        self.motor_serial = MotorSerial(arduino_port) 
        self.imu = VectorNavIMU(vectornav_port, vectornav_baud)
        self.depth = DepthSensor(bus=depth_bus)

        self.prev = {}

    def set_motor(self, pin, magnitude):
        if pin in self.prev and self.prev[pin] == magnitude:
            return
        self.prev[pin] = magnitude

        self.motor_serial.send({pin: magnitude})

    def initialize_motor(self, pin):
        return 0

    def kill(self):
        self.motor_serial.kill()