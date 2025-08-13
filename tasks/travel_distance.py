from ezauv.mission.mission import Task
from ezauv.utils.pid import PID
from ezauv import AccelerationState

import numpy as np


class TravelTo(Task):
    def __init__(self, position, Kp, Ki, Kd):
        # gains are 3d vectors

        self.target = position
        self.pid_x = PID(Kp[0], Ki[0], Kd[0])
        self.pid_y = PID(Kp[1], Ki[1], Kd[1])
        self.pid_z = PID(Kp[2], Ki[2], Kd[2])

    def name(self) -> str:
        return "Travel distance"

    def update(self, sensor_data: dict):
        # this will just use 1 pid per-axis for now but in the future we might want cascaded
        pos = sensor_data["position"]

        if pos is None:
            raise Exception("Travel distance task cannot run without position data")
        if self.start_pos is None:
            self.start_pos = pos
        
        signal_x = self.pid_x.signal(pos[0])
        signal_y = self.pid_y.signal(pos[1])
        signal_z = self.pid_y.signal(pos[2])
        return AccelerationState(Tx=signal_x,Ty=signal_y,Tz=signal_z, local=False)