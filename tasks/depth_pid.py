from ezauv.mission.mission import Subtask
from ezauv.utils.pid import PID
from ezauv import AccelerationState

import numpy as np


class DepthPID(Subtask):

    def __init__(self, wanted_heading, Kp, Ki, Kd):
        super().__init__()
        self.pid = PID(Kp, Ki, Kd, 0)
        self.wanted = wanted_heading

    def name(self) -> str:
        return "Depth PID"

    def update(self, sensor_data: dict) -> np.ndarray:
        depth = sensor_data["depth"]
        if depth is None:
            raise Exception("Depth PID cannot run without depth data")
        
        signal = self.pid.signal(self.wanted - depth)
        return AccelerationState(Tz=-signal, local=True)