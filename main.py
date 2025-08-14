import numpy as np

from ezauv.auv import AUV
from ezauv.hardware import MotorController, Motor, SensorInterface
from ezauv.utils.inertia import InertiaBuilder, Cuboid
from ezauv.mission.tasks.main import AccelerateVector
from ezauv.mission.tasks.subtasks import HeadingPID
from ezauv.mission import Path
from ezauv import AccelerationState


from hardware_interface import SubHardware
from tasks.depth_pid import DepthPID

motor_locations = [
    # horizontals
    np.array([-1., -1., 0.]), # 2
    np.array([-1., 1., 0.]), # 8
    np.array([1., 1., 0.]), # 9
    np.array([1., -1., 0.]), # 3

    # verticals
    np.array([0., 0., 0.]) 
    ]

motor_directions = [
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.]),
    np.array([1., -1., 0.]),
    np.array([1., 1., 0.]),

    np.array([0, 0, 4.])
    ]

motor_indices = [
    2,
    8,
    9,
    3,
    4
]
bounds = [[-0.2, 0.2]] * 4 + [[-0.8, 0.8]]
deadzone = [[-0.11, 0.11]] * 4 + [[-0.44, 0.44]]

hardware = SubHardware(
    arduino_port='/dev/ttyUSB0',
    vectornav_port='/dev/ttyUSB1'
)

anchovy = AUV(
    motor_controller = MotorController(
        inertia = InertiaBuilder(
            Cuboid(
                mass=1,
                width=1,
                height=1,
                depth=0.1,
                center=np.array([0,0,0])
            )).moment_of_inertia(),
            motors = [
                Motor(
                    direction,
                    loc,
                    lambda magnitude, idx=motor_indices[i]: hardware.set_motor(idx, magnitude),
                    lambda idx=motor_indices[i]: hardware.initialize_motor(idx),
                    Motor.Range(bounds[i][0], bounds[i][1]),
                    Motor.Range(-deadzone[i][0], deadzone[i][1])
                    )
                for i, (loc, direction) in enumerate(zip(motor_locations, motor_directions))
                ]
        ),
        sensors = SensorInterface(sensors=[hardware.imu, hardware.depth]),
        pin_kill = hardware.kill,
        lock_to_yaw=True
    )

#anchovy.register_subtask(HeadingPID(0, 0.03, 0.0, 0.01))
#anchovy.register_subtask(DepthPID(-0.6, 0.7, 0.0, 0.1))


mission = Path(
    AccelerateVector(AccelerationState(), 20)
)

anchovy.travel_path(mission)