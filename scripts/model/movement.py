import numpy as np
from core import forward_kinematics, inverse_kinematics, is_valid, joints_to_pwm
from typing import Callable, NewType, Tuple, Optional

StatusType = NewType('StatusType', str)

MotorSequence = NewType('MotorSequence', Callable[[float], Tuple[np.ndarray, StatusType]])

class set_height:
    def __init__(self,
                 current_position: np.ndarray,
                 height: float,
                 duration: float):
        self.x = np.empty_like(current_position)
        self.x_target = np.empty_like(current_position)
        self.x[:] = current_position[:]
        self.x_target[:] = current_position[:]
        self.x_target[2] = height
        self.duration = duration
        target_angles = inverse_kinematics(self.x_target)
        assert target_angles is not None

    def __call__(self, t):
        s = min(t / self.duration, 1)
        x_s = np.empty_like(self.x)
        x_s[:] = (1 - s) * self.x + s * self.x_target
        print(f'solving for {s}: {x_s}')
        angles = inverse_kinematics(x_s)
        assert angles is not None
        result = joints_to_pwm(angles)
        status = ""
        return result, status

#
# def set_height(current_position: np.ndarray,
#                height: float,
#                duration: float) -> Optional[MotorSequence]:
#     # Inputs:
#     # - Current Point
#     # - Target Height
#     # - Duration
#     # Output:
#     # - Motor Sequence [function t-> (motor_values, status)]
#     x = np.empty_like(current_position)
#     x_target = np.empty_like(current_position)
#     x[:] = current_position[:]
#     x_target[:] = current_position[:]
#     x_target[2] = height
#
#     target_angles = inverse_kinematics(x_target)
#     if target_angles is None or not is_valid(target_angles):
#         return None
#
#     def function(t):
#         s = min(t / duration, 1)
#         x_s = np.empty_like(x)
#         x_s[:] = (1 - s) * x + s * x_target
#         print(f'solving for {s}: {x_s}')
#         angles = inverse_kinematics(x_s)
#         assert angles is not None
#         result = joints_to_pwm(angles)
#         status = ""
#         return result, status
#
#     return function


def reset_to(current_position,
             target_position,
             max_height,
             duration):

    pass


