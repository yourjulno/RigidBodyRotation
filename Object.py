import numpy as np
from math import sqrt
from math import cos, sin
from casadi import *
from typing import Callable, List, Tuple
from quat import Quaternion


# w/dt

class Object:

    def __init__(self, omega: np.array,
                 moments: np.array,
                 Ixx: float,
                 Iyy: float,
                 Izz: float):
        self.w_x = omega[0]
        self.w_y = omega[1]
        self.w_z = omega[2]

        self.Mx = moments[0]
        self.My = moments[1]
        self.Mz = moments[2]

        self.Ixx = Ixx
        self.Iyy = Iyy
        self.Izz = Izz

    # w/dt = I^-1(M - w x (I*w))
    @staticmethod
    def dw_dt(self) -> np.array:
        w_x_dot = (self.Mx - (self.Izz - self.Iyy) * self.w_y * self.w_z) / self.Ixx
        w_y_dot = (self.My - (self.Ixx - self.Izz) * self.w_z * self.w_x) / self.Iyy
        w_z_dot = (self.Mz - (self.Iyy - self.Ixx) * self.w_x * self.w_y) / self.Izz

        dw_dt: List[float] = [w_x_dot, w_y_dot, w_z_dot]

        return dw_dt

    # q * w = (q0, qx, qy, qz) * (0, wx, wy, wz)^T
    @staticmethod
    def dq_dt(self, q: Quaternion) -> np.array:
        q_x_dot = 0.5 * q.l_0 * self.w_x + 0.5 * (q.l[1] * self.w_z - q.l[2] * self.w_y)
        q_y_dot = 0.5 * q.l_0 * self.w_y + 0.5 * (q.l[2] * self.w_x - q.l[0] * self.w_z)
        q_z_dot = 0.5 * q.l_0 * self.w_z + 0.5 * (q.l[0] * self.w_y - q.l[1] * self.w_x)
        q_w_dot = q.l[0] * self.w_x + q.l[1] * self.w_y + q.l[2] * self.w_z
        dq_dt: List[float] = [q_x_dot, q_y_dot, q_z_dot, q_w_dot]

        return dq_dt

    @staticmethod
    def state_vector(self, q: Quaternion) -> np.array:
        a = self.dq_dt(self, q)
        b = self.dw_dt(self)
        state: np.array = ([[a], [b]])
        # for i in a:
        #     state.append(i)
        # for j in b:
        #     state.append(j)
        return state
