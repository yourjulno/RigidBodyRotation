from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt

import numpy as np

import pyquaternion
from pyquaternion import Quaternion


class BaseMoment(ABC):

    @abstractmethod
    def calcTorque(self, t: float, y: np.ndarray) -> np.ndarray:
        """

        :param t:
        :param y:
        :return:
        """

####
class ConstatntTorque(BaseMoment):

    def __init__(self, const_torque: np.ndarray):
        self.const_torque = const_torque

    def calcTorque(self, t: float, y: np.ndarray) -> np.ndarray:
        return self.const_torque


#######################################################################
# Правая часть уравнения движения q' = 1/2 q * w

class RightPart:
    def __init__(self, torque_list: List[BaseMoment],
                 tensor: np.array,
                 ):
        self.torque_list = torque_list
        self.tensor = tensor

    def calc_of_dw_dt(self, t: float,
                      y: np.array) -> np.array:
        res_torque = np.zeros(3)
        for torque in self.torque_list:
            res_torque += torque.calcTorque(t, y)
        result = np.zeros(3)
        w_x_dot = (res_torque[0] - (self.tensor[2] - self.tensor[1]) * y[5] * y[6]) / self.tensor[0]
        result[0] = w_x_dot
        w_y_dot = (res_torque[1] - (self.tensor[0] - self.tensor[2]) * y[6] * y[4]) / self.tensor[1]
        result[1] = w_y_dot
        w_z_dot = (res_torque[2] - (self.tensor[1] - self.tensor[0]) * y[4] * y[5]) / self.tensor[2]
        result[2] = w_z_dot

        return result

    @staticmethod
    def calc_of_dq_dt(t: float,
                      y: np.array) -> np.array:
        q_quat = Quaternion(y[3], y[0], y[1], y[2])
        q_w = Quaternion(0, y[4], y[5], y[6])
        result = q_quat * q_w

        return result

    def func(self, t: float,
             y: np.ndarray,
             ) -> np.array:
        dot_vector = np.zeros(len(y))
        dw_dt = self.calc_of_dw_dt(t, y)
        dq_dt = self.calc_of_dq_dt(t, y)

        dot_vector[0:3] = dq_dt.vector
        dot_vector[3] = dq_dt.w
        dot_vector[4:7] = dw_dt

        return dot_vector


#######################################################
# Решение диффура 1-ого порядка
class Runge_Kutta:

    @staticmethod
    def make_step(time: float,
                  step: float,
                  state: np.array,
                  right_part: RightPart,
                  ) -> np.array:
        k1 = right_part.func(time, state)
        k2 = right_part.func(time + step / 2, state + (step / 2) * k1)
        k3 = right_part.func(time + step / 2, state + (step / 2) * k2)
        k4 = right_part.func(time + step, state + step * k3)
        y = state + (step / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y

    @staticmethod
    def integrate(
            y0: np.array,
            time_0: float,
            time_end: float,
            step: float,
            right_part: RightPart,
    ) -> Tuple[List[float], List[np.ndarray]]:
        time = time_0
        times: List[float] = [time]
        result: List[np.ndarray] = [y0]
        while time < time_end:
            result.append(Runge_Kutta.make_step(times[-1], step, result[-1], right_part))
            time = time + step
            times.append(time)
        return times, result


initial_cond = np.array([0, 0., 0., 1., 0., 1., 0])
step = 0.003
initial_time = 0
end_time = 40
constant_torque = [0., 0., -1]
tensor = [2, 2, 1]

c = ConstatntTorque(np.array(constant_torque))
mom = [c]


################################################################################
# Возвращает вектор состояния (qw, qx, qy, qz, wx, wy, wz)
class Res:

    @staticmethod
    def return_results():
        times, states = Runge_Kutta.integrate(initial_cond, initial_time, end_time, step, RightPart(mom, tensor))
        result_of_times = np.array(times)
        result_of_states = np.array(states)
        return result_of_states


time, state = Runge_Kutta.integrate(initial_cond, initial_time, end_time, step, RightPart(mom, tensor))
# print(Res.return_results())
wx = []
wy = []
wz = []
Kx = []
Ky = []
Kz = []
for i in Res.return_results():
    w_x = i[4]
    w_y = i[5]
    w_z = i[6]
    wx.append(w_x)
    wy.append(w_y)
    wz.append(w_z)
    q = Quaternion(i[3], i[0], i[1], i[2])
    w = w_x * w_x + w_y * w_y + w_z * w_z
    K = [w_x * tensor[0], w_y * tensor[1], w_z * tensor[2]]

    K_rotate = q.rotate(K)
    Kx.append(K_rotate[0])
    Ky.append(K_rotate[1])
    Kz.append(K_rotate[2])
    # print(K_rotate)
    # print(w)
    # print(w_x * w_x * tensor[0] + w_y * w_y * tensor[1] + w_z * w_z * tensor[2])

fig, ax = plt.subplots()

ax.scatter(time, Kx, color='green', s=5, marker='o')
ax.scatter(time, Ky, color='blue', s=5, marker='o')
ax.scatter(time, Kz, color='red', s=5, marker='o')

ax.set_xlabel("time")
ax.set_ylabel("K")
ax.minorticks_on()
# plt.title("T = 50C")
ax.grid(which='major', linewidth=0.3)
ax.grid(which="minor", linestyle=':')
# ax.set(xlim=(14, 17), ylim=(0, 0.4))
ax.legend()
plt.savefig('calibration.png')

# print(k[1])
plt.show()
