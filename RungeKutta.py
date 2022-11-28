from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import numpy as np
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


#
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

################################################################################
# Возвращает вектор состояния (qw, qx, qy, qz, wx, wy, wz)
class Res:

    def __init__(self, initial_cond, initial_time, end_time, step, tensor, mom):
        self.initial_cond = initial_cond
        self.initial_time = initial_time
        self.end_time = end_time
        self.step = step
        self.tensor = tensor
        self.mom = mom
        self.results = []
        self.times = []

    def return_results_of_state(self):
        times, states = Runge_Kutta.integrate(self.initial_cond, self.initial_time, self.end_time, self.step,
                                              RightPart(self.mom, self.tensor))
        self.results = np.array(states)
        return self.results

    def return_time_(self):
        times, states = Runge_Kutta.integrate(self.initial_cond, self.initial_time, self.end_time, self.step,
                                              RightPart(self.mom, self.tensor))
        self.times = np.array(times)
        return self.times

    def GetKinTorqueFromResults(self):
        Kx, Ky, Kz = [], [], []
        for i in self.return_results_of_state():
            w_x = i[4]
            w_y = i[5]
            w_z = i[6]
            q = Quaternion(i[3], i[0], i[1], i[2])
            K = [w_x * self.tensor[0], w_y * self.tensor[1], w_z * self.tensor[2]]

            K_rotate = q.rotate(K)
            Kx.append(K_rotate[0])
            Ky.append(K_rotate[1])
            Kz.append(K_rotate[2])
        return Kx, Ky, Kz

    def GetEnergyFromResults(self):
        T_sum = []
        for i in self.return_results_of_state():
            w_x = i[4]
            w_y = i[5]
            w_z = i[6]
            T = self.tensor[0] * w_x * w_x + self.tensor[1] * w_y * w_y + self.tensor[2] * w_z * w_z
            T_sum.append(T)
        return np.array(T_sum)
