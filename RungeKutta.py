from typing import Callable, List, Tuple
from abc import ABC, abstractmethod
import numpy as np
from pyquaternion import Quaternion
from matplotlib import pyplot as plt


class BaseMoment(ABC):

    @abstractmethod
    def calcTorque(self, t: float, y: np.ndarray) -> np.ndarray:
        """

        :param t:
        :param y:
        :return:
        """


####
# class ConstatntTorque(BaseMoment):
#
#     def __init__(self, const_torque: np.ndarray):
#         self.const_torque = const_torque
#
#     def calcTorque(self, t: float, y: np.ndarray) -> np.ndarray:
#         return self.const_torque


class PID(BaseMoment):

    def __init__(self,
                 y_end: np.ndarray):
        self.q_target = Quaternion(y_end[3], y_end[0], y_end[1], y_end[2])
        self.Kq: float = 1
        self.Kw: float = 1

    @staticmethod
    def get_W(t: float,
              y: np.ndarray) -> np.array:
        return [y[4], y[5], y[6]]

    def calcDeltaQuat(self, t: float,
                      y: np.ndarray) -> Quaternion:
        # delta_quat = self.q_target - Quaternion(y[3], y[0], y[1], y[2])
        delta_quat = self.q_target*y[3]
        return delta_quat

    def calcTorque(self, t: float,
                   y: np.ndarray) -> np.ndarray:
        w_begin = np.zeros(3)

        torque_t = np.array(self.Kq * self.calcDeltaQuat(t, y).vector + self.Kw * (w_begin - self.get_W(t, y)))
        # print(torque_t)
        return torque_t


#
#######################################################################
# Правая часть уравнения движения q' = 1/2 q * w

class RightPart:
    def __init__(self, torque_list: List[BaseMoment],
                 tensor: np.array,
                 ):
        self.torque_list = torque_list
        self.tensor = tensor

    def calc_of_torque_in_different_times(self, t: float,
                                          y: np.array) -> np.array:
        res_torque = np.zeros(3)

        for torque in self.torque_list:
            res_torque += torque.calcTorque(t, y)
        return res_torque

    def calc_of_dw_dt(self, t: float,
                      y: np.array) -> np.array:
        result = np.zeros(3)
        w_x_dot = (self.calc_of_torque_in_different_times(t, y)[0]
                   - (self.tensor[2] - self.tensor[1]) * y[5] * y[6]) / self.tensor[0]
        result[0] = w_x_dot
        w_y_dot = (self.calc_of_torque_in_different_times(t, y)[1]
                   - (self.tensor[0] - self.tensor[2]) * y[6] * y[4]) / self.tensor[1]
        result[1] = w_y_dot
        w_z_dot = (self.calc_of_torque_in_different_times(t, y)[2]
                   - (self.tensor[1] - self.tensor[0]) * y[4] * y[5]) / self.tensor[2]
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
            # print(result[-1])
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
            print(w_x * w_x + w_y * w_y + w_z * w_z)
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

    @staticmethod
    def GetWFromResults(y: np.ndarray) -> np.array:
        W_sum = []
        for i in y:
            w_x = i[4]
            w_y = i[5]
            w_z = i[6]
            W_sum.append(w_x ** 2 + w_y ** 2 + w_z ** 2)
        return np.array(W_sum)

    @staticmethod
    def GetXQuatComponent(y: np.ndarray) -> np.array:
        qx = []
        for i in y:
            x = i[1]

            qx.append(x)

        return qx

    @staticmethod
    def GetYQuatComponent(y: np.ndarray) -> np.array:
        qy = []
        for i in y:
            y = i[2]

            qy.append(y)

        return qy

    @staticmethod
    def GetZQuatComponent(y: np.ndarray) -> np.array:
        qz = []
        for i in y:
            z = i[3]

            qz.append(z)

        return qz


#для быстрой отрисовки
initial_cond = [0., 0, 0, 1, 2, 2, 2]  # начальные условия удовлетворяют случай эйлера
initial_time = 0
end_time = 40
tensor = [2, 2, 1]
step = 0.002
initial = np.array([0., 0, 0, 1, 0, 0, 0])
pid = PID(initial)
c = [pid]
Res = Res(initial_cond, initial_time, end_time, step, tensor, c)
res = Res.return_results_of_state()
Time = Res.return_time_()
fig, ax = plt.subplots()
x = Res.GetXQuatComponent(res)
y = Res.GetYQuatComponent(res)
z = Res.GetZQuatComponent(res)
ax.scatter(Time, x, color='green', s=5, marker='o')
ax.scatter(Time, y, color='blue', s=5, marker='o')
ax.scatter(Time, z, color='red', s=5, marker='o')
ax.legend()
plt.savefig("calibration.png")
plt.show()
