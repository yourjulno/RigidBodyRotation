from typing import Callable, List, Tuple
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt, cos, sin, tan, acos
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from quat import Quaternion
from Object import Object


class RightPart:
    def __init__(self, moments: np.array,
                 tensor: np.array,
                 ):
        self.moments = moments
        self.tensor = tensor
        # self.omegas = omega
        # self.quaternion = quaternion

    def calc_of_dw_dt(self, t: float,
                      y: np.array) -> np.array:
        w_x_dot = (self.moments[0] - (self.tensor[2] - self.tensor[1]) * y[5] * y[6]) / self.tensor[0]
        w_y_dot = (self.moments[1] - (self.tensor[0] - self.tensor[2]) * y[6] * y[4]) / self.tensor[1]
        w_z_dot = (self.moments[2] - (self.tensor[1] - self.tensor[0]) * y[4] * y[5]) / self.tensor[2]
        result: np.array = [w_x_dot, w_y_dot, w_z_dot]
        return result

    @staticmethod
    def calc_of_dq_dt(t: float,
                      y: np.array) -> np.array:
        result = np.zeros(4)
        q_x_dot = 0.5 * y[3] * y[4] + 0.5 * (y[1] * y[6] - y[2] * y[5])
        result += q_x_dot
        q_y_dot = 0.5 * y[3] * y[5] + 0.5 * (y[2] * y[4] - y[0] * y[6])
        result += q_y_dot
        q_z_dot = 0.5 * y[3] * y[6] + 0.5 * (y[0] * y[5] - y[1] * y[6])
        result += q_z_dot
        q_w_dot = y[0] * y[4] + y[1] * y[5] + y[2] * y[6]
        result += q_w_dot

        return result

    def func(self, t: float,
             y: np.ndarray,
             ) -> np.ndarray:
        state_vector = np.zeros(len(y))
        dw_dt = self.calc_of_dw_dt(t, y)
        dq_dt = self.calc_of_dq_dt(t, y)
        # sum = np.zeros(len(y))
        for i in range(4):
            state_vector[i] = state_vector[i] + dq_dt[i]
        for j in range(3):
            state_vector[j] = state_vector[j] + dw_dt[j]

        return state_vector


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


initial_cond = np.array([1., 0., 1.0, 0., 0., 1., 0.])
step = 0.01
initial_time = 0
end_time = 10
moments = [1, 1, 1]
tensor = [1, 1, 1]

times, states = Runge_Kutta.integrate(initial_cond, initial_time, end_time, step, RightPart(moments, tensor))
result_of_times = np.array(times)
result_of_states = np.array(states)
print(result_of_states)
print(result_of_times)
fig = plt.figure()
ax = plt.axes(projection='3d')

ax.plot(result_of_states[:, 0], result_of_states[:, 1], result_of_states[:, 2])

# plt.show()



def normalize(vector, tolerance=0.00001):
    mag2 = sum(n * n for n in vector)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        vector = tuple(n / mag for n in vector)
    return vector
def quaternionMult(quaternionOne, quaternionTwo):
    w1, x1, y1, z1 = quaternionOne
    w2, x2, y2, z2 = quaternionTwo
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def quaternionConjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)

def quaternionvectorProduct(quaternion, vector):

    quaternion2 = (0.0,) + vector

    return quaternionMult(quaternionMult(quaternion, quaternion2), quaternionConjugate(quaternion))[1:]
# def angletoQuaternion(vector, theta):
#     vector = normalize(vector)
#     x, y, z = vector
#     theta /= 2
#     w = cos(theta/2.)
#     x = x * sin(theta/2.)
#     y = y * sin(theta/2.)
#     z = z * sin(theta/2.)
#
#     return w, x, y, z

# def quaterniontoAngle(quaternion):
#     w, vector = quaternion[0], quaternion[1:]
#     theta = acos(w) * 2.0
#     return normalize(vector), theta

class Motion:

    @staticmethod
    def basis_vectors():
        e_x = (1, 0, 0)
        e_y = (0, 1, 0)
        e_z = (0, 0, 1)
        vector = (1, 1, 1)
        result_of_motion: np.array = []
        for i in range(len(result_of_states)):
            q = Quaternion(result_of_states[3][i], [result_of_states[0][i], result_of_states[1][i], result_of_states[2][i]])
            # q_ = q.conjugate()
            # q_norm = q.normalize()
            # q__norm = q_.normalize()
            # div = quaternionMult(q, q_)
            rotated_vector = q.rotate(vector)
            result_of_motion.append(rotated_vector)

        return vector



print(Motion.basis_vectors())
