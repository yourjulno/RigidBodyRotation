from typing import Callable, List, Tuple
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from draw import Drawing_Schema
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from math import sqrt, cos, sin, tan, acos

from matplotlib.patches import Ellipse
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


initial_cond = np.array([0., 1.0, 0.0, 0., 1., 0., 0.])
step = 0.01
initial_time = 0
end_time = 10
moments = [1, 0, 0]
tensor = [1, 1, 1]

times, states = Runge_Kutta.integrate(initial_cond, initial_time, end_time, step, RightPart(moments, tensor))
result_of_times = np.array(times)
result_of_states = np.array(states)



class Motion:

    @staticmethod
    def basis_vectors(x, y, z):
        result_of_motion: np.array = []
        for i in range(len(result_of_states)):
            q = Quaternion(result_of_states[i][3], [result_of_states[i][0],
                                                    result_of_states[i][1], result_of_states[i][2]])
            # q_ = q.conjugate()
            # q_norm = q.normalize()
            # q__norm = q_.normalize()
            # div = quaternionMult(q, q_)
            rotated_vector_x = q.rotate((x, y, z))
            result_of_motion.append(rotated_vector_x)

        return result_of_motion


fig = plt.figure()
ax = plt.axes(projection='3d')
x = Motion.basis_vectors(1, 0, 0)
print(x)
y = Motion.basis_vectors(0, 1, 0)
z = Motion.basis_vectors(0, 0, 1)

# ax.plot(c, result_of_times )
xs = []
xy = []
xz = []
for basis in x:
    xs.append(basis[0])
    xy.append(basis[1])
    xz.append(basis[2])
ex = None
# ax.set_xlim([0, 1])
# ax.set_ylim([0, 1])
# ax.set_zlim([0, 1])
for i in range(len(xs)):

    if ex:
        ex.remove()
        ey.remove()
        ez.remove()
    ex = ax.quiver(0,0,0,x[i][0],x[i][1],x[i][2],color='red')
    ey = ax.quiver(0,0,0,y[i][0],y[i][1],y[i][2])
    ez = ax.quiver(0,0,0,z[i][0],z[i][1],z[i][2],color="green")
    plt.pause(.001)

# sphere = plt.line(axes=ax, view=[-25,12])























# wframe = None
# for phi in np.linspace(0, 180. / np.pi, 100):
#     if wframe:
#         wframe.remove()
#     wframe = ax.plot(x, y, z)
#     plt.pause(1)

plt.show()
# Plot the new wireframe and pause briefly before continuing.
#     wframe = ax.quiver([0,0,0],[0,0,0],[0,0,0],[b[0],0,0],[0,b[1],0],[0,0,b[2]], colors=cols)

# print(Motion.basis_vectors())
