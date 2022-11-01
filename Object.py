import numpy as np
from math import sqrt
from math import cos, sin
from casadi import *


# w/dt


def dw_dt_moments(w, m, Ixx, Iyy, Izz):
    w_x = w[0]
    w_y = w[1]
    w_z = w[2]

    Mx = m[0]
    My = m[1]
    Mz = m[2]

    w_x_dot = (Mx - (Izz - Iyy) * w_y * w_z) / Ixx
    w_y_dot = (My - (Ixx - Izz) * w_z * w_x) / Iyy
    w_z_dot = (Mz - (Iyy - Ixx) * w_x * w_y) / Izz

    dw_dt = vertcat(w_x_dot, w_y_dot, w_z_dot)

    return dw_dt

print(dw_dt_moments([1, 0, 1], [0, 1, 1], 1, 1, 1))
