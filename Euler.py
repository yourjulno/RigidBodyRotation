from RungeKutta import Res as res
from RungeKutta import time as time
from pyquaternion import Quaternion
import numpy as np
import matplotlib.pyplot as plt
import initial_conditions as ic


class Procession:

    def __init__(self, kx=[], ky=[], kz=[], wx=[], wy=[], wz=[]):
        # init_list = [kx, ky, kz, wx, wy, wz]
        # for arg in init_list:
        #     if arg is None:
        #         arg = []
        self.kz = kz
        self.ky = ky
        self.kx = kx
        self.wx = wx
        self.wy = wy
        self.wz = wz

    def save_data(self, data):
        for i in data:
            self.wx.append(i[4])
            self.wy.append(i[5])
            self.wz.append(i[6])
            q = Quaternion(i[3], i[0], i[1], i[2])
            w = i[4]**2 + i[5]**2 + i[6]**2
            k = [i[4] * ic.tensor[0], i[5] * ic.tensor[1], i[6] * ic.tensor[2]]
            k_rot = q.rotate(k)
            self.kx.append(k_rot[0])
            self.ky.append(k_rot[1])
            self.kz.append(k_rot[2])

    def procession_graph(self):
        fig, ax = plt.subplots()

        ax.scatter(time, self.kx, color='blue', s=5, marker='o')
        ax.scatter(time, self.ky, color='red', s=5, marker='o')
        ax.scatter(time, self.kz, color='pink', s=5, marker='o')
        ax.set_xlabel("time")
        ax.set_ylabel("K")
        ax.minorticks_on()
        ax.grid(which='major', linewidth=0.3)
        ax.grid(which="minor", linestyle=':')
        ax.legend()
        plt.show()
        plt.savefig('calibration.png')


data = res.return_results()
Euler = Procession()
Euler.save_data(data)
Euler.procession_graph()