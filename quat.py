import numpy as np
from math import sqrt


class Quaternion:

    def __init__(self, l_0: float,
                l: np.array):
        self.l_0 = l_0
        self.l = np.array(l)

    def sum_quat(self, other):
        other: Quaternion

        return Quaternion(self.l_0 + other.l_0, np.add(self.l, other.l))

    def norm(self):
        return sqrt((self.l_0 ** 2) + (self.l[0] ** 2) + (self.l[1] ** 2) + (self.l[2] **2))

    def division(self, other):
        return Quaternion(self.l_0 * other.l_0 - self.l.dot(other.l), self.l * other.l
                          + other.l_0 * self.l + np.cross(self.l, other.l))

    def size(self, le: float):
        self.l_0 = self.l_0 * le
        self.l = le * self.l
        return self

    # нормированный кватернион
    def normalize(self):
        return self.size(self.norm())

    def inv(self):
        return self.conjugate().normalize()

    # сопряжённый кватернион
    def conjugate(self):
        return Quaternion(self.l_0, [-self.l[0], -self.l[1], -self.l[2]])

    # произвольное положение твердого тела с неподвижной точкой
    # задаётся нормированным кватернионом L по формуле: e_k = L * i_k * L'
    def rotate(self, vector: np.array):
        return np.array(self.division(Quaternion(0, vector)).division(self.inv()).l)



