import numpy as np
from pyquaternion import Quaternion
from matplotlib import pyplot as plt
from matplotlib import animation


class DrawObject:
    def __init__(self, res):
        self.q = None
        self.fig = plt.figure()
        self.quats = []
        self.ax = self.fig.add_axes([0, 0, 1, 1], projection='3d')
        self.iterator_quats = iter(self.quats)
        self.canvas = []
        self.res = res

    def SetAxis(self):
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.ax.set_xlim((-2, 2))
        self.ax.set_ylim((-2, 2))
        self.ax.set_zlim((-2, 2))

    def GetResultFromEquations(self):
        y = self.res
        for i in y:
            self.quats.append(Quaternion(i[3], i[0], i[1], i[2]))

    def __set_canvas__(self):
        e1, = self.ax.plot([], [], c='green')
        e2, = self.ax.plot([], [], c='blue')
        e3, = self.ax.plot([], [], c='red')
        return [e1, e2, e3]

    def GetCanvas(self):
        self.canvas = self.__set_canvas__()

    def get_axis_state(self):
        e1 = next(self.iterator_quats).rotate([1, 0, 0])
        e2 = next(self.iterator_quats).rotate([0, 1, 0])
        e3 = next(self.iterator_quats).rotate([0, 0, 1])

        return [e1, e2, e3]

    def update_canvas(self):
        data = self.get_axis_state()
        e1x, e1y, e1z = data[0]
        e2x, e2y, e2z = data[1]
        e3x, e3y, e3z = data[2]
        self.canvas[0].set_data([0, e1x], [0, e1y])
        self.canvas[1].set_data([0, e2x], [0, e2y])
        self.canvas[2].set_data([0, e3x], [0, e3y])
        self.canvas[0].set_3d_properties([0, e1z])
        self.canvas[1].set_3d_properties([0, e2z])
        self.canvas[2].set_3d_properties([0, e3z])
        return self.canvas

    def animate(self, i):
        # self.q = next(self.iterator_quats)
        e1, e2, e3 = self.update_canvas()

        return e1, e2, e3

    def Prepare(self):
        self.SetAxis()
        self.GetResultFromEquations()
        self.__set_canvas__()
        self.GetCanvas()

    def Draw(self):
        t = np.linspace(0, 30, 1000)
        self.Prepare()
        ani = animation.FuncAnimation(self.fig, self.animate, frames=len(t), interval=1000 * t.max() / len(t),
                                      blit=True)
        plt.show()


class DrawPlot:
    def __init__(self, res, time):
        self.fig, self.ax = plt.subplots()
        self.res = res
        self.time = time

    def SetScatter(self):
        self.ax.scatter(self.time, self.res, color='green', s=5, marker='o')

    def SetSettingsonGraphic(self):
        self.ax.set_xlabel("time")
        self.ax.set_ylabel("K")
        self.ax.minorticks_on()
        # plt.title("T = 50C")
        self.ax.grid(which='major', linewidth=0.3)
        self.ax.grid(which="minor", linestyle=':')
        # ax.set(xlim=(14, 17), ylim=(0, 0.4))
        self.ax.legend()

    def ShowPlot(self):
        self.SetScatter()
        self.SetSettingsonGraphic()
        plt.show()
