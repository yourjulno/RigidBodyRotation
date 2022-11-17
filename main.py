import numpy as np
from matplotlib.backends._backend_tk import blit

from pyquaternion import Quaternion

from matplotlib import pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

from RungeKutta import Res as res

fig = plt.figure()
ax = fig.add_axes([0, 0, 1, 1], projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

colors = ['r', 'g', 'b']

lines = sum([ax.plot([], [], [], c=c)
             for c in colors], [])

# prepare the axes limits
ax.set_xlim((-2, 2))
ax.set_ylim((-2, 2))
ax.set_zlim((-2, 2))

ax.view_init(30, 0)


def init():
    for line in lines:
        line.set_data([], [])
        line.set_3d_properties([])

    return lines


quats = []

y = res.return_results()
for i in y:
    quats.append(Quaternion(i[3], i[0], i[1], i[2]))
iterator_quats = iter(quats)

startpoints = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
endpoints = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
e1 = [1, 0, 0]
e2 = [0, 1, 0]
e3 = [0, 0, 1]

def __set_canvas__(ax):
    e1, = ax.plot([], [])
    e2, = ax.plot([], [])
    e3, = ax.plot([], [])
    return [e1,e2,e3]
canvas = __set_canvas__(ax)
e1 = [1, 0, 0]
e2 = [0, 1, 0]
e3 = [0, 0, 1]

def get_axis_state(q):

        e1 = np.add(0, q.rotate([1, 0, 0]))
        e2 = np.add(0, q.rotate([0, 1, 0]))
        e3 = np.add(0, q.rotate([0, 0, 1]))

        return [e1, e2, e3]
def update_canvas(canvas, q):

    data = get_axis_state(q)
    e1x, e1y, e1z = data[0]
    e2x, e2y, e2z = data[1]
    e3x, e3y, e3z = data[2]
    canvas[0].set_data([0, e1x], [0, e1y])
    canvas[1].set_data([0, e2x], [0, e2y])
    canvas[2].set_data([0, e3x], [0, e3y])
    canvas[0].set_3d_properties([0, e1z])
    canvas[1].set_3d_properties([0, e2z])
    canvas[2].set_3d_properties([0, e3z])
    return canvas
def animate(i):



    q = quats
    e1,e2,e3 = update_canvas(canvas, q[i])

    return e1, e2, e3

animate(0)
interval = None
t = np.linspace(0,30,1000)
if interval is None:
    ani = animation.FuncAnimation(fig, animate, frames=len(t), interval=1000 * t.max() / len(t), blit=blit)
else:
    ani = animation.FuncAnimation(fig, animate, frames=len(t), interval=interval / len(t), blit=blit)

plt.show()
