import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random
from RungeKutta import Res as res

################################################################################
# Класс трехмерного объекта
class Object_3D:

    ############################################################################
    # Конструктор
    # Параметры:
    # Размеры трехмерного объекта:
    # L - длина, W - ширина, H - высота
    # xyz_ax_len - размеры связанной системы координат (ССК) объекта
    def __init__(self, L, W, H, xyz_ax_len):

        ########################################
        # Задание векторов для описания точек ССК и
        # вершин трехмерного объекта при вращении

        # ССК
        self.v_x = np.array([xyz_ax_len, 0, 0])
        self.v_y = np.array([0, xyz_ax_len, 0])
        self.v_z = np.array([0, 0, xyz_ax_len])
        # Объект
        self.v_1 = np.array([-L / 2, -W / 2, H / 2])
        self.v_2 = np.array([-L / 2, W / 2, H / 2])
        self.v_3 = np.array([L / 2, -W / 2, H / 2])
        self.v_4 = np.array([L / 2, W / 2, H / 2])
        self.v_5 = np.array([-L / 2, -W / 2, -H / 2])
        self.v_6 = np.array([-L / 2, W / 2, -H / 2])
        self.v_7 = np.array([L / 2, -W / 2, -H / 2])
        self.v_8 = np.array([L / 2, W / 2, -H / 2])

        ########################################
        # Задание начальных значений
        # массивов координат абсолютного местоположения
        # центра объекта, ССК и вершин трехмерного объекта при перемещении

        # Центр ССК и объекта
        self.pos = np.array([0, 0, 0])

        # Вектора для осей ССК (x, y, z)
        self.p_x = np.array(self.v_x)
        self.p_y = np.array(self.v_y)
        self.p_z = np.array(self.v_z)

        # Вектора для вершин объекта (x, y, z)
        self.p_1 = np.array(self.v_1)
        self.p_2 = np.array(self.v_2)
        self.p_3 = np.array(self.v_3)
        self.p_4 = np.array(self.v_4)
        self.p_5 = np.array(self.v_5)
        self.p_6 = np.array(self.v_6)
        self.p_7 = np.array(self.v_7)
        self.p_8 = np.array(self.v_8)

    ############################################################################
    # Начальная графическая прорисовка объекта
    # Параметр: пределы трехмерного графика
    def init_plot(self, xyz_lim):

        # Окно фигуры
        self.fig = plt.figure(figsize=(10, 6))
        self.ax = plt.axes(projection='3d', proj_type='ortho')

        # Координаты граней объекта
        self.top_verts = [self.p_1, self.p_2, self.p_4, self.p_3]
        self.bottom_verts = [self.p_5, self.p_6, self.p_8, self.p_7]
        self.left_verts = [self.p_1, self.p_3, self.p_7, self.p_5]
        self.right_verts = [self.p_4, self.p_2, self.p_6, self.p_8]
        self.front_verts = [self.p_3, self.p_4, self.p_8, self.p_7]
        self.back_verts = [self.p_2, self.p_1, self.p_5, self.p_6]

        # Грани объекта
        self.top = Poly3DCollection([self.top_verts], alpha=0.5, \
                                    linewidth=1, edgecolors='k')
        self.bottom = Poly3DCollection([self.bottom_verts], alpha=0.5, \
                                       linewidth=1, edgecolors='k')
        self.left = Poly3DCollection([self.left_verts], alpha=0.5, \
                                     linewidth=1, edgecolors='k')
        self.right = Poly3DCollection([self.right_verts], alpha=0.5, \
                                      linewidth=1, edgecolors='k')
        self.front = Poly3DCollection([self.front_verts], alpha=0.5, \
                                      linewidth=1, edgecolors='k')
        self.back = Poly3DCollection([self.back_verts], alpha=0.5, \
                                     linewidth=1, edgecolors='k')

        # Добавление трехмерного объекта
        alpha = 0.2
        self.top.set_facecolor((0.7, 0.7, 0.7, alpha))
        self.bottom.set_facecolor((0.7, 0.7, 0.7, alpha))
        self.left.set_facecolor((0.7, 0.7, 0.7, alpha))
        self.right.set_facecolor((0.7, 0.7, 0.7, alpha))
        self.front.set_facecolor((0.7, 0.7, 0.7, alpha))
        self.back.set_facecolor((0.7, 0.7, 0.7, alpha))

        self.ax.add_collection3d(self.top)
        self.ax.add_collection3d(self.bottom)
        self.ax.add_collection3d(self.left)
        self.ax.add_collection3d(self.right)
        self.ax.add_collection3d(self.front)
        self.ax.add_collection3d(self.back)

        # Добавление осей ССК
        self.X_ax = self.ax.plot([self.pos[0], self.p_x[0]], \
                                 [self.pos[1], self.p_x[1]], \
                                 [self.pos[2], self.p_x[2]], \
                                 linewidth=2.0, color='blue')
        self.Y_ax = self.ax.plot([self.pos[0], self.p_y[0]], \
                                 [self.pos[1], self.p_y[1]], \
                                 [self.pos[2], self.p_y[2]], \
                                 linewidth=2.0, color='red')
        self.Z_ax = self.ax.plot([self.pos[0], self.p_z[0]], \
                                 [self.pos[1], self.p_z[1]], \
                                 [self.pos[2], self.p_z[2]], \
                                 linewidth=2.0, color='green')

        # Настройка осей графика
        self.ax.set_xlim(-xyz_lim, xyz_lim)
        self.ax.set_ylim(-xyz_lim, xyz_lim)
        self.ax.set_zlim(-xyz_lim, xyz_lim)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Подпись графика
        plt.title('3D object')

        # Угол наблюдения
        self.ax.view_init(40, 40)

        # Вывести фигуру
        plt.ion()
        plt.show()

    ############################################################################
    # Обновление графического изображения объекта после перемещения или поворота
    def update_plot(self):

        # Вектора осей ССК (x, y, z)
        np.put(self.p_x, [0, 1, 2], self.v_x + self.pos)
        np.put(self.p_y, [0, 1, 2], self.v_y + self.pos)
        np.put(self.p_z, [0, 1, 2], self.v_z + self.pos)

        # Вектора для вершин объекта (x, y, z)
        np.put(self.p_1, [0, 1, 2], self.v_1 + self.pos)
        np.put(self.p_2, [0, 1, 2], self.v_2 + self.pos)
        np.put(self.p_3, [0, 1, 2], self.v_3 + self.pos)
        np.put(self.p_4, [0, 1, 2], self.v_4 + self.pos)
        np.put(self.p_5, [0, 1, 2], self.v_5 + self.pos)
        np.put(self.p_6, [0, 1, 2], self.v_6 + self.pos)
        np.put(self.p_7, [0, 1, 2], self.v_7 + self.pos)
        np.put(self.p_8, [0, 1, 2], self.v_8 + self.pos)

        # Обновить положение граней
        self.top.set_verts([self.top_verts])
        self.bottom.set_verts([self.bottom_verts])
        self.left.set_verts([self.left_verts])
        self.right.set_verts([self.right_verts])
        self.front.set_verts([self.front_verts])
        self.back.set_verts([self.back_verts])

        # Обновить положение ССК
        self.X_ax[0].set_data_3d([self.pos[0], self.p_x[0]], \
                                 [self.pos[1], self.p_x[1]], \
                                 [self.pos[2], self.p_x[2]])
        self.Y_ax[0].set_data_3d([self.pos[0], self.p_y[0]], \
                                 [self.pos[1], self.p_y[1]], \
                                 [self.pos[2], self.p_y[2]])
        self.Z_ax[0].set_data_3d([self.pos[0], self.p_z[0]], \
                                 [self.pos[1], self.p_z[1]], \
                                 [self.pos[2], self.p_z[2]])

        # Вывести обновленный график
        self.fig.canvas.draw()

    ############################################################################
    # Элементарный шаг вращения
    # Параметры: парметры кватрениона
    def rotate_step(self, w, qx, qy, qz):

        # Ось, вдоль которой идет поворот
        # if axis == 'x':
        #     axis_rot = self.v_x
        # if axis == 'y':
        #     axis_rot = self.v_y
        # if axis == 'z':
        #     axis_rot = self.v_z

        # Кватернион поворота
        q_rot = Quaternion(w, qx, qy, qz)

        # Поворот векторов объекта на кватерион поворота
        # ССК
        self.v_x = q_rot.rotate(self.v_x)
        self.v_y = q_rot.rotate(self.v_y)
        self.v_z = q_rot.rotate(self.v_z)
        # Объект
        self.v_1 = q_rot.rotate(self.v_1)
        self.v_2 = q_rot.rotate(self.v_2)
        self.v_3 = q_rot.rotate(self.v_3)
        self.v_4 = q_rot.rotate(self.v_4)
        self.v_5 = q_rot.rotate(self.v_5)
        self.v_6 = q_rot.rotate(self.v_6)
        self.v_7 = q_rot.rotate(self.v_7)
        self.v_8 = q_rot.rotate(self.v_8)

    ############################################################################
    # Вращение
    # Параметры: количество шагов
    def rotate(self, repeats):
        for i in range(0, repeats):
            self.rotate_step(res.return_results()[i][3], res.return_results()[i][0],
                                                    res.return_results()[i][1],
                                                    res.return_results()[i][2])

            # Обновить изображение
            self.update_plot()
            plt.pause(.001)


################################################################################

# Размеры цветных отрезков связанной системы координат объекта
xyz_ax_len = 150

# Размеры трехмерного объекта
# (L - длина, W - ширина, H - высота)
L = 80
W = 80
H = 50

# Пределы трехмерного пространства
xyz_lim = 300

# Создание объекта
obj = Object_3D(L, W, H, xyz_ax_len)

# Вывод графического изображения
obj.init_plot(xyz_lim)

# Повороты вокруг осей связанной системы координат

obj.rotate(len(res.return_results()))
