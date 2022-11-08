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


        # ССК
        self.v_x = np.array([xyz_ax_len, 0, 0])
        self.v_y = np.array([0, xyz_ax_len, 0])
        self.v_z = np.array([0, 0, xyz_ax_len])


        ########################################
        # Задание начальных значений
        # массивов координат абсолютного местоположения
        # центра ССК

        # Центр ССК
        self.pos = np.array([0, 0, 0])

        # Вектора для осей ССК (x, y, z)
        self.p_x = np.array(self.v_x)
        self.p_y = np.array(self.v_y)
        self.p_z = np.array(self.v_z)



    ############################################################################
    # Начальная графическая прорисовка
    # Параметр: пределы трехмерного графика
    def init_plot(self, xyz_lim):

        # Окно фигуры
        self.fig = plt.figure(figsize=(10, 6))
        self.ax = plt.axes(projection='3d', proj_type='ortho')


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
        # plt.ion()
        plt.show(block=False)

    ############################################################################
    # Обновление графического изображения объекта после перемещения или поворота
    def update_plot(self):

        # Вектора осей ССК (x, y, z)
        np.put(self.p_x, [0, 1, 2], self.v_x + self.pos)
        np.put(self.p_y, [0, 1, 2], self.v_y + self.pos)
        np.put(self.p_z, [0, 1, 2], self.v_z + self.pos)

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

        # Кватернион поворота
        q_rot = Quaternion(w, qx, qy, qz)

        # Поворот векторов объекта на кватерион поворота
        # ССК
        self.v_x = q_rot.rotate(self.v_x)
        self.v_y = q_rot.rotate(self.v_y)
        self.v_z = q_rot.rotate(self.v_z)

        print(self.v_x)
        print(self.v_y)
        print(self.v_z)
        print("\b")

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
