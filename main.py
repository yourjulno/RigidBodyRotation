# import numpy as np
# from typing import Callable, List, Tuple
# from quat import Quaternion
# from Object import Object
# from Runge_Kutta import Runge_Kutta
#
# obj = Object([1, 1, 1], [1, 1, 1], 1, 1, 1)
#
# quaternion = Quaternion(1, [1, 1, 1])
#
# obj.dq_dt(obj, quaternion)
# obj.dw_dt(obj)
#
# state = obj.state_vector(obj, quaternion)
# print(state)
# RK = Runge_Kutta
#
# step_of_integrate = RK.make_step(5, 1, state, RK.right_function(obj, quaternion))
# print(RK.integrate(step_of_integrate, [1, 1, 1], 1, 3, 0.5, np.array))

