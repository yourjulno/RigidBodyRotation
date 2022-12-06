import RungeKutta as RK
import numpy as np
import Draw


########
# в случае Эйлера первыми интегралами являются кинетический момент в ИСО K, кинетическая энергия Т
initial_cond = [0., 0, 0, 1, 2, 2, 2]  # начальные условия удовлетворяют случай эйлера
initial_time = 0
end_time = 40
step = 0.002
initial = np.array([0., 0, 0, 1, 0, 0, 0])
pid = RK.PID(initial)
c = [pid]
# constant_torque = [0., 0., 0]  # моменты, действующие на тело
tensor = [2, 2, 1]  # динамически симметричное тело
#
# c = RK.PID(np.array(constant_torque))
# mom = [c]

time, state = RK.Runge_Kutta.integrate(initial_cond, initial_time, end_time, step, RK.RightPart(c, tensor))
draw = Draw.DrawObject(state)
draw.Draw()
# c = RK.ConstatntTorque(np.array(constant_torque))
# mom = [c]

Res = RK.Res(initial_cond, initial_time, end_time, step, tensor, c)
res = Res.return_results_of_state()
# Kx, Ky, Kz = Res.GetKinTorqueFromResults()
W = Res.GetWFromResults(res)
Time = Res.return_time_()
X_comp = Res.GetXQuatComponent(res)
Draw.DrawPlot(W, Time)
Draw.DrawPlot(X_comp, time)
Draw.DrawPlot.ShowPlot()
