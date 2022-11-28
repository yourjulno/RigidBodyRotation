import RungeKutta as RK
import numpy as np
import Draw

########
# в случае Эйлера первыми интегралами являются кинетический момент в ИСО K, кинетическая энергия Т
initial_cond = [0., 0, 0, 1, 1, 0, 0]  # начальные условия удовлетворяют случай эйлера
initial_time = 0
end_time = 40
step = 0.002

constant_torque = [0., 0., 0]  # моменты, действующие на тело
tensor = [2, 2, 1]  # динамически симметричное тело

c = RK.ConstatntTorque(np.array(constant_torque))
mom = [c]

time, state = RK.Runge_Kutta.integrate(initial_cond, initial_time, end_time, step, RK.RightPart(mom, tensor))
draw = Draw.DrawObject(state)
draw.Draw()
c = RK.ConstatntTorque(np.array(constant_torque))
mom = [c]

Res = RK.Res(initial_cond, initial_time, end_time, step, tensor, mom)
Kx, Ky, Kz = Res.GetKinTorqueFromResults()
T = Res.GetEnergyFromResults()
Time = Res.return_time_()
draw_plot = Draw.DrawPlot(T, Time)

draw_plot.ShowPlot()
