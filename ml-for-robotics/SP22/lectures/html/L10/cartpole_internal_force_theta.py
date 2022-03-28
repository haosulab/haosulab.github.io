from sympy import init_session
from sympy import *
from sympy.physics.vector import dynamicsymbols
from sympy import diff, Symbol

init_session()

M, m, t, g, l, lmd, f = symbols('M m t g l lmd f')
q = Matrix([dynamicsymbols('x1 x2 th')]).T

dqdt = diff(q, t)
dx1dt = dqdt[0]
dx2dt = dqdt[1]
dthdt = dqdt[2]
y2 = l * cos(q[2])
dy2dt = diff(y2, t)

T = 0.5 * M * dx1dt * dx1dt + 0.5 * m * (dx2dt * dx2dt + dy2dt * dy2dt)
V = m * g * y2
C = (q[0]-q[1])**2+q[2]**2-l**2
L = T - V + C*lmd

Lagrangian = diff(diff(L, dqdt), t)- diff(L, q)

F = Matrix([f, 0, 0])
stationarity = Lagrangian - F

A = diff(C, q).T
feasibility = diff(A, t)* dqdt + A * diff(dqdt, t)

ddqdt = diff(dqdt, t)
sol = simplify(solve((stationarity[0], stationarity[1], stationarity[2], feasibility), lmd, ddqdt[0], ddqdt[1], ddqdt[2]))
