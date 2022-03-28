from sympy import init_session
from sympy import *
from sympy.physics.vector import dynamicsymbols
from sympy import diff, Symbol
import sympy
import numpy as np
import scipy.linalg as sl

init_session()

##### system dynamics
dof = 2

p, th = dynamicsymbols('p, th')
M, m, l, g = symbols('M, m, l, g')
f, tau = symbols('f, tau')
q = Matrix([p, th])

dqdt = diff(q, t)
dpdt= dqdt[0]
dthdt = dqdt[1]

Mass = Matrix([[M+m, -m*l*cos(th)], [-cos(th), l]])
N = Matrix([m*l*sin(th)*dthdt*dthdt, -g*sin(th)])
F = Matrix([f, tau])
a = diff(dqdt, t)

ID = Mass*a+N
FD = Inverse(Mass)*(F-N)

##### target
qd = Matrix([0, 0])

##### dynamics in control form
qbar = q-qd
dqbardt = diff(qbar, t)
x = Matrix.vstack(qbar, dqbardt)
u = F

A = zeros(2*dof, 2*dof)
A[0:dof, dof:(2*dof)] = eye(dof,dof)
A[dof:(2*dof), 0:dof] = diff(FD, q).reshape(dof,dof)
A[dof:(2*dof), dof:(2*dof)] = diff(FD, dqdt).reshape(dof,dof)

B = zeros(2*dof, 1)
B[dof:(2*dof), :] = diff(FD, f).reshape(dof,1)
# B = zeros(2*dof, dof)
# B[dof:(2*dof), :] = diff(FD, F).reshape(dof,dof)
B = simplify(B)

##### assign values
Ataylor = simplify(A.subs(f, 0).subs(tau, 0).subs(p, qd[0]).subs(th, qd[1]))

# Aeval = simplify(Ataylor.subs(M, 10).subs(m, 1).subs(l, 1).subs(g, 9.81))
# Beval = simplify(B.subs(M, 10).subs(m, 1).subs(l, 1).subs(th, 0))
#
# ##### controller params
# Qmat = eye(2*dof, 2*dof)
# Rmat = eye(1, 1)
#
# Anp = np.array(Aeval.tolist()).astype(np.float64)
# Bnp = np.array(Beval.tolist()).astype(np.float64)
# Qmatnp = np.array(Qmat.tolist()).astype(np.float64)
# Rmatnp = np.array(Rmat.tolist()).astype(np.float64)
#
# S = sl.solve_continuous_are(Anp, Bnp, Qmatnp, Rmatnp)
#
# K = np.linalg.inv(Rmatnp)@Bnp.T@S
