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

dpdt = diff(p,t)
dthdt = diff(th,t)

E = 0.5*(M+m)*(dpdt*dpdt)-m*l*dpdt*dthdt*cos(th)-0.5*m*l*l*dthdt*dthdt+m*g*l*cos(th)
Ed = m*g*l
Ediff=E-Ed
print(latex(simplify(diff(Ediff,t))))

