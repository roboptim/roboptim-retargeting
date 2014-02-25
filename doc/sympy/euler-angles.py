# -*- coding: utf-8 -*-
from sympy import init_printing
init_printing(use_unicode=True)

from sympy import *
from sympy.matrices import *

#from sympy.abc import gamma, beta, alpha

gamma, beta, alpha = symbols('r p y')


Rz = Matrix([
        [cos(alpha), -sin(alpha), 0],
        [sin(alpha), cos(alpha), 0],
        [0, 0, 1]])

Ry = Matrix([
        [cos(beta), 0, sin(beta)],
        [0, 1, 0],
        [-sin(beta), 0, cos(beta)]])

Rx = Matrix([
        [1, 0, 0],
        [0, cos(gamma), -sin(gamma)],
        [0, sin(gamma), cos(gamma)]])

R = Rz * Ry * Rx


pprint("Rz:")
pprint(Rz)

pprint("Ry:")
pprint(Ry)

pprint("Rx:")
pprint(Rx)

R0 = R[:,0]
R1 = R[:,1]
R2 = R[:,2]

dR0 = R0.jacobian(Matrix([gamma, beta, alpha]))
dR1 = R1.jacobian(Matrix([gamma, beta, alpha]))
dR2 = R2.jacobian(Matrix([gamma, beta, alpha]))

pprint("dR0")
pprint(dR0)
pprint("dR1")
pprint(dR1)
pprint("dR2")
pprint(dR2)

J_global = \
    R2 * R1.transpose() * dR0 + \
    R1 * R0.transpose() * dR2 + \
    R0 * R2.transpose() * dR1

pprint("EVALUATION")
gamma_val = 1.1
beta_val = 1.2
alpha_val = 1.3
values = {alpha: alpha_val, beta: beta_val, gamma: gamma_val}

pprint("dR1")
pprint(dR0.evalf(subs=values))
pprint("dR2")
pprint(dR1.evalf(subs=values))
pprint("dR3")
pprint(dR2.evalf(subs=values))

pprint("J_global")
pprint(J_global.evalf(subs=values))
