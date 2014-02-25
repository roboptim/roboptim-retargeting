# -*- coding: utf-8 -*-
from sympy import init_printing
init_printing(use_unicode=True)

from sympy import *
from sympy.matrices import *

#from sympy.abc import alpha, beta, gamma

alpha,beta, gamma = symbols('a b c', real=True)

def skew(v): return Matrix( [ [0 ,-v[2 ,0 ],v[1 ,0 ]] , [v[2 ,0 ],0 ,-v[0 ,0 ]] , [-v[1 ,0 ],v[0 ,0 ],0 ] ] )

v = Matrix([alpha, beta, gamma])
theta = v.norm()
k = v / v.norm()
hatk = skew(k)

R = eye(3) + hatk * sin(theta) + (1 - cos(theta)) * hatk * hatk

#pprint(v)
#pprint(theta)
#pprint(k)
#pprint(hatk)
#pprint(R)

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
