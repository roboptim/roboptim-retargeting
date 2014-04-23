# -*- coding: utf-8 -*-
from sympy import init_printing
init_printing(use_unicode=True)

from sympy import *
from sympy.matrices import *

#from sympy.abc import gamma, beta, alpha

r00, r01, r02 = symbols('r00 r01 r02', is_real=True)
r10, r11, r12 = symbols('r10 r11 r12', is_real=True)
r20, r21, r22 = symbols('r20 r21 r22', is_real=True)

alpha = atan2(r10, r00)
beta = atan2(-r20, sqrt(r21 * r21 + r22 * r22))
gamma = atan2(r21, r22)

Jalpha = Matrix(
    [[diff(alpha, r00), diff(alpha, r01), diff(alpha, r02)],
     [diff(alpha, r10), diff(alpha, r11), diff(alpha, r12)],
     [diff(alpha, r20), diff(alpha, r21), diff(alpha, r22)],
     ])

Jbeta = Matrix(
    [[diff(beta, r00), diff(beta, r01), diff(beta, r02)],
     [diff(beta, r10), diff(beta, r11), diff(beta, r12)],
     [diff(beta, r20), diff(beta, r21), diff(beta, r22)],
     ])

Jgamma = Matrix(
    [[diff(gamma, r00), diff(gamma, r01), diff(gamma, r02)],
     [diff(gamma, r10), diff(gamma, r11), diff(gamma, r12)],
     [diff(gamma, r20), diff(gamma, r21), diff(gamma, r22)],
     ])

#pprint(diff(alpha, r00))
#pprint(diff(alpha, r01))
#pprint(diff(alpha, r02))

#pprint(diff(alpha, r10))
#pprint(diff(alpha, r11))
#pprint(diff(alpha, r12))

#pprint(diff(alpha, r20))
#pprint(diff(alpha, r21))
#pprint(diff(alpha, r22))

#alpha.jacobian(Matrix([r00, r01, r02, r11, r12, r20, r21, r22]))

pprint(Jalpha)
pprint(Jbeta)
pprint(Jgamma)
