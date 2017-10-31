#import numpy as np
from sympy import *  #symbolic maths
#from math import sin, cos, pi
from data_dummy import dummy

#install sympy, first install mpmath: https://github.com/fredrik-johansson/mpmath
#then install sympy: https://github.com/sympy/sympy.git


################### Symbols #################### 

th1 = Symbol('th1', rational=True)
l1 = Symbol('l1', rational=True)

th2 = Symbol('th2', rational=True)
l2 = Symbol('l2', rational=True)

th3 = Symbol('th3', rational=True)
l3 = Symbol('l3', rational=True)

nx = Symbol('nx', rational=True)
ox = Symbol('ox', rational=True)
ax = Symbol('ax', rational=True)
px = Symbol('px', rational=True)

ny = Symbol('ny', rational=True)
oy = Symbol('oy', rational=True)
ay = Symbol('ay', rational=True)
py = Symbol('py', rational=True)

nz = Symbol('nz', rational=True)
oz = Symbol('oz', rational=True)
az = Symbol('az', rational=True)
pz = Symbol('pz', rational=True)

################################################ 

class DHMatrix:
    def __init__(self, theta, d, a, alpha):
        self.theta_ = theta
        self.d_ = d
        self.a_ = a
        self.alpha_ = alpha

        self.dhmatrix_ = Matrix ([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*cos(theta), a*cos(theta)], \
                                  [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)], \
                                  [0, sin(alpha), cos(alpha), d], \
                                  [0, 0, 0, 1]])

	def print_(self):
		print "A matrix with denavit params:"
		print " - theta = ", self.theta_, " d = ", self.d_, " a = ", self.a_, " alpha = ", self.alpha_
		print " - ", self.dhmatrix_
        print "#############################"

class DirectKinematics:
    def __init__(self, a1, a2, a3):
        self.T_ = simplify(a1.dhmatrix_ * a2.dhmatrix_ * a3.dhmatrix_)

        self.direct_k_x_ = simplify(self.T_[0,3])
        self.direct_k_y_ = simplify(self.T_[1,3])
        self.direct_k_z_ = simplify(self.T_[2,3])

    def setConstants(self, _l1, _l2, _l3):
        self.eq_direct_x_ = self.direct_k_x_.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)
        self.eq_direct_y_ = self.direct_k_y_.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)
        self.eq_direct_z_ = self.direct_k_z_.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)

    def calculate(self, _th1, _th2, _th3):
        print "Compute direct kinematiks for: th1 = ", _th1, ", th2 = ", _th2, ", th3 = ", _th2
        print " - X = ", N(self.eq_direct_x_.subs(th1, _th1).subs(th2, _th2).subs(th3, _th3))
        print " - X = ", N(self.eq_direct_y_.subs(th1, _th1).subs(th2, _th2).subs(th3, _th3))
        print " - X = ", N(self.eq_direct_z_.subs(th1, _th1).subs(th2, _th2).subs(th3, _th3))
        print "#############################"

    def printMatrix(self):
        print "Matrix direct kinematiks"
        print " - ", self.T_
        print "#############################"

    def print_(self):
        print "Direct kinematiks:"
        print " - x = ", self.direct_k_x_
        print " - y = ", self.direct_k_y_
        print " - z = ", self.direct_k_z_
        print "#############################"

class InverseKinematics:
    def __init__(self):
        self.inverse_k_th1_ = simplify(atan2(py,px))
        self.costh3 = (px**2 + py**2 + pz**2 - l2**2 - l3**2)/(2*l2*l3)
        self.inverse_k_th3_ = simplify((+sqrt(1-self.costh3**2))/(self.costh3)) #could be + or - depending on the joint
        self.inverse_k_th2_ = simplify(atan2(pz, (sqrt(px**2 + py**2))) - atan2(l3*sin(self.inverse_k_th3_),l2+l3*cos(self.inverse_k_th3_)))
    
    def setConstants(self, _l1, _l2, _l3):
        self.eq_inverse_th1_ = self.inverse_k_th1_.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)
        self.eq_inverse_th2_ = self.inverse_k_th2_.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)
        self.eq_inverse_th3_ = self.inverse_k_th3_.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)

    def calculate(self, _px, _py, _pz):
        print "Compute direct kinematiks for: px = ", _px, ", py = ", _py, ", pz = ", _pz
        print " - th1 = ", N(self.eq_inverse_th1_.subs(px, _px).subs(py, _py).subs(pz, _pz))
        print " - th2 = ", N(self.eq_inverse_th2_.subs(px, _px).subs(py, _py).subs(pz, _pz))
        print " - th3 = ", N(self.eq_inverse_th3_.subs(px, _px).subs(py, _py).subs(pz, _pz))
        print "#############################"

    def print_(self):
        print "Inverse kinematiks:"
        print " - th1 = ", self.inverse_k_th1_
        print " - th2 = ", self.inverse_k_th2_
        print " - th3 = ", self.inverse_k_th3_
        print "#############################"

class Jacobian:
    def __init__(self, _direct_k):
        self.J_ = dummy.J_ #simplify(Matrix ([[diff(_direct_k.direct_k_x_, th1), diff(_direct_k.direct_k_x_, th2), diff(_direct_k.direct_k_x_, th3)], \
        #                     [diff(_direct_k.direct_k_y_, th1), diff(_direct_k.direct_k_y_, th2), diff(_direct_k.direct_k_y_, th3)], \
        #                     [diff(_direct_k.direct_k_z_, th1), diff(_direct_k.direct_k_z_, th2), diff(_direct_k.direct_k_z_, th3)]]))
        self.J_inverse_ = dummy.J_inverse_#self.J_**-1

    def calculateSingularities(self):
        self.singularities = solve(simplify(self.J_.det()))
        print " Singularities in: ", self.singularities
        print "#############################"

    def printMatrix(self):
        print "Jacobian Matrix"
        print " - ", self.J_
        print "\n"
        print "Inverse Jacobian Matrix"
        print " - ", self.J_inverse_
        print "#############################"

    def setConstants(self, _l1, _l2, _l3, _J, _inverse_J = 0):
        self.eq_J_ = _J.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)
        self.eq_J_inverse_ = _inverse_J.subs(l1, _l1).subs(l2, _l2).subs(l3, _l3)

    def setArticularPos(self, _th1, _th2, _th3, _J, _inverse_J = 0):
        self.eq_J_ = _J.subs(th1, _th1).subs(th2, _th2).subs(th3, _th3)
        self.eq_J_inverse_ = _inverse_J.subs(th1, _th1).subs(th2, _th2).subs(th3, _th3)

    def calculateCartesianSpeed(self, speed_th1, speed_th2, speed_th3):
        print "Compute direct jacobian for speeds: th1 = ", speed_th1, ", th2 = ", speed_th2, ", th3 = ", speed_th2
        articular_speed = Matrix ([[speed_th1],[speed_th2],[speed_th3]])

        self.cartesian_speed_ = N(simplify(self.eq_J_ * articular_speed))
        print " - speed_x = ", self.cartesian_speed_[0]
        print " - speed_y = ", self.cartesian_speed_[1]
        print " - speed_z = ", self.cartesian_speed_[2]
        print "#############################"

    def calculateArticularSpeed(self, speed_x, speed_y, speed_z):
        print "Compute direct kinematiks for: speed_x = ", speed_x, ", speed_y = ", speed_y, ", speed_z = ", speed_z
        cartesian_speed = Matrix ([[speed_x],[speed_y],[speed_z]])

        self.articular_speed_ = N(simplify(self.eq_J_inverse_ * cartesian_speed))
        print " - speed_th1 = ", self.articular_speed_[0]
        print " - speed_th2 = ", self.articular_speed_[1]
        print " - speed_th3 = ", self.articular_speed_[2]
        print "#############################"

#################### A matrix #################### 

A1 = DHMatrix(th1, l1, 0, 90)
A2 = DHMatrix(th2, 0, l2, 0)
A3 = DHMatrix(th3, 0, l3, 0)

##################################################

T = DirectKinematics(A1, A2, A3)
T.print_()
#T.setConstants(40, 40, 40)
#T.calculate(30*pi/180,30*pi/180,30*pi/180)
#T.calculate(0*pi/180,0*pi/180,0*pi/180)
print "###################################################"
print "###################################################"

##################################################

T2 = InverseKinematics()
T2.print_()
#T2.setConstants(40, 40, 40)
#T2.calculate(59.56,6.11,88.84)
print "###################################################"
print "###################################################"

##################################################

J = Jacobian(T)
J.printMatrix()
J.setConstants(40, 40, 40, J.J_, J.J_inverse_)
J.setArticularPos(30,30,30, J.eq_J_, J.eq_J_inverse_)
J.calculateCartesianSpeed(10*pi/180,10*pi/180,10*pi/180)
J.calculateArticularSpeed(5,5,1)
J.calculateSingularities()
print "###################################################"
print "###################################################"

##################################################

print "End of script"



