from scipy import *
from pylab import *
from sympy import *
from numpy import *

"""
Takes two functions containing variables x, y and computes Newton's method.
"""

# allows variables x and y to be inputs
x, y = symbols('x, y', real=True)


class fractal2D:
    def __init__(self, f1, f2):
        self.f1 = f1
        self.f2 = f2

    # prints the two functions in the form of a Matrix
    def __repr__(self):
        return 'Matrix([[{}], [{}]])'.format(self.f1, self.f2)

    # implements Newton's method
    def newton(self, initial):
        """
        initial should be in the form [x0, y0]
        """
        # creates a matrix that can be turned into a Jacobian matrix
        F = Matrix([self.f1, self.f2])
        # creates a Jacobian matrix with the two given functions
        J = F.jacobian([x, y])
        # turns initial values into an array
        v = array(initial)
        for i in range(400):
            # used for comparison
            vold = v
            # computes Jacobian matrix at a given point
            compJac = J.subs([(x, v[0]), (y, v[1])])
            invert = compJac**-1
            # implements Newton's method
            v = v - invert.dot(F.subs([(x, v[0]), (y, v[1])]))
            # tolerance check
            tol = 1.e-8
            if abs(v[0]-vold[0]) < tol:
                if abs(v[1]-vold[1]) < tol:
                    conv = print('Convergence was observed after {} iterations'.format(i))
                    break
        else:
            raise Exception('No convergence (yet)')
        return conv

    # implements simplified Newton's method
    def simplifiednewton(self, initial):
        """
        initial should be in the form [x0, y0]
        """
        # creates a matrix that can be turned into a Jacobian matrix
        F = Matrix([self.f1, self.f2])
        # creates a Jacobian matrix with the two given functions
        J = F.jacobian([x, y])
        # computes Jacobian matrix at a given point
        compJac = J.subs([(x, v[0]), (y, v[1])])
        invert = compJac**-1
        # turns initial values into an array
        v = array(initial)
        for i in range(400):
            # used for comparison
            vold = v
            # implements Newton's method
            v = v - invert.dot(F.subs([(x, v[0]), (y, v[1])]))
            # tolerance check
            tol = 1.e-8
            if abs(v[0]-vold[0]) < tol:
                if abs(v[1]-vold[1]) < tol:
                    conv = print('Convergence was observed after {} iterations'.format(i))
                    break
        else:
            raise Exception('No convergence (yet)')
        return conv

### TESTS ###
# F = fractal2D(x**3-3*x*y**2-1, 3*x**2*y-y**3)
# F.newton([-0.6, 0.6])
# F.newton([1, 0])
