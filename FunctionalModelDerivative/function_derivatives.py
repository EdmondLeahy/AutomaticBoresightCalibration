import numpy
import sympy as sy



x = sy.symbols('x')

f = x**2

dx = sy.Derivative(f).doit()

print(f'\n{f}\n{dx}')
