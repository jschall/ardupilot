from sympy import *
from sympy.stats import Normal, density, cdf



weights = symbols('w0:2', real=True, nonnegative=True)
mu = symbols('mu0:2', real=True)
y = symbols('y0:2', real=True)
sigma = symbols('sigma0:2', real=True, nonnegative=True)


distr = Normal(y[0], mu[0], sigma[0])/4 + 3*Normal(y[1], mu[1], sigma[1])/4

pprint(density(distr))
