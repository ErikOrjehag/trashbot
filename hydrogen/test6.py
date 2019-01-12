import sympy as sp
sp.init_printing(use_latex='mathjax')

x, y, z = sp.symbols('x y z')
f = sp.sin(x * y) + sp.cos(y * z)
sp.integrate(f, x)
