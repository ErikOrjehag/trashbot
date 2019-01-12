from casadi import *

x = SX.sym("x")
y = SX.sym("y")
z = SX.sym("z")

f = x**2 + 100*z**2
g = z + (1 - x)**2 - y

P = dict(f=f, g=g, x=vertcat(x, y, z))

opts = {
    "ipopt.print_level": 0,
    "ipopt.sb": "yes",
    "print_time": 0
    }

F = nlpsol("F", "ipopt", P, opts)

r = F(x0=[2.5, 3.0, 0.75], ubg=0, lbg=0)

print(r["x"])
