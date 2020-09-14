import sympy as sp

# Constants
A1, L1, L2, L3 = sp.symbols("A1 L1 L2 L3")
t = sp.symbols("t")

# Variables
beta, x, y, a, b = sp.symbols("beta x y a b", cls=sp.Function)

alpha = sp.acos(
    ((L2**2) + ((b(t) - a(t))**2) - (L1**2)) / (2 * L2 * (b(t) - a(t))))
beta = sp.acos(
    ((L1**2) + ((b(t) - a(t))**2) - (L2**2)) / (2 * L1 * (b(t) - a(t))))
x = b(t) - L3 * sp.cos(beta + A1)
y = L3 * sp.sin(beta + A1)

sp.print_latex(beta)
sp.print_latex(x)
sp.print_latex(y)

# sp.pprint(sp.diff(x, t))
# print(sp.diff(x, t))
# print("\n\n-----------------\n\n")
# sp.pprint(sp.simplify(sp.diff(y, t)))
# print(sp.simplify(sp.diff(y, t)))
dxdt = sp.simplify(sp.diff(x, t))
dydt = sp.simplify(sp.diff(y, t))

sp.print_latex(dxdt)
sp.print_latex(dydt)

# Manual substitution

j = -L3 * (L1**2 - L2**2 - a(t)**2 + 2 * a(t) * b(t) - b(t)**2)
k = A1 + sp.acos(
    (L1**2 - L2**2 + (a(t) - b(t))**2) / (2 * L1 * (-a(t) + b(t))))
l = 2 * L1 * sp.sqrt(1 - ((L1**2 - L2**2 + (a(t) - b(t))**2)**2 /
                          (4 * L1**2 * (a(t) - b(t))**2))) * (a(t) - b(t))**2

sp.pprint(j)
sp.pprint(k)
sp.pprint(l)

sp.print_latex(j)
sp.print_latex(k)
sp.print_latex(l)


f = j * sp.sin(k) / l
g = j * sp.cos(k) / l

sp.pprint(f)
sp.pprint(g)

sp.print_latex(f)
sp.print_latex(g)

sp.print_ccode(j)
sp.print_ccode(k)
sp.print_ccode(l)
sp.print_ccode(f)
sp.print_ccode(g)