import sympy as sp
import numpy as np


def angles_to_complex(
        equations,
        angles,
        control_angles
):

    x = {theta: sp.symbols(f'z_{i}') for i, theta in enumerate(angles)}
    u = {theta: sp.symbols(f'w_{i}') for i, theta in enumerate(control_angles)}
    substitutions = []

    X = list(x.values()) + list(u.values())

    for i, x_i in enumerate(x):
        z = x[x_i]
        substitutions += [(sp.sin(x_i), (z - 1/z)/2j), (sp.cos(x_i), (z+1/z)/2)]

    for i, u_i in enumerate(u):
        z = u[u_i]
        substitutions += [(sp.sin(u_i), (z - 1/z)/2j), (sp.cos(u_i), (z+1/z)/2)]

    out_equations = [
        sp.expand_trig(equation).subs(substitutions)
        for equation in equations
    ]

    return X, out_equations, substitutions, (x, u)


def remove_nonzero_factors(eqn,
                           reduce_monomials=False,
                           atom_bounds=None
                           ):
    if eqn == 0:
        return eqn
    result = 1
    domains = {}
    for atom in [a for a in eqn.atoms() if a.is_symbol]:
        if atom_bounds and atom in atom_bounds:
            domains[atom] = sp.Interval(*atom_bounds[atom])
        else:
            domains[atom] = sp.Reals

    for factor in sp.factor(sp.cancel(eqn)).args:
        if isinstance(factor, sp.Pow) and factor.args[1] < 0:
            continue
        if reduce_monomials:
            atoms = [a for a in factor.atoms() if a.is_symbol]
            if len(atoms) == 1:
                a, = atoms
                solve_set = sp.solveset(factor, domain=domains[a])
                if not solve_set:
                    continue

        result *= factor

    return result


def RotateXY(angle):
    c = sp.cos(angle)
    s = sp.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def RotateXZ(angle):
    c = sp.cos(angle)
    s = sp.sin(angle)
    return np.array([[c,  0, -s, 0], [0,  1, 0, 0], [s,  0, c, 0], [0,  0, 0, 1]])


def RotateYZ(angle):
    c = sp.cos(angle)
    s = sp.sin(angle)
    return np.array([[1,  0,  0, 0], [0,  c, -s, 0], [0,  s, c, 0], [0,  0, 0, 1]])


def ReflectY():
    return np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def ReflectX():
    return np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def Translate(x, y, z):
    out = np.eye(4, dtype=np.float)
    for i, v in enumerate((x, y, z)):
        out[i, 3] = v
    return out


def to_line(point, origin):
    return [float(origin[0]), float(point[0])], [float(origin[1]), float(point[1])]


def _simplify_all(eqn):
    out_eqn = eqn
    atoms = out_eqn.atoms()
    for atm in atoms:
        if atm.is_symbol:
            out_eqn = tan_simplify(out_eqn, atm)
    return out_eqn


def simplify_array(arr, subs=None):
    n, m = arr.shape
    for i in range(0, n):
        for j in range(0, m):
            arr[i, j] = sp.simplify(arr[i, j])
            if subs:
                arr[i, j] = sp.simplify(arr[i, j].subs(subs))
    return arr


def tan_simplify(eqn, atom, implicit=True, positive_branch=False):

    s = sp.Dummy('s')
    c = sp.Dummy('c')

    eq_temp = eqn.subs([(sp.sin(atom), s), (sp.cos(atom), c)]).collect(c).collect(s)
    # A sin + B cos + C = 0
    # R = sign(A)/(A^2 +B^2)^{1/2}
    # A / R sin(x) + B/R cos(x)  + C/R = 0
    # sin(x + phi) + C / R= 0
    # B / R = sin(phi)
    # A / R  = cos(phi)
    # tan = sin/cos = (B/A)
    # phi = arctan(B / A)
    # sin(x + phi)  + C / R = 0
    A = eq_temp.coeff(s, 1).coeff(c, 0)
    B = eq_temp.coeff(c, 1).coeff(s, 0)
    remainder = eq_temp.coeff(c, 0).coeff(s, 0)

    difference = sp.expand(A*s + B*c + remainder - eq_temp)

    if difference != 0 or not A or not B:
        return eqn
    #A = _simplify_all(A)
    #B = _simplify_all(B)
    magnitude = sp.simplify(A ** 2 + B ** 2)
    R = sp.sign(A) * sp.sqrt(magnitude)
    C = remainder / R

    ratio = sp.atan2(B / R, A / R)

    phi = sp.atan(ratio)

    if implicit:
        return sp.simplify(R * sp.sin(atom + phi) + remainder)
    else:
        return -ratio - sp.asin(sp.simplify(C))


def Maclaurin_series(expr, atom, order=4):
    return Taylor_series(expr, atom, 0, order=order)


def Taylor_series(expr, x, x0, order=4):
    f = expr
    output = 0
    output += sp.re(f.subs(x, x0))

    for i in range(1, order + 1):
        f = f.diff(x)
        output += sp.re((f.subs(x, x0 / sp.factorial(i)))) * (x - x0)**i

    return output


def hyperbolic_substitution(expr, angle, x):
    subs = [(sp.sin(angle), 2 * x / (1 + x**2)),  (sp.cos(angle), (1 - x**2)/(1 + x**2)) ]
    # sin/cos = tan = 2 x / (1 - x*2)
    # x_i = tan (angle / 2)
    return expr.subs(subs)


def real_to_integer_polynomial(poly):
    numbers = [n for n in sp.expand(poly).atoms() if n.is_number]

    factor = 1

    for n in numbers:
        remainder = abs(factor * n) - int(abs(factor * n))
        if remainder > 1e-3:
            factor *= int(1 / remainder)

    result = sum(
        basis * int(coeff * factor)
        for basis, coeff in poly.as_coefficients_dict().items()
    )

    return result


def truncate(eqn, order=4):
    out_eqn = 0

    coeff_dict = eqn.as_coefficients_dict()
    cutoff = max(coeff_dict.values()) / np.power(10, order)

    for basis, term in coeff_dict.items():
        if term > cutoff:
            out_eqn += basis * term

    return out_eqn


def diff_on_S1(eqn, ignore=None):

    symbols = {
        a: sp.symbols(f'w_{{{a}}}')
        for a in eqn.atoms() if a.is_symbol and (not ignore or a not in ignore)
    }

    out_eqns = {da:  sp.simplify(1j * a * eqn.diff(a)) for a, da in symbols.items()}

    return out_eqns


def jacobian(eqns):

    basis_vectors = []
    values = {}
    for row, eqn in enumerate(eqns):
        derivatives = diff_on_S1(eqn)
        for dx, dfdx in derivatives.items():
            try:
                col = basis_vectors.index(dx)
            except ValueError:
                col = len(basis_vectors)
                basis_vectors.append(dx)
            values[(row, col)] = dfdx

    matrix = sp.Matrix(
        [[0 if (row, col) not in values else values[(row, col)] for col in range(len(basis_vectors))]
         for row in range(len(eqns))
        ]
    )
    return matrix, sp.Matrix([[b] for b in basis_vectors])


def filter_tiny_numbers(eqn, eps=1e-4):
    atoms = eqn.atoms()
    subs = []
    for a in atoms:
        try:
            if abs(a) < eps:
                subs.append((a, 0))
        except TypeError:
            continue
    return eqn.subs(subs)


def eqn_to_mul_add(eqn):

    atoms = {s: 0 for s in eqn.atoms() if s.is_symbol}
    if not atoms:
        return eqn

    coeff, factors = sp.factor_list(eqn)

    if len(factors) > 1:
        for factor, power in factors:
            coeff *= eqn_to_mul_add(factor)
        return coeff
    elif not factors:
        return coeff

    this_factor, power = factors[0]
    _, terms = this_factor.as_coeff_add()

    for term in terms:
        for atom in term.atoms():
            if atom in atoms:
                atoms[atom] += 1

    root, _ = max(atoms.items(), key=lambda x: x[1])

    b = this_factor.coeff(root, 0)
    a = 0

    for i in range(10):
        a += this_factor.coeff(root, i + 1) * (root ** i)

    a = eqn_to_mul_add(a)
    b = eqn_to_mul_add(b)

    result = coeff
    for i in range(power):
        result *= (a*root + b)
    return result