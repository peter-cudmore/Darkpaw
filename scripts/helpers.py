import sympy as sp
import numpy as np
import cvxpy as cp


def angles_to_quadratics(
        equations,
        angles,
        control_angles
):

    x = {theta: sp.symbols(f's_{i}, c_{i}') for i, theta in enumerate(angles)}
    u = {theta: sp.symbols(f'v_{i}, w_{i}') for i, theta in enumerate(control_angles)}
    substitutions = []

    X = [q_i for q in x.values() for q_i in q] + [q_i for q in u.values() for q_i in q]

    for i, x_i in enumerate(x):
        (s_i, c_i) = x[x_i]
        substitutions += [(sp.sin(x_i), s_i), (sp.cos(x_i), c_i)]

    for i, u_i in enumerate(u):
        (s_i, c_i) = u[u_i]
        substitutions += [(sp.sin(u_i), s_i), (sp.cos(u_i), c_i)]

    out_equations = [
        sp.expand_trig(equation).subs(substitutions)
        for equation in equations
    ]

    out_equations += [q_i[0] ** 2 + q_i[1] ** 2 - 1 for q_i in x.values()]
    out_equations += [q_i[0] ** 2 + q_i[1] ** 2 - 1 for q_i in u.values()]

    return X, out_equations, substitutions, (x, u)


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


def quadratic_form(eqn, X, X0):
    subs = list(zip(X, X0))
    P = sp.Matrix([[eqn.diff(t).diff(s) for t in X] for s in X]).subs(subs)/2
    q = sp.Matrix([eqn.diff(t) for t in X]).T.subs(subs)
    r = eqn.subs(subs)

    XM = sp.Matrix(X)
    rem = (XM.T @ P @ XM)[0] + (q @ XM)[0] + r - eqn
    ev = [sp.re(e) for e in P.eigenvals()]

    if min(ev) < 0:
        P -= min(ev)*sp.eye(len(X))
        r += min(ev) * len(X)
        #P, r_added = make_positive(np.asarray(P))
        #r += r_added

    return np.asarray(P).astype(np.float), np.asarray(q).astype(np.float), float(r), rem


def remove_nonzero_factors(eqn):
    if eqn == 0:
        return eqn
    result = 1
    for factor in sp.factor(eqn).args:
        if factor.is_number:
            continue
        if isinstance(factor, sp.Pow) and factor.args[1] < 0:
            continue

        result *= factor

    return result


def make_positive(matrix):

    n, _ = matrix.shape
    X = cp.Variable(n)
    A = np.zeros(shape=(n//2, n), dtype=np.float)

    for i in range(n//2):
        A[i, 2*i] = 1
        A[i, 2 * i + 1] = -1

    constraints = [A @ X]
    constraints.append(cp.lambda_min(matrix + cp.diag(X)) >= 0)

    objective = cp.norm(X, 1)

    problem = cp.Problem(cp.Minimize(objective), constraints)

    problem.solve(verbose=True)

    out_matrix = matrix + np.diag(X.value)

    return out_matrix, - sum(X.value) / 2

def RotateXY(angle):
    c = sp.cos(angle)
    s = sp.sin(angle)
    return np.array([[c, -s, -s, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


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

def R(th):
    c = sp.cos(th)
    s = sp.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def S(s):
    return np.array([[s, 0, 0], [0, s, 0], [0, 0, 1]])

def T(x, y, th=0):
    c = sp.cos(th)
    s = sp.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])


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

    A = eq_temp.coeff(c, 1).coeff(s, 0)
    B = eq_temp.coeff(s, 1).coeff(c, 0)
    remainder = eq_temp.coeff(c, 0).coeff(s, 0)

    difference = sp.expand(A*c + B*s + remainder - eq_temp)

    if difference != 0 or not A or not B:
        return eqn
    A = _simplify_all(A)
    B = _simplify_all(B)
    remainder = _simplify_all(remainder)
    ratio = sp.simplify(B / A)

    magnitude = _simplify_all(sp.simplify(A**2 + B**2))
    phase_shift = sp.atan(ratio)
    if positive_branch:
        C = sp.simplify(sp.sqrt(magnitude))
    else:
        C = sp.simplify(sp.sign(A) * sp.sqrt(magnitude))
    # C = sp.simplify(sp.sqrt(magnitude))
    # A sin(theta) + B cos(theta)  + R = C cos(theta - delta) + R = 0
    # so cos(theta + delta) =  - R / C
    # theta - delta = arccos(-R/C)
    # theta = arctan(B/A) + arccos(-R/C)
    if implicit:
        return sp.simplify(C * sp.cos(atom - phase_shift) + remainder)
    else:
        return sp.atan(ratio) + sp.acos(sp.simplify(-remainder / C))


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
