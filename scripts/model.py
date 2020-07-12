import numpy as np
import sympy as sp
from scipy.optimize import root

l_a = 40
l_b = 36
l_c = 32
l_d = 26

Y_h = 39
Y_o = -7.5
Y_i = 7.5


motor_arm_top = 15
motor_arm_bottom = -12.5
motor_offset_top = 6.5
motor_offset_bottom = -6.5
motor_offset_x = 32
D_section_b = -26
D_section_e = -32
leg_mid_to_foot = 78
leg_top_to_foot = 112.5
leg_mid_to_top = 39.5
leg_pivot_to_foot_x = -15
leg_pivot_to_foot_y = -75
body_length = 84
arm_anchor_to_pivot = 66
body_width = 84
body_linkage_offset_radial = 36
body_linkage_offset_inner = 16
motor_arm_body = 12.5

def R(th):
    c = sp.cos(th)
    s = sp.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def S(s):
    return np.array([[s, 0, 0], [0, s, 0], [0, 0, 1]])


angles = {}


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
                arr[i ,j] = sp.simplify(arr[i, j].subs(subs))
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


beta_D, beta_Y, beta_A = sp.symbols(r'\beta_D, \beta_Y, \beta_A', real=True)
u_r, u_h, u_b = sp.symbols('u_r, u_h, u_b', real=True)
Leg_Pivot = R(beta_D) @ T(D_section_e, 0) @ R(-beta_D)
Leg_Bottom = Leg_Pivot @ R(beta_Y) @ T(leg_pivot_to_foot_x, leg_pivot_to_foot_y)
# For R(beta_Y)
Y_inner = R(beta_Y) @ T(Y_i, Y_h) @ R(-beta_Y)
Y_outer = R(beta_Y) @ T(Y_o, Y_h) @ R(-beta_Y)
D_bottom = R(beta_D) @ T(0, D_section_b) @ R(-beta_D)
Top_Motor = T(motor_offset_x, motor_offset_top)
Top_Motor_Anchor = Top_Motor @ R(u_r) @ T(0, motor_arm_top) @ R(-u_r)
Bottom_Motor = T(motor_offset_x, motor_offset_bottom)
Bottom_Motor_Anchor = Bottom_Motor @ R(u_h) @ T(0, motor_arm_bottom) @ R(-u_h)
# Front left, form top down POV is xy
Body_Anchor = T(body_width / 2, -body_length/2) @ R(beta_A) @ T(-body_linkage_offset_inner, body_linkage_offset_radial) @ R(-beta_A)
Body_Motor_Arm = T(0, -body_length / 2) @ R(u_b) @ T(0, -motor_arm_body) @ R(-u_h)

C_a = simplify_array((Body_Anchor - Body_Motor_Arm).T @ (Body_Anchor - Body_Motor_Arm))
f_a = sp.simplify(sp.expand(C_a[2, 2] - l_a**2))
f_a_lambda = sp.lambdify((beta_A, u_b), f_a)


def beta_a(body_motor, last_value=0):
    return root(
        lambda x: [abs(f_a_lambda(x[0], x[1])), x[1]],
        x0=np.array([last_value, body_motor])
    ).x[0]


C_b = simplify_array((Bottom_Motor_Anchor - D_bottom).T @ (Bottom_Motor_Anchor - D_bottom))
f_b = sp.simplify(sp.expand(C_b[2, 2] - l_b**2))
f_b_lambda = sp.lambdify((beta_D, u_h), f_b)


def beta_d(bottom_motor, last_value=0):
    return root(
        lambda x: [abs(f_b_lambda(x[0], x[1])), x[1]],
        x0=np.array([last_value, bottom_motor])
    ).x[0]


C_t = simplify_array((Top_Motor_Anchor - Y_inner).T @ (Top_Motor_Anchor - Y_inner))
f_t = sp.simplify(sp.expand(C_t[2, 2] - l_d ** 2))
f_t_lambda = sp.lambdify((beta_Y, u_r), f_t)


def beta_y(top_motor, last_value=0):
    return root(
        lambda x: [abs(f_t_lambda(x[0], x[1])), x[1]],
        x0=np.array([last_value, top_motor])
    ).x[0]


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


def get_leg_position(front=True, left=True):
    anchor = Translate(body_width / 2, body_width / 2, 0)

    if not left:
        anchor = ReflectX() @ anchor
    if not front:
        anchor = ReflectY() @ anchor
    Leg_Bottom = (
         anchor @ RotateXY(beta_A) @ Translate(arm_anchor_to_pivot, 0, 0)
         @ RotateXZ(beta_D) @ Translate(-D_section_e, 0, 0) @ RotateXZ(-beta_D) @ RotateXZ(beta_Y)
         @ Translate(-leg_pivot_to_foot_x, 0, leg_pivot_to_foot_y)
    ) @ np.array([0, 0, 0, 1])

    return Leg_Bottom[0:3], (f_a, f_b, f_t)


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


def compute_normal_form(equation, substitution, order):

    x, y = substitution
    h = sp.Dummy('h')

