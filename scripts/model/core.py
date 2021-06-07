
import numpy as np
import sympy as sp
import scipy.optimize as spo
from model.helpers import *
from model.codegen import LookupTable, Function
from functools import lru_cache
from typing import Optional
from collections import namedtuple

TableEntry = namedtuple('TableEntry', ['pwm', 'low', 'high', 'value', 'gradient'])

l_a = 40
l_b = 36
l_c = 32
l_d = 26

Y_h = 39
Y_o = -12.5
Y_i = 12.5

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

origin = np.array([0, 0, 0, 1])

pwm_factor = np.pi / 300
pwm_zero = 300


body_angle_bounds = (-np.pi/2, np.pi / 2)
radial_angle_bounds = (-np.pi/2, np.pi/2)
height_angle_bounds = (-np.pi/2, np.pi/2)


def get_active_kinematic_chain(body_angle, rad_ext_angle, height_angle):

    w_b = body_angle
    w_d = height_angle
    w_y = rad_ext_angle

    knee = RotateXY(sp.pi / 2) @ RotateXY(w_b) @ Translate(arm_anchor_to_pivot, 0, 0)
    ankle = knee @ RotateXZ(w_d) @  Translate(-D_section_e, 0, 0) @ RotateXZ(-w_d)
    foot = ankle @ RotateXZ(w_y) @ Translate(-leg_pivot_to_foot_x, 0, leg_pivot_to_foot_y)
    chain = [knee, ankle, foot]

    return chain


def get_analytic_ik(x, y, z):
    w_r, w_h = sp.symbols('w_r, w_h')

    # step one:
    # work out the body angle in the xy plane
    # ie (r - hip)
    xp, zp = sp.symbols('xp, zp')
    anchor = Translate(body_width / 2, body_width / 2, 0)
    hip = anchor.dot(origin)

    w_bf = sp.atan2(y - hip[1], x - hip[0]) - np.pi / 2
    r_planar = sp.simplify(RotateXY(-w_bf - np.pi/2).dot(np.array([x, y, z, 1]) - hip))

    foot_transform = get_active_kinematic_chain(-sp.pi/2, w_r, w_h)[-1]
    r = foot_transform.dot(origin)
    r = [sp.trigsimp(eqn) for eqn in r]

#    assert r[0].coeff(sp.cos(w_h)) == r[2].coeff(sp.sin(w_h)), r[0]
    eqn_s = (r[0] - r[0].coeff(sp.cos(w_h))*sp.cos(w_h) - xp)**2 + (r[2] - zp - r[2].coeff(sp.sin(w_h)) * sp.sin(w_h))**2 - r[2].coeff(sp.sin(w_h))**2
    eqn_s = sp.trigsimp(sp.expand(eqn_s))
    eqn_s = sp.collect(sp.collect(eqn_s, sp.sin(w_r)), sp.cos(w_r))

    w_rf = eqn_s.subs([(xp, r_planar[0]), (zp, r_planar[2])])

    w_rf = sp.simplify(tan_simplify(w_rf, w_r, implicit=False))
    # solve s
    c_wh = r[2].coeff(sp.sin(w_h))
    w_hf = sp.simplify(sp.asin((r_planar[2] / c_wh - r[2] / c_wh + sp.sin(w_h))))
    w_hf = w_hf.subs(w_r, w_rf)

    return w_bf, w_rf, w_hf

# create evaluation functions for forward and inverse kinematics.


def generate_kinematic_solvers():

    angles = sp.symbols('w_b, w_r, w_h')
    position = sp.symbols('x, y, z')
    anchor = Translate(body_width / 2, body_width / 2, 0)
    foot = (anchor @ get_active_kinematic_chain(*angles)[-1] @ origin)[0:3]
    J = sp.Matrix([[f.diff(x_j) for x_j in angles] for f in foot])
    ik_angles = get_analytic_ik(*position)

    fk = lambda x: np.array(sp.lambdify(angles, foot, 'numpy')(*x))
    jac = lambda x: np.array(sp.lambdify(angles, J, 'numpy')(*x))
    ik = lambda x: np.array(sp.lambdify(position, ik_angles, 'numpy')(*x))

    return fk, jac, ik


def get_body_linkage_constraint(rotation_from_frame, motor):
    # Consider point at anchor joint
    # +x is forwards,
    # +y is left
    body_motor_linkage_leg = (
            RotateXY(sp.pi/2)
            @ RotateXY(rotation_from_frame)
            @ Translate(body_linkage_offset_radial, body_linkage_offset_inner, 0)
    )
    body_motor_linkage_arm = (
        Translate(-body_length/2, 0, 0)
        @ RotateXY(motor)
        @ Translate(0, motor_arm_body, 0)
    )

    difference = (body_motor_linkage_arm - body_motor_linkage_leg) @ origin
    return difference.T @ difference - l_a ** 2


def get_leg_top_constraint(frame_angle_from_z, motor):
    # for the left leg, assume
    # +x is along the leg
    # +y is towards the back
    # +z is up
    # frame origin is at the leg out pivot point

    motor_linkage = (
        Translate(-motor_offset_x, 0, motor_offset_top)
        @ RotateXZ(motor) @ Translate(0, 0, motor_arm_top)
    )

    ybar_linkage = (
        RotateXZ(frame_angle_from_z)
        @ Translate(-Y_i, 0, Y_h)
    )
    difference = (motor_linkage - ybar_linkage) @ origin

    return difference.T @ difference - l_d ** 2


def get_leg_bottom_constraint(frame_angle, motor):
    # for the left leg, assume
    # +x is along the leg
    # +y is towards the back
    # +z is up
    # frame origin is at the leg out pivot point
    motor_linkage = (
        Translate(-motor_offset_x, 0, motor_offset_bottom)
        @ RotateXZ(motor)
        @ Translate(0, 0, motor_arm_bottom)
    )
    dbar_linkage = (
        RotateXZ(frame_angle)
        @ Translate(0, 0, D_section_b)
    )

    difference = (motor_linkage - dbar_linkage) @ origin

    return difference.T @ difference - l_b ** 2


def get_leg_position(front=True, left=True):
    anchor = Translate(body_width / 2, body_width / 2, 0)

    if not left:
        anchor = ReflectX() @ anchor
    if not front:
        anchor = ReflectY() @ anchor
    atoms = sp.symbols(r'w_b, w_r, w_h, u_b, u_r, u_h')
    rot_from_pivot, rot_y_from_z, rot_d_from_z, u_a, u_r, u_h = atoms
    chain = get_active_kinematic_chain(rot_from_pivot, rot_y_from_z, rot_d_from_z)

    Leg_Bottom = anchor @ chain[-1]  @ origin

    foot_pos = []
    f_a = get_body_linkage_constraint(rot_from_pivot, u_a)
    f_t = get_leg_top_constraint(rot_y_from_z, u_r)
    f_b = get_leg_bottom_constraint(rot_d_from_z, u_h)

    for i in range(0, 3):
        foot_pos.append(filter_tiny_numbers(sp.expand_trig(sp.simplify(Leg_Bottom[i]))))

    constraints = [
        2 * sp.expand_trig(sp.simplify(sp.expand(f))) for f in (f_a, f_t, f_b)
    ]

    return atoms, foot_pos, constraints


def create_hyperbolic_problem():
    atoms, pos, constraints = get_leg_position()

    u = sp.symbols([f'u_{i}' for i in range(0, 3)])
    q = sp.symbols([f'q_{i}' for i in range(0, 3)])
    r = sp.symbols('x, y, z')
    X = q + u
    eqns = [pos_i - r_i for pos_i, r_i in zip(pos, r)] + constraints
    atom_bounds = {a_i: (-1, 1) for a_i in X}
    for i in range(6):
        for angle_i, x_i in zip(atoms, X):
            eqns[i] = hyperbolic_substitution(eqns[i], angle_i, x_i)
        eqns[i] = sp.simplify(eqns[i])
        eqns[i] = remove_nonzero_factors(eqns[i], True, atom_bounds)
        eqns[i] = real_to_integer_polynomial(eqns[i])

    return eqns, X, r


def generate_functions():
    e, z, _ = create_hyperbolic_problem()

    f_1 = sp.lambdify((z[0], z[3]), abs(e[3]) ** 2, 'numpy')
    f_2 = sp.lambdify((z[1], z[4]), abs(e[4]) ** 2, 'numpy')
    f_3 = sp.lambdify((z[2], z[5]), abs(e[5]) ** 2, 'numpy')

    _beta_a = lambda a: (spo.minimize_scalar(
        lambda x: f_1(np.tan(x / 2), np.tan(a / 2)),
        bounds=body_angle_bounds,
        method='bounded'
    )).x

    _beta_Y = lambda a: (spo.minimize_scalar(
        lambda x: f_2(np.tan(x / 2), np.tan(a / 2)),
        bounds=radial_angle_bounds,
        method='bounded'
    )).x

    _beta_D = lambda a: (spo.minimize_scalar(
        lambda x: f_3(np.tan(x / 2), np.tan(a / 2)),
        bounds=height_angle_bounds,
        method='bounded'
    )).x

    return _beta_a, _beta_Y, _beta_D


rest_angles = [beta(0) for beta in generate_functions()]


def generate_gradients():
    e, z, _ = create_hyperbolic_problem()

    DF = [
        lambda x:
        sp.lambdify((z[i], z[i + 3]),
                    sp.simplify(-e[i + 3].diff(z[i + 3]) / e[i + 3].diff(z[i])) * (4 + z[i+3] ** 2) / (4 + z[i]**2))
            (np.tan(x[0]/2), np.tan(x[1]/2))
        for i in range(3)
    ]
    return DF


def create_tables():
    F = generate_functions()
    dF = generate_gradients()

    pwm = [i for i in range(-200, 201)]
    motor_angles = [((pwm_i - 0.5) * pwm_factor,
                     pwm_i * pwm_factor,
                     (pwm_i + 0.5) * pwm_factor)
                    for pwm_i in pwm]

    tables = ([], [], [])

    for pwm_i, (low, center, high) in zip(pwm, motor_angles):
        for table, f, df in zip(tables, F, dF):
            x = f(center)
            x_0 = f(low)
            x_1 = f(high)
            table.append(TableEntry(
                pwm=pwm_i + pwm_zero,
                low=x_0 if x_0 < x_1 else x_1,
                high=x_1 if x_1 > x_0 else x_0,
                value=x,
                gradient=df((x, center))
            ))
    return tables


forward_kinematics, forward_jacobian, inverse_kinematics = generate_kinematic_solvers()


def generate_kinematics_source():
    atoms, pos, _ = get_leg_position()
    angles = atoms[0:3]

    variables = []
    subs = []
    in_name = "angles"
    for i, th in enumerate(atoms[0:3]):
        s_i, c_i = sp.symbols(f"s_{i}, c_{i}")
        sin_i = sp.sin(th)
        cos_i = sp.cos(th)
        subs.append((sin_i, s_i))
        subs.append((cos_i, c_i))
        variables.append(f"float s_{i} = sin({in_name}[{i}]);")
        variables.append(f"float c_{i} = cos({in_name}[{i}]);")

    X = [p.subs(subs) for p in pos]
    J = [[pos[i].diff(x_j).subs(subs) for x_j in angles] for i in range(3)]
    Jdet = eqn_to_mul_add(sp.Matrix(J).det() / 10000)

    f1 = Function(name='get_leg_position', ret_type='float*')
    Df = Function(name='get_leg_jacobian', ret_type='float*')
    detDf = Function(name='get_jacobian_determinant', ret_type='float')

    f1.arguments = [
        ('float*',  f'{in_name}'),
        ('float*', f'position')
    ]
    f1.statements = [v for v in variables]
    f1.statements += [f'position[{i}] = {x};' for i, x in enumerate(X)]
    f1.statements.append('return position;\n')

    Df.arguments = [
        ('float*', f'{in_name}'),
        ('float*', 'jacobian')
    ]
    Df.statements = [v for v in variables] + [
        f'jacobian[{col + 3 * row}] = {J[row][col]};'
        for row in range(3) for col in range(3)
    ] + ['return jacobian;\n']

    detDf.arguments = [('float*',  f'{in_name}')]
    detDf.statements = [v for v in variables] + [
        f'float out = {Jdet};', 'return out;\n'
    ]
    header = "\n".join(
        [f_i.get_declaration() for f_i in [f1, Df, detDf]]

    )
    source = "\n\n".join(
        [f_i.get_definition() for f_i in [f1, Df, detDf]]
    )
    source = source.replace("**", "^")
    return header, source


def cholesky_LDL(A):
    rows, cols = A.shape
    L = np.zeros((rows, cols))
    D = np.zeros((rows, cols))

    for j in range(rows):
        D[j,j] = A[j, j] - sum(D[k, k] * L[j, k]**2 for k in range(j))
        L[j, j] = 1
        for i in range(j, cols):
            L[i,j] = (A[i,j] - sum(L[i,k]*L[j,k]*D[k,k] for k in range(j))) / D[j, j]

    return L, D


from pickle import dump, load
file = 'tables.pkl'


def write_tables(tables):
    with open(file, 'wb') as fp:
        dump(tables, fp)


def load_tables():
    try:
        with open(file, 'rb') as fp:
            data = load(fp)
        return data
    except Exception as ex:
        print(ex)
        return None


@lru_cache(maxsize=1)
def get_tables():

    tables = load_tables()
    if tables is not None:
        return tables

    tables = create_tables()
    filtered_tables = ([], [], [])

    for i, table in enumerate(tables):
        n = len(table)
        assert n > 100
        table.sort(key=lambda x: x.value)
        df_zero = table[n // 2].gradient
        low = table[n // 2].low
        high = table[n // 2].high
        mid_idx = n // 2

        filtered_tables[i].append(table[mid_idx])

        for entry in reversed(table[0:mid_idx]):
            if entry.gradient * df_zero < 0 or low < entry.value < high or entry.high == entry.low:
                break
            filtered_tables[i].append(entry)
            low = entry.low

        for entry in table[mid_idx + 1:]:
            if entry.gradient * df_zero < 0 or low < entry.value < high or entry.high == entry.low:
                break
            filtered_tables[i].append(entry)
            high = entry.high

        filtered_tables[i].sort(key=lambda x: x.pwm)

    write_tables(filtered_tables)
    return filtered_tables


def pwm_to_joints(pwms: np.ndarray) -> Optional[np.ndarray]:
    tables = get_tables()
    results = []
    for pwm, table in zip(pwms.tolist(), tables):
        if pwm < table[0].pwm or pwm > table[-1].pwm:
            return None
        idx = 0

        while idx < len(table):
            if pwm == table[idx].pwm:
                results.append(table[idx].value)
                break
            idx += 1

        if idx == len(table):
            return None

    return np.array(results)


def joints_to_pwm(values):
    tables = get_tables()
    results = []
    for table, value in zip(tables, values.tolist()):
        entries = [entry for entry in table if entry.low < value < entry.high]
        if not entries:
            return None
        results.append(entries[0].pwm)

    return np.array(results)


def is_valid(motor_values: np.ndarray):
    tables = get_tables()
    for table, value in zip(tables, motor_values.tolist()):
        result = [entry for entry in table if entry.low < value < entry.high]
        if not result:
            return False

    return True


def generate_tables_source():
    lt = LookupTable('AngleLookupTable', pwm=int, low=float, high=float, value=float, gradient=float)
    for table, name in zip(get_tables(), ['BodyJoint', 'RadialJoint', 'HeightJoint']):
        for e in table:
            lt.add_entry(name, *e)

    return lt.get_header(), lt.get_table_implementation()
