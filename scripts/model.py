import numpy as np
import scipy.optimize as spo
import sympy as sp
from helpers import *
from codegen import LookupTable, Function

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

origin = np.array([0, 0, 0, 1])

pwm_factor = np.pi / 300
pwm_zero = 300

def get_body_linkage_constraint(rotation_from_frame, motor):
    body_motor_linkage_leg = (
            RotateXY(rotation_from_frame)
            @ Translate(body_linkage_offset_radial, body_linkage_offset_inner, 0)
            @ RotateXY(-rotation_from_frame)
    )
    body_motor_linkage_arm = (
        Translate(0, body_length/2, 0)
        @ RotateXY(motor)
        @ Translate(motor_arm_body, 0, 0)
        @ RotateXY(-motor)
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
        @ RotateXZ(-motor)
    )

    ybar_linkage = (
        RotateXZ(frame_angle_from_z)
        @ Translate(-Y_i, 0, Y_h)
        @ RotateXZ(-frame_angle_from_z)
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
        @ RotateXZ(-motor)
    )
    dbar_linkage = (
        RotateXZ(frame_angle)
        @ Translate(0, 0, D_section_b)
        @ RotateXZ(-frame_angle)
    )

    difference = (motor_linkage - dbar_linkage) @ origin

    return difference.T @ difference - l_b ** 2


def get_leg_position(front=True, left=True):
    anchor = Translate(body_width / 2, body_width / 2, 0)

    if not left:
        anchor = ReflectX() @ anchor
    if not front:
        anchor = ReflectY() @ anchor
    atoms = sp.symbols(r'bodyangle, radial_angle, height_angle, u_b, u_r, u_h')
    rot_from_pivot, rot_y_from_z, rot_d_from_z, u_a, u_r, u_h = atoms

    Leg_Bottom = (
         anchor
         @ RotateXY(np.pi/2) @ RotateXY(rot_from_pivot)
         @ Translate(arm_anchor_to_pivot, 0, 0)
         @ RotateXZ(rot_d_from_z) @ Translate(-D_section_e, 0, 0) @ RotateXZ(-rot_d_from_z)
         @ RotateXZ(rot_y_from_z)
         @ Translate(-leg_pivot_to_foot_x, 0, leg_pivot_to_foot_y)
    ) @ origin

    foot_pos = []
    f_a = get_body_linkage_constraint(rot_from_pivot, u_a)
    f_t = get_leg_top_constraint(rot_y_from_z, u_r)
    f_b = get_leg_bottom_constraint(rot_d_from_z, u_h)

    for i in range(0, 3):
        foot_pos.append(filter_tiny_numbers(sp.expand_trig(sp.simplify(Leg_Bottom[i]))))

    constraints = [
        filter_tiny_numbers(sp.expand_trig(sp.simplify(sp.expand(f)))) for f in (f_a, f_b, f_t)
    ]

    return atoms, foot_pos, constraints


def create_leg_complex_problem(front=True, left=True):
    atoms, pos, constraints = get_leg_position(front, left)

    u = sp.symbols([f'u_{i}' for i in range(0, 3)])
    q = sp.symbols([f'q_{i}' for i in range(0, 3)])

    substitutions = [(atoms[i], q[i]) for i in range(0, 3)]
    substitutions += [(atoms[i + 3], u[i]) for i in range(0, 3)]

    r = [filter_tiny_numbers(p.subs(substitutions)) for p in pos]
    constraints = [filter_tiny_numbers(c.subs(substitutions)) for c in constraints]
    x, eqns, subs, (xq, xu) = angles_to_complex(r + constraints, q, u)

    eqns = [remove_nonzero_factors(e) if i >= len(r) else e for i, e in enumerate(eqns)]

    return x, eqns, (xq, xu)


def generate_tables():
    # pwm = np.linspace(-np.pi / 2, np.pi / 2, 401)
    pwm = [i for i in range(-200, 201)]

    z, e, maps = create_leg_complex_problem()
    temp = e[4]
    e[4] = e[5]
    e[5] = temp
    DF = [
        sp.simplify(-e[i + 3].diff(z[i + 3]) / e[i + 3].diff(z[i])) for i in range(3)
    ]
    bounds = []
    positions = LookupTable('AngleLookupTable', pwm=int, value=float, grad=float)

    f_1 = sp.lambdify((z[0], z[3]), abs(e[3]) ** 2, 'numpy')
    df_1 = sp.lambdify((z[0], z[3]), DF[0], 'numpy')
    for i in range(401):
        f = lambda x: f_1(np.exp(1j * x), np.exp(1j * pwm_factor * pwm[i]))
        res = spo.minimize_scalar(f, bounds=(-1, 1))
        positions.add_entry('BodyMotor',
            pwm[i] + pwm_zero,
            res.x,
            np.real(df_1(np.exp(1j * res.x), np.exp(1j * pwm_factor * pwm[i])))
        )

    f_2 = sp.lambdify((z[1], z[4]), abs(e[4]) ** 2, 'numpy')
    df_2 = sp.lambdify((z[1], z[4]), DF[1], 'numpy')
    for i in range(401):
        f = lambda x: 1000 * f_2(np.exp(1j * x), np.exp(1j * pwm_factor * pwm[i]))
        res = spo.minimize_scalar(f, bounds=(-1, 0.5), method='bounded')
        positions.add_entry('TopMotor',
            pwm[i] + pwm_zero,
            res.x,
            np.real(df_2(np.exp(1j * res.x), np.exp(1j * pwm_factor * pwm[i])))
        )

    f_3 = sp.lambdify((z[2], z[5]), abs(e[5]) ** 2, 'numpy')
    df_3 = sp.lambdify((z[2], z[5]), DF[2], 'numpy')

    for i in range(401):
        f = lambda x: f_3(np.exp(1j * x), np.exp(1j * pwm_factor * pwm[i]))
        res = spo.minimize_scalar(f, bounds=(-1, 0.5), method='bounded')
        positions.add_entry('BottomMotor',
            pwm[i] + pwm_zero,
            res.x,
            np.real(df_3(np.exp(1j * res.x), np.exp(1j * pwm_factor * pwm[i])))
        )

    return positions.get_header(),\
           positions.get_table_implementation()


def generate_kinematics():
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


if __name__ == '__main__':
    header_text, source_text = generate_tables()
    h1, s1 = generate_kinematics()
    header_text += "\n" + h1
    source_text += "\n" + s1
    includes = [
        '<math.h>',
        '\"model.h\"'
    ]

    with open('../src/gen/model.h', 'w') as header:
        header.writelines(header_text)

    with open('../src/gen/model.c', 'w') as source:
        source.writelines(
            "\n".join([f'#include {file}\n' for file in includes])
        )
        source.writelines(source_text)

