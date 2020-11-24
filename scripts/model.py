import numpy as np
import cvxpy as cp
from scipy.optimize import root
from helpers import *

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
angles = {}

#
# beta_D, beta_Y, beta_A = sp.symbols(r'\beta_D, \beta_Y, \beta_A', real=True)
# u_r, u_h, u_b = sp.symbols('u_r, u_h, u_b', real=True)
# Leg_Pivot = R(beta_D) @ T(D_section_e, 0) @ R(-beta_D)
# Leg_Bottom = Leg_Pivot @ R(beta_Y) @ T(leg_pivot_to_foot_x, leg_pivot_to_foot_y)
# # For R(beta_Y)
# Y_inner = R(beta_Y) @ T(Y_i, Y_h) @ R(-beta_Y)
# Y_outer = R(beta_Y) @ T(Y_o, Y_h) @ R(-beta_Y)
# D_bottom = R(beta_D) @ T(0, D_section_b) @ R(-beta_D)
# Top_Motor = T(motor_offset_x, motor_offset_top)
# Top_Motor_Anchor = Top_Motor @ R(u_r) @ T(0, motor_arm_top) @ R(-u_r)
# Bottom_Motor = T(motor_offset_x, motor_offset_bottom)
# Bottom_Motor_Anchor = Bottom_Motor @ R(u_h) @ T(0, motor_arm_bottom) @ R(-u_h)
# # Front left, form top down POV is xy
# Body_Anchor = T(body_width / 2, body_length/2) @ R(np.pi/2) @ R(beta_A) @ T(body_linkage_offset_inner, body_linkage_offset_radial) @ R(-beta_A)
# Body_Motor_Arm = T(body_width/2,  0) @ R(u_b) @ T(0, -motor_arm_body) @ R(-u_h)
#
# C_a = simplify_array((Body_Anchor - Body_Motor_Arm).T @ (Body_Anchor - Body_Motor_Arm))
# f_a = sp.expand_trig(sp.simplify(sp.expand(C_a[2, 2] - l_a**2)))
# f_a_lambda = sp.lambdify((beta_A, u_b), f_a)
#
#
# def beta_a(body_motor, last_value=0):
#     return root(
#         lambda x: [abs(f_a_lambda(x[0], x[1])), x[1]],
#         x0=np.array([last_value, body_motor])
#     ).x[0]
#
#
# C_b = simplify_array((Bottom_Motor_Anchor - D_bottom).T @ (Bottom_Motor_Anchor - D_bottom))
# f_b = sp.expand_trig(sp.simplify(sp.expand(C_b[2, 2] - l_b**2)))
# f_b_lambda = sp.lambdify((beta_D, u_h), f_b)
#
#
# def beta_d(bottom_motor, last_value=0):
#     return root(
#         lambda x: [abs(f_b_lambda(x[0], x[1])), x[1]],
#         x0=np.array([last_value, bottom_motor])
#     ).x[0]
#
#
# C_t = simplify_array((Top_Motor_Anchor - Y_inner).T @ (Top_Motor_Anchor - Y_inner))
# f_t = sp.expand_trig(sp.simplify(sp.expand(C_t[2, 2] - l_d ** 2)))
# f_t_lambda = sp.lambdify((beta_Y, u_r), f_t)
#
#
# def beta_y(top_motor, last_value=0):
#     return root(
#         lambda x: [abs(f_t_lambda(x[0], x[1])), x[1]],
#         x0=np.array([last_value, top_motor])
#     ).x[0]

origin = np.array([0, 0, 0, 1])


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

def find_lock_values(theta_1, theta_2, constraint):
    # we have f(theta_1, theta_2) = 0
    # assume there is a singularity/gimbal lock
    # we want to find where this is
    # so df = 0 = f_{\theta_1}dTheta_1 + f_{theta_2}dTheta_2
    # so we want to solve f_{\theta_1} = 0 and f(theta_1, theta_2) = 0
    # also, f_{\theta_2} = 0  and f(theta_1, theta_2) = 0
    pass



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


def build_full_system():

    # order is FL, BL, FR, BR
    leg_args = {
        #id, #front  #left
        0: (True, True),
        1: (False, True),
        2: (True, False),
        3: (False, False)
    }

    u = sp.symbols([f'u_{i}' for i in range(0, 12)])
    q = sp.symbols([f'q_{i}' for i in range(0, 12)])

    legs = {}
    constraints = []
    for idx, (front, left) in leg_args.items():
        atoms, pos, constraints_ = get_leg_position(front, left)

        substitutions = [(atoms[i], q[i + 3 * idx]) for i in range(0, 3)]
        substitutions += [(atoms[i + 3], u[i + 3*idx]) for i in range(0, 3)]

        legs[idx] = sp.Matrix([filter_tiny_numbers(p.subs(substitutions)) for p in pos])
        constraints += [filter_tiny_numbers(c.subs(substitutions)) for c in constraints_]

    return q, u, legs, constraints


def construct_system():
    q_base, u_base, legs, constraints = build_full_system()

    X_temp, eqns, subs, maps = angles_to_quadratics(constraints, q_base, u_base)

    legs = {l: legs[l].subs(subs) for l in legs}

    return X_temp, legs, eqns, maps


def get_motor_bounds_constraints(decision_vars, cfg_space, u_map):

    constraints = []
    for (_, c_i) in u_map.values():
        idx = cfg_space.index(c_i)
        constraints.append(decision_vars[idx] >= 0)

    return constraints


def get_quadratic_constraints(decision_vars, cfg_space, eqns):

    constraints = []
    X0 = [0] * len(cfg_space)

    for eqn in eqns:
        P, qT, r, _ = quadratic_form(eqn, cfg_space, X0)
        M = np.block([[P, qT.T / 2], [qT/2, r]])
        #print(np.linalg.eigvals(M))
        constraints.append(cp.quad_form(decision_vars, M) == 0)

    return constraints


def get_quadratic_ojectives(decision_vars, cfg_space, eqns):

    objectives = []
    X0 = [0] * len(cfg_space)

    for eqn in eqns:
        P, qT, r, _, = quadratic_form(eqn, cfg_space, X0)
        expr = cp.quad_form(decision_vars, P) + qT @ decision_vars + r
        objectives.append(expr)
    return objectives


def get_leg_as_quadratics(cfg_space, leg):
    X0 = [0] * len(cfg_space)
    out = []
    for i in range(3):
        p, q, r, _, _ = quadratic_form(leg[i], cfg_space, X0)
        out += [(p, q, r)]
    return out


def create_leg_qp(front, left, gamma=0.01):
    atoms, pos, constraints = get_leg_position(front, left)

    r_cmd = cp.Parameter(3)

    u = sp.symbols([f'u_{i}' for i in range(0, 3)])
    q = sp.symbols([f'q_{i}' for i in range(0, 3)])

    substitutions = [(atoms[i], q[i]) for i in range(0, 3)]
    substitutions += [(atoms[i + 3], u[i]) for i in range(0, 3)]

    r = [filter_tiny_numbers(p.subs(substitutions)) for p in pos]
    constraints = [filter_tiny_numbers(c.subs(substitutions)) for c in constraints]

    x, eqns, subs, (xq, xu) = angles_to_quadratics(constraints, q, u)

    X = cp.Variable(len(x))

    cp_constraints = get_motor_bounds_constraints(X, x, xu)
    cp_constraints += get_quadratic_constraints(X, x, eqns)

    # objective = r_error + gamma * cp.quad_form(X, Pi_u)

    problem = cp.Problem(cp.Minimize(cp.norm(X, 1)), cp_constraints)

    return problem, X, cp_constraints


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

# def solve_leg_qp(r_cmd, q_current=None):


def get_differential_ik(front=True, left=True, motor_cost=0.1):

    x, eqns, (xq, xu) = create_leg_complex_problem(front, left)

    jacobian_terms = [[row.diff(x_i) for x_i in x] for row in eqns]
    jacobian_terms += [[motor_cost if x_i == u_i else 0 for x_i in x] for u_i in xu]
    dphi_dq = sp.Matrix(jacobian_terms)

    dq_dtheta = sp.Matrix([[1j/x_i if x_i == x_j else 0 for x_i in x] for x_j in x])

    A = dphi_dq * dq_dtheta

    def step(desired_position, current_angles):
        q_now = np.exp(1j*current_angles)
        Aq = sp.matrix2numpy(A.subs([(x_i, q_i) for x_i, q_i in zip(x, q_now)]), dtype=np.complex)

        b = np.zeros((Aq.shape[0],))
        b[0:3] = desired_position
        # r = Aq.T @ b
        # inverse = np.linalg.inv(Aq.T @ Aq)
        # return current_angles + np.real(inverse @ r)
        pinv = np.linalg.pinv(Aq)
        return np.real(pinv @ b)

    return step, (xq, xu)


def get_implicit_tan_coefficients(free_angle, eqn):
    s, c = sp.symbols('s, c')
    eqn_temp = sp.expand_trig(eqn.subs([(sp.sin(free_angle), s), (sp.cos(free_angle), c)]))
    cos_coeff = eqn_temp.diff(c)
    sin_coeff = eqn_temp.diff(s)
    remainder = (eqn_temp - cos_coeff * c - sin_coeff * s).simplify()

    return cos_coeff, sin_coeff, remainder

def generate_solve_leg_angles():
    (ba, br, bh, x1, x2, x3), foot_pos, (f_a, f_h, f_r) = get_leg_position(True, True)
    template = """
        float {angle}_cos_coeff = {rhs_cos};
        float {angle}_sin_coeff = {rhs_sin};
        float {angle}_remainder = {rhs_rem};\n
    """

    text = ""

    for pair in [(ba, f_a), (br, f_r), (bh, f_h)]:
        cos_coeff, sin_coeff, remainder = get_implicit_tan_coefficients(*pair)

        rhs_cos = str(cos_coeff).replace('sin', 'sinf').replace('cos', 'cosf')
        rhs_sin = str(sin_coeff).replace('sin', 'sinf').replace('cos', 'cosf')
        rhs_rem = str(remainder).replace('sin', 'sinf').replace('cos', 'cosf')

        text += template.format(
            angle=str(pair[0]),
            rhs_cos=rhs_cos,
            rhs_sin=rhs_sin,
            rhs_rem=rhs_rem
        )

    return text


def generate_leg_position_code():
    (x1, x2, x3, _, _, _), foot_pos, _ = get_leg_position()
    X = (x1, x2, x3)
    y = []
    Xs = {
        sp.symbols('s_a'): sp.sin(x1),
        sp.symbols('s_r'): sp.sin(x2),
        sp.symbols('s_h'): sp.sin(x3),
    }
    Xc = {
        sp.symbols('c_a'): sp.cos(x1),
        sp.symbols('c_r'): sp.cos(x2),
        sp.symbols('c_h'): sp.cos(x3),
    }
    for i, x_i in enumerate(X):
        for j, x_j in enumerate(X):
            if j > i:
                y.append(x_i - x_j)
                y.append(x_i + x_j)

    from sympy.simplify.fu import TR8

    Ys = {sp.symbols(f's_{i}'): sp.sin(v) for i, v in enumerate(y)}
    Yc = {sp.symbols(f'c_{i}'): sp.cos(v) for i, v in enumerate(y)}
    subs = list(Ys.items()) + list(Yc.items()) + list(Xs.items()) + list(Xc.items())
    subs = [(v, k) for k, v in subs]

    px = TR8(foot_pos[0]).subs(subs)
    py = TR8(foot_pos[1]).subs(subs)
    pz = TR8(foot_pos[2]).subs(subs)

    "bodyangle -> angles[TorsoLeg]"



    return px, py, pz, (Ys, Yc)


if __name__ == '__main__':
    x, eqns, _ = create_leg_complex_problem()
    print(eqns[3:])