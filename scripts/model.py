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
f_a = sp.expand_trig(sp.simplify(sp.expand(C_a[2, 2] - l_a**2)))
f_a_lambda = sp.lambdify((beta_A, u_b), f_a)


def beta_a(body_motor, last_value=0):
    return root(
        lambda x: [abs(f_a_lambda(x[0], x[1])), x[1]],
        x0=np.array([last_value, body_motor])
    ).x[0]


C_b = simplify_array((Bottom_Motor_Anchor - D_bottom).T @ (Bottom_Motor_Anchor - D_bottom))
f_b = sp.expand_trig(sp.simplify(sp.expand(C_b[2, 2] - l_b**2)))
f_b_lambda = sp.lambdify((beta_D, u_h), f_b)


def beta_d(bottom_motor, last_value=0):
    return root(
        lambda x: [abs(f_b_lambda(x[0], x[1])), x[1]],
        x0=np.array([last_value, bottom_motor])
    ).x[0]


C_t = simplify_array((Top_Motor_Anchor - Y_inner).T @ (Top_Motor_Anchor - Y_inner))
f_t = sp.expand_trig(sp.simplify(sp.expand(C_t[2, 2] - l_d ** 2)))
f_t_lambda = sp.lambdify((beta_Y, u_r), f_t)


def beta_y(top_motor, last_value=0):
    return root(
        lambda x: [abs(f_t_lambda(x[0], x[1])), x[1]],
        x0=np.array([last_value, top_motor])
    ).x[0]


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

    foot_pos = []

    for i in range(0, 3):
        foot_pos.append(filter_tiny_numbers(sp.expand_trig(sp.simplify(Leg_Bottom[i]))))

    atoms = [beta_A, beta_Y, beta_D, u_b, u_h, u_r]

    return atoms, foot_pos, (filter_tiny_numbers(f) for f in (f_a, f_b, f_t))


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
