import numpy as np
import sympy as sp

l_a = 39
l_b = 36
l_c = 32
l_d = 26

Y_h = 39
Y_o = 7.5
Y_i = -7.5


motor_arm_top = 15
motor_arm_bottom = -12.5
motor_offset_top = 6.5
motor_offset_bottom = -6.5
motor_offset_x = 32
D_section_b = 26
D_section_e = -32
leg_mid_to_foot = 78
leg_top_to_foot = 112.5
leg_mid_to_top = 38.5

leg_inner_angle = np.arccos(leg_mid_to_foot/leg_top_to_foot)
y_base_angle = np.arcsin((Y_o - Y_i)/ Y_h)

m_t = sp.symbols('m_t', real=True)
m_b = sp.symbols('m_b', real=True)
phi = sp.symbols('phi', real=True)  # angle y makes with OX
theta = sp.symbols('theta', real=True)  # angle D makes with OX
beta = sp.symbols('beta', real=True)    # angle between D and Leg

def compose(args, value):
    recursing_args = args[1:]
    last_arg = args[0]
    if recursing_args:
        return last_arg(compose(recursing_args, value))
    else:
        return last_arg(value)

def R(th):
    c = sp.cos(th)
    s = sp.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

angles = {}


def Ri(th):
    global angles

    try:
        if th.is_constant():
            return lambda z: z * (sp.cos(th) - sp.I*sp.sin(th))
    except:
        return lambda z: z * (sp.cos(th) - sp.I * sp.sin(th))

    if th in angles:
        z_a = angles[th]
    else:
        z_a = sp.symbols(f'z_{len(angles)}', real=False)
        angles[th] = z_a
    return lambda z: z / z_a


def T(x, y):
    return np.array([[1, 0, x], [0, 1, y], [0, 0, 1]])

def Ti(x,y):
    return lambda z: z - x - sp.I*y

o = np.zeros(3,)
o[2] = 1

Leg_Mid = R(theta) @ T(D_section_e, 0) @ o
Leg_Top = R(beta) @ T(Y_h, Y_o) @ Leg_Mid

def c_norm(cv):
    return sp.sqrt(cv.conjugate()*cv)


Complex_Leg_Mid = compose([Ri(theta), Ti(D_section_e, 0)], 0)
Complex_Leg_Top = compose([Ri(beta), Ti(Y_h, Y_o)], Complex_Leg_Mid)
Complex_Leg_Bottom = compose([
    Ti(leg_mid_to_foot, 0),
    Ri(leg_inner_angle)
],
    Complex_Leg_Mid * (Complex_Leg_Top - Complex_Leg_Mid)/leg_mid_to_top)
s_t, c_t = sp.symbols('s_t, c_t')
s_b, c_b = sp.symbols('s_b, c_b')
# Linkage D

def Linkage_B_Constraint():
    L_Ba = (T(motor_offset_x, motor_offset_bottom) @ R(m_b) @ T(0, motor_arm_bottom)) @ o
    L_Bb = (R(theta) @ T(0, -D_section_b)) @ o
    L_B = L_Ba - L_Bb
    return sp.simplify(sp.expand(L_B.T @ L_B - l_b**2))

def Linkage_D_Constraint():
    L_Da = (T(motor_offset_x, motor_offset_top) @ R(m_t) @ T(0, motor_arm_top)) @ o
    L_Db = (R(phi) @ T(Y_h, - Y_i)) @ o
    L_D = L_Da - L_Db

    return sp.simplify(sp.expand(L_D.T @ L_D - l_d ** 2))
    # constraint = constraint.subs([(sp.sin(m_t), s_t), (sp.cos(m_t), c_t)])
    # constraint = sp.collect(constraint, [sp.cos(phi), sp.sin(phi)])
    # At = 195.0*c_t - 960.0*s_t + 2192.5     # constant
    # Bt = (-1170.0*c_t - 225.0*s_t - 27.0)   # coeff of sin(phi)
    # Ct = (225.0*c_t - 1170.0*s_t + 2593.5)  #coeff of cos(phi)
    # return constraint

def Linkage_C_Constraint():
    LC_a = R(phi) @ T(Y_h, Y_o) @ o
    L_C = Leg_Top - LC_a
    return sp.simplify(sp.expand(L_C.T @ L_C - l_c ** 2))


PHI = sp.acos((-195.0*c_t + 960.0*s_t - 2192.5)/sp.sqrt(1230255.0*c_t - 6056640.0*s_t + 8146496.25)) + sp.atan((1170.0*c_t + 225.0*s_t + 27.0)/(225.0*c_t - 1170.0*s_t + 2593.5))
THETA = sp.acos((-162.5*c_b - 800.0*s_b + 602.5)/sp.sqrt(-439400.0*c_b - 2163200.0*s_b + 3305640.0)) + sp.atan((1664.0 - 650.0*s_b)/(650.0*c_b - 338.0))
# constraint_c = constraint_c.subs([(theta, THETA)])
# constraint_c = sp.expand(constraint_c).simplify()
# constraint_c = constraint_c.subs([(phi, PHI)])
# constraint_c = sp.expand(constraint_c).simplify()


def Complex_Linkage_B_Constraint():
    L_Ba = compose([Ti(motor_offset_x, motor_offset_bottom), Ri(m_b), Ti(0, motor_arm_bottom)], 0)
    L_Bb = compose([Ri(theta), Ti(0, -D_section_b)], 0)
    # L_Ba = Ti(motor_offset_x, motor_offset_bottom)(Ri(m_b)(Ti(0, motor_arm_bottom)(0)))
    #L_Bb = Ri(theta)(Ti(0, -D_section_b))(0)
    substitutions = [(cv.conjugate(), 1 / cv) for cv in angles.values()]
    L_B = L_Ba - L_Bb
    return sp.simplify(
        sp.expand(
            (L_B.conjugate() * L_B).subs(substitutions) - l_b**2
        ) * angles[theta] * angles[m_b]
    )

def Complex_Linkage_D_Constraint():
    L_Da = compose(
        [Ti(motor_offset_x, motor_offset_top), Ri(m_t), Ti(0, motor_arm_top)],
        0
    )
    L_Db = compose((Ri(phi), Ti(Y_h, - Y_i)), 0)
    # L_Da = Ti(motor_offset_x, motor_offset_top)(Ri(m_t)(Ti(0, motor_arm_top)(0)))
    # L_Db = Ri(phi)(Ti(Y_h, - Y_i)(0))
    L_D = L_Da - L_Db

    substitutions = [(cv.conjugate(), 1 / cv) for cv in angles.values()]
    return sp.simplify(
            sp.expand(
                (L_D.conjugate() * L_D).subs(substitutions) - l_d ** 2
            ) * angles[phi] * angles[m_t]
    )
    # constraint = constraint.subs([(sp.sin(m_t), s_t), (sp.cos(m_t), c_t)])
    # constraint = sp.collect(constraint, [sp.cos(phi), sp.sin(phi)])
    # At = 195.0*c_t - 960.0*s_t + 2192.5     # constant
    # Bt = (-1170.0*c_t - 225.0*s_t - 27.0)   # coeff of sin(phi)
    # Ct = (225.0*c_t - 1170.0*s_t + 2593.5)  #coeff of cos(phi)
    # return constraint

def Complex_Linkage_C_Constraint():
    LC_a = compose([Ri(phi), Ti(Y_h, Y_o)], 0)
    L_C = Complex_Leg_Top - LC_a

    substitutions = [(cv.conjugate(), 1 / cv) for cv in angles.values()]
    return sp.simplify(
        sp.expand(
            (L_C.conjugate() * L_C).subs(substitutions) - l_d ** 2
        ) * angles[theta]*angles[beta]*angles[phi]
    )
    # constraint = constraint.subs([(sp.sin(m_t), s_t), (sp.cos(m_t), c_t)])
    # constraint = sp.collect(constraint, [sp.cos(phi), sp.sin(phi)])
    # At = 195.0*c_t - 960.0*s_t + 2192.5     # constant
    # Bt = (-1170.0*c_t - 225.0*s_t - 27.0)   # coeff of sin(phi)
    # Ct = (225.0*c_t - 1170.0*s_t + 2593.5)  #coeff of cos(phi)
    # return constraint


if __name__ == '__main__':
    print(Complex_Linkage_B_Constraint())
    print(Complex_Linkage_C_Constraint())
    print(Complex_Linkage_D_Constraint())