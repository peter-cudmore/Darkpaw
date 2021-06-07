import numpy as np
import model as m
import pytest

# x_rest = np.array([127.9448021, 144.63271919, -70.78863683])
u_rest = np.array([300, 300, 300])

e_x = np.array([1, 0, 0], dtype=float)
e_z = np.array([0, 0, 1], dtype=float)

epsilon_rad = 0.1
epsilon_mm = 1


def test_tables():
    tables = m.get_tables()

    for table in tables:
        for entry in table:
            assert entry.low < entry.value < entry.high, "Entry is mis-ordered"

            same_values = {
                e for e in table if e.low < entry.value < e.high
                and e != entry
            }
            assert not same_values, "Duplicate intervals"

        different_gradient = {e for e in table[1:] if e.gradient*table[0].gradient < 0}
        assert not different_gradient, "Gradient crosses zero"


def pwm_sampler(n_points):
    n_samples = 0
    while n_samples < n_points:
        yield np.random.randint(m.pwm_zero - 200, m.pwm_zero + 200, (3,))
        n_samples += 1


def angle_sampler(n_points):
    t_1, t_2, t_3 = m.get_tables()
    n_samples = 0
    while n_samples < n_points:
        i_1 = np.random.randint(0, len(t_1))
        i_2 = np.random.randint(0, len(t_2))
        i_3 = np.random.randint(0, len(t_3))

        result = np.array([t_1[i_1].value, t_2[i_2].value, t_3[i_3].value])

        yield result
        n_samples += 1


def int_array_approx(array1, array2):
    return all((int(a1) - int(a2)) < 4 for a1, a2 in zip(array1.tolist(), array2.tolist()))


def test_angles():
    q = m.pwm_to_joints(u_rest)
    u = m.joints_to_pwm(q)
    assert np.linalg.norm(u - u_rest) < epsilon_rad
    assert all(-np.pi / 2 < q[i] < np.pi / 2 for i in range(3))
    u_test = np.zeros_like(u_rest)
    n_samples = 100000

    for m_list in pwm_sampler(n_samples):
        u_test[:] = m_list
        q_test = m.pwm_to_joints(u_test)
        if q_test is None:
            continue
        u = m.joints_to_pwm(q_test)
        assert u is not None, f"Failed to find values for {q_test.tolist()}, should be {u_test.tolist()}"
        assert all(u_i == ut_i for u_i, ut_i in zip(u.tolist(), m_list)), f"Inverse is not equal: {u.tolist()} != {m_list}"
        assert all(-np.pi / 2 < q[i] < np.pi / 2 for i in range(3))


def position_is_valid(x):
    if (x[0] < 0) or (x[1] < 90) or (x[2] > 0):
        return False
    else:
        return True


def test_kinematics():

    # for each motor value
    # check if it is within the domain, where the jacobian
    # is invertible.
    #
    # if so,
    # 1. get the forward kinematics solution and make sure
    #    the signs are right.
    n_samples = 1000
    for angles in angle_sampler(n_samples):
        x = m.forward_kinematics(angles)
        jac = m.forward_jacobian(angles)
        assert position_is_valid(x)
        assert abs(np.linalg.det(jac)) > 1
        u_ik = m.inverse_kinematics(x)
        assert np.linalg.norm(u_ik - angles) < epsilon_rad, f"IK Angles are not close enough at {x}"
        x_ik = m.forward_kinematics(u_ik)
        assert np.linalg.norm(x - x_ik) < epsilon_mm

