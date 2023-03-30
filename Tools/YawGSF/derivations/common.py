from helpers import *
from sys import *
x = Matrix([])
state_idx_defines = []

def add_states(name, n):
    global x, state_idx_defines
    syms = []
    names = []

    for i in range(n):
        names.append('%s%u' % (name.upper(), i))
        state_idx_defines.append('static const int STATE_IDX_%s = %u;' % (names[-1], len(state_idx_defines)))

    ret = Matrix([Symbol('x[STATE_IDX_%s]' % (name,), real=True) for name in names])
    x = Matrix([x, ret])
    return ret

def get_state_index(state_symbol):
    global x

    for i in range(len(x)):
        if x[i] == state_symbol:
            return i
    return None

est_quat = Matrix(symbols('quat[0] quat[1] quat[2] quat[3]', real=True))

rot_err = add_states('rot_err', 3)
gbias = add_states('gbias', 3)
#gscale = add_states('gscale', 3)
abias_z = add_states('abias_z', 1)
vel = add_states('vel', 3)

abias = Matrix([0,0,abias_z[0]])

n_states = len(x)

P = compressedSymmetricMatrix('P',n_states)
#P = copy_upper_to_lower_offdiagonals(Matrix(n_states, n_states, symbols(r'P(0:%u\,0:%u)' % (n_states,n_states), real=True)))

for i in range(n_states):
    P[i,i] = Symbol(str(P[i,i]), real=True, nonnegative=True)

err_quat = gibbs_to_quat(rot_err)
Tbn = quat_to_matrix(quat_multiply(est_quat, err_quat))

def derive_zero_rot_err(_x, _P):
    _P = copy_upper_to_lower_offdiagonals(_P)
    del_gibbs = Matrix(symbols('del_gibbs((0:3))', real=True))
    #err_quat = gibbs_to_quat(rot_err)
    rotErrNew = gibbs_multiply(-del_gibbs, rot_err)

    f = x[:,:]
    f[0:3,:] = rotErrNew
    assert f.shape == x.shape

    F = f.jacobian(x)

    soln = solve(rotErrNew, del_gibbs, dict=True)[0]
    f = f.xreplace(soln).xreplace(dict(zip(x, _x)))
    F = F.xreplace(soln).xreplace(dict(zip(x, _x)))
    del_quat = gibbs_to_quat(del_gibbs).xreplace(soln).xreplace(dict(zip(x, _x)))

    quat_n = quat_multiply(est_quat,del_quat)
    x_n = f
    P_n = F*_P*F.T

    P_n = copy_upper_to_lower_offdiagonals(P_n)

    return quat_n, x_n, P_n
