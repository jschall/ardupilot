#!/usr/bin/python
from sympy import *
from sympy.printing.ccode import *
from math import sqrt, floor, fmod
import math
from helpers import *
import sys

output_stream = sys.stdout


# Parameters
dt = Symbol('dt')  # Time step
del_vel_noise = Symbol('del_vel_noise')

# Inputs
del_vel = Matrix(symbols('del_vel[0:3]'))

# States
x = Matrix(symbols('state[0:6]'))
nStates = len(x)

pos = x[0:3,0]
vel = x[3:6,0]

# Covariance matrix
P = compressedSymmetricMatrix('cov', nStates)

def deriveSyntheticPos(jsonfile):
    print('Beginning synthetic position derivation')
    t1 = datetime.datetime.now()

    align = toVec(symbols('align_x align_y align_z'))
    align_R = toVec(symbols('align_x_R align_y_R align_z_R'))

    los_unit_ned = toVec(symbols('los_n los_e los_d'))
    height = Symbol('height')
    height_R = Symbol('height_R')

    synthetic_pos = los_unit_ned*height/los_unit_ned[2]

    corrected_meas = quat_to_matrix(rot_vec_to_quat_approx(align)) * synthetic_pos
    corrections = toVec(align, height)
    corrections_R = diag(*toVec(align_R, height_R))

    H = corrected_meas.jacobian(corrections)
    synthetic_pos_R = H*corrections_R*H.T

    subs = {
        align[0]:0,
        align[1]:0,
        align[2]:0
        }

    synthetic_pos_R = simplify(upperTriangularToVec(synthetic_pos_R).xreplace(subs))

    # Output generation
    funcParams = {'align_R':align_R,'los_unit_ned':los_unit_ned,'height':height,'height_R':height_R}

    funcs = {}

    #funcs['subx'] = {}
    #funcs['subx']['params'] = funcParams
    #funcs['subx']['ret'] = toVec([x[1] for x in subx])
    #funcs['subx']['retsymbols'] = toVec([x[0] for x in subx])

    #funcParams = funcParams.copy()
    #funcParams['subx'] = toVec([x[0] for x in subx])

    funcs['z'] = {}
    funcs['z']['params'] = funcParams
    funcs['z']['ret'] = synthetic_pos

    funcs['R'] = {}
    funcs['R']['params'] = funcParams
    funcs['R']['ret'] = synthetic_pos_R

    check_funcs(funcs)

    saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    op_count, subx_count = getOpStats(funcs)
    t2 = datetime.datetime.now()
    print('%s synthetic position: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,jsonfile,op_count,subx_count))

def printInitialization():
    vel = Matrix(symbols('vel[0:3]'))
    vel_var = Matrix(symbols('vel_xy_var vel_xy_var vel_z_var'))
    align = Matrix(symbols('align[0:3]'))
    align_var = Matrix(symbols('align_xy_var align_xy_var align_z_var'))

    los_unit_ned = Matrix(symbols('los_unit_ned[0:3]'))
    dist = Symbol('dist')
    dist_var = Symbol('dist_var')

    x_n = zeros(6,1)
    x_n[0:3,0] = quat_to_matrix(rot_vec_to_quat_approx(align)) * los_unit_ned*dist
    x_n[3:6,0] = vel

    params = toVec(align, dist, vel)
    params_R = diag(*toVec(align_var, dist_var, vel_var))

    H = x_n.jacobian(params)
    P_n = H*params_R*H.T

    x_n = x_n.xreplace(dict(zip(align, zeros(3,1))))
    P_n = P_n.xreplace(dict(zip(align, zeros(3,1))))

    output_stream.write('// INITIALIZATION\n')

    for i in range(len(x_n)):
        output_stream.write('state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_n[i])))

    for i in range(len(P_n)):
        output_stream.write('cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_n[i])))
    output_stream.write('\n\n')

def printPrediction():
    # f: state-transtition model
    f = zeros(6,1)
    f[0:3,0] = pos+vel*dt
    f[3:6,0] = vel+del_vel

    assert f.shape == x.shape

    # F: linearized state-transition model, AKA "A" in literature
    F = f.jacobian(x)

    # u: control input vector
    u = del_vel

    # G: control-influence matrix, AKA "B" in literature
    G = f.jacobian(u)

    # w_u_sigma: additive noise on u
    w_u_sigma = Matrix([del_vel_noise, del_vel_noise, del_vel_noise])

    # Q_u: covariance of additive noise on u
    Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

    # Q: covariance of additive noise on x
    Q = G*Q_u*G.T

    # x_n: state at time k+1
    x_n = f

    # P_n: covariance matrix at time k+1
    P_n = F*P*F.T + Q
    P_n = upperTriangularToVec(P_n)

    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=1)

    output_stream.write('// PREDICTION\n')
    output_stream.write('// %u operations\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(subx),))

    for i in range(len(subx)):
        output_stream.write('float %s = %s;\n' % (subx[i][0], CCodePrinter_float().doprint(subx[i][1])))

    for i in range(len(x_n)):
        output_stream.write('state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_n[i])))

    for i in range(len(P_n)):
        output_stream.write('cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_n[i])))
    output_stream.write('\n\n')


def printFusion(name, h, R_type='matrix'):
    # z: observation
    z = Matrix(symbols('z[0:%u]' % (len(h),)))

    # R: observation covariance
    if R_type == 'matrix':
        R = compressedSymmetricMatrix('R', len(h))
    elif R_type == 'vector':
        R = diag(*symbols('R[0:3]'))
    elif R_type == 'scalar':
        R = diag(*[Symbol('R') for _ in z])

    # y: innovation vector
    y = z-h

    # H: measurement sensitivity matrix
    H = h.jacobian(x)


    # S: innovation covariance
    S = H*P*H.T + R

    S_I = quickinv_sym(S)

    # K: Kalman gain
    K = P*H.T*S_I

    I = eye(nStates)

    NIS = y.T*S_I*y # normalized innovation squared

    x_n = x + K*y
    P_n = (I-K*H)*P
    P_n = upperTriangularToVec(P_n)

    x_n,P_n,NIS,subx = extractSubexpressions([x_n,P_n,NIS],'subx',threshold=1)

    output_stream.write('// %s FUSION\n' % (name.upper(),))
    output_stream.write('// %u operations\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(subx),))
    for i in range(len(subx)):
        output_stream.write('float %s = %s;\n' % (subx[i][0], CCodePrinter_float().doprint(subx[i][1])))

    for i in range(len(x_n)):
        output_stream.write('state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_n[i])))

    for i in range(len(P_n)):
        output_stream.write('cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_n[i])))

    output_stream.write('\nfloat NIS = %s;\n' % (CCodePrinter_float().doprint(NIS[0]),))
    output_stream.write('\n\n')


printInitialization()
printPrediction()
printFusion('los', pos/vec_norm(pos), 'scalar')
printFusion('velocity', vel, 'vector')
sys.exit()
