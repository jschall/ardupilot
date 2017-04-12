#!/usr/bin/python
from sympy import *
from sympy.printing.ccode import *
from math import sqrt, floor, fmod
import math
from helpers import *
import sys

output_stream = sys.stdout
subx_threshold = 5

# Parameters
dt = Symbol('dt')  # Time step
del_vel_noise = Symbol('del_vel_noise')

# Inputs
del_vel = Matrix(symbols('del_vel[0:3]'))

# States
x = Matrix(symbols('state[0:10]'))
nStates = len(x)

# position of the vehicle relative to the location the initial frame was taken, NED
vehicle_pos = x[0:3,0]
# velocity of the vehicle
vehicle_vel = x[3:6,0]
# position of the target relative to the location the initial frame was taken, inverse-scaled NED
target_pos_nedw = x[6:10,0]

target_pos_ned_rel_init = target_pos_nedw[0:3,0]*1/target_pos_nedw[3]
target_pos_ned_rel_vehicle = target_pos_ned_rel_init-vehicle_pos

# Covariance matrix
P = compressedSymmetricMatrix('cov', nStates)

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
    f = zeros(10,1)
    f[0:3,0] = vehicle_pos+vehicle_vel*dt
    f[3:6,0] = vehicle_vel+del_vel
    f[6:10,0] = x[6:10,0]

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

    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=subx_threshold)

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

    x_n,P_n,NIS,subx = extractSubexpressions([x_n,P_n,NIS],'subx',threshold=subx_threshold)

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
printFusion('los', target_pos_ned_rel_vehicle/vec_norm(target_pos_ned_rel_vehicle), 'scalar')
printFusion('velocity', vehicle_vel, 'vector')
sys.exit()
