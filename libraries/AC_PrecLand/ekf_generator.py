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
del_vel_sigma = Symbol('del_vel_sigma')

# Inputs
del_vel = Matrix(symbols('del_vel[0:3]'))

# States
x = Matrix(symbols('state[0:10]'))
nStates = len(x)

# position of the vehicle relative to the location the initial frame was taken, NED
vehicle_pos = x[0:3,0]
# velocity of the vehicle, NED
vehicle_vel = x[3:6,0]
# normalized position of the target relative to the location the initial frame was taken, NED
target_pos_ned_unit = x[6:9,0]
# inverse of target distance from the location the initial frame was taken
rho = x[9]

target_pos_ned_rel_init = target_pos_ned_unit*1/rho
target_pos_ned_rel_vehicle = target_pos_ned_unit*1/rho - vehicle_pos

# Covariance matrix
P = compressedSymmetricMatrix('cov', nStates)

def printPos():
    output_stream.write('// TARGET POS RELATIVE TO VEHICLE\n')
    for i in range(len(target_pos_ned_rel_vehicle)):
        output_stream.write('ret[%u] = %s;\n' % (i, CCodePrinter_float().doprint(target_pos_ned_rel_vehicle[i])))
    output_stream.write('\n\n')

def printVel():
    output_stream.write('// TARGET VEL RELATIVE TO VEHICLE\n')
    for i in range(len(vehicle_vel)):
        output_stream.write('ret[%u] = %s;\n' % (i, CCodePrinter_float().doprint(-vehicle_vel[i])))
    output_stream.write('\n\n')

def printInitialization():
    min_d = 2
    vel = Matrix(symbols('vel[0:3]'))
    vel_sigma = Matrix(symbols('vel_xy_sigma vel_xy_sigma vel_z_sigma'))
    vel_var = vel_sigma.multiply_elementwise(vel_sigma)

    los_unit_ned = Matrix(symbols('los_unit_ned[0:3]'))
    align = Matrix(symbols('align[0:3]'))
    align_sigma = Matrix(symbols('align_xy_sigma align_xy_sigma align_z_sigma'))
    align_var = align_sigma.multiply_elementwise(align_sigma)

    inv_scale = Symbol('inv_scale')
    inv_scale_var = Symbol('inv_scale_var')

    x_n = zeros(10,1)
    x_n[3:6,0] = vel
    x_n[6:9,0] = quat_to_matrix(rot_vec_to_quat_approx(align)) * los_unit_ned
    x_n[9] = inv_scale

    params = toVec(align, inv_scale, vel)
    params_R = diag(*toVec(align_var, inv_scale_var, vel_var))

    H = x_n.jacobian(params)
    P_n = H*params_R*H.T

    x_n = x_n.xreplace(dict(zip(align, zeros(3,1)))).xreplace({inv_scale: 1./(2*min_d), inv_scale_var: (1./(4*min_d))**2})
    P_n = P_n.xreplace(dict(zip(align, zeros(3,1)))).xreplace({inv_scale: 1./(2*min_d), inv_scale_var: (1./(4*min_d))**2})
    P_n = upperTriangularToVec(P_n)

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
    w_u_sigma = Matrix([del_vel_sigma, del_vel_sigma, del_vel_sigma])

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

def printLOSFusion():
    los_unit_ned = Matrix(symbols('los_unit_ned[0:3]'))
    align = Matrix(symbols('align[0:3]'))
    align_sigma = Matrix(symbols('align_xy_sigma align_xy_sigma align_z_sigma'))
    align_var = align_sigma.multiply_elementwise(align_sigma)

    params = toVec(align)
    params_R = diag(*toVec(align_var))

    z = quat_to_matrix(rot_vec_to_quat_approx(align)) * los_unit_ned
    H = z.jacobian(params)

    R = H*params_R*H.T

    z = z.xreplace(dict(zip(align, zeros(3,1))))
    R = R.xreplace(dict(zip(align, zeros(3,1))))

    h = -vehicle_pos*rho + target_pos_ned_unit

    z *= vec_norm(-vehicle_pos*rho + target_pos_ned_unit)

    printFusion("los", h, z, R)

def printVelFusion():
    vel = Matrix(symbols('vel[0:3]'))
    vel_sigma = Matrix(symbols('vel_xy_sigma vel_xy_sigma vel_z_sigma'))
    vel_var = vel_sigma.multiply_elementwise(vel_sigma)

    h = vehicle_vel
    z = vel
    R = diag(*vel_var)

    printFusion("vel", h, z, R)

def printVelZFusion():
    vel_z = Matrix([Symbol('vel_z')])
    vel_z_sigma = Matrix([Symbol('vel_z_sigma')])
    vel_z_var = vel_z_sigma.multiply_elementwise(vel_z_sigma)

    h = vehicle_vel[2:3,0]
    z = vel_z
    R = diag(*vel_z_var)

    printFusion("vel_z", h,z,R)

def printFusion(name, h, z, R):
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
printVelFusion()
printLOSFusion()
printPos()
printVel()
printVelZFusion()
