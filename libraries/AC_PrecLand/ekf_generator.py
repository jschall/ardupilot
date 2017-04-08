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


def printFusion(name, h):
    # z: observation
    z = Matrix(symbols('z[0:%u]' % (len(h),)))

    # R: observation covariance
    R = compressedSymmetricMatrix('R', len(h))

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

#def deriveSyntheticPos(jsonfile):
    #print('Beginning synthetic position derivation')
    #t1 = datetime.datetime.now()

    #align = toVec(symbols('align_x align_y align_z'))
    #align_R = toVec(symbols('align_x_R align_y_R align_z_R'))

    #los_unit_ned = toVec(symbols('los_n los_e los_d'))
    #height = Symbol('height')
    #height_R = Symbol('height_R')

    #synthetic_pos = los_unit_ned*height/los_unit_ned[2]

    #corrected_meas = quat_to_matrix(rot_vec_to_quat_approx(align)) * synthetic_pos
    #corrections = toVec(align, height)
    #corrections_R = diag(*toVec(align_R, height_R))

    #H = corrected_meas.jacobian(corrections)
    #synthetic_pos_R = H*corrections_R*H.T

    #subs = {
        #align[0]:0,
        #align[1]:0,
        #align[2]:0
        #}

    #synthetic_pos_R = simplify(upperTriangularToVec(synthetic_pos_R).xreplace(subs))

    ## Output generation
    #funcParams = {'align_R':align_R,'los_unit_ned':los_unit_ned,'height':height,'height_R':height_R}

    #funcs = {}

    ##funcs['subx'] = {}
    ##funcs['subx']['params'] = funcParams
    ##funcs['subx']['ret'] = toVec([x[1] for x in subx])
    ##funcs['subx']['retsymbols'] = toVec([x[0] for x in subx])

    ##funcParams = funcParams.copy()
    ##funcParams['subx'] = toVec([x[0] for x in subx])

    #funcs['z'] = {}
    #funcs['z']['params'] = funcParams
    #funcs['z']['ret'] = synthetic_pos

    #funcs['R'] = {}
    #funcs['R']['params'] = funcParams
    #funcs['R']['ret'] = synthetic_pos_R

    #check_funcs(funcs)

    #saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    #op_count, subx_count = getOpStats(funcs)
    #t2 = datetime.datetime.now()
    #print('%s synthetic position: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,jsonfile,op_count,subx_count))

def printSynthPos():
    los_m = Matrix(symbols('los_m[0:3]'))
    dist_m = Symbol('dist_m')

    los_R = Symbol('los_')

    synthetic_pos_m = los_m*dist_m



printPrediction()
printFusion('pos', pos)
printFusion('vertical velocity', vel[2:3,0])
sys.exit()


output_stream.write(
'// PREDICTION'
)



output_stream.write(
'#pragma once\n'
'\n'
)
output_stream.write(
'struct ekf_state_s {\n'
'    float x[%u];\n'
'    float P[%u];\n'
'    float innov[%u];\n'
'    float NIS;\n'
'};\n' % (nStates, len(P_p), len(y))
)
output_stream.write(
'\n'
'static struct ekf_state_s ekf_state[2];\n'
'static uint8_t ekf_idx = 0;\n'
'\n'
'static void ekf_init(const Vector3f& init_pos, const Matrix3f& init_pos_R, const Vector3f& init_vel, const Matrix3f& init_vel_R) {\n'
'    float* state = ekf_state[ekf_idx].x;\n'
'    float* cov = ekf_state[ekf_idx].P;\n'
'\n'
)

for i in range(len(init_x)):
    output_stream.write('    state[%u] = %s;\n' % (i, CCodePrinter_float().doprint(init_x[i])))

for i in range(len(init_P)):
    output_stream.write('    cov[%u] = %s;\n' % (i, CCodePrinter_float().doprint(init_P[i])))

output_stream.write(
'    memset(&ekf_state[ekf_idx], 0, sizeof(ekf_state[ekf_idx]));\n'
'    state[1] = init_theta;\n'
'}\n'
'\n'
)

output_stream.write('static float subx[%u];\n' % (max(len(pred_subx),len(fuse_subx)),))
output_stream.write(
'static void ekf_predict(float dt, Vector3f ) {\n'
'    uint8_t next_ekf_idx = (ekf_idx+1)%2;\n'
'    float* state = ekf_state[ekf_idx].x;\n'
'    float* cov = ekf_state[ekf_idx].P;\n'
'    float* state_n = ekf_state[next_ekf_idx].x;\n'
'    float* cov_n = ekf_state[next_ekf_idx].P;\n'
)

output_stream.write('    // %u operations\n' % (count_ops(x_p)+count_ops(P_p)+count_ops(pred_subx),))

for i in range(len(pred_subx)):
    output_stream.write('    %s = %s;\n' % (pred_subx[i][0], CCodePrinter_float().doprint(pred_subx[i][1])))

for i in range(len(x_p)):
    output_stream.write('    state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_p[i])))

for i in range(len(P_p)):
    output_stream.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_p[i])))

output_stream.write(
'\n'
'    ekf_idx = next_ekf_idx;\n'
'}\n'
)

output_stream.write(
'\n'
'static void ekf_update(float i_alpha_m, float i_beta_m) {\n'
'    uint8_t next_ekf_idx = (ekf_idx+1)%2;\n'
'    float* state = x_at_curr_meas;\n'
'    float* cov = ekf_state[ekf_idx].P;\n'
'    float* state_n = ekf_state[next_ekf_idx].x;\n'
'    float* cov_n = ekf_state[next_ekf_idx].P;\n'
'    float* innov = ekf_state[next_ekf_idx].innov;\n'
'    float* NIS = &ekf_state[next_ekf_idx].NIS;\n'
'\n'
)

output_stream.write('    // pos fusion\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(fuse_subx),))
output_stream.write('    // %u operations\n' % (count_ops(x_n)+count_ops(P_n)+count_ops(fuse_subx),))
for i in range(len(fuse_subx)):
    output_stream.write('    %s = %s;\n' % (fuse_subx[i][0], CCodePrinter_float().doprint(fuse_subx[i][1])))

for i in range(len(x_n)):
    output_stream.write('    state_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(x_n[i])))

for i in range(len(P_n)):
    output_stream.write('    cov_n[%u] = %s;\n' % (i, CCodePrinter_float().doprint(P_n[i])))

for i in range(len(y)):
    output_stream.write('    innov[%u] = %s;\n' % (i, CCodePrinter_float().doprint(y[i])))

output_stream.write('    *NIS = %s;\n' % (CCodePrinter_float().doprint(NIS[0]),))

output_stream.write(
'\n'
'    ekf_idx = next_ekf_idx;\n'
'}\n'
)





