from sympy import *
from sympy.solvers import solve
from helpers import *
from sys import exit
import math

# EKF to estimate target position and velocity relative to vehicle

# Goals:
# - Provide target height estimation (i.e. depth-from-motion) for cases where
#   there is no range finder or the target is on an elevated platform (like the
#   roof of a car)
# - Decouple precision landing performance from navigation performance and
#   potentially allow precision landings indoors.
# - Allow fusion of multiple sources of data relating to the target location
#   (e.g. if your computer vision algorithm provides an estimate of distance to
#   target based on its size in the frame)


# Parameters
dt = Symbol('dt')
Tbn = Matrix(3,3,symbols('Tbn[0:3][0:3]'))
cameraOffset = toVec(symbols('cam_ofs_x cam_ofs_y cam_ofs_z'))

vehicleVelNED_R = toVec(symbols('vv_n_R vv_e_R vv_d_R'))
terrainDistD_R_sca = Symbol('gnd_dist_d_R')
targetDist_R_sca = Symbol("target_dist_R")
initFocLen = Symbol("foc_len_init")
initFocLen_R = Symbol("foc_len_init_R")

terrainDistD_R = toVec(terrainDistD_R_sca)
targetDist_R = toVec(targetDist_R_sca)
targetCameraPos_R = Matrix(2,2,symbols('cam_pos_R[0:2][0:2]'))
targetCameraPos_R = copy_upper_to_lower_offdiagonals(targetCameraPos_R)

vehicleDelVelNED_noise = toVec(symbols('vdv_n_noise vdv_e_noise vdv_d_noise'))

# Observations
terrainDistD = Symbol('gnd_dist_d')
targetCameraPos = toVec(symbols('cam_pos_x cam_pos_y'))
vehicleVelNED = toVec(symbols('vv_n vv_e vv_d'))
targetDist = Symbol("target_dist")

# Inputs
vehicleDeltaVelocityNED = toVec(symbols('dvv_n dvv_e dvv_d'))

# States
# Parameterization: Target position is encoded as posNED = normalize(p_n, p_e, 1)/range_inv, and inverse range.
targetPosNED = toVec(symbols('pt_n pt_e pt_d'))
targetVelNED = toVec(symbols('vt_n vt_e vt_d'))
stateVector = toVec(targetPosNED, targetVelNED)

nStates = len(stateVector)

# Covariance matrix
P = Matrix(nStates,nStates,symbols('P[0:%u][0:%u]' % (nStates,nStates)))
P = copy_upper_to_lower_offdiagonals(P)

def deriveInitialization(jsonfile):
    print('Beginning initialization derivation')
    t1 = datetime.datetime.now()

    unitVecToTargetBody = toVec(targetCameraPos[0], targetCameraPos[1], 1.)
    unitVecToTargetBody /= vec_norm(unitVecToTargetBody)
    unitVecToTargetNED = Tbn*unitVecToTargetBody

    heightInit, heightInit_R = symbols('height_init height_init_R')

    initTargetPosNED = unitVecToTargetNED*heightInit/unitVecToTargetNED[2]
    initTargetVelNED = -vehicleVelNED

    # x_n: initial state
    x_n = toVec(initTargetPosNED, initTargetVelNED)

    assert x_n.shape == stateVector.shape

    # z: initialization measurement vector
    z = toVec(targetCameraPos, heightInit, vehicleVelNED)

    # R: covariance of additive noise on z
    R = diag(*toVec(zeros(2,1), heightInit_R, vehicleVelNED_R))
    assert R.shape[0] == R.shape[1] and R.shape[0] == z.shape[0]

    R[0:2,0:2] = targetCameraPos_R

    # H: initialization measurement influence matrix
    H = x_n.jacobian(z)

    # P_n: initial covariance
    P_n = H*R*H.T

    assert P_n.shape == P.shape

    # Optimizations
    P_n = upperTriangularToVec(P_n)
    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=1)

    # Output generation
    funcParams = {'Tbn': Tbn, 'cam_pos': targetCameraPos, 'hgt':heightInit, 'hgt_R': heightInit_R, 'cam_pos_R': upperTriangularToVec(targetCameraPos_R), 'vel': vehicleVelNED, 'vel_R': vehicleVelNED_R}

    funcs = {}

    funcs['subx'] = {}
    funcs['subx']['params'] = funcParams
    funcs['subx']['ret'] = toVec([x[1] for x in subx])
    funcs['subx']['retsymbols'] = toVec([x[0] for x in subx])

    funcParams = funcParams.copy()
    funcParams['subx'] = toVec([x[0] for x in subx])

    funcs['state'] = {}
    funcs['state']['params'] = funcParams
    funcs['state']['ret'] = x_n

    funcs['cov'] = {}
    funcs['cov']['params'] = funcParams
    funcs['cov']['ret'] = P_n

    check_funcs(funcs)

    saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    op_count, subx_count = getOpStats(funcs)
    t2 = datetime.datetime.now()
    print(('%s initialization: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,jsonfile,op_count,subx_count)))

def derivePrediction(jsonfile):
    print('Beginning prediction derivation')
    t1 = datetime.datetime.now()

    # account for changes in yaw angle since initialization
    yawRate, yawRateSigma = symbols('yaw_rate yaw_rate_sigma')
    yawRot = Rz(yawRate*dt)

    # States at time k+1
    targetPosNEDNew = yawRot*(targetPosNED+targetVelNED*dt)
    targetVelNEDNew = yawRot*(targetVelNED-vehicleDeltaVelocityNED)

    # f: state-transtition model
    f = simplify(toVec(targetPosNEDNew, targetVelNEDNew))


    assert f.shape == stateVector.shape

    # F: linearized state-transition model
    F = f.jacobian(stateVector)

    # u: control input vector
    u = toVec(vehicleDeltaVelocityNED, yawRate)

    # G: control-influence matrix, AKA "B" in literature
    G = f.jacobian(u)

    # w_u_sigma: additive noise on u
    w_u_sigma = toVec(vehicleDelVelNED_noise, yawRateSigma)

    # Q_u: covariance of additive noise on u
    Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

    # Q: covariance of additive noise on x
    Q = G*Q_u*G.T

    # P_n: covariance matrix at time k+1
    P_n = F*P*F.T + Q
    assert P_n.shape == P.shape

    x_n = f

    subs = {yawRate:0, yawRateSigma:math.radians(1.)}
    x_n = x_n.xreplace(subs)
    P_n = P_n.xreplace(subs)

    # Optimizations
    P_n = upperTriangularToVec(P_n)
    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=1)

    # Output generation
    funcParams = {'x':stateVector,'P':upperTriangularToVec(P),'u':vehicleDeltaVelocityNED,'w_u_sigma':vehicleDelVelNED_noise,'dt':dt}

    funcs = {}

    funcs['subx'] = {}
    funcs['subx']['params'] = funcParams
    funcs['subx']['ret'] = toVec([x[1] for x in subx])
    funcs['subx']['retsymbols'] = toVec([x[0] for x in subx])

    funcParams = funcParams.copy()
    funcParams['subx'] = toVec([x[0] for x in subx])

    funcs['state'] = {}
    funcs['state']['params'] = funcParams
    funcs['state']['ret'] = x_n

    funcs['cov'] = {}
    funcs['cov']['params'] = funcParams
    funcs['cov']['ret'] = P_n

    check_funcs(funcs)

    saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    op_count, subx_count = getOpStats(funcs)
    t2 = datetime.datetime.now()
    print(('%s prediction: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,jsonfile,op_count,subx_count)))

def deriveCameraRObs(jsonfile):
    print('Beginning cameraR derivation')
    t1 = datetime.datetime.now()

    scaleFactorX, scaleFactorX_R, scaleFactorY, scaleFactorY_R, alignX, alignY, alignZ, alignX_R, alignY_R, alignZ_R, timeDelay, timeDelay_R, gx, gy, gz = symbols('scale_x scale_x_R scale_y scale_y_R align_x align_y align_z align_x_R align_y_R align_z_R time_delay time_delay_R gx gy gz')

    gyro = toVec(gx, gy, gz)

    unitVecToTargetBody = toVec(targetCameraPos[0]*scaleFactorX, targetCameraPos[1]*scaleFactorY, 1.)
    unitVecToTargetBody /= vec_norm(unitVecToTargetBody)
    unitVecToTargetBody = quat_to_matrix(rot_vec_to_quat_approx(gyro*timeDelay)) * quat_to_matrix(rot_vec_to_quat_approx(toVec(alignX, alignY, alignZ))) * unitVecToTargetBody

    corrected_meas = toVec(unitVecToTargetBody[0:2])/unitVecToTargetBody[2]

    corrections = toVec(scaleFactorX, scaleFactorY, alignX, alignY, alignZ, timeDelay)

    corrections_R = diag(*toVec(scaleFactorX_R, scaleFactorY_R, alignX_R, alignY_R, alignZ_R, timeDelay_R))

    H = corrected_meas.jacobian(corrections)

    cov = H*corrections_R*H.T

    subs = {
        scaleFactorX:1,
        scaleFactorY:1,
        alignX:0,
        alignY:0,
        alignZ:0,
        timeDelay:0,
        scaleFactorX_R:0.02**2,
        scaleFactorY_R:0.02**2,
        alignX_R:math.radians(2.)**2,
        alignY_R:math.radians(2.)**2,
        alignZ_R:math.radians(5.)**2,
        timeDelay_R: 0.02**2
        }

    assert simplify(corrected_meas.xreplace(subs)-targetCameraPos) == zeros(2,1)

    cov = upperTriangularToVec(cov)
    cov = simplify(cov.xreplace(subs))
    cov,subx = extractSubexpressions([cov],'subx',threshold=1)

    # Output generation
    funcParams = {'z':targetCameraPos,'gyro':gyro}

    funcs = {}

    funcs['subx'] = {}
    funcs['subx']['params'] = funcParams
    funcs['subx']['ret'] = toVec([x[1] for x in subx])
    funcs['subx']['retsymbols'] = toVec([x[0] for x in subx])

    funcParams = funcParams.copy()
    funcParams['subx'] = toVec([x[0] for x in subx])

    funcs['R'] = {}
    funcs['R']['params'] = funcParams
    funcs['R']['ret'] = cov

    check_funcs(funcs)

    saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    op_count, subx_count = getOpStats(funcs)
    t2 = datetime.datetime.now()
    print(('%s cameraR: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,jsonfile,op_count,subx_count)))

def deriveCameraFusion(jsonfile):
    targetPosBody = Tbn.T*targetPosNED - cameraOffset
    measPred = toVec(targetPosBody[0]/targetPosBody[2], targetPosBody[1]/targetPosBody[2])

    deriveFusionSimultaneous('camera', jsonfile, measPred, additionalinputs={'Tbn':Tbn, 'ofs':cameraOffset}, R_type='matrix')

def deriveVelNEFusion(jsonfile):
    measPred = -targetVelNED[0:2,0]

    deriveFusionSimultaneous('velNE',jsonfile,measPred,subx_threshold=1)

def deriveVelDFusion(jsonfile):
    measPred = -targetVelNED[2:3,0]

    deriveFusionSimultaneous('velD',jsonfile,measPred,subx_threshold=1)

def deriveHeightFusion(jsonfile):
    measPred = targetPosNED[2:3,0]

    deriveFusionSimultaneous('height',jsonfile,measPred,subx_threshold=1)

def deriveFusionSimultaneous(fusionName,jsonfile,measPred,additionalinputs={},subs={},R_type='scalar',subx_threshold=10):
    assert isinstance(measPred,MatrixBase) and measPred.cols == 1
    print(('Beginning %s fusion derivation' % (fusionName,)))
    t1 = datetime.datetime.now()

    nObs = measPred.rows
    I = eye(nStates)

    # Define symbols
    z = toVec(symbols('z[0:%u]' % (nObs,))) # Measurement
    if R_type == 'matrix':
        R_param = copy_upper_to_lower_offdiagonals(Matrix(nObs,nObs, symbols('R[0:%u][0:%u]' % (nObs,nObs))))
        R = R_param
        R_param = upperTriangularToVec(R_param)
    elif R_type == 'vector':
        R_param = toVec(symbols('R[0:%u]' % (nObs,)))
        R = diag(*R_param)
    elif R_type == 'scalar':
        R_param = Symbol('R')
        R = eye(nObs)*R_param

    # Intermediates
    y = z-measPred                       # Innovation
    H = measPred.jacobian(stateVector)   # Obervation sensitivity matrix
    S = H*P*H.T + R                      # Innovation covariance
    S_I = quickinv_sym(S)                # Innovation covariance inverse
    K = P*H.T*S_I                        # Near-optimal Kalman gain

    y,H,S_I,K,temp_subx = extractSubexpressions([y,H,S_I,K],'temp')

    # Outputs

    # NOTE: The covariance update involves subtraction and can result in loss
    # of symmetry and positive definiteness due to rounding errors. Joseph's
    # form covariance update avoids this at the expense of computation burden.

    NIS = y.T*S_I*y                      # Normalized innovation squared
    x_n = stateVector+K*y                # Updated state vector
    P_n = (I-K*H)*P*(I-K*H).T+K*R*K.T    # Updated covariance matrix

    # Apply specified substitutions
    y = y.xreplace(subs)
    NIS = NIS.xreplace(subs)
    x_n = x_n.xreplace(subs)
    P_n = P_n.xreplace(subs)
    temp_subx = [(x[0], x[1].xreplace(subs)) for x in temp_subx]

    # Optimizations
    P_n = upperTriangularToVec(P_n)
    y, NIS, x_n, P_n, subx = extractSubexpressions([y,NIS,x_n,P_n],'subx',threshold=subx_threshold,prev_subx=temp_subx)

    funcParams = {'x':stateVector,'P':upperTriangularToVec(P),'R':R_param,'z':z}
    funcParams.update(additionalinputs)

    funcs = {}
    funcs['subx'] = {}
    funcs['subx']['params'] = funcParams
    funcs['subx']['ret'] = toVec([x[1] for x in subx])
    funcs['subx']['retsymbols'] = toVec([x[0] for x in subx])

    funcParams = funcParams.copy()
    funcParams['subx'] = toVec([x[0] for x in subx])

    funcs['innov'] = {}
    funcs['innov']['params'] = funcParams
    funcs['innov']['ret'] = y

    funcs['NIS'] = {}
    funcs['NIS']['params'] = funcParams
    funcs['NIS']['ret'] = NIS

    funcs['state'] = {}
    funcs['state']['params'] = funcParams
    funcs['state']['ret'] = x_n

    funcs['cov'] = {}
    funcs['cov']['params'] = funcParams
    funcs['cov']['ret'] = P_n


    saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    op_count, subx_count = getOpStats(funcs)
    t2 = datetime.datetime.now()
    print(('%s %s fusion: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,fusionName,jsonfile,op_count,subx_count)))

def getOpStats(funcs):
    op_count = sum([count_ops(x['ret']) for x in list(funcs.values())])
    subx_count = len(funcs['subx']['ret']) if 'subx' in funcs else 0
    return op_count, subx_count
