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

# States
# The origin is the vehicle position at feature initialization. Thus, it is not included in the state vector.
# Vehicle orientation is not included in the state vector. Instead, the inertial navigation is relied upon
# to provide an accurate orientation.
vehiclePosNED = toVec(symbols('pv_n pv_e pv_d')) # Vehicle position in NED frame relative to feature initialization. Starts at 0,0,0.
vehicleVelNED = toVec(symbols('vv_n vv_e vv_d')) # Vehicle velocity in NED frame
targetPosHomogeneousNED = toVec(symbols('pt_n pt_e pt_d_inv')) # Target position relative to feature initialization, homogeneous.
stateVector = toVec(vehiclePosNED, vehicleVelNED, targetPosHomogeneousNED)

# Target position relative to vehicle, NED
targetPosNED = homogeneousNEDtoNED(targetPosHomogeneousNED)-vehiclePosNED

nStates = len(stateVector)

# Covariance matrix
P = Matrix(nStates,nStates,symbols('P[0:%u][0:%u]' % (nStates,nStates)))
P = copy_upper_to_lower_offdiagonals(P)

def deriveInitialization(jsonfile):
    print('Beginning initialization derivation')
    t1 = datetime.datetime.now()

    heightLowerBound, heightUpperBound = symbols('height_lower height_upper')
    invHeightInit, invHeightInit_R = symbols('inv_height_init inv_height_init_R')
    heightInit = 1/invHeightInit
    vehicleVelNEDObs = toVec(symbols('vv_n_obs vv_e_obs vv_d_obs'))
    vehicleVelNEDObs_R = toVec(symbols('vv_n_obs_R vv_e_obs_R vv_d_obs_R'))
    targetCameraPosObs = toVec(symbols('cam_pos_x_obs cam_pos_y_obs'))
    targetCameraPosObs_R = copy_upper_to_lower_offdiagonals(Matrix(2,2,symbols('cam_pos_R[0:2][0:2]')))

    unitVecToTargetBody = toVec(targetCameraPosObs[0], targetCameraPosObs[1], 1.)
    unitVecToTargetBody /= vec_norm(unitVecToTargetBody)
    unitVecToTargetNED = Tbn*unitVecToTargetBody

    initTargetPosNED = unitVecToTargetNED*heightInit/unitVecToTargetNED[2] + Tbn*cameraOffset

    initVehiclePosNED = toVec(0,0,0)
    initVehicleVelNED = vehicleVelNEDObs
    initTargetPosHomogeneousNED = NEDtoHomogeneousNED(initTargetPosNED)

    # x_n: initial state
    x_n = toVec(initVehiclePosNED, initVehicleVelNED, initTargetPosHomogeneousNED)

    assert x_n.shape == stateVector.shape

    # z: initialization measurement vector
    z = toVec(targetCameraPosObs, invHeightInit, vehicleVelNEDObs)

    # R: covariance of additive noise on z
    R = diag(*toVec(zeros(2,1), invHeightInit_R, vehicleVelNEDObs_R))
    assert R.shape[0] == R.shape[1] and R.shape[0] == z.shape[0]

    R[0:2,0:2] = targetCameraPosObs_R

    # H: initialization measurement influence matrix
    H = x_n.jacobian(z)

    # P_n: initial covariance
    P_n = H*R*H.T

    assert P_n.shape == P.shape

    # p0 and sigma_p0 are initialized so that (p0-2*p0_sigma, p0+2*p0_sigma) == (1/heightUpperBound, 1/heightLowerBound)
    p0_sym, p0_sigma_sym = symbols('p0, p0_sigma')
    soln = solve([p0_sym-2*p0_sigma_sym-1/heightUpperBound, p0_sym+2*p0_sigma_sym-1/heightLowerBound],(p0_sym, p0_sigma_sym))
    subs = {invHeightInit:soln[p0_sym], invHeightInit_R:soln[p0_sigma_sym]**2}

    x_n = x_n.xreplace(subs)
    P_n = P_n.xreplace(subs)

    # Optimizations
    P_n = upperTriangularToVec(P_n)
    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=1)

    # Output generation
    funcParams = {'Tbn': Tbn, 'cam_pos': targetCameraPosObs, 'hgtLwr':heightLowerBound, 'hgtUpr': heightUpperBound, 'cam_pos_R': upperTriangularToVec(targetCameraPosObs_R), 'vel': vehicleVelNEDObs, 'vel_R': vehicleVelNEDObs_R, 'cam_ofs':cameraOffset}

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

    # Inputs
    vehicleDeltaVelocityNED = toVec(symbols('dvv_n dvv_e dvv_d'))
    vehicleDelVelNEDSigma = toVec(symbols('vdv_n_sigma vdv_e_sigma vdv_d_sigma'))

    # The reference frame can be expected to change over time due to upstream estimator accuracy.
    frameRotRate = toVec(symbols('fr_x fr_y fr_z'))
    frameRotRateSigma = toVec(symbols('fr_x_sigma fr_y_sigma fr_z_sigma'))
    frameRot = quat_to_matrix(rot_vec_to_quat_approx(frameRotRate*dt))

    # States at time k+1
    vehicleVelNEDNew = frameRot*vehicleVelNED+vehicleDeltaVelocityNED
    vehiclePosNEDNew = frameRot*vehiclePosNED+vehicleVelNED*dt
    targetPosHomogeneousNEDNew = NEDtoHomogeneousNED(frameRot*homogeneousNEDtoNED(targetPosHomogeneousNED))
    # f: state-transtition model
    f = simplify(toVec(vehiclePosNEDNew, vehicleVelNEDNew, targetPosHomogeneousNEDNew))

    assert f.shape == stateVector.shape

    # F: linearized state-transition model
    F = f.jacobian(stateVector)

    # u: control input vector
    u = toVec(vehicleDeltaVelocityNED, frameRotRate)

    # G: control-influence matrix, AKA "B" in literature
    G = f.jacobian(u)

    # w_u_sigma: additive noise on u
    w_u_sigma = toVec(vehicleDelVelNEDSigma, frameRotRateSigma)

    # Q_u: covariance of additive noise on u
    Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

    # Q: covariance of additive noise on x
    Q = G*Q_u*G.T

    # P_n: covariance matrix at time k+1
    P_n = F*P*F.T + Q
    assert P_n.shape == P.shape

    x_n = f

    subs = dict(zip(frameRotRate, toVec(0,0,0))+zip(frameRotRateSigma, toVec(.003,.003,.03)))
    x_n = x_n.xreplace(subs)
    P_n = P_n.xreplace(subs)

    # Optimizations
    P_n = upperTriangularToVec(P_n)
    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=1)

    # Output generation
    funcParams = {'x':stateVector,'P':upperTriangularToVec(P),'u':vehicleDeltaVelocityNED,'w_u_sigma':vehicleDelVelNEDSigma,'dt':dt}

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

def deriveTargetPosCov(jsonfile):
    print('Beginning targetPosCov derivation')
    t1 = datetime.datetime.now()

    # H: sensitivity matrix
    H = targetPosNED.jacobian(stateVector)

    # P_n: covariance of additive noise on targetPosNED
    cov = H*P*H.T

    funcs = {}

    funcParams = {'x':stateVector,'P':upperTriangularToVec(P)}

    funcs['cov'] = {}
    funcs['cov']['params'] = funcParams
    funcs['cov']['ret'] = upperTriangularToVec(cov)

    check_funcs(funcs)

    saveExprsToJSON(jsonfile, {'funcs':serialize_exprs_in_structure(funcs.copy())})

    op_count, subx_count = getOpStats(funcs)
    t2 = datetime.datetime.now()
    print(('%s targetPosCov: derivation saved to %s. %u ops, %u subexpressions.' % (t2-t1,jsonfile,op_count,subx_count)))

def deriveCameraRObs(jsonfile):
    print('Beginning cameraR derivation')
    t1 = datetime.datetime.now()

    targetCameraPosObs = toVec(symbols('cam_pos_x_obs cam_pos_y_obs'))

    scaleFactorX, scaleFactorX_R, scaleFactorY, scaleFactorY_R, alignX, alignY, alignZ, alignX_R, alignY_R, alignZ_R, timeDelay, timeDelay_R, gx, gy, gz = symbols('scale_x scale_x_R scale_y scale_y_R align_x align_y align_z align_x_R align_y_R align_z_R time_delay time_delay_R gx gy gz')

    gyro = toVec(gx, gy, gz)

    unitVecToTargetBody = toVec(targetCameraPosObs[0]*scaleFactorX, targetCameraPosObs[1]*scaleFactorY, 1.)
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
        alignZ_R:math.radians(2.)**2,
        timeDelay_R: 0.02**2
        }

    assert simplify(corrected_meas.xreplace(subs)-targetCameraPosObs) == zeros(2,1)

    cov = upperTriangularToVec(cov)
    cov = simplify(cov.xreplace(subs))
    cov,subx = extractSubexpressions([cov],'subx',threshold=1)

    # Output generation
    funcParams = {'z':targetCameraPosObs,'gyro':gyro}

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
    measPred = targetPosBody[0:2,0]/targetPosBody[2]

    deriveFusionSimultaneous('camera', jsonfile, measPred, additionalinputs={'Tbn':Tbn, 'cam_ofs':cameraOffset}, R_type='matrix')

def deriveVelNEFusion(jsonfile):
    measPred = vehicleVelNED[0:2,0]

    deriveFusionSimultaneous('velNE',jsonfile,measPred,subx_threshold=1)

def deriveVelDFusion(jsonfile):
    measPred = vehicleVelNED[2:3,0]

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
