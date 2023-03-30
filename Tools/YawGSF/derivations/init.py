from common import *

# Generate covariance initialization

accel_vector = Matrix(symbols('init_accel_vec((0:3))', real=True))
yaw_angle = Symbol('yaw_angle', real=True)
accel_sigma = Symbol('accel_sigma', real=True, nonnegative=True)
yaw_sigma = Symbol('yaw_sigma', real=True, nonnegative=True)

accel_unit_vector = accel_vector/accel_vector.norm()

roll = atan2(-accel_unit_vector[1],-accel_unit_vector[2])
pitch = asin(accel_unit_vector[0])

init_quat_sym = Matrix(symbols('init_quat((0:4))', real=True))
truth_quat_sym = Matrix(symbols('truth_quat(0:4)', real=True))
init_rot_err_sym = Matrix(symbols('init_rot_err((0:3))', real=True))

init_quat = quat_from_euler(roll,pitch,yaw_angle)

err_quat_influence = init_quat.jacobian(Matrix([accel_vector, [yaw_angle]]))
init_sigma = Matrix([accel_sigma, accel_sigma, accel_sigma, yaw_sigma])
err_quat_param_covariance = diag(*init_sigma.multiply_elementwise(init_sigma))
err_quat_cov = err_quat_influence*err_quat_param_covariance*err_quat_influence.T

truth_quat = quat_multiply(init_quat_sym, gibbs_to_quat(init_rot_err_sym))
init_rot_err_influence = truth_quat.jacobian(init_rot_err_sym).xreplace(dict(zip(init_rot_err_sym, zeros(3,1)))).pinv()
init_rot_err_cov = init_rot_err_influence*err_quat_cov*init_rot_err_influence.T

init_rot_err_cov = packSymmetricMatrix(init_rot_err_cov.xreplace(dict(zip(init_quat_sym, init_quat))))

rot_err_cov = packSymmetricMatrix(P[0:3,0:3])

subs = dict(zip(rot_err_cov, init_rot_err_cov))

P_n = P.xreplace(subs)

init_quat, P_n, subx = extractSubexpressions([init_quat,P_n], 'subx', threshold=4)


# Generate C code for quaternion covariance initialization
print('{ //////// Begin generated code: Attitude covariance initialization ////////')
for i in range(len(subx)):
    print('    float %s = %s;' % (subx[i][0], ccode(subx[i][1])))

print('')

for i in range(init_quat.rows):
    print('    quat[%u] = %s;' % (i, ccode(init_quat[i])))

print('')

for i in range(len(P_n)):
    if P_n[i] != P[i]:
        print('    P[%u] = %s;' % (i, ccode_double(P_n[i])))

print('} //////// End generated code: Attitude covariance initialization   ////////\n')
