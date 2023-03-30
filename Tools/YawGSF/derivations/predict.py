from common import *

# Generate prediction model
dt = Symbol('imu_dt', real=True, nonnegative=True)
del_ang = Matrix(symbols('del_ang((0:3))', real=True))
del_vel = Matrix(symbols('del_vel((0:3))', real=True))
gbias_pnoise = Symbol('gbias_pnoise', real=True, nonnegative=True)
abias_pnoise_xy = Symbol('abias_pnoise_xy', real=True, nonnegative=True)
abias_pnoise_z = Symbol('abias_pnoise_z', real=True, nonnegative=True)
gscale_pnoise = Symbol('gscale_pnoise', real=True, nonnegative=True)
tsca_err_pnoise = Symbol('tsca_err_pnoise', real=True, nonnegative=True)
accel_sigma = Symbol('accel_sigma', real=True, nonnegative=True)
gyro_sigma = Symbol('gyro_sigma', real=True, nonnegative=True)
accel_scale_sigma = Symbol('accel_scale_sigma', real=True, nonnegative=True)
gyro_cross_sigma = Symbol('gyro_cross_sigma', real=True, nonnegative=True)
accel_cross_sigma = Symbol('accel_cross_sigma', real=True, nonnegative=True)
gravity = 9.80655
gravity_ned = Matrix([0,0,gravity])

del_ang_corrected = del_ang-gbias*dt
#del_ang_corrected = del_ang.multiply_elementwise(gscale+ones(3,1))-gbias*dt
del_vel_corrected = del_vel-abias*dt
del_vel_coordinate_ned = Tbn*del_vel_corrected+gravity_ned*dt

rot_err_new_approx = quat_to_gibbs(quat_rotate_approx(err_quat, del_ang_corrected))
rot_err_new = quat_to_gibbs(quat_rotate(err_quat, del_ang_corrected))

# f: state-transtition model for the purpose of linearization
f = Matrix([rot_err_new_approx, gbias, abias_z, vel+del_vel_coordinate_ned])

F = f.jacobian(x)

# u: control input vector
u = Matrix([del_ang, del_vel])

# G: control-influence matrix, AKA "B" in literature
G = f.jacobian(u)

del_ang_sigma = ones(3,1)*gyro_sigma*dt# + Matrix([[0, gyro_cross_sigma, gyro_cross_sigma], [gyro_cross_sigma, 0, gyro_cross_sigma], [gyro_cross_sigma, gyro_cross_sigma, 0]])*del_ang_corrected

del_vel_sigma = ones(3,1)*accel_sigma*dt# + Matrix([[accel_scale_sigma, accel_cross_sigma, accel_cross_sigma], [accel_cross_sigma, accel_scale_sigma, accel_cross_sigma], [accel_cross_sigma, accel_cross_sigma, accel_scale_sigma]])*del_vel_corrected

# w_u_sigma: additive noise on u
w_u_sigma = Matrix([del_ang_sigma, del_vel_sigma])

# Q_u: covariance of additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

# Q: covariance of additive noise on x
Q = G*Q_u*G.T

for sym in gbias:
    i = get_state_index(sym)
    Q[i,i] += (gbias_pnoise*dt)**2

#for sym in gscale:
    #i = get_state_index(sym)
    #Q[i,i] += (gscale_pnoise*dt)**2

for sym in abias_z:
    i = get_state_index(sym)
    if sym == abias[2]:
        Q[i,i] += (abias_pnoise_z*dt)**2
    else:
        Q[i,i] += (abias_pnoise_xy*dt)**2

f[0:3,:] = rot_err_new

f = f.xreplace(dict(zip(rot_err, zeros(3,1))))
F = F.xreplace(dict(zip(rot_err, zeros(3,1))))
Q = Q.xreplace(dict(zip(rot_err, zeros(3,1))))

P_n = F*P*F.T+Q

quat_n, f, P_n = derive_zero_rot_err(f,P_n)

P_n = packSymmetricMatrix(P_n)


#pprint(f)

# Generate C code for prediction model
quat_n, x_n, P_n, subx = extractSubexpressions([quat_n,f, P_n], 'subx', threshold=4)

print('{ //////// Begin generated code: Prediction model      ////////')
for i in range(len(subx)):
    print('    float %s = %s;' % (subx[i][0], ccode_float(subx[i][1])))

print('')

for i in range(len(x_n)):
    print('    x_n[%u] = %s;' % (i, ccode_float(x_n[i])))

print('')

for i in range(len(P_n)):
    print('    P_n[%u] = %s;' % (i, ccode_float(P_n[i])))

print('')

for i in range(len(quat_n)):
    print('    quat_n[%u] = %s;' % (i, ccode_float(P_n[i])))

print('} //////// End generated code: Prediction model        ////////\n')
