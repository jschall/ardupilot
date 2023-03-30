from common import *

velNE_sigma = Matrix(symbols('velNE_sigma velNE_sigma', real=True))


z = Matrix(symbols('velN_obs velE_obs',real=True))
R = diag(*velNE_sigma.multiply_elementwise(velNE_sigma))
h = vel[0:2,:]
H = h.jacobian(x)
y = z-h
S = H*P*H.T + R

S_I = quickinv_sym(S)

NIS = y.T*S_I*y

K = P*H.T*S_I
x_n = x+K*y
P_n = (eye(n_states,n_states)-K*H)*P

quat_n, x_n, P_n = derive_zero_rot_err(x_n,P_n)

P_n = packSymmetricMatrix(P_n)

quat_n, x_n, P_n, subx = extractSubexpressions([quat_n, x_n, P_n], 'subx', threshold=4)


print('{ //////// Begin generated code: Fuse velocity      ////////')
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
print('} //////// End generated code: Fuse velocity        ////////\n')
