from common import *

quat_sym = Matrix(symbols('qw qx qy qz', real=True))
yaw_angle_sym = Symbol('yaw_angle', real=True)
down_vec_body_sym = Matrix(symbols('down_vec_body((0:3))', real=True))

eqns = []

yaw_angle = quat_to_euler(quat_sym)[2]
Tbn = quat_to_matrix(quat_sym)

down_vec_body = Tbn.T * Matrix([0,0,1])


eqns.append(Eq(quat_sym.norm()**2,1))
eqns.append(Eq(down_vec_body_sym.norm()**2,1))
eqns.append(Eq(yaw_angle_sym,yaw_angle))

for i in range(3):
    eqns.append(Eq(down_vec_body_sym[i],down_vec_body[i]))


for eq in eqns:
    pprint(eq)
    print('\n\n')

pprint(solve(eqns, [quat_sym]))
