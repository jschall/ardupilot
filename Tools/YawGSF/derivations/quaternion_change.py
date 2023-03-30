from common import *

new_quat_sym = Matrix(symbols('new_quat.w() new_quat.x() new_quat.y() new_quat.z()',real=True))


del_gibbs = Matrix(symbols('del_gibbs((0:3))', real=True))
rot_err_new = gibbs_multiply(-del_gibbs, rot_err)

f = x[:,:]
f = f.xreplace(dict(zip(rot_err, rot_err_new)))

F = f.jacobian(x)

del_quat_sym = Matrix(symbols('del_quat((0:4))',real=True))
subs = solve(quat_multiply(est_quat,del_quat_sym)-new_quat_sym, del_quat_sym)

del_quat = del_quat_sym.xreplace(subs)
subs.update(dict(zip(del_gibbs, quat_to_gibbs(del_quat))))

f = f.xreplace(subs)
F = F.xreplace(subs)

f, F, subx = extractSubexpressions([f, F], 'subx', threshold=4)

for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(3):
    print '    f(%u) = %s;' % (i, ccode_double(f[i]))

for i in range(3):
    for j in range(3):
        print '    F(%u,%u) = %s;' % (i, j, ccode_double(F[i,j]))
