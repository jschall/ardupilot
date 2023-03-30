from common import *

pprint(P)

lat, lon = symbols('pos_lla(0) pos_lla(1)', real=True)

quat_ecef_to_ned = quat_ecef_to_ned_from_lat_lon(lat,lon)

err_quat = gibbs_to_quat(rot_err)

att_euler = quat_to_euler(quat_multiply(quat_inverse(quat_ecef_to_ned), quat_multiply(est_quat, err_quat)))

G = att_euler[2,:].jacobian(x)

heading = att_euler[2]
heading_sigma = sqrt((G*P*G.T)[0])

subs = dict(zip(rot_err,zeros(3,1)))

heading = heading.xreplace(subs)
heading_sigma = heading_sigma.xreplace(subs)
#pprint(heading_sigma)

heading, heading_sigma, subx = extractSubexpressions([heading, heading_sigma], 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_float(subx[i][1]))

print '    heading = %s;' % (ccode_float(heading),)
print '    heading_sigma = %s;' % (ccode_float(heading_sigma),)
