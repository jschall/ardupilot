from common import *

pos_ecef = Matrix(symbols('pos_ecef((0:3))', real=True))


print '\n\n###### ecef2lla ######'
lat, lon, alt, subx = extractSubexpressions(ecef2lla(pos_ecef), 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

print '    lat = %s;' % (ccode_double(lat),)
print '    lon = %s;' % (ccode_double(lon),)
print '    alt = %s;' % (ccode_double(alt),)

print '\n\n###### lla2ecef ######'

lat, lon, alt = symbols('lat lon alt', real=True)
pos_ecef, subx = extractSubexpressions([lla2ecef(lat, lon, alt)], 'subx', threshold=4)


for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(len(pos_ecef)):
    print '    ret(%u) = %s;' % (i, ccode_double(pos_ecef[i]))

print '\n\n###### gravity_ecef ######'

pos_ecef = Matrix(symbols('pos_ecef((0:3))', real=True))
vel_ecef = Matrix(symbols('vel_ecef((0:3))', real=True))
gravity_ecef, subx = extractSubexpressions([gravity_ecef(pos_ecef, vel_ecef)], 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(len(pos_ecef)):
    print '    ret(%u) = %s;' % (i, ccode_double(gravity_ecef[i]))

print '\n\n###### ecef2ned_from_lat_lon ######'

lat, lon, alt = symbols('lat lon alt', real=True)
ecef2ned_from_lat_lon, subx = extractSubexpressions([rotation_ecef_to_ned_from_lat_lon(lat, lon)], 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(ecef2ned_from_lat_lon.rows):
    for j in range(ecef2ned_from_lat_lon.cols):
        print '    ret(%u, %u) = %s;' % (i, j, ccode_double(ecef2ned_from_lat_lon[i,j]))


print '\n\n###### quat_ecef2ned_from_lat_lon ######'

lat, lon, alt = symbols('lat lon alt', real=True)
quat_ecef2ned_from_ecef, subx = extractSubexpressions([quat_ecef_to_ned_from_lat_lon(lat,lon)], 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(4):
    print '    ret.%s() = %s;' % ('wxyz'[i], ccode_double(quat_ecef2ned_from_ecef[i]))

print '\n\n###### ecef2ned_from_ecef ######'

pos_ecef = Matrix(symbols('pos_ecef((0:3))', real=True))
ecef2ned_from_ecef, subx = extractSubexpressions([rotation_ecef_to_ned_from_ecef(pos_ecef)], 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(ecef2ned_from_ecef.rows):
    for j in range(ecef2ned_from_ecef.cols):
        print '    ret(%u, %u) = %s;' % (i, j, ccode_double(ecef2ned_from_ecef[i,j]))

print '\n\n###### quat_ecef2ned_from_ecef ######'

pos_ecef = Matrix(symbols('pos_ecef((0:3))', real=True))
quat_ecef2ned_from_ecef, subx = extractSubexpressions([quat_ecef_to_ned_from_ecef(pos_ecef)], 'subx', threshold=4)

for i in range(len(subx)):
    print '    auto %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(4):
    print '    ret.%s() = %s;' % ('wxyz'[i], ccode_double(quat_ecef2ned_from_ecef[i]))
