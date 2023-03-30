from common import *

# Generate tag observation model
other_pos = Matrix(symbols(r'other_pos((0:3)\,0)', real=True))
c = Symbol('SPEED_OF_LIGHT')

# Tag obs model
h_tag = Matrix([(pos-other_pos).norm()/c])
H_tag = h_tag.jacobian(x)

# Generate C code for tag obs model
h_tag, H_tag, subx = extractSubexpressions([h_tag[:16,:],H_tag[:,:16]], 'subx', threshold=4)

print '{ //////// Begin generated code: Tag observation model ////////'
for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

print ''

for i in range(len(h_tag)):
    print '    h_tag_row(%u,0) = %s;' % (i, ccode_double(h_tag[i]))

print ''

for i in range(len(H_tag)):
    if H_tag[i] != 0:
        print '    H_tag_row(0,%u) = %s;' % (i, ccode_double(H_tag[i]))

print '} //////// End generated code: Tag observation model   ////////\n'
