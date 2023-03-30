from math import sqrt, floor, fmod
from sympy import *
from sympy.printing.c import *
import math

QUAT_ORDER_WLAST = False

class C99CodePrinterTweaked(C99CodePrinter):
    def __init__(self, *args, **kwargs):
        self.float_precision = kwargs.pop('fprec', 'double')
        assert self.float_precision in ['single', 'double']

        self.use_fast_trig = kwargs.pop('use_fast_trig', False)
        C99CodePrinter.__init__(self, *args, **kwargs)

        self.sqrt_fname = 'sqrt' if self.float_precision == 'double' else 'sqrtf'
        self.pow_fname = 'pow' if self.float_precision == 'double' else 'powf'

        if self.float_precision == 'single':
            self.known_functions = {
                "Abs": [(lambda x: not x.is_integer, "fabsf")],
                "gamma": "tgammaf",
                "sin": "sinf" if not self.use_fast_trig else "sinf_fast",
                "cos": "cosf" if not self.use_fast_trig else "cosf_fast",
                "tan": "tanf",
                "asin": "asinf",
                "acos": "acosf",
                "atan": "atanf",
                "atan2": "atan2f",
                "exp": "expf",
                "log": "logf",
                "erf": "erff",
                "sinh": "sinhf",
                "cosh": "coshf",
                "tanh": "tanhf",
                "asinh": "asinhf",
                "acosh": "acoshf",
                "atanh": "atanhf",
                "floor": "floorf",
                "ceiling": "ceilf",
            }

    def _print_Pow(self, expr):
        if "Pow" in self.known_functions:
            return self._print_Function(expr)
        PREC = precedence(expr)

        invert = expr.exp < 0

        if expr.exp == -1:
            ret = self._print(expr.base)
        else:
            if invert:
                expr = 1/(expr)

            if expr.exp == 0.5:
                ret = '%s(%s)' % (self.sqrt_fname, self._print(expr.base))
            elif expr.exp.is_integer and expr.exp <= 4:
                ret = "(%s)" % ('*'.join(["(%s)" % (self._print(expr.base)) for _ in range(expr.exp)]),)
            else:
                ret = '%s(%s, %s)' % (self.pow_fname, self._print(expr.base), self._print(expr.exp))

        if invert:
            return '1/(%s)' % (ret,)
        else:
            return ret

    def _print_Piecewise(self, expr):
        if expr.args[-1].cond != True:
            # We need the last conditional to be a True, otherwise the resulting
            # function may not return a result.
            raise ValueError("All Piecewise expressions must contain an "
                             "(expr, True) statement to be used as a default "
                             "condition. Without one, the generated "
                             "expression may not evaluate to anything under "
                             "some condition.")
        lines = []
        if expr.has(Assignment):
            for i, (e, c) in enumerate(expr.args):
                if i == 0:
                    lines.append("if (%s) {" % self._print(c))
                elif i == len(expr.args) - 1 and c == True:
                    lines.append("else {")
                else:
                    lines.append("else if (%s) {" % self._print(c))
                code0 = self._print(e)
                lines.append(code0)
                lines.append("}")
            return " ".join(lines)
        else:
            # The piecewise was used in an expression, need to do inline
            # operators. This has the downside that inline operators will
            # not work for statements that span multiple lines (Matrix or
            # Indexed expressions).
            ecpairs = ["((%s) ? ( %s ) " % (self._print(c),
                                               self._print(e))
                    for e, c in expr.args[:-1]]
            last_line = ": ( %s )" % self._print(expr.args[-1].expr)
            return ": ".join(ecpairs) + last_line + " ".join([")"*len(ecpairs)])

    def _print_Float(self,flt):
        ret = C99CodePrinter._print_Float(self,flt)
        if self.float_precision == 'single':
            return ret+'f'
        return ret
    
    def _print_Rational(self, expr):
        p, q = int(expr.p), int(expr.q)
        if self.float_precision == 'single':
            return '%d.0f/%d.0f' % (p, q)
        else:
            return '%d.0/%d.0' % (p, q)

def ccode_float(*args, **kwargs):
    return C99CodePrinterTweaked(fprec='single').doprint(*args, **kwargs)

def ccode_double(*args, **kwargs):
    return C99CodePrinterTweaked(fprec='double').doprint(*args, **kwargs)

def skew(v):
    return Matrix([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def euler_from_matrix(m):
    return atan2(m[2,1], m[2,2]), -asin(m[2,0]), atan2(m[1,0],m[0,0])

def quat_from_euler(roll, pitch, yaw):
    ret = Matrix([
        sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2),
        cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2),
        cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2),
        cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        ])
    
    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]])

def quat_to_euler(q):
    if QUAT_ORDER_WLAST:
        qi = q[0]
        qj = q[1]
        qk = q[2]
        qr = q[3]
    else:
        qr = q[0]
        qi = q[1]
        qj = q[2]
        qk = q[3]

    return Matrix([
        atan2(2*(qr*qi+qj*qk), 1-2*(qi**2+qj**2)),
        asin(2*(qr*qj-qk*qi)),
        atan2(2*(qr*qk+qi*qj), 1-2*(qj**2+qk**2))
        ])

def vec_norm(v):
    return sqrt(sum([x**2 for x in v]))

def Rx(theta):
    return Matrix([[1, 0, 0],
                   [0,cos(theta), -sin(theta)],
                   [0,sin(theta),cos(theta)]])

def Ry(theta):
    return Matrix([[cos(theta), 0, sin(theta)],[0, 1, 0],[-sin(theta), 0, cos(theta)]])

def Rz(theta):
    return Matrix([[cos(theta), -sin(theta), 0],[sin(theta),cos(theta), 0], [0,0,1]])

def quat_to_gibbs(q):
    if QUAT_ORDER_WLAST:
        return q[0:3,:] / q[3]
    else:
        return q[1:4,:] / q[0]

def gibbs_to_quat(g):
    assert g.rows == 3

    ret = Matrix([
        [g[0]/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)],
        [g[1]/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)],
        [g[2]/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)],
        [   1/sqrt(g[0]**2 + g[1]**2 + g[2]**2 + 1)]
        ])
    
    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]])

def gibbs_multiply(g1,g2):
    return g1+g2-g2.cross(g1) / (1-g1.dot(g2))

def rot_vec_to_quat_approx(v):
    ret = Matrix([v[0]*Rational(1,2),v[1]*Rational(1,2),v[2]*Rational(1,2),1])
    
    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]])

def axis_angle_to_quat(theta, axis):
    ret = Matrix([sin(theta/2.) * axis[0], sin(theta/2.) * axis[1], sin(theta/2.) * axis[2], cos(theta/2.)])
    
    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]]) 
        

def quat_rotate_axis(q, theta, axis):
    return quat_multiply(q,axis_angle_to_quat(theta, axis))

def quat_rotate_x(q, theta):
    return quat_rotate_axis(q, theta, Matrix([1,0,0]))

def quat_rotate_y(q, theta):
    return quat_rotate_axis(q, theta, Matrix([0,1,0]))

def quat_rotate_z(q, theta):
    return quat_rotate_axis(q, theta, Matrix([0,0,1]))


def quat_rotate(q, v):
    return quat_multiply(q,rot_vec_to_quat(v))

def quat_rotate_approx(q, v):
    return quat_multiply(q,rot_vec_to_quat_approx(v))



def rot_vec_to_quat(v):
    theta = sqrt(v[0]**2+v[1]**2+v[2]**2)
    axis = v/Piecewise((theta, theta>0), (1, True))
    ret = Matrix([sin(theta/2.) * axis[0], sin(theta/2.) * axis[1], sin(theta/2.) * axis[2], cos(theta/2.)])

    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]]) 

def quat_to_rot_vec_approx(q):
    if QUAT_ORDER_WLAST:
        return 2.*Matrix([q[0],q[1],q[2]])
    else:
        return 2.*Matrix([q[1],q[2],q[3]])

def quat_to_rot_vec(q):
    if QUAT_ORDER_WLAST:
        q = Matrix([q[3],q[0],q[1],q[2]])

    theta = 2*acos(q[0])
    l = sqrt(q[1]**2+q[2]**2+q[3]**2)

    axis = Matrix([q[1],q[2],q[3]])/Piecewise((l, l>0), (1, True))

    return theta*axis

def quat_inverse(q):    
    if QUAT_ORDER_WLAST:
        q[0] = -q[0]
        q[1] = -q[1]
        q[2] = -q[2]
    else:
        q[1] = -q[1]
        q[2] = -q[2]
        q[3] = -q[3]
        
    return q

def quat_normalize(q):
    return q/sqrt(q[0]**2+q[1]**2+q[2]**2+q[3]**2)

def quat_multiply(q1, q2):
    if QUAT_ORDER_WLAST:
        q1i = q1[0]
        q1j = q1[1]
        q1k = q1[2]
        q1w = q1[3]

        q2i = q2[0]
        q2j = q2[1]
        q2k = q2[2]
        q2w = q2[3]
    else:
        q1w = q1[0]
        q1i = q1[1]
        q1j = q1[2]
        q1k = q1[3]

        q2w = q2[0]
        q2i = q2[1]
        q2j = q2[2]
        q2k = q2[3]

    ret = Matrix([q1w*q2i + q1i*q2w + q1j*q2k - q1k*q2j,
                  q1w*q2j - q1i*q2k + q1j*q2w + q1k*q2i,
                  q1w*q2k + q1i*q2j - q1j*q2i + q1k*q2w,
                  q1w*q2w - q1i*q2i - q1j*q2j - q1k*q2k])
    
    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]]) 

def quat_to_matrix(q):
    if QUAT_ORDER_WLAST:
        q_vec = q[0:3,0]
        q_w = q[3]
    else:
        q_vec = q[1:,0]
        q_w = q[0]
    return (q_w**2-(q_vec.T*q_vec)[0])*eye(3) + 2.*(q_vec*q_vec.T) + 2.*q_w*skew(q_vec)

def quat_from_matrix(m):
    S = 2*sqrt(m[0,0]+m[1,1]+m[2,2]+1)
    method1 = Matrix([(m[2,1] - m[1,2]) / S, (m[0,2] - m[2,0]) / S, (m[1,0] - m[0,1]) / S, S/4])

    S = sqrt(1 + m[0,0] - m[1,1] - m[2,2]) * 2
    method2 = Matrix([S/4, (m[0,1] + m[1,0]) / S, (m[0,2] + m[2,0]) / S, (m[2,1] - m[1,2]) / S])

    S = sqrt(1 + m[1,1] - m[0,0] - m[2,2]) * 2
    method3 = Matrix([(m[0,1] + m[1,0]) / S, S/4, (m[1,2] + m[2,1]) / S, (m[0,2] - m[2,0]) / S])

    S = sqrt(1 + m[2,2] - m[0,0] - m[1,1]) * 2
    method4 = Matrix([((m[0,2] + m[2,0]) / S, (m[1,2] + m[2,1]) / S, S/4, m[1,0] - m[0,1]) / S])

    criteria1 = m[0,0]+m[1,1]+m[2,2] > 0
    criteria2 = (m[0,0]>m[1,1]) & (m[0,0]>m[2,2])
    criteria3 = m[1,1] > m[2,2]
    criteria4 = True

    ret = zeros(4,1)
    for i in range(4):
        ret[i] = Piecewise((method1[i], criteria1), (method2[i], criteria2), (method3[i], criteria3), (method4[i], criteria4))

    if QUAT_ORDER_WLAST:
        return ret
    else:
        return Matrix([ret[3],ret[0],ret[1],ret[2]])

def packSymmetricMatrix(M):
    assert M.rows == M.cols

    N = M.rows
    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))
    return Matrix([M[r(k),c(k)] for k in range((N**2-N)//2+N)])

def unpackSymMatrix(M):
    x = len(M)
    N = int(floor(sqrt(8*x + 1)/2 - 1/2))
    ret = zeros(N)
    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))
    for k in range(x):
        ret[r(k),c(k)] = ret[c(k),r(k)] = M[k]
    return ret


def copy_upper_to_lower_offdiagonals(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols

    ret = M[:,:]

    for r in range(ret.rows):
        for c in range(ret.cols):
            if r > c:
                ret[r,c] = ret[c,r]
    return ret

def compressedSymmetricMatrix(prefix, N):
    ret = zeros(N,N)

    r = lambda k: int(floor((2*N+1-sqrt((2*N+1)*(2*N+1)-8*k))/2))
    c = lambda k: int(k - N*r(k) + r(k)*(r(k)-1)/2 + r(k))

    for k in range((N**2-N)//2+N):
        ret[r(k),c(k)] = ret[c(k),r(k)] = Symbol('%s[%u]' % (prefix,k))
    return ret

def count_subexpression(subexpr, expr):
    if hasattr(expr, "__getitem__"):
        return sum(map(lambda x: count_subexpression(subexpr, x), expr))
    else:
        return expr.count(subexpr)

def extractSubexpressions(inexprs, prefix='X', threshold=0, prev_subx=[]):
    subexprs, outexprs = cse(inexprs, symbols=numbered_symbols('__TMP__'), order='none')

    subexprs = prev_subx+subexprs

    for i in reversed(range(len(subexprs))):
        from sympy.logic.boolalg import Boolean
        ops_saved = (count_subexpression(subexprs[i][0], [[x[1] for x in subexprs], outexprs])-1)*subexprs[i][1].count_ops()
        if ops_saved < threshold or isinstance(subexprs[i][1], Boolean):
            sub = dict([subexprs.pop(i)])
            subexprs = list(map(lambda x: (x[0],x[1].xreplace(sub)), subexprs))
            outexprs = list(map(lambda x: x.xreplace(sub), outexprs))

    for i in range(len(subexprs)):
        newSym = Symbol('%s%u' % (prefix,i+len(prev_subx)))
        sub = {subexprs[i][0]:newSym}
        subexprs[i] = (newSym,subexprs[i][1])
        subexprs = list(map(lambda x: (x[0],x[1].xreplace(sub)), subexprs))
        outexprs = list(map(lambda x: x.xreplace(sub), outexprs))

    outexprs = list(map(lambda x: Matrix(x) if type(x) is ImmutableDenseMatrix else x, outexprs))

    return tuple(outexprs+[subexprs])

def quickinv_sym(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols
    n = M.rows
    A = Matrix(n,n,symbols('_X[0:%u][0:%u]' % (n,n)))
    A = copy_upper_to_lower_offdiagonals(A)
    B = Matrix(simplify(Matrix(A.cholesky_solve(eye(A.rows,A.cols)))))
    B = copy_upper_to_lower_offdiagonals(B)

    return B.xreplace(dict(zip(A,M)))

def wrap_pi(x):
    while x >= math.pi:
        x -= 2*math.pi

    while x < -math.pi:
        x += 2*math.pi

    return x

def UDUdecomposition(M):
    assert M.rows == M.cols
    assert M.is_symmetric()

    P = M[:,:]

    n = P.rows

    U = zeros(*P.shape)
    D = zeros(*P.shape)

    for j in range(n-1, 0, -1):
        D[j,j] = P[j,j]
        alpha = 1/D[j,j]
        for k in range(j):
            beta = P[k,j]
            U[k,j] = alpha*beta
            for i in range(k+1):
                P[i,k] = P[i,k]-beta*U[i,j]
    D[0,0] = P[0,0]
    for i in range(n):
        U[i,i] = 1

    return U,D

def ecef2lla(pos_ecef):
    # Michael Kleder (2020). Convert Cartesian (ECEF) Coordinates to lat, lon, alt (https://www.mathworks.com/matlabcentral/fileexchange/7941-convert-cartesian-ecef-coordinates-to-lat-lon-alt), MATLAB Central File Exchange. Retrieved May 30, 2020.
    x = pos_ecef[0]
    y = pos_ecef[1]
    z = pos_ecef[2]

    a = 6378137
    e = 8.1819190842622e-2

    # calculations:
    b   = sqrt(a**2*(1-e**2))
    ep  = sqrt((a**2-b**2)/b**2)
    p   = sqrt(x**2+y**2)
    th  = atan2(a*z,b*p)
    lon = atan2(y,x)
    lat = atan2((z+ep**2*b*sin(th)**3),(p-e**2*a*cos(th)**3))
    N   = a/sqrt(1-e**2*sin(lat)**2)
    alt = p/cos(lat)-N


    # correct for numerical instability in altitude near exact poles:
    # (after this correction, error is about 2 millimeters, which is about
    # the same as the numerical precision of the overall function)

    alt = Piecewise((alt, (abs(x)>1) & (abs(y)>1)), (abs(z)-b, True))

    return lat, lon, alt

def lla2ecef(lat, lon, alt):
    # Michael Kleder (2020). Covert lat, lon, alt to ECEF Cartesian (https://www.mathworks.com/matlabcentral/fileexchange/7942-covert-lat-lon-alt-to-ecef-cartesian), MATLAB Central File Exchange. Retrieved May 30, 2020.
    a = 6378137
    e = 8.181919084261345e-2
    e_sq = e**2
    b_sq = a**2*(1 - e_sq)

    N = a/sqrt(1 - e_sq*sin(lat)**2)
    x = (N+alt)*cos(lat)*cos(lon)
    y = (N+alt)*cos(lat)*sin(lon)
    z = ((b_sq/a**2)*N+alt)*sin(lat)

    return Matrix([x,y,z])

def rotation_ecef_to_ned_from_lat_lon(lat, lon):
    return Matrix([[-sin(lat)*cos(lon), -sin(lon), -cos(lat)*cos(lon)],
                   [-sin(lat)*sin(lon), cos(lon), -cos(lat)*sin(lon)],
                   [cos(lat), 0, -sin(lat)]
                   ])


def quat_ecef_to_ned_from_lat_lon(lat, lon):
    if QUAT_ORDER_WLAST:
        q = Matrix([0,0,0,1])
    else:
        q = Matrix([1,0,0,0])
        
    return quat_rotate_y(quat_rotate_x(quat_rotate_y(q, -pi/2), lon), -lat)

def quat_ecef_to_ned_from_ecef(pos_ecef):
    lat, lon, alt = ecef2lla(pos_ecef)
    
    return quat_ecef_to_ned_from_lat_lon(lat, lon)

def rotation_ecef_to_ned_from_ecef(pos_ecef):
    lat, lon, alt = ecef2lla(pos_ecef)
    return rotation_ecef_to_ned_from_lat_lon(lat, lon)

def gravitation_ecef(pos_ecef):
    earth_omega = Matrix([0,0,7.2921159e-5])

    x = pos_ecef[0]
    y = pos_ecef[1]
    z = pos_ecef[2]

    J2 = .00108263
    mu = 3.986004418e14
    R = 6378137
    r = sqrt(x**2+y**2+z**2)
    sub1 = 1.5*J2*(R/r)**2
    sub2 = 5*z**2/r**2
    sub3 = -mu/r**3
    sub4 = sub3*(1-sub1*(sub2-1))

    return Matrix([x * sub4,
                   y * sub4,
                   z * sub3*(1-sub1*(sub2-3))])

def coriolis_ecef(vel_ecef):
    omega = Matrix([0,0,7.2921159e-5])
    coriolis = -2*omega.cross(vel_ecef)
    return coriolis

def centrifugal_ecef(pos_ecef):
    omega = Matrix([0,0,7.2921159e-5])
    centrifugal = -omega.cross(omega.cross(pos_ecef))
    return centrifugal

def gravity_ecef(pos_ecef, vel_ecef=zeros(3,1)):
    return gravitation_ecef(pos_ecef) + centrifugal_ecef(pos_ecef) + coriolis_ecef(vel_ecef)

def quat_from_vectors(inertial_aligned, body_aligned, inertial_planar, body_planar):
    inertial_aligned = inertial_aligned.normalized()
    body_aligned = body_aligned.normalized()
    inertial_planar = inertial_planar.normalized()
    body_planar = body_planar.normalized()

    inertial_dcm = zeros(3,3)
    body_dcm = zeros(3,3)

    inertial_dcm[0:3,0] = inertial_aligned
    inertial_dcm[0:3,1] = inertial_aligned.cross(inertial_planar).normalized()
    inertial_dcm[0:3,2] = inertial_aligned.cross(inertial_aligned.cross(inertial_planar).normalized()).normalized()

    body_dcm[0:3,0] = body_aligned
    body_dcm[0:3,1] = body_aligned.cross(body_planar).normalized()
    body_dcm[0:3,2] = body_aligned.cross(body_aligned.cross(body_planar).normalized()).normalized()

    return quat_from_matrix(body_dcm.T*inertial_dcm)

