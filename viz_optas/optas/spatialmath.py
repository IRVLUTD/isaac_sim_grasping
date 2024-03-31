"""! @brief This is a partial port to Python/CasADi, with some modifications and additions, of the Spatial Math Toolbox for MATLAB. See the following. https://github.com/petercorke/spatialmath-matlab """

import array
import functools
import casadi as cs
from casadi import sin, cos, vec
from typing import List, Dict, Tuple, Union, Callable

## Accepted array types.
ArrayType = Union[cs.DM, cs.SX, cs.MX, List[float], Tuple[float], cs.np.ndarray, float, int]

## CasADi array types typically returned by OpTaS methods.
CasADiArrayType = Union[cs.DM, cs.SX, cs.MX]

## The number pi (i.e. 3.141...).
pi = cs.np.pi

## The machine epsilon.
eps = cs.np.finfo(float).eps


def arrayify_args(fun: Callable) -> Callable:
    """! Decorator that ensures all input arguments are casadi arrays (i.e. either DM or SX).

    @param fun Callable function
    @return Wrapper that ensures the input to the given function are casadi arrays.
    """

    _arraylike_types = (cs.DM, cs.SX, list, tuple, cs.np.ndarray, float, int)

    def _handle_arraylike_args(args, handle):
        """Helper method that applies the handle to array like arguments."""
        args_out = []

        for a in args:
            if isinstance(a, _arraylike_types):
                args_out.append(handle(a))
            else:
                args_out.append(a)
        return args_out

    def _handle_arraylike_kwargs(kwargs, handle, default_kwargs):
        kwargs_out = {}

        for label, value in kwargs.items():
            if isinstance(value, _arraylike_types):
                kwargs_out[label] = handle(value)
            else:
                kwargs_out[label] = value

        for label, default_value in default_kwargs.items():
            if label not in kwargs_out and isinstance(default_value, _arraylike_types):
                kwargs_out[label] = handle(default_value)

        return kwargs_out

    @functools.wraps(fun)
    def wrap(*args, **kwargs):
        # Extract default values for kwargs from fun
        arg_names = fun.__code__.co_varnames[: fun.__code__.co_argcount]
        arg_defaults = fun.__defaults__
        if arg_defaults is not None:
            default_kwargs = dict(zip(arg_names[-len(arg_defaults) :], arg_defaults))
        else:
            default_kwargs = {}

        args_use = _handle_arraylike_args(args, cs.horzcat)
        kwargs_use = _handle_arraylike_kwargs(kwargs, cs.horzcat, default_kwargs)
        return fun(*args_use, **kwargs_use)

    return wrap


def I3() -> cs.DM:
    """! 3-by-3 identity matrix.

    @return Identity matrix of order 3.
    """
    return cs.DM.eye(3)


def I4() -> cs.DM:
    """! 4-by-4 identity matrix.

    @return Identity matrix of order 4.
    """
    return cs.DM.eye(4)


@arrayify_args
def angvec2r(theta: ArrayType, v: ArrayType) -> CasADiArrayType:
    """! Convert angle and vector orientation to a rotation matrix. This method uses Rodrigue's formula.

    @param theta Angle of rotation (radians).
    @param v Direction vector to rotate about.
    @return Rotation matrix.
    """
    sk = skew(unit(v))
    R = I3() + sin(theta) * sk + (1.0 - cos(theta)) * sk @ sk  # Rodrigue's equation
    return R


@arrayify_args
def r2t(R: ArrayType) -> CasADiArrayType:
    """! Convert rotation matrix to a homogeneous transform.

    @param R A 3-by-3 rotation matrix.
    @return Homogenous transformation with given rotation and zero translation.
    """
    return cs.vertcat(
        cs.horzcat(R, cs.DM.zeros(3)),
        cs.DM([[0.0, 0.0, 0.0, 1]]),
    )


@arrayify_args
def rotx(theta: ArrayType) -> CasADiArrayType:
    """! SO(3) rotation about X axis.

    @param theta Angle of rotation (radians).
    @return A 3-by-3 rotation matrix.
    """
    ct, st = cos(theta), sin(theta)
    return cs.vertcat(
        cs.DM([[1.0, 0.0, 0.0]]),
        cs.horzcat(0.0, ct, -st),
        cs.horzcat(0, st, ct),
    )


@arrayify_args
def roty(theta: ArrayType) -> CasADiArrayType:
    """! SO(3) rotation about Y axis.

    @param theta Angle of rotation (radians).
    @return A 3-by-3 rotation matrix.
    """
    ct, st = cos(theta), sin(theta)
    return cs.vertcat(
        cs.horzcat(ct, 0.0, st),
        cs.DM([[0.0, 1.0, 0.0]]),
        cs.horzcat(-st, 0.0, ct),
    )


@arrayify_args
def rotz(theta: ArrayType) -> CasADiArrayType:
    """! SO(3) rotation about Z axis.

    @param theta Angle of rotation (radians).
    @return A 3-by-3 rotation matrix.
    """
    ct, st = cos(theta), sin(theta)
    return cs.vertcat(
        cs.horzcat(ct, -st, 0.0),
        cs.horzcat(st, ct, 0.0),
        cs.DM([[0.0, 0.0, 1.0]]),
    )

@arrayify_args
def standoff(offset: ArrayType, axis: str = "x") -> CasADiArrayType:
    # define the standoff pose
    if axis == 'x':
        return cs.vertcat(
            cs.horzcat(1.0, 0.0, 0.0, offset),
            cs.DM([[0.0, 1.0, 0.0, 0.0]]),
            cs.DM([[0.0, 0.0, 1.0, 0.0]]),
            cs.DM([[0.0, 0.0, 0.0, 1.0]]),
        )
    elif axis == 'y':
        return cs.vertcat(
            cs.DM([[1.0, 0.0, 0.0, 0.0]]),
            cs.horzcat(0.0, 1.0, 0.0, offset),
            cs.DM([[0.0, 0.0, 1.0, 0.0]]),
            cs.DM([[0.0, 0.0, 0.0, 1.0]]),
        )
    elif axis == 'z':
        return cs.vertcat(
            cs.DM([[1.0, 0.0, 0.0, 0.0]]),
            cs.DM([[0.0, 1.0, 0.0, 0.0]]),
            cs.horzcat(0.0, 0.0, 1.0, offset),
            cs.DM([[0.0, 0.0, 0.0, 1.0]]),
        )


@arrayify_args
def rpy2r(rpy: ArrayType, opt: str = "zyx") -> CasADiArrayType:
    """! Roll-pitch-yaw angles to SO(3) rotation matrix.

    @param rpy Roll-Pitch-Yaw angles in radians.
    @param opt Order option. Acceptable inputs:
        - 'xyz'      Rotations about X, Y, Z axes (for a robot gripper)
        - 'zyx'      Rotations about Z, Y, X axes (for a mobile robot, default)
        - 'yxz'      Rotations about Y, X, Z axes (for a camera)
        - 'arm'      Rotations about X, Y, Z axes (for a robot arm)
        - 'vehicle'  Rotations about Z, Y, X axes (for a mobile robot)
        - 'camera'   Rotations about Y, X, Z axes (for a camera)
    @return A 3-by-3 rotation matrix.
    """

    order = ["zyx", "xyz", "yxz", "arm", "vehicle", "camera"]
    r, p, y = cs.vertsplit(rpy)

    if opt in {"xyz", "arm"}:
        return rotx(y) @ roty(p) @ rotz(r)
    elif opt in {"zyx", "vehicle"}:
        return rotz(y) @ roty(p) @ rotx(r)
    elif opt in {"yxz", "camera"}:
        return roty(y) @ rotx(p) @ rotz(r)
    else:
        raise ValueError(f"didn't recognize given option {opt}, only allowed {order}")


@arrayify_args
def rt2tr(R: ArrayType, t: ArrayType) -> CasADiArrayType:
    """! Convert rotation and translation to homogeneous transform.

    @param R A 3-by-3 rotation matrix.
    @param t A vector with 3 elements.
    @return Homogeneous transformation matrix.
    """
    return cs.vertcat(
        cs.horzcat(R, vec(t)),
        cs.DM([[0.0, 0.0, 0.0, 1.0]]),
    )


@arrayify_args
def skew(v: ArrayType) -> CasADiArrayType:
    """! Create skew-symmetric matrix.

    If V (1x1) then (order 2) S =

          | 0  -v |
          | v   0 |

    and if V (1x3) then (order 3) S =

          |  0  -vz   vy |
          | vz    0  -vx |
          |-vy   vx    0 |

    @param v The form of the skew-symmetric matrix, either a scalar or vector with 3 elements.
    @return Skew-symmetric matrix of order 2 or 3.
    """
    if v.shape[0] == 1:
        return cs.vertcat(
            cs.horzcat(0.0, -v),
            cs.horzcat(v, 0.0),
        )
    elif v.shape[0] == 3:
        return cs.vertcat(
            cs.horzcat(0.0, -v[2], v[1]),
            cs.horzcat(v[2], 0.0, -v[0]),
            cs.horzcat(-v[1], v[0], 0.0),
        )
    else:
        raise ValueError(f"expecting a scalar or 3-vector")


@arrayify_args
def t2r(T: ArrayType) -> CasADiArrayType:
    """! Rotational submatrix.

    @param Homogenous transformation matrix.
    @return A 3-by-3 rotation matrix.
    """
    return T[:3, :3]


@arrayify_args
def invt(T: ArrayType) -> CasADiArrayType:
    """! Inverse of a homogeneous transformation matrix.

    @param T Homogeneous transformation matrix.
    @return Homogeneous transformation matrix such that Tinv @ T = I where I is the identity.
    """
    R = t2r(T)
    t = transl(T)
    return rt2tr(R.T, -R.T @ t)


@arrayify_args
def transl(T: ArrayType) -> CasADiArrayType:
    """! SE(3) translational homogeneous transform.

    @param T Homogeneous transformation matrix.
    @return Translation part of the homogeneous transformation.
    """
    return T[:3, 3]


@arrayify_args
def unit(v: ArrayType) -> CasADiArrayType:
    """! Unitize a vector.

    @param v A vector of order 3.
    @return A vector of order 3 with unit magnitude that is parralel to input vector.
    """
    return v / cs.norm_fro(v)


class Quaternion:
    """! Quaternion class"""

    def __init__(self, x: ArrayType, y: ArrayType, z: ArrayType, w: ArrayType):
        """! Quaternion constructor.

        @param x x-value of the quaternion.
        @param y y-value of quaternion.
        @param z z-value of quaternion.
        @param w w-value of quaternion.
        @return An instance of the Quaternion class.
        """
        self._q = cs.vertcat(x, y, z, w)

    def split(self) -> Tuple[ArrayType]:
        """! Split the quaternion into its xyzw parts.

        @return The xyzw parts of the quaternion.
        """
        return cs.vertsplit(self._q)

    def __mul__(self, quat):
        """! Quaternion multiplication.

        @param quat Another quaternion to multiply.
        @return Quaternion product.
        """
        assert isinstance(quat, Quaternion), "unsupported type"
        x0, y0, z0, w0 = self.split()
        x1, y1, z1, w1 = quat.split()
        return Quaternion(
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        )

    def sumsqr(self) -> CasADiArrayType:
        """! Sum the square values of the quaternion elements.

        @return Result of the sum of square values of the quaternion.
        """
        return cs.sumsqr(self._q)

    def inv(self):
        """! Quaternion inverse.

        @return Inverse quaternion.
        """
        q = self.getquat()
        qinv = cs.vertcat(-q[:3], q[3]) / self.sumsqr()
        return Quaternion(qinv[0], qinv[1], qinv[2], qinv[3])

    @staticmethod
    def fromrpy(rpy: ArrayType):
        """! Return a quaternion from Roll-Pitch-Yaw angles.

        @param rpy Roll-Pitch-Yaw angles (radians).
        @return An instance of the Quaternion class.
        """

        r, p, y = cs.vertsplit(vec(rpy))
        cr, sr = cs.cos(0.5 * r), cs.sin(0.5 * r)
        cp, sp = cs.cos(0.5 * p), cs.sin(0.5 * p)
        cy, sy = cs.cos(0.5 * y), cs.sin(0.5 * y)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        n = cs.sqrt(x * x + y * y + z * z + w * w)
        return Quaternion(x / n, y / n, z / n, w / n)

    @staticmethod
    def fromvec(q: ArrayType):
        """! Return a quaternion from a quaternion vector array.

        @param q Quaternion as an array.
        @return An instance of the Quaternion class.
        """
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]
        return Quaternion(x, y, z, w)

    @staticmethod
    def fromangvec(theta: ArrayType, v: ArrayType):
        """! Return a quaternion from angle-vector form.

        @param theta Angle of rotation (radians).
        @param v Direction vector to rotate about.
        @return An instance of the Quaternion class.
        """
        w = cos(0.5 * theta)
        xyz = sin(0.5 * theta) * unit(vec(v))
        x, y, z = cs.vertsplit(xyz)
        return Quaternion(x, y, z, w)

    def getquat(self) -> CasADiArrayType:
        """! Return the quaternion vector.

        @return The quaternion as an array.
        """
        return self._q

    def getrpy(self) -> CasADiArrayType:
        """! Return the quaternion as Roll-Pitch-Yaw angles.

        @return The RPY Euler angles (in radians).
        """
        qx, qy, qz, qw = self.split()

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = cs.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)

        pitch = cs.if_else(cs.fabs(sinp) >= 1.0, pi / 2.0, cs.asin(sinp))

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        yaw = cs.atan2(siny_cosp, cosy_cosp)

        return cs.vertcat(roll, pitch, yaw)
    

    def getrotm(self) ->CasADiArrayType:
        """
        
        @ return rot matrix
        """
        x, y, z, w = self.split()

        return cs.vertcat(
            cs.horzcat(1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*y+2*w*y),
            cs.horzcat(2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*z),
            cs.horzcat(2*x*z-2*w*y, 2*y*z+w*w*x, 1-2*x*x-2*y*y),
        )
    
    def getT(self, tx, ty, tz) ->CasADiArrayType:
        """
        
        @ return rot matrix
        """
        x, y, z, w = self.split()

        return cs.vertcat(
            cs.horzcat(1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*y+2*w*y, tx),
            cs.horzcat(2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*z, ty),
            cs.horzcat(2*x*z-2*w*y, 2*y*z+w*w*x, 1-2*x*x-2*y*y, tz),
            cs.horzcat(0.0, 0.0, 0.0, 1.0),
        )    
