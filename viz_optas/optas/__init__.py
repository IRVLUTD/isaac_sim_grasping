import casadi as cs
from casadi import *
from .spatialmath import *
from .models import RobotModel, TaskModel
from .builder import OptimizationBuilder
from .solver import CasADiSolver, OSQPSolver, CVXOPTSolver, ScipyMinimizeSolver
from .visualize import Visualizer


@arrayify_args
def deg2rad(x: ArrayType) -> Union[cs.DM, cs.SX]:
    """! Convert degrees to radians.

    @param x An array containing angles in degrees.
    @return An array containing angles in radians.
    """
    return (pi / 180.0) * x


@arrayify_args
def rad2deg(x: ArrayType) -> Union[cs.DM, cs.SX]:
    """! Convert radians to degrees.

    @param x An array containing angles in radians.
    @return An array containing angles in degrees.
    """
    return (180.0 / pi) * x


@arrayify_args
def clip(
    x: ArrayType, lo: Union[float, ArrayType], hi: Union[float, ArrayType]
) -> Union[cs.DM, cs.SX]:
    """! Clip (limit) the values in an array.

    @param x Array containing elements to clip.
    @param lo Minimum value.
    @param hi Maximum value.
    @return An array with the elements of x, but where values < lo are replaced with lo, and those > hi with hi.
    """
    return cs.fmax(cs.fmin(x, hi), lo)
