"""! @brief Several Model classes are defined."""
import os
import warnings
import functools
import pathlib

import casadi as cs

from typing import Callable, Union

# https://wiki.ros.org/xacro
import xacro

# https://pypi.org/project/urdf-parser-py
from urdf_parser_py.urdf import URDF, Joint, Link, Pose

from .spatialmath import *


def listify_output(fun: Callable) -> Callable:
    """! Decorator that handles the output of Model methods.

    @param fun A method from a Model sub-class.
    """

    @functools.wraps(fun)
    def listify(*args, **kwargs):
        args = list(args)  # makes concatenation easier later

        # Handle cases where a list is required
        output = None
        if len(args) > 1:
            q = args[2]  # joint states are always given at index 2

            # Check if q is a trajectory
            if q.shape[1] > 1:
                # Convert output to list
                output = []
                for i in range(q.shape[1]):
                    args_ = [args[0], args[1], q[:, i]] + args[3:]
                    output.append(fun(*args_, **kwargs))

                # Merge list when elements are vectors
                if output[0].shape[1] == 1:
                    output = cs.horzcat(*output)

        # When list wasn't required (i.e. output is still None), just evaluate function
        if output is None:
            output = fun(*args, **kwargs)

        return output

    return listify


def deprecation_warning(name_to):
    def decorator(function):
        def wrapper(*args, **kwargs):
            name_from = function.__name__
            warn = f"'{name_from}' will be deprecated, please use '{name_to}' instead"
            msg = "\033[93m" + warn + "\033[0m"  # add
            warnings.warn(msg, DeprecationWarning, stacklevel=2)
            self_ = args[0]
            new_function = getattr(self_, name_to)
            while hasattr(new_function, "__wrapped__"):
                new_function = new_function.__wrapped__

            args_use = list(args)
            if function.__name__.endswith("_function"):
                args_use = args_use[1:]

            return new_function(*args_use, **kwargs)

        return wrapper

    return decorator


class Model:
    """! The Model base class.
    Defines the base class utilized by all models.
    """

    def __init__(
        self,
        name: str,
        dim: int,
        time_derivs: List[int],
        symbol: str,
        dlim: Dict[int, Tuple[List[float]]],
        T: Union[None, int],
    ):
        """! The Model base class initializer.

        @param name The name of the model.
        @param dim Model dimension (for robots this is ndof).
        @param time_derivs Time derivatives required for model, 0 means not time derivative, 1 means first derivative wrt to time is required, etc.
        @param symbol A short symbol to represent the model.
        @param dlim Limits on each time derivative, index should correspond to a time derivative (i.e. 0, 1, ...) and the value should be a tuple of two lists containing the lower and upper bounds.
        @param T Optionally use this to override the number of time-steps given in the OptimizationBuilder constructor.

        @return An instance of the Model class.
        """

        ## The name of the model.
        self.name = name

        ## Model dimension.
        self.dim = dim

        ## Time derivatives required for the model.
        self.time_derivs = time_derivs

        ## A short symbol to represent the model.
        self.symbol = symbol

        ## Model limits
        self.dlim = dlim

        ## Number of time steps
        self.T = T

    def get_name(self) -> str:
        """! Return the name of the model.

        @return Name of the model.
        """
        return self.name

    def state_name(self, time_deriv: int) -> str:
        """! Return the state name.

        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return The state name in the form {name}/{d}{symbol}, where "name" is the model name, d is a string given by 'd'*time_deriv, and symbol is the symbol for the model state.
        """
        assert (
            time_deriv in self.time_derivs
        ), f"Given time derivative time_deriv={time_deriv} is not recognized, only allowed {self.time_derivs}"
        return self.name + "/" + "d" * time_deriv + self.symbol

    def state_parameter_name(self, time_deriv: int) -> str:
        """! Return the parameter name.

        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return The parameter name in the form {name}/{d}{symbol}/p, where "name" is the model name, d is a string given by 'd'*time_deriv, and symbol is the symbol for the model parameters.
        """
        assert (
            time_deriv in self.time_derivs
        ), f"Given time derivative time_deriv={time_deriv} is not recognized, only allowed {self.time_derivs}"
        return self.name + "/" + "d" * time_deriv + self.symbol + "/" + "p"

    def state_optimized_name(self, time_deriv: int) -> str:
        """! Return the sate optimized name.

        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return The parameter name in the form {name}/{d}{symbol_param}/x, where "name" is the model name, d is a string given by 'd'*time_deriv, and symbol_param is the symbol for the model parameters.
        """
        assert (
            time_deriv in self.time_derivs
        ), f"Given time derivative time_deriv={time_deriv} is not recognized, only allowed {self.time_derivs}"
        return self.name + "/" + "d" * time_deriv + self.symbol + "/" + "x"

    def get_limits(self, time_deriv: int) -> Tuple[ArrayType]:
        """! Return the model limits.

        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return lower The model lower limit.
        @return upper The model upper limit.
        """
        assert (
            time_deriv in self.time_derivs
        ), f"Given time derivative time_deriv={time_deriv} is not recognized, only allowed {self.time_derivs}"
        assert (
            time_deriv in self.dlim.keys()
        ), f"Limit for time derivative time_deriv={time_deriv} has not been given"
        return self.dlim[time_deriv]

    def in_limit(self, x: ArrayType, time_deriv: int) -> cs.DM:
        """! Check if array is within model limits.

        @param x The array containing values to be checked.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return Returns DM(1) if the array x is within the model limits, DM(0) otherwise.
        """
        lo, up = self.get_limits(time_deriv)
        return cs.logic_all(cs.logic_and(lo <= x, x <= up))


class TaskModel(Model):
    """! Model class for tasks, e.g. position trajectory."""

    def __init__(
        self,
        name: str,
        dim: int,
        time_derivs: List[int] = [0],
        symbol: str = "y",
        dlim: Dict[int, Tuple[List[float]]] = {},
        T: Union[None, int] = None,
        is_discrete: bool = False,
    ):
        """! Task model initializer.

        @param name The name of the model.
        @param dim Model dimension.
        @param time_derivs Time derivatives required for model, 0 means not time derivative, 1 means first derivative wrt to time is required, etc.
        @param symbol A short symbol to represent the model.
        @param dlim Limits on each time derivative, index should correspond to a time derivative (i.e. 0, 1, ...) and the value should be a tuple of two lists containing the lower and upper bounds.
        @param T Optionally use this to override the number of time-steps given in the OptimizationBuilder constructor.
        @param is_discrete When True, the variables are treated as discrete variables.
        """

        super().__init__(name, dim, time_derivs, symbol, dlim, T)
        self.is_discrete = is_discrete


class JointTypeNotSupported(NotImplementedError):
    """! Exception to be thrown when a joint type is not supported."""

    def __init__(self, joint_type: str):
        """! Initializer for the JointTypeNotSupported exception.

        @param joint_type The joint type given
        @return An instance of the exception class.
        """

        msg = f"{joint_type} joints are currently not supported\n"
        msg += "if you require this joint type please raise an issue at "
        msg += "https://github.com/cmower/optas/issues"
        super().__init__(msg)


class RobotModel(Model):
    """! A class that defines a robot model. Many methods here model the robot kinematics."""

    def __init__(
        self,
        urdf_filename: Union[None, str] = None,
        urdf_string: Union[None, str] = None,
        xacro_filename: Union[None, str] = None,
        name: Union[None, str] = None,
        time_derivs: List[int] = [0],
        qddlim: Union[None, ArrayType] = None,
        T: Union[None, int] = None,
        param_joints: List[str] = [],
    ):
        """! Initializer for the robot model class.

                Note, at least one of the parameters urdf_filename, urdf_string, or xacro_filename must be specified.

        @param urdf_filename Filename for a URDF.
        @param urdf_string URDF as a string.
        @param xacro_filename Filename for a xacro file.
        @param name Name of the robot model.
        @param time_derivs Time derivatives required for model, 0 means not time derivative, 1 means first derivative wrt to time is required, etc.
        @param qddlim Optionally specify limits on the joint acceleration.
        @param T Optionally use this to override the number of time-steps given in the OptimizationBuilder constructor.
        @param param_joints A list of joints that are considered parameters.
        """

        # If xacro is passed then convert to urdf string

        ## Filename for xacro file.
        self.xacro_filename = xacro_filename
        if xacro_filename is not None:
            try:
                urdf_string = xacro.process(xacro_filename)
            except AttributeError:
                from io import StringIO

                xml = xacro.process_file(xacro_filename)
                str_io = StringIO()
                xml.writexml(str_io)
                urdf_string = str_io.getvalue()

        # Load URDF

        ## URDF instance.
        self.urdf = None

        ## Filename for URDF file.
        self.urdf_filename = None

        ## URDF string.
        self.urdf_string = None
        if urdf_filename is not None:
            self.urdf_filename = urdf_filename
            self.urdf = URDF.from_xml_file(urdf_filename)
        if urdf_string is not None:
            self.urdf = URDF.from_xml_string(urdf_string)
        assert (
            self.urdf is not None
        ), "You need to supply a urdf, either through filename or as a string"

        # Setup joint limits, joint position/velocity limits

        ## List of parameterized joints.
        self.param_joints = param_joints
        dlim = {
            0: (self.lower_optimized_joint_limits, self.upper_optimized_joint_limits),
            1: (
                -self.velocity_optimized_joint_limits,
                self.velocity_optimized_joint_limits,
            ),
        }

        # Handle potential acceleration limit
        if qddlim:
            qddlim = vec(qddlim)
            if qddlim.shape[0] == 1:
                qddlim = qddlim * cs.DM.ones(self.ndof)
            assert (
                qddlim.shape[0] == self.ndof
            ), f"expected ddlim to have {self.ndof} elements"
            dlim[2] = -qddlim, qddlim

        # If user did not supply name for the model then use the one in the URDF
        if name is None:
            name = self.urdf.name

        super().__init__(name, self.ndof, time_derivs, "q", dlim, T)

    def get_urdf(self):
        return self.urdf

    def get_urdf_dirname(self):
        if self.urdf_filename is not None:
            return pathlib.Path(os.path.dirname(self.urdf_filename))
        elif self.xacro_filename is not None:
            return pathlib.Path(os.path.dirname(self.xacro_filename))

    @property
    def joint_names(self) -> List[str]:
        """! Property that gives the list of joint names.

        @return List of joint names.
        """
        return [jnt.name for jnt in self.urdf.joints]

    @property
    def link_names(self) -> List[str]:
        """! Property that gives the list of link names.

        @return List of link names.
        """
        return [lnk.name for lnk in self.urdf.links]

    @property
    def actuated_joint_names(self) -> List[str]:
        """! Property that gives the names of the actuated joints.

        @return List of actuated joint names.
        """
        return [jnt.name for jnt in self.urdf.joints if jnt.type != "fixed"]

    @property
    def parameter_joint_names(self) -> List[str]:
        """! Property that gives the names of the parameterized joints.

        @return List of the parameterized joint names.
        """
        return [
            joint for joint in self.actuated_joint_names if joint in self.param_joints
        ]

    @property
    def optimized_joint_indexes(self) -> List[int]:
        """! Property that gives the indexes of the optimized joints.

        @return List of the optimized joint indexes.
        """
        return [
            self.get_actuated_joint_index(joint) for joint in self.optimized_joint_names
        ]

    @property
    def optimized_joint_names(self) -> List[str]:
        """! Property that gives the names of the optimized joints names.

        @return List of the optimized joint names.
        """
        return [
            joint
            for joint in self.actuated_joint_names
            if joint not in self.parameter_joint_names
        ]

    @property
    def parameter_joint_indexes(self) -> List[int]:
        """! Property that gives the indexes of the parameterized joints.

        @return List of the parameterized joint indexes.
        """
        return [
            self.get_actuated_joint_index(joint) for joint in self.parameter_joint_names
        ]

    def extract_parameter_dimensions(self, values: ArrayType) -> ArrayType:
        """! Return the elements that correspond to the model dimensions.

        @param values The values to extract elements from.
        @return A sub-array of the given values corresponding to the model parameterized dimension.
        """
        return values[self.parameter_joint_indexes, :]

    def extract_optimized_dimensions(self, values: ArrayType) -> ArrayType:
        """! Return the elements that correspond to the model optimized dimensions.

        @param values The values to extract elements from.
        @return A sub-array of the given values corresponding to the model optimized dimensions.
        """
        return values[self.optimized_joint_indexes, :]

    @property
    def ndof(self) -> int:
        """! Number of degrees of freedom.

        @return The number of degrees of freedom for the robot.
        """
        return len(self.actuated_joint_names)

    @property
    def num_opt_joints(self) -> int:
        """! Number of optimized joints.

        @return The number of optimized joints.
        """
        return len(self.optimized_joint_names)

    @property
    def num_param_joints(self) -> int:
        """! Number of parameterized joints.

        @return The number of parameterized joints.
        """
        return len(self.parameter_joint_names)

    def get_joint_lower_limit(self, joint: Joint) -> float:
        """! Return the lower limit for a given joint.

        @param joint The joint instance from the URDF.
        @return The lower limit, when undefined the value -1e9 is returned.
        """
        if joint.limit is None:
            return -1e9
        return joint.limit.lower

    def get_joint_upper_limit(self, joint: Joint) -> float:
        """! Return the upper limit for a given joint.

        @param joint The joint instance from the URDF.
        @return The upper limit, when undefined the value 1e9 is returned.
        """
        if joint.limit is None:
            return 1e9
        return joint.limit.upper

    def get_velocity_joint_limit(self, joint: Joint) -> float:
        """! Return the velocity limit for a given joint.

        @param joint The joint instance from the URDF.
        @return The velocity limit, when undefined the value 1e9 is returned.
        """
        if joint.limit is None:
            return 1e9
        return joint.limit.velocity

    @property
    def lower_actuated_joint_limits(self) -> cs.DM:
        """! Property that defines the lower actuated joint limits.

        @return The lower joint position limits.
        """
        return cs.DM(
            [
                self.get_joint_lower_limit(jnt)
                for jnt in self.urdf.joints
                if jnt.type != "fixed"
            ]
        )

    @property
    def upper_actuated_joint_limits(self) -> cs.DM:
        """! Property that defines the upper actuated joint limits.

        @return The upper joint position limits.
        """
        return cs.DM(
            [
                self.get_joint_upper_limit(jnt)
                for jnt in self.urdf.joints
                if jnt.type != "fixed"
            ]
        )

    @property
    def velocity_actuated_joint_limits(self) -> cs.DM:
        """! Property that defines the velocity actuated joint limits.

        @return The velocity joint limits.
        """
        return cs.DM(
            [
                self.get_velocity_joint_limit(jnt)
                for jnt in self.urdf.joints
                if jnt.type != "fixed"
            ]
        )

    @property
    def lower_optimized_joint_limits(self) -> cs.DM:
        """! Property that defines the lower optimized joint limits.

        @return The lower joint position limits.
        """
        return cs.DM(
            [
                self.get_joint_lower_limit(jnt)
                for jnt in self.urdf.joints
                if jnt.name in self.optimized_joint_names
            ]
        )

    @property
    def upper_optimized_joint_limits(self) -> cs.DM:
        """! Property that defines the upper optimized joint limits.

        @return The upper joint position limits.
        """
        return cs.DM(
            [
                self.get_joint_upper_limit(jnt)
                for jnt in self.urdf.joints
                if jnt.name in self.optimized_joint_names
            ]
        )

    @property
    def velocity_optimized_joint_limits(self) -> cs.DM:
        """! Property that defines the velocity limits for the optimized joints.

        @return The joint velocity limits.
        """
        return cs.DM(
            [
                self.get_velocity_joint_limit(jnt)
                for jnt in self.urdf.joints
                if jnt.name in self.optimized_joint_names
            ]
        )

    def add_base_frame(
        self,
        base_link: str,
        xyz: Union[None, List[float]] = None,
        rpy: Union[None, List[float]] = None,
        joint_name: str = None,
    ) -> None:
        """! Add new base frame, note this changes the root link.

        @param base_link The name for the new root link.
        @param xyz The position of the new link with respect to the current root link. Defaults to [0.0, 0.0, 0.0].
        @param rpy The orientation as Euler (RPY) angles, defined in radians, with respect to the current root link. Defaults to [0.0, 0.0, 0.0].
        @param joint_name The name for the joint that connects the current root link with the new base frame. Defaults to "{base_link}_and_{current_root_link}_joint".
        """

        parent_link = base_link
        child_link = self.urdf.get_root()  # i.e. current root

        if xyz is None:
            xyz = [0.0] * 3

        if rpy is None:
            rpy = [0.0] * 3

        if not isinstance(joint_name, str):
            joint_name = parent_link + "_and_" + child_link + "_joint"

        self.urdf.add_link(Link(name=parent_link))

        joint = Joint(
            name=joint_name,
            parent=parent_link,
            child=child_link,
            joint_type="fixed",
            origin=Pose(xyz=xyz, rpy=rpy),
        )
        self.urdf.add_joint(joint)

    # def add_fixed_link(self, link, parent_link, xyz=None, rpy=None, joint_name=None):
    #     """Add a fixed link"""

    #     assert link not in self.link_names, f"'{link}' already exists"
    #     assert (
    #         parent_link in self.link_names
    #     ), f"{parent_link=}, does not appear in link names"

    #     child_link = link

    #     if xyz is None:
    #         xyz = [0.0] * 3

    #     if rpy is None:
    #         rpy = [0.0] * 3

    #     if not isinstance(joint_name, str):
    #         joint_name = parent_link + "_and_" + child_link + "_joint"

    #     self.urdf.add_link(Link(name=child_link))

    #     origin = Pose(xyz=xyz, rpy=rpy)
    #     self.urdf.add_joint(
    #         Joint(
    #             name=joint_name,
    #             parent=parent_link,
    #             child=child_link,
    #             joint_type="fixed",
    #             origin=origin,
    #         )
    #     )

    def get_root_link(self) -> str:
        """! The root link name.

        @return The name of the root link.
        """
        return self.urdf.get_root()

    def get_link_visual_origin(self, link: Link) -> Tuple[cs.DM]:
        """! Get the link position and orientation for the link visual.

        @param link The link of interest.
        @return The position and orientation.
        """
        xyz, rpy = cs.DM.zeros(3), cs.DM.zeros(3)
        if link.visual is not None:
            if link.visual.origin is not None:
                origin = link.visual.origin
                xyz, rpy = cs.DM(origin.xyz), cs.DM(origin.rpy)
        return xyz, rpy

    def get_joint_origin(self, joint: Joint) -> Tuple[cs.DM]:
        """! Get the origin for the joint.

        @param The joint of interest.
        @return The position and orientation of the joint.
        """
        xyz, rpy = cs.DM.zeros(3), cs.DM.zeros(3)
        if joint.origin is not None:
            xyz, rpy = cs.DM(joint.origin.xyz), cs.DM(joint.origin.rpy)
        return xyz, rpy

    def get_joint_axis(self, joint: Joint) -> cs.DM:
        """! Get the axis of a joint.

        @param The joint of interest.
        @return The normalized joint axis."""
        axis = cs.DM(joint.axis) if joint.axis is not None else cs.DM([1.0, 0.0, 0.0])
        return unit(axis)

    def get_actuated_joint_index(self, joint_name: str) -> int:
        """! Get the joint index for a given joint name.

        @param The name of the joint.
        @return Index for the joint.
        """
        return self.actuated_joint_names.index(joint_name)

    def get_random_joint_positions(
        self,
        n: int = 1,
        xlim: Union[None, Tuple[ArrayType]] = None,
        ylim: Union[None, Tuple[ArrayType]] = None,
        zlim: Union[None, Tuple[ArrayType]] = None,
        base_link: Union[None, str] = None,
    ) -> cs.DM:
        """! Random joint positions within actuator limits and optionally within a box for a given base link.

        @param n Number of joint positions. Default is 1.
        @param xlim Limit the robot link positions in the x axis for the base frame. None means there are no limits.
        @param ylim Limit the robot link positions in the y axis for the base frame. None means there are no limits.
        @param zlim Limit the robot link positions in the z axis for the base frame. None means there are no limits.
        @param base_link The link to define the x, y, z limits. None means the root link is used.
        @return Random joint positions with dimension ndof-by-n.
        """

        lo = self.lower_actuated_joint_limits.toarray()
        hi = self.upper_actuated_joint_limits.toarray()

        pos = None
        if isinstance(base_link, str):
            pos = {
                self.get_link_position_function(link, base_link)
                for link in self.link_names
            }

        def _in_limit(q):
            if pos is not None:
                for p in pos.values():
                    pp = p(q)
                    if xlim is not None:
                        if not (xlim[0] <= pp[0] <= xlim[1]):
                            return False
                    if ylim is not None:
                        if not (ylim[0] <= pp[1] <= ylim[1]):
                            return False
                    if zlim is not None:
                        if not (zlim[0] <= pp[2] <= zlim[1]):
                            return False
            return True

        def randq():
            qr = cs.vec(cs.np.random.uniform(lo, hi))
            while not _in_limit(qr):
                qr = cs.vec(cs.np.random.uniform(lo, hi))
            return qr

        return cs.horzcat(*[randq() for _ in range(n)])

    def get_random_pose_in_global_link(self, link_name: str) -> cs.DM:
        """Random end-effector pose within robot limits.

        @param link_name Name of the end-effector link.
        @return Random homogeneous transformation array within robot limits defined in the global link frame.
        """
        q = self.get_random_joint_positions()
        return self.get_global_link_transform(link_name, q)

    def make_function(
        self,
        label: str,
        link: str,
        method: Callable,
        n: int = 1,
        base_link: Union[None, str] = None,
        axis: Union[None, cs.DM] = None,
        numpy_output=False,
    ):
        """! Automate function generation. This is an internal function for the RobotModel class and <b>SHOULD NOT</b> be used."""
        q = cs.SX.sym("q", self.ndof)
        args = [link, q]
        if axis is not None:
            args.append(axis)
        kwargs = {}
        if base_link is not None:
            kwargs["base_link"] = base_link
        out = method(*args, **kwargs)
        F = cs.Function(label, [q], [out])

        class ListFunction:
            def __init__(self, F, n):
                self.F = F
                self.n = n

            @arrayify_args
            def __call__(self, Q):
                assert (
                    Q.shape[1] == self.n
                ), f"expected input to have shape {self.ndof}-by-{n}, got {Q.shape[0]}-by-{Q.shape[1]}"
                return [self.F(q) for q in cs.horzsplit(Q)]

            def size_in(self, i):
                return self.F.size_in(i)

            def size_out(self, i):
                return self.F.size_out(i)

            def size1_in(self, i):
                return self.F.size1_in(i)

            def size1_out(self, i):
                return self.F.size1_out(i)

            def size2_in(self, i):
                return self.F.size2_in(i)

            def size2_out(self, i):
                return self.F.size2_out(i)

            def numel_in(self):
                return self.F.numel_in()

            def numel_out(self):
                return self.F.numel_out()

        if n > 1 and out.shape[1] == 1:
            F = F.map(n)

        elif n > 1 and out.shape[1] > 1:
            F = ListFunction(F, n)

        if numpy_output:

            class NumpyOutputFunction:
                def __init__(self, F):
                    self.F = F

                    if isinstance(F, ListFunction):
                        self.call = self._call_list
                    else:
                        self.call = self._call

                @staticmethod
                def handle(fout):
                    if fout.shape[1] == 1:
                        return fout.flatten()
                    else:
                        return fout

                def _call_list(self, Q):
                    return [self.handle(fout.toarray()) for fout in self.F(Q)]

                def _call(self, q):
                    return self.handle(self.F(q).toarray())

                def __call__(self, q):
                    assert not isinstance(
                        q, cs.SX
                    ), "numpy_output=False was specified, you can not pass symbolic variables"
                    return self.call(q)

            F = NumpyOutputFunction(F)

        return F

    @arrayify_args
    @listify_output
    def get_global_link_transform(self, link: str, q: ArrayType) -> CasADiArrayType:
        """! Get the link transform in the global frame for a given joint state q.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Homogeneous transform array.
        """

        # Setup
        assert (
            link in self.urdf.link_map.keys()
        ), f"given link '{link}' does not appear in URDF"
        root = self.urdf.get_root()
        T = I4()
        if link == root:
            return T

        # Iterate over joints in chain
        for joint_name in self.urdf.get_chain(root, link, links=False):
            joint = self.urdf.joint_map[joint_name]
            xyz, rpy = self.get_joint_origin(joint)

            if joint.type == "fixed":
                T = T @ rt2tr(rpy2r(rpy), xyz)
                continue

            joint_index = self.get_actuated_joint_index(joint.name)
            qi = q[joint_index]

            T = T @ rt2tr(rpy2r(rpy), xyz)

            if joint.type in {"revolute", "continuous"}:
                T = T @ r2t(angvec2r(qi, self.get_joint_axis(joint)))

            elif joint.type == "prismatic":
                T = T @ rt2tr(I3(), qi * self.get_joint_axis(joint))

            else:
                raise JointTypeNotSupported(joint.type)

        return T

    def get_global_link_transform_function(
        self, link: str, n: int = 1, numpy_output: bool = False
    ) -> cs.Function:
        """! Get the function which computes the link transform in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the transform for a given joint state. If n > 1 then a list of transform are given for each corresponding joint state.
        """
        return self.make_function(
            "T", link, self.get_global_link_transform, n=n, numpy_output=numpy_output
        )

    @arrayify_args
    @listify_output
    def get_link_transform(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the link transform in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Homogeneous transform array.
        """
        # T_L_W = self.get_global_link_transform(link, q)
        # T_B_W = self.get_global_link_transform(base_link, q)
        # return T_L_W @ invt(T_B_W)

        T_W_L = self.get_global_link_transform(link, q)
        T_W_B = self.get_global_link_transform(base_link, q)
        return invt(T_W_B) @ T_W_L

    def get_link_transform_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> CasADiArrayType:
        """! Get the function that computes the transform in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the transform for a given joint state. If n > 1 then a list of transform are given for each corresponding joint state.
        """
        return self.make_function(
            "T",
            link,
            self.get_link_transform,
            base_link=base_link,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_position(self, link: str, q: ArrayType) -> CasADiArrayType:
        """! Get the link position in the global frame for a given joint state q.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Position array.
        """
        return transl(self.get_global_link_transform(link, q))

    def get_global_link_position_function(
        self, link: str, n: int = 1, numpy_output: bool = False
    ) -> cs.Function:
        """! Get the function that computes the global position of a given link.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the position for a given joint state. If n > 1 then an array is computed whose columns each correspond to the respective joint state in the input.
        """
        return self.make_function(
            "p", link, self.get_global_link_position, n=n, numpy_output=numpy_output
        )

    @arrayify_args
    @listify_output
    def get_link_position(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the link position in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Position array.
        """
        return transl(self.get_link_transform(link, q, base_link))

    def get_link_position_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the position of a link in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the position for a given joint state. If n > 1 then an array is computed whose columns each correspond to the respective joint state in the input.
        """
        return self.make_function(
            "p",
            link,
            self.get_link_position,
            n=n,
            base_link=base_link,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_rotation(self, link: str, q: ArrayType) -> CasADiArrayType:
        """! Get the link rotation in the global frame for a given joint state q.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Rotation array.
        """
        return t2r(self.get_global_link_transform(link, q))

    def get_global_link_rotation_function(
        self, link: str, n: int = 1, numpy_output: bool = False
    ) -> cs.Function:
        """! Get the function that computes a rotation matrix in the global link.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the rotation for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "R", link, self.get_global_link_rotation, n=n, numpy_output=numpy_output
        )

    @arrayify_args
    @listify_output
    def get_link_rotation(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the rotation matrix for a link in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Rotation array.
        """
        return t2r(self.get_link_transform(link, q, base_link))

    def get_link_rotation_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the rotation matrix for a link in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the rotation for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "R",
            link,
            self.get_link_rotation,
            base_link=base_link,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_quaternion(self, link: str, q: ArrayType) -> CasADiArrayType:
        """! Get a quaternion in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Quaternion array.
        """

        # Setup
        assert link in self.urdf.link_map.keys(), "given link does not appear in URDF"
        root = self.urdf.get_root()
        quat = Quaternion(0.0, 0.0, 0.0, 1.0)
        if link == root:
            return quat.getquat()

        # Iterate over joints in chain
        for joint_name in self.urdf.get_chain(root, link, links=False):
            joint = self.urdf.joint_map[joint_name]
            xyz, rpy = self.get_joint_origin(joint)

            if joint.type == "fixed":
                quat = Quaternion.fromrpy(rpy) * quat
                continue

            joint_index = self.get_actuated_joint_index(joint.name)
            qi = q[joint_index]

            quat = Quaternion.fromrpy(rpy) * quat
            if joint.type in {"revolute", "continuous"}:
                quat = Quaternion.fromangvec(qi, self.get_joint_axis(joint)) * quat

            elif joint.type == "prismatic":
                pass

            else:
                raise JointTypeNotSupported(joint.type)

        return quat.getquat()

    def get_global_link_quaternion_function(
        self, link: str, n: int = 1, numpy_output: bool = False
    ) -> cs.Function:
        """! Get the function that computes a quaternion in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the quaternion for a given joint state. If n > 1 then an array is computed whose columns correspond to the respective joint state in the input.
        """
        return self.make_function(
            "quat",
            link,
            self.get_global_link_quaternion,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_quaternion(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the quaternion defined in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Quaternion array.
        """
        quat_L_W = Quaternion.fromvec(self.get_global_link_quaternion(link, q))
        quat_B_W = Quaternion.fromvec(self.get_global_link_quaternion(base_link, q))
        return (quat_L_W * quat_B_W.inv()).getquat()

    def get_link_quaternion_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes a quaternion defined in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the quaternion for a given joint state. If n > 1 then an array is computed whose columns correspond to the respective joint state in the input.
        """
        return self.make_function(
            "quat",
            link,
            self.get_link_quaternion,
            n=n,
            base_link=base_link,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_rpy(self, link: str, q: ArrayType) -> CasADiArrayType:
        """! Get the Roll-Pitch-Yaw angles in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return RPY euler angles in radians.
        """
        return Quaternion.fromvec(self.get_global_link_quaternion(link, q)).getrpy()

    def get_global_link_rpy_function(
        self, link: str, n: int = 1, numpy_output: bool = False
    ):
        """! Get the function that computes the Roll-Pitch-Yaw angles in the global frame."""
        return self.make_function(
            "quat", link, self.get_global_link_rpy, n=n, numpy_output=numpy_output
        )

    @arrayify_args
    @listify_output
    def get_link_rpy(self, link: str, q: ArrayType, base_link: str) -> CasADiArrayType:
        """! Get the the Roll-Pitch-Yaw angles defined in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return RPY euler angles in radians.
        """
        return Quaternion.fromvec(self.get_link_quaternion(link, q, base_link)).getrpy()

    def get_link_rpy_function(
        self, link: str, base_link: str, n: int = 1, numpy_output: bool = False
    ) -> cs.Function:
        """! Get the function that computes the Roll-Pitch-Yaw angles defined in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the RPY angles for a given joint state. If n > 1 then an array is computed whose columns correspond to the respective joint state in the input.
        """
        return self.make_function(
            "rpy",
            link,
            self.get_link_rpy,
            n=n,
            base_link=base_link,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_geometric_jacobian(
        self, link: str, q: ArrayType
    ) -> CasADiArrayType:
        """! Compute the geometric Jacobian matrix in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Geometric Jacobian.
        """

        root = self.urdf.get_root()

        e = self.get_global_link_position(link, q)

        pdot = cs.DM.zeros(3)

        joint_index_order = []
        jacobian_columns = []

        for joint in self.urdf.joints:
            if joint.type == "fixed":
                continue

            joint_index = self.get_actuated_joint_index(joint.name)
            joint_index_order.append(joint_index)
            qi = q[joint_index]

            if joint.name in self.urdf.get_chain(root, link, links=False):
                # if joint in chain check type of join
                if joint.type in {"revolute", "continuous"}:
                    axis = self.get_joint_axis(joint)
                    R = self.get_global_link_rotation(joint.child, q)
                    R = R @ angvec2r(qi, axis)
                    p = self.get_global_link_position(joint.child, q)

                    z = R @ axis
                    pdot = cs.cross(z, e - p)

                    jcol = cs.vertcat(pdot, z)
                    jacobian_columns.append(jcol)

                elif joint.type == "prismatic":
                    axis = self.get_joint_axis(joint)
                    R = self.get_global_link_rotation(joint.child, q)
                    z = R @ axis
                    jcol = cs.vertcat(z, cs.DM.zeros(3))
                    jacobian_columns.append(jcol)

                else:
                    raise JointTypeNotSupported(joint.type)
            else:
                # if joint not in chain add zeros
                jcol = cs.DM.zeros(6)
                jacobian_columns.append(jcol)

        # Sort columns of jacobian
        jacobian_columns_ordered = [jacobian_columns[idx] for idx in joint_index_order]

        # Build jacoian array
        J = jacobian_columns_ordered.pop(0)
        while jacobian_columns_ordered:
            J = cs.horzcat(J, jacobian_columns_ordered.pop(0))

        return J

    def get_global_link_geometric_jacobian_function(
        self,
        link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the geometric jacobian in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the geometric jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "J", link, self.get_global_link_geometric_jacobian, n=n
        )

    @arrayify_args
    @listify_output
    def get_global_link_analytical_jacobian(
        self, link: str, q: ArrayType
    ) -> CasADiArrayType:
        """! Compute the analytical Jacobian matrix in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Analytic Jacobian.
        """
        return cs.vertcat(
            self.get_global_link_linear_jacobian(link, q),
            self.get_global_link_angular_analytical_jacobian(link, q),
        )

    def get_global_link_analytical_jacobian_function(
        self,
        link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the analytical jacobian in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the analytic jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "J_a",
            link,
            self.get_global_link_analytical_jacobian,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_geometric_jacobian(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the geometric jacobian in a given base link.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Geometric Jacobian.
        """

        J = self.get_global_link_geometric_jacobian(link, q)

        # Transform jacobian to given base link
        R = self.get_global_link_rotation(base_link, q).T
        O = cs.DM.zeros(3, 3)
        K = cs.vertcat(
            cs.horzcat(R, O),
            cs.horzcat(O, R),
        )
        J = K @ J

        return J

    def get_link_geometric_jacobian_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """Get the function that computes the geometric jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the geometric jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "J",
            link,
            self.get_link_geometric_jacobian,
            base_link=base_link,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_analytical_jacobian(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Compute the analytical Jacobian matrix in a given base link.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Analytic Jacobian.
        """
        return cs.vertcat(
            self.get_link_linear_jacobian(link, q, base_link),
            self.get_link_angular_analytical_jacobian(link, q, base_link),
        )

    def get_link_analytical_jacobian_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the analytical jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the analytic jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "J_a",
            link,
            self.get_link_analytical_jacobian,
            n=n,
            base_link=base_link,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_linear_jacobian(
        self, link: str, q: ArrayType
    ) -> CasADiArrayType:
        """! Compute the linear part of the geometric jacobian in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Linear part of the Jacobian.
        """
        J = self.get_global_link_geometric_jacobian(link, q)
        return J[:3, :]

    def get_global_link_linear_jacobian_function(
        self,
        link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the linear part of the geometric jacobian in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the linear part of the Jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "Jl",
            link,
            self.get_global_link_linear_jacobian,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_linear_jacobian(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the linear part of the geometric jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Linear part of the Jacobian.
        """
        J = self.get_link_geometric_jacobian(link, q, base_link)
        return J[:3, :]

    def get_link_linear_jacobian_function(
        self, link: str, base_link: str, n: int = 1, numpy_output: bool = False
    ) -> cs.Function:
        """! Get the function that computes the linear part of the geometric jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the linear part of the Jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "Jl",
            link,
            self.get_link_linear_jacobian,
            base_link=base_link,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_angular_geometric_jacobian(
        self, link: str, q: ArrayType
    ) -> CasADiArrayType:
        """! Compute the angular part of the geometric jacobian in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Angular part of the geometric Jacobian.
        """
        J = self.get_global_link_geometric_jacobian(link, q)
        return J[3:, :]

    def get_global_link_angular_geometric_jacobian_function(
        self,
        link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the angular part of the geometric jacobian in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the angular part of the geometric Jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "Ja",
            link,
            self.get_global_link_angular_geometric_jacobian,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_angular_analytical_jacobian(
        self, link: str, q: ArrayType
    ) -> CasADiArrayType:
        """! Compute the angular part of the analytical Jacobian matrix in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @return Angular part of the analytical Jacobian.
        """
        return self.get_link_angular_analytical_jacobian(link, q, self.get_root_link())

    def get_global_link_angular_analytical_jacobian_function(
        self,
        link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the angular part of the analytical jacobian in the global frame.

        @param link Name of the end-effector link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the angular part of the geometric Jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "Ja",
            link,
            self.get_global_link_angular_analytical_jacobian,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_angular_geometric_jacobian(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Get the angular part of the geometric jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Angular part of the geometric Jacobian.
        """
        J = self.get_link_geometric_jacobian(link, q, base_link)
        return J[3:, :]

    def get_link_angular_geometric_jacobian_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the angular part of the geometric jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the angular part of the geometric Jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "Ja",
            link,
            self.get_link_angular_geometric_jacobian,
            base_link=base_link,
            n=n,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_angular_analytical_jacobian(
        self, link: str, q: ArrayType, base_link: str
    ) -> CasADiArrayType:
        """! Compute the angular part of the analytical Jacobian matrix in a given base frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param base_link Name of the base frame link.
        @return Angular part of the analytical Jacobian.
        """

        # Compute rpy derivative Ja
        q_sym = cs.SX.sym("q_sym", self.ndof)
        rpy = self.get_link_rpy(link, q_sym, base_link)
        Ja = cs.jacobian(rpy, q_sym)

        # Functionize Ja
        Ja = cs.Function("Ja", [q_sym], [Ja])

        return Ja(q)

    def get_link_angular_analytical_jacobian_function(
        self,
        link: str,
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get the function that computes the angular part of the analytical jacobian in a given base frame.

        @param link Name of the end-effector link.
        @param base_link Name of the base frame link.
        @param n Number of joint states to expect when the function is called. Default is 1.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the angular part of the analytical Jacobian for a given joint state. If n > 1 then a list of arrays are computed whose corresponding to the respective joint state in the input.
        """
        return self.make_function(
            "Ja",
            link,
            self.get_link_angular_analytical_jacobian,
            n=n,
            base_link=base_link,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_link_axis(
        self, link: str, q: ArrayType, axis: Union[str, ArrayType], base_link: str
    ) -> CasADiArrayType:
        """! Compute the link axis, this is a direction vector defined in the end-effector frame (e.g. the x/y/z link axis).

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param axis The axis (direction vector) defined in the end-effector frame. If 'x', 'y', 'z' is passed then the corresponding sub-array of the homogenous transform is used.
        @param base_link Name of the base frame link.
        @return Axis defined in the end-effector frame as function of the joint angles.
        """
        Tf = self.get_link_transform(link, q, base_link)

        axis2index = {"x": 0, "y": 1, "z": 2}
        if isinstance(axis, str):
            assert axis in axis2index, "axis must be either 'x', 'y', 'z' or a 3-array"
            index = axis2index[axis]
            vector = Tf[:3, index]

        elif isinstance(axis, (cs.DM, cs.SX)):
            a = axis / cs.norm_fro(axis)  # normalize

            x = Tf[:3, 0]
            y = Tf[:3, 1]
            z = Tf[:3, 2]

            vector = a[0] * x + a[1] * y + a[2] * z

        else:
            raise ValueError(f"did not recognize input for axis: {axis}")

        return vector

    def get_link_axis_function(
        self,
        link: str,
        axis: Union[str, ArrayType],
        base_link: str,
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get function for computing the link axis.

        @param link Name of the end-effector link.
        @param axis The axis (direction vector) defined in the end-effector frame. If 'x', 'y', 'z' is passed then the corresponding sub-array of the homogenous transform is used.
        @param base_link Name of the base frame link.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the link axis for a given joint state. If n > 1 then an array is computed whose columns correspond to the respective joint state in the input.
        """
        return self.make_function(
            "a",
            link,
            self.get_link_axis,
            n=n,
            base_link=base_link,
            axis=axis,
            numpy_output=numpy_output,
        )

    @arrayify_args
    @listify_output
    def get_global_link_axis(
        self, link: str, q: ArrayType, axis: Union[str, ArrayType]
    ) -> CasADiArrayType:
        """! Compute the link axis, this is a direction vector defined in the end-effector frame (e.g. the x/y/z link axis) in the global frame.

        @param link Name of the end-effector link.
        @param q Joint position array.
        @param axis The axis (direction vector) defined in the end-effector frame. If 'x', 'y', 'z' is passed then the corresponding sub-array of the homogenous transform is used.
        @return Axis defined in the end-effector frame as function of the joint angles.
        """
        return self.get_link_axis(link, q, axis, self.get_root_link())

    def get_global_link_axis_function(
        self,
        link: str,
        axis: Union[str, ArrayType],
        n: int = 1,
        numpy_output: bool = False,
    ) -> cs.Function:
        """! Get function for computing the link axis in the global frame.

        @param link Name of the end-effector link.
        @param axis The axis (direction vector) defined in the end-effector frame. If 'x', 'y', 'z' is passed then the corresponding sub-array of the homogenous transform is used.
        @param numpy_output When true, the output will be a NumPy array.
        @return A CasADi function that computes the link axis for a given joint state. If n > 1 then an array is computed whose columns correspond to the respective joint state in the input.
        """
        get_global_link_axis = functools.partial(self.get_global_link_axis, axis=axis)
        return self.make_function(
            "a", link, get_global_link_axis, n=n, numpy_output=numpy_output
        )

    @arrayify_args
    def rnea(self, q: ArrayType, qd: ArrayType, qdd: ArrayType) -> ArrayType:
        """! Compute inverse dynamics with RNEA.

        NOTE, currently this method only supports revolute/continuous joints.
        The first joint should be fixed and there is no fixed joints except the first joint and the last joint.

        @ param  q: Joint position (unit rad).
        @ param  qd: Joint velocity.
        @ param  qdd: Joint accerleration.
        """
        # Ensure URDF only contains continuous/revolute joints
        rnea_supported_joint_types = {"revolute", "continuous", "fixed"}
        for joint_name, joint in self.urdf.joint_map.items():
            if joint.type not in rnea_supported_joint_types:
                raise JointTypeNotSupported(joint.type)

        if list(self.urdf.joint_map.items())[0][1].type not in {"fixed"}:
            raise JointTypeNotSupported("First joint should be fixed")

        # This is a implementation from the RNEA method Chapter 6 (Page 176) in  Introduction to robotics mechanics and control (3rd version)

        # list of link informations
        masses = [
            link.inertial.mass for link in self.urdf.links if link.inertial is not None
        ]
        massesCenter = [
            link.inertial.origin.xyz
            for link in self.urdf.links
            if link.inertial is not None
        ]
        Inertia = [
            link.inertial.inertia.to_matrix()
            for link in self.urdf.links
            if link.inertial is not None
        ]

        m = cs.np.array(masses[1:])
        cm = cs.np.array(massesCenter[1:]).T
        Icm = cs.np.hstack(tuple(Inertia[1:]))

        xyzs = []
        rpys = []
        axes = []
        # list of joint informations
        joints_list = self.urdf.get_chain(
            self.urdf.get_root(), self.link_names[-1], links=False
        )
        joints_list_r = joints_list[1:]
        for joint_name in joints_list_r:
            joint = self.urdf.joint_map[joint_name]
            xyz, rpy = self.get_joint_origin(joint)
            axis = self.get_joint_axis(joint)
            xyzs.append(xyz)
            rpys.append(rpy)
            axes.append(axis)

        # omega0 (angle velocity of base_link).
        om0 = cs.DM([0.0, 0.0, 0.0])
        # omegaDot0 (angle accerleration of base_link).
        om0D = cs.DM([0.0, 0.0, 0.0])
        # Gravity accerlation of base_link (equals to a gravity force for every joint).
        gravity_para = cs.DM([0.0, 0.0, -9.81])
        # external force of the base_link.
        fs = [cs.DM([0.0, 0.0, 0.0])]
        # external force of the torque.
        ns = [cs.DM([0.0, 0.0, 0.0])]

        oms = [om0]
        omDs = [om0D]
        vDs = [-gravity_para]

        """
        $$ Forward part of RNEA $$
        link0->1, link1->2 ...., link7->end-effector (for example)
        joint0,   joint1 ....
        """

        oms = [om0]
        omDs = [om0D]
        vDs = [-gravity_para]

        # 2.1 forward part of RNEA
        """
        link0->1, link1->2 ...., link7->end-effector (for example).
        joint0,   joint1 ....
        
        """
        for i in range(len(joints_list_r)):
            if i != len(joints_list_r) - 1:
                iRp = (rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])).T
                iaxisi = iRp @ axes[i]
                omi = iRp @ oms[i] + iaxisi * qd[i]
                omDi = (
                    iRp @ omDs[i]
                    + skew(iRp @ oms[i]) @ (iaxisi * qd[i])
                    + iaxisi * qdd[i]
                )
            else:
                iRp = rpy2r(rpys[i]).T
                omi = iRp @ oms[i]
                omDi = iRp @ omDs[i]

            vDi = iRp @ (
                vDs[i]
                + skew(omDs[i]) @ xyzs[i]
                + skew(oms[i]) @ (skew(oms[i]) @ xyzs[i])
            )

            fi = m[i] * (
                vDi + skew(omDi) @ cm[:, i] + skew(omi) @ (skew(omi) @ cm[:, i])
            )
            ni = (
                Icm[:, i * 3 : i * 3 + 3] @ omDi
                + skew(omi) @ Icm[:, i * 3 : i * 3 + 3] @ omi
            )

            oms.append(omi)
            omDs.append(omDi)
            vDs.append(vDi)
            fs.append(fi)
            ns.append(ni)

        """
        $$ Backward part of RNEA $$
        """

        ifi = fs[-1]
        ini = ns[-1] + skew(cm[:, -1]) @ fs[-1]

        taus = []

        for i in range(len(joints_list_r) - 1, 0, -1):
            if i < len(joints_list_r) - 1:
                pRi = rpy2r(rpys[i]) @ angvec2r(q[i], axes[i])
            elif i == len(joints_list_r) - 1:
                pRi = rpy2r(rpys[i])
            else:
                pRi = rpy2r(rpys[i])

            ini = (
                ns[i]
                + pRi @ ini
                + skew(cm[:, i - 1]) @ fs[i]
                + skew(xyzs[i]) @ pRi @ ifi
            )
            ifi = pRi @ ifi + fs[i]
            pRi = rpy2r(rpys[i - 1]) @ angvec2r(q[i - 1], axes[i - 1])
            _tau = ini.T @ pRi.T @ axes[i - 1]
            taus.append(_tau)

        tau_ = cs.vertcat(*[taus[k] for k in range(len(taus) - 1, -1, -1)])

        return tau_
