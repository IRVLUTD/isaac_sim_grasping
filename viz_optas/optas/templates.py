import abc
import time
import yaml
import functools
import casadi as cs
from scipy import interpolate
from typing import Union, Dict, Callable, Any


class Manager(abc.ABC):
    """! Base manager class."""

    def __init__(
        self, config_filename: Union[None, str] = None, record_solver_perf: bool = False
    ):
        """! Initializer for the Manger class.

        @param config_filename Filename for a YAML configuration file. When None is passed, it is assumed there is no configuration.
        @param record_solver_perf When true the solver duration is recorded when there is a call to solve.
        @return An instance of the Manger class.
        """
        self.reset_manager()
        self.config_filename = config_filename
        self.record_solver_perf = record_solver_perf
        self.config = self._load_configuration(self.config_filename)
        self.solver = self.setup_solver()
        self.solve = self._initialize_solve_method()

    def reset_manager(self) -> None:
        """! Reset basic variables in the manager."""
        self.num_solves = 0  # number of times _solve is called
        self.solver_duration = None
        self.solution = None

    def _load_configuration(self, filename) -> Dict:
        """! Load the configuration file.

        @param filename Filename for the YAML file.
        @return Dictionary of the configuration.
        """
        config = {}
        if filename:
            with open(filename, "rb") as config_file:
                config = yaml.load(config_file, Loader=yaml.FullLoader)
        return config

    def _initialize_solve_method(self) -> Callable:
        """! Initialize the solve method, i.e. whether to record solver duration or not."""
        if self.record_solver_perf:
            solve = self._solve_and_time
        else:
            solve = self._solve
        return solve

    def _solve(self) -> None:
        """! Solves the optimization problem."""
        self.solution = self.solver.solve()
        self.num_solves += 1

    def _solve_and_time(self) -> None:
        """! Solves the optimization problem and times."""
        t0 = time.perf_counter()
        self._solve()
        t1 = time.perf_counter()
        self.solver_duration = t1 - t0

    def get_solver_duration(self) -> float:
        """! Returns the duration of the solver.

        @return Duration in secs.
        """
        return self.solver_duration

    def is_first_solve(self) -> bool:
        """! True when the solver is being run for the first time.

        @return Boolean indicating if this is the first time the solver is called.
        """
        return self.num_solves == 0

    ####################################################################################
    ## Abstract methods

    @abc.abstractmethod
    def setup_solver(self) -> None:
        """! Setup the optimization solver. This is an abstract method."""
        pass

    @abc.abstractmethod
    def is_ready(self) -> bool:
        """! True when manager is ready to use. This is an abstract method."""
        pass

    @abc.abstractmethod
    def reset(self) -> None:
        """! Reset the optimization problem. This is an abstract method."""
        pass

    @abc.abstractmethod
    def get_target(self) -> cs.DM:
        """! Return the target from the solution. For example, the target joint states. This is an abstract method.

        @return The part of the solution that represents the target. E.g. the next step in a motion plan.
        """
        pass


class ROSManager(Manager):
    """! Manager setup specifically for ROS."""

    ## ROS state listener.
    ## Dictionary that defines the state listener. This is used to
    ## setup subscribers that listen to the input that defines the
    ## parameters for the solver. The key should be the topic name, and
    ## the value should be the message type. E.g.
    ##     "joint_states": JointState
    state_listener = {}

    def __init__(
        self,
        rosapi: Any,
        rosver: int,
        config_filename: Union[None, str],
        record_solver_perf: bool = False,
    ):
        """! Initializer for the ROSManger class.

        @param rosapi For ROS 1, this is rospy (ie. import rospy, and pass the module). For ROS 2, this is the node.
        @param rosver When using ROS 1, pass 1, and if using ROS 2, pass 2.
        @param config_filename Filename for a YAML configuration file. When None is passed, it is assumed there is no configuration.
        @param record_solver_perf When true the solver duration is recorded when there is a call to solve.
        @return An instance of the ROSManager class.
        """
        super().__init__(
            config_filename=config_filename, record_solver_perf=record_solver_perf
        )

        ## For ROS 1, this is rospy (ie. import rospy, and pass the module). For ROS 2, this is the node.
        self.rosapi = rosapi

        ## ROS version (i.e. 1 or 2).
        self.rosver = rosver

        # Setup target publisher
        from std_msgs.msg import Float64MultiArray  # available in ros1/2

        ## Float array message type
        self.Float64MultiArray = Float64MultiArray

        ## Target publisher
        self.target_pub = self._setup_target_publisher()

        # Setup publishers/subscriber

        ## Dictionary containing messages from the state listener
        self.msgs = (
            {}
        )  # for storing messages from subscribers setup in create_state_listener
        self.create_state_listener()  # user defined method

    def _setup_target_publisher(self) -> Any:
        """! Creates the target publisher.

        @return Instance of the target publisher.
        """
        if self.rosver == 1:
            target_pub = self.rosapi.Publisher(
                "target", Float64MultiArray, queue_size=10
            )
        elif self.rosver == 2:
            target_pub = self.rosapi.create_publisher(
                self.Float64MultiArray, "target", 10
            )
        else:
            raise ValueError(f"Did not recognize ros version, given '{self.rosver}'")
        return target_pub

    def add_subscriber(self, topic_name: str, msg_type: Any) -> None:
        """! Creates a subscriber.

        @param topic_name Name of the topic to subscribe.
        @param msg_type The message type for the given topic.
        """
        self.msgs[topic_name] = None
        callback = functools.partial(self._callback, topic_name=topic_name)
        if self.rosver == 1:
            self.rosapi.Subscriber(topic_name, msg_type, callback)
        elif self.rosver == 2:
            self.rosapi.create_subscription(msg_type, topic_name, callback, 10)
        else:
            raise ValueError(f"Did not recognize ros version, given '{self.rosver}'")

    def _callback(self, msg: Any, topic_name: str) -> None:
        """! The subcriber callback.

        @param msg The message from ROS.
        @param topic_name the name of the topic the message is from.
        """
        self.msgs[topic_name] = msg

    def get_state(self, topic_name: str) -> Any:
        """! Get the most current message.

        @return The message from ROS. None is returned if the message is not yet received.
        """
        return self.msgs[topic_name]

    def is_ready(self) -> bool:
        """! True when messages have been recieved on all topics.

        @return Boolean indicating if the manager is ready to use.
        """
        return all(m is not None for m in self.msgs.values())

    def publish_target(self, target: cs.DM) -> None:
        """! Publish a target to ROS.

        @param target The target array.
        """
        msg = self.Float64MultiArray(data=target.toarray().flatten().tolist())
        self.target_pub.publish(msg)

    def create_state_listener(self) -> None:
        """! Creates the state listener."""
        for topic_name, msg_type in self.state_listener.items():
            if topic_name == "tf2":
                raise NotImplementedError(
                    "Listening to frames from the tf2 library has not yet been implemented."
                )
            self.add_subscriber(topic_name, msg_type)


class ROSController(ROSManager):
    """! A class for implementing controllers."""

    def __init__(
        self,
        rosapi: Any,
        rosver: int,
        config_filename: Union[None, str],
        hz: int,
        urdf_string: str,
        record_solver_perf: bool = False,
    ):
        """! Initializer for the ROSController class.

        @param rosapi For ROS 1, this is rospy (ie. import rospy, and pass the module). For ROS 2, this is the node.
        @param rosver When using ROS 1, pass 1, and if using ROS 2, pass 2.
        @param config_filename Filename for a YAML configuration file. When None is passed, it is assumed there is no configuration.
        @param hz The sampling frequency (in Hz) that the controller will run.
        @param urdf_string The URDF for the robot passed as a string.
        @param record_solver_perf When true the solver duration is recorded when there is a call to solve.
        @return An instance of the ROSController class.
        """

        ## The URDF for the robot as a string.
        self.urdf_string = urdf_string

        ## Sampling frequency for the controller.
        self.hz = hz

        super().__init__(
            rosapi, rosver, config_filename, record_solver_perf=record_solver_perf
        )

    def __call__(self) -> None:
        """! Step the controller. Reset the problem, solve the problem, and pubslih the target."""
        self.reset()
        self.solve()
        self.publish_target(self.get_target())


class ROSPlanner(ROSManager):
    """! A class for implementing motion planners."""

    def __init__(
        self,
        rosapi: Any,
        rosver: int,
        config_filename: Union[None, str],
        record_solver_perf: bool = False,
    ):
        """! Initializer for the ROSPlanner class.

        @param rosapi For ROS 1, this is rospy (ie. import rospy, and pass the module). For ROS 2, this is the node.
        @param rosver When using ROS 1, pass 1, and if using ROS 2, pass 2.
        @param config_filename Filename for a YAML configuration file. When None is passed, it is assumed there is no configuration.
        @param record_solver_perf When true the solver duration is recorded when there is a call to solve.
        @return An instance of the ROSPlanner class.
        """

        ## Duration of the motion plan (set using set_duration).
        self.duration = None

        super().__init__(
            rosapi, rosver, config_filename, record_solver_perf=record_solver_perf
        )

    def set_duration(self, duration: float) -> None:
        """Set the duration for the motion plan."""
        self.duration = duration

    def get_duration(self) -> Union[None, float]:
        """Return duration for the motion plan. When None is returned, this means that the duration is not yet set.
        
        @return Duration of the motion plan.
        """
        return self.duration

    def plan(self) -> interpolate.interp1d:
        """! Solve the planning problem. Reset the problem, solve the problem, and interpolate the result.
        
        @return The interpolated plan as a function of time.
        """
        self.reset()
        self.solve()
        duration = self.get_duration()
        solution = self.get_target()
        return self.solver.interpolate(solution, duration, **interp_kwargs)
