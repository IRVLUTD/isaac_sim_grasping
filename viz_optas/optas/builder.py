"""! @brief The optimization builder class is defined."""

import casadi as cs
from .sx_container import SXContainer
from .mx_container import MXContainer
from .spatialmath import arrayify_args, ArrayType, CasADiArrayType
from .optimization import *
from .models import Model, TaskModel, RobotModel
from typing import List, Union


class OptimizationBuilder:
    """! OptimizationBuilder allows you to build/specify an optimization problem."""

    def __init__(
        self,
        T: int,
        robots: List[RobotModel] = [],
        tasks: List[TaskModel] = [],
        derivs_align: bool = False,
    ):
        """! OptimizationBuilder constructor.

        @param T Number of time steps for the trajectory.
        @param robots A list of robot models.
        @param tasks A list of task models.
        @param derivs_align When true, the time derivatives align for each time step. Default is False.
        @return An instance of the OptimizationBuilder class.
        """

        # Input check
        assert T > 0, f"T must be strictly positive"

        if not isinstance(robots, list):
            robots = [robots]  # allow user to pass a single robot

        if not isinstance(tasks, list):
            tasks = [tasks]  # all user to pass a single task

        # Class attributes

        ## Number of time steps for the trajectory.
        self.T = T

        ## List of models.
        self._models = robots + tasks

        ## When true, the time derivatives align for each time step.
        self.derivs_align = derivs_align

        # Ensure T is sufficiently large
        if not derivs_align and len(self._models) > 0:
            # Get max time deriv
            all_time_derivs = []
            for m in self._models:
                all_time_derivs += m.time_derivs
            max_time_deriv = max(all_time_derivs)

            # Check T is large enough
            Tmin = max_time_deriv + 1
            # assert T >= Tmin, f"T={T} is too low, it should be at least {Tmin}"

        model_names = [m.get_name() for m in self._models]
        is_unique_names = len(model_names) == len(set(model_names))
        assert is_unique_names, "each model should have a unique name"

        # Setup containers for decision variables, parameters, cost terms, ineq/eq constraints

        ## MXContainer containing decision variables.
        self._decision_variables = MXContainer()

        ## MXContainer containing parameters.
        self._parameters = MXContainer()

        ## MXContainer containing the cost terms.
        self._cost_terms = MXContainer()

        ## MXContainer containing the linear equality constraints.
        self._lin_eq_constraints = MXContainer()

        ## MXContainer containing the linear inequality constraints.
        self._lin_ineq_constraints = MXContainer()

        ## MXContainer containing the inequality constraints.
        self._ineq_constraints = MXContainer()

        ## MXContainer containing the equality constraints.
        self._eq_constraints = MXContainer()

        # Setup decision variables and parameters
        for model in self._models:
            for d in model.time_derivs:
                n_s_x = model.state_optimized_name(d)
                t = T - d if not derivs_align else T
                if isinstance(model, RobotModel):
                    self.add_decision_variables(n_s_x, model.num_opt_joints, t)
                    n_s_p = model.state_parameter_name(d)
                    self.add_parameter(n_s_p, model.num_param_joints, t)
                else:
                    self.add_decision_variables(n_s_x, model.dim, t, model.is_discrete)

    def get_model_names(self) -> List[str]:
        """! Return the names of each model.

        @return List of model names.
        """
        return [model.name for model in self._models]

    def get_model_index(self, name: str) -> int:
        """! Return the index of the model in the list of models.

        @param name Name of the model.
        @return Index of the model in the list of models.
        """
        return self.get_model_names().index(name)

    def get_model(self, name: str) -> Model:
        """! Return the model with given name.

        @param name Name of the model.
        @return A task or robot model.
        """
        return self._models[self.get_model_index(name)]

    def get_model_state(
        self, name: str, t: int, time_deriv: int = 0
    ) -> CasADiArrayType:
        """! Get the model state at a given time.

        @param name Name of the model.
        @param t Index of the desired state.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return The state vector where dim is the model dimension.
        """
        states = self.get_model_states(name, time_deriv)
        return states[:, t]

    def get_model_states(self, name: str, time_deriv: int = 0) -> CasADiArrayType:
        """! Get the full state trajectory for a given model.

        @param name Name of the model.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.).
        @return The state vector where dim is the model dimension, and T is the number of time-steps in the trajectory.
        """
        model = self.get_model(name)
        assert (
            time_deriv in model.time_derivs
        ), f"model '{name}', was not specified with time derivative to order {time_deriv}"
        name = model.state_optimized_name(time_deriv)
        return self._decision_variables[name]

    def get_model_parameters(self, name: str, time_deriv: int = 0) -> CasADiArrayType:
        """! Get the array of parameters for a given model.

        @param name Name of the model.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.).
        @return The array of parameters where dim is the number of model parameters, and T is the number of time-steps in the trajectory.
        """
        model = self.get_model(name)
        assert (
            time_deriv in model.time_derivs
        ), f"model '{name}', was not specified with time derivative to order {time_deriv}"
        name = model.state_parameter_name(time_deriv)
        return self._parameters[name]

    def get_model_parameter(
        self, name: str, t: int, time_deriv: int = 0
    ) -> CasADiArrayType:
        """! Get the model parameter at a given time.

        @param name Name of the model.
        @param t Index of the desired state.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.).
        @return The parameter vector where dim is the model number of parameters
        """
        parameters = self.get_model_parameters(name, time_deriv)
        return parameters[:, t]

    def get_robot_states_and_parameters(
        self, name: str, time_deriv: int = 0
    ) -> CasADiArrayType:
        """! Get the vector of states and parameters for a given model.

        Note that method only applies to to RobotModel.
        To be replaced by get_model_states_and_parameters once parameters are added to base class Model.

        @param name Name of the model.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.)
        @return The vector of parameters where dim is the number of model states and parameters (for a robot it should correspond to the degrees of freedom), and T is the number of time-steps in the trajectory.
        """

        model = self.get_model(name)
        assert isinstance(model, RobotModel), "this method only applies to robot models"

        states = self.get_model_states(name, time_deriv=time_deriv)
        parameters = self.get_model_parameters(name, time_deriv=time_deriv)

        states_and_params = cs.MX.zeros(model.dim, max(1, self.T - time_deriv))
        for idx in range(model.num_param_joints):
            states_and_params[model.parameter_joint_indexes[idx], :] = parameters[
                idx, :
            ]
        for idx in range(model.num_opt_joints):
            states_and_params[model.optimized_joint_indexes[idx], :] = states[idx, :]
        return states_and_params

    def _x(self) -> CasADiArrayType:
        """! Return the decision variables as a casadi.SX vector.

        @return Symbolic decision variables.
        """
        return self._decision_variables.vec()

    def _p(self) -> CasADiArrayType:
        """! Return the parameters as a casadi.SX vector.

        @return Symbolic parameters.
        """
        return self._parameters.vec()

    def _is_linear_in_x(self, y: cs.MX) -> cs.DM:
        """! Returns true DM(1) if y is a linear function of the decision variables, false DM(0) otherwise.

        @param y Symbolic function of interest.
        @return True if y is linear in x.
        """
        return cs.is_linear(y, self._x())

    def _cost(self) -> CasADiArrayType:
        """! Returns the cost function.

        @return The symbolic cost function.
        """
        return cs.sum1(self._cost_terms.vec())

    def is_cost_quadratic(self) -> cs.DM:
        """! True DM(1) when cost function is quadratic in the decision variables, False DM(0) otherwise.

        @return Truth value if the cost function is quadratic or not.
        """
        return cs.is_quadratic(self._cost(), self._x())

    #
    # Upate optimization problem
    #

    def add_decision_variables(
        self, name: str, m: int = 1, n: int = 1, is_discrete: bool = False
    ) -> cs.MX:
        """! Add decision variables to the optimization problem.

        @param name Name of decision variable array.
        @param m Number of rows in decision variable array. Default is 1.
        @param n Number of columns in decision variable array. Default is 1.
        @param is_discret If true, then the decision variables are treated as discrete variables. Default is False.
        @return Array of the decision variables.
        """
        x = cs.MX.sym(name, m, n)
        self._decision_variables[name] = x
        if is_discrete:
            self._decision_variables.variable_is_discrete(name)
        return x

    def add_parameter(self, name: str, m: int = 1, n: int = 1) -> cs.MX:
        """! Add a parameter to the optimization problem.

        @param name Name of parameter array.
        @param m Number of rows in parameter array. Default is 1.
        @param n Number of columns in parameter array. Default is 1.
        @return Array of the parameters.
        """
        p = cs.MX.sym(name, m, n)
        self._parameters[name] = p
        return p

    @arrayify_args
    def add_cost_term(self, name: str, cost_term: cs.MX) -> None:
        """! Add cost term to the optimization problem.

        @param name Name for cost function.
        @param cost_term Cost term, must be an array with shape 1-by-1.
        """
        cost_term = cs.vec(cost_term)
        m, n = cost_term.shape
        assert m == 1 and n == 1, "cost term must be scalar"
        self._cost_terms[name] = cost_term

    @arrayify_args
    def add_geq_inequality_constraint(
        self, name: str, lhs: CasADiArrayType, rhs: Union[None, CasADiArrayType] = None
    ) -> None:
        """! Add the inequality constraint lhs >= rhs to the optimization problem.

        @param name Name for the constraint.
        @param lhs Left-hand side for the inequality constraint.
        @param rhs Right-hand side for the inequality constraint. If None, then it is replaced with the zero array with the same shape as lhs.
        """
        if rhs is None:
            rhs = cs.DM.zeros(*lhs.shape)
        self.add_leq_inequality_constraint(name, rhs, lhs)

    @arrayify_args
    def add_leq_inequality_constraint(
        self, name: str, lhs: CasADiArrayType, rhs: Union[None, CasADiArrayType] = None
    ) -> None:
        """! Add the inequality constraint lhs <= rhs to the optimization problem.

        @param name Name for the constraint.
        @param lhs Left-hand side for the inequality constraint.
        @param rhs Right-hand side for the inequality constraint. If None, then it is replaced with the zero array with the same shape as lhs.
        """
        if rhs is None:
            rhs = cs.DM.zeros(*lhs.shape)
        diff = rhs - lhs  # diff >= 0
        if self._is_linear_in_x(diff):
            self._lin_ineq_constraints[name] = diff
        else:
            self._ineq_constraints[name] = diff

    @arrayify_args
    def add_bound_inequality_constraint(
        self,
        name: str,
        lhs: CasADiArrayType,
        mid: CasADiArrayType,
        rhs: CasADiArrayType,
    ) -> None:
        """! Add the inequality constraint lhs <= mid <= rhs to the optimization problem.

        @param name Name for the constraint.
        @param lhs Left-hand side for the inequality constraint.
        @param mid Middle part of the inequality constraint.
        @param rhs Right-hand side for the inequality constraint.
        """
        self.add_leq_inequality_constraint(name + "_l", lhs, mid)
        self.add_leq_inequality_constraint(name + "_r", mid, rhs)

    @arrayify_args
    def add_equality_constraint(
        self,
        name: str,
        lhs: CasADiArrayType,
        rhs: Union[None, CasADiArrayType] = None,
        reduce_constraint=False,
    ) -> None:
        """! Add the equality constraint lhs == rhs to the optimization problem.

        @param name Name for the constraint.
        @param lhs Left-hand side for the inequality constraint.
        @param rhs Right-hand side for the inequality constraint. If None, then it is replaced with the zero array with the same shape as lhs.
        @param reduce_constraint When True, the constraints are specified by ||lhs - rhs||^2 = 0. This has the effect of reducing the number of equality constraints in the problem. However, if the constraints are linear they will be treated as nonlinear. Default is False.
        """
        if rhs is None:
            rhs = cs.DM.zeros(*lhs.shape)
        diff = rhs - lhs  # diff == 0
        if reduce_constraint:
            diff = cs.sumsqr(diff)
        if self._is_linear_in_x(diff):
            self._lin_eq_constraints[name] = diff
        else:
            self._eq_constraints[name] = diff

    #
    # Common constraints
    #

    def sphere_collision_avoidance_constraints(
        self, name, obstacle_names, link_names=None, base_link=None
    ) -> None:
        """! Add collision avoidance constraints, based on spherical representations attached to the robot links and placed in the world.

        @param name Name of the model.
        @param obstacle_names A list of strings with names of obstacles. For each object, parameters will be created with labels NAME_position (size 3) and NAME_radii (size 1) that represent the obstacle position and radius respectively.
        @param link_names The links you want to attach collision avoidance constraints onto; for each link a parameter is created with label NAME_radii (size 1). When None is passed, a sphere is attcahed to each link specified in the URDF.
        @param base_link Name of the base frame link. When None is passed, the root link in the URDF is used.
        """
        # Get model
        model = self.get_model(name)
        assert isinstance(model, RobotModel), "this method only applies to robot models"

        # Update base link
        if base_link is None:
            base_link = model.get_root_link()

        # Get model states
        Q = self.get_model_states(name)
        n = Q.shape[1]

        # Setup inks
        if link_names is None:
            link_names = model.link_names
        assert len(link_names), "at least one link should be named"
        links = {}
        for name in link_names:
            links[name] = (
                model.get_link_position_function(name, base_link),
                self.add_parameter(name + "_radii"),
            )

        # Setup obstacles
        assert len(obstacle_names), "at least one obstacle should be named"
        obstacles = {}
        for name in obstacle_names:
            obstacles[name] = (
                self.add_parameter(name + "_position", 3),
                self.add_parameter(name + "_radii"),
            )

        # Add constraints
        for t in range(n):
            q = Q[:, t]
            for link_name, (pos, linkrad) in links.items():
                p = pos(q)
                for obs_name, (obs, obsrad) in obstacles.items():
                    name = f"sphere_col_avoid_{t}_{link_name}_{obs_name}"
                    dist2 = cs.sumsqr(p - obs)
                    bnd2 = (linkrad + obsrad) ** 2
                    self.add_leq_inequality_constraint(name, bnd2, dist2)

    def integrate_model_states(
        self, name: str, time_deriv: int, dt: CasADiArrayType
    ) -> None:
        """! Integrates the model states over time.

        @param name Name of the model.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.).
        @param dt Integration time step. If the value is a scalar then the value is used as the time step for the trajectory. When an array is passed, it should have the correct length: ```n = T - (1 if derivs_align else time_deriv)``` where T and derivs_align are flags passed to the optimization builder constructor.
        """

        def setup_integr_func(dim, n):
            """Returns an integration function where m is the state dimension,
            and n is the number of trajectory points.
            """
            xd = cs.MX.sym("xd", dim)
            x0 = cs.MX.sym("x0", dim)
            x1 = cs.MX.sym("x1", dim)
            dt = cs.MX.sym("dt")
            integr = cs.Function("integr", [x0, x1, xd, dt], [x0 + dt * xd - x1])
            return integr.map(n)

        # Ensure dt is an 1-by-n array
        n = self.T - (1 if self.derivs_align else time_deriv)
        if isinstance(dt, (float, int)):
            dt = dt * cs.DM.ones(n)

        if isinstance(dt, (cs.DM, cs.MX)):
            dt = cs.vec(dt)
            if dt.shape[0] == 1:
                dt = dt * cs.DM.ones(n)

        dt = cs.vec(dt).T
        assert (
            dt.shape[1] == n
        ), f"The array for dt has an incorrect length, expected {n}, got {dt.shape[1]}"

        # Extract model states
        model = self.get_model(name)
        xd = self.get_model_states(name, time_deriv)
        x = self.get_model_states(name, time_deriv - 1)

        if self.derivs_align:
            xd = xd[:, :-1]

        # Integrate states
        if isinstance(model, RobotModel):
            integr = setup_integr_func(model.num_opt_joints, n)
        else:
            integr = setup_integr_func(model.dim, n)
        name = f"__integrate_model_states_{name}_{time_deriv}__"
        self.add_equality_constraint(name, integr(x[:, :-1], x[:, 1:], xd, dt))

    def enforce_model_limits(
        self,
        name: str,
        time_deriv: int = 0,
        lo: Union[None, CasADiArrayType] = None,
        up: Union[None, CasADiArrayType] = None,
        safe_frac=1.0,
    ) -> None:
        """! Enforce model limits.

        @param name Name of model.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.).
        @param lo Lower limits, if None then model limits specified in the model class are used.
        @param up Upper limits, if None then model limits specified in the model class are used.
        @param safe_frac When safe_frac < 1.0, the joint limits are reduced by a fraction of their original values.
        """
        assert (
            0.0 < safe_frac <= 1.0
        ), f"Given safe_frac '{safe_frac}' must be in range (0, 1]."
        x = self.get_model_states(name, time_deriv)
        xlo = lo
        xup = up
        if (xlo is None) or (xup is None):
            mlo, mup = self.get_model(name).get_limits(time_deriv)
            if xlo is None:
                xlo = mlo
            if xup is None:
                xup = mup
        if safe_frac < 1.0:
            mid = 0.5 * (xlo + xup)
            diff = xup - xlo
            small_diff = safe_frac * diff
            xlo_use = mid - 0.5 * small_diff
            xup_use = mid + 0.5 * small_diff
        else:
            xlo_use = xlo
            xup_use = xup
        n = f"__{name}_model_limit_{time_deriv}__"
        self.add_bound_inequality_constraint(n, xlo_use, x, xup_use)

    def initial_configuration(
        self, name: str, init: Union[None, CasADiArrayType] = None, time_deriv: int = 0
    ) -> None:
        """! Set initial configuration.

        @param name Name of model.
        @param init Initial configuration. If None is passed then this is assumed to be zero.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.). The default is 0.
        """
        t0 = 0
        x0 = self.get_model_state(name, t0, time_deriv=time_deriv)
        n = f"__{name}_initial_configuration_{time_deriv}__"
        self.add_equality_constraint(n, lhs=x0, rhs=init)  # init will be zero when None

    def fix_configuration(
        self, name: str, config: CasADiArrayType = None, time_deriv: int = 0, t: int = 0
    ) -> None:
        """! Fix configuration.

        @param name Name of model.
        @param config The configuration. When None is passed then it is considered to be zero.
        @param time_deriv The time-deriviative required (i.e. position is 0, velocity is 1, etc.).
        @param t Index for the configuration in trajectory (by default this is the first element but it could also be the last for example in moving horizon estimation).
        """
        x0 = self.get_model_state(name, t, time_deriv=time_deriv)
        n = f"__{name}_fix_configuration_{time_deriv}_{t}__"
        self.add_equality_constraint(
            n, lhs=x0, rhs=config
        )  # config will be zero when None

    #
    # Main build method
    #

    def build(self) -> Optimization:
        """! Build the optimization problem."""

        # Setup optimization
        nlin = (
            self._lin_ineq_constraints.numel() + self._lin_eq_constraints.numel()
        )  # total no. linear constraints
        nnlin = (
            self._ineq_constraints.numel() + self._eq_constraints.numel()
        )  # total no. nonlinear constraints

        if self.is_cost_quadratic():
            # True -> use QP formulation

            if self._decision_variables.has_discrete_variables():
                opt = MixedIntegerNonlinearCostNonlinearConstrained(
                    self._decision_variables,
                    self._parameters,
                    self._cost_terms,
                    self._lin_eq_constraints,
                    self._lin_ineq_constraints,
                    self._eq_constraints,
                    self._ineq_constraints,
                )
            else:
                if nnlin > 0:
                    opt = QuadraticCostNonlinearConstraints(
                        self._decision_variables,
                        self._parameters,
                        self._cost_terms,
                        self._lin_eq_constraints,
                        self._lin_ineq_constraints,
                        self._eq_constraints,
                        self._ineq_constraints,
                    )
                elif nlin > 0:
                    opt = QuadraticCostLinearConstraints(
                        self._decision_variables,
                        self._parameters,
                        self._cost_terms,
                        self._lin_eq_constraints,
                        self._lin_ineq_constraints,
                    )
                else:
                    opt = QuadraticCostUnconstrained(
                        self._decision_variables,
                        self._parameters,
                        self._cost_terms,
                    )
        else:
            # False -> use (nonlinear) Optimization formulation

            if self._decision_variables.has_discrete_variables():
                opt = MixedIntegerNonlinearCostNonlinearConstrained(
                    self._decision_variables,
                    self._parameters,
                    self._cost_terms,
                    self._lin_eq_constraints,
                    self._lin_ineq_constraints,
                    self._eq_constraints,
                    self._ineq_constraints,
                )
            else:
                if nnlin > 0:
                    opt = NonlinearCostNonlinearConstraints(
                        self._decision_variables,
                        self._parameters,
                        self._cost_terms,
                        self._lin_eq_constraints,
                        self._lin_ineq_constraints,
                        self._eq_constraints,
                        self._ineq_constraints,
                    )
                elif nlin > 0:
                    opt = NonlinearCostLinearConstraints(
                        self._decision_variables,
                        self._parameters,
                        self._cost_terms,
                        self._lin_eq_constraints,
                        self._lin_ineq_constraints,
                    )
                else:
                    opt = NonlinearCostUnconstrained(
                        self._decision_variables,
                        self._parameters,
                        self._cost_terms,
                    )

        opt.set_models(self._models)

        return opt
