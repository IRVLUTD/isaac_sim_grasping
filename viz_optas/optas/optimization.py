import casadi as cs
from .models import Model
from .mx_container import MXContainer
from .spatialmath import CasADiArrayType
from typing import List, Tuple


def derive_jacobian_and_hessian_functions(
    name: str, fun: cs.Function, x: CasADiArrayType, p: CasADiArrayType
) -> Tuple[cs.Function]:
    """! Compute the Jacobian and Hessian for a given function using automatic differentiation.

    @param name The function name.
    @param fun The CasADi function.
    @param x The variables of the function.
    @param p The parameters of the function.
    @return The Jacobian and Hessian that are the derivatives of the function wrt the variables x.
    """
    fun_input = [x, p]
    jac = cs.jacobian(fun(x, p), x)
    hes = cs.jacobian(jac, x)
    Jac = cs.Function("d" + name, fun_input, [jac])
    Hes = cs.Function("dd" + name, fun_input, [hes])
    return Jac, Hes


def vertcon(
    x: CasADiArrayType,
    p: CasADiArrayType,
    ineq: List[cs.Function] = [],
    eq: List[cs.Function] = [],
) -> cs.Function:
    """! Align inequality and equality constraints vertically.

    Given an inequality constraint ineq(x, p) and equality constraint eq(x, p). The method that is returned evaluates the following array.

                  [ ineq(x, p) ]
        v(x, p) = [   eq(x, p) ]
                  [  -eq(x, p) ]

    @param x The variables of the functions.
    @param p The parameters of the functions.
    @param ineq A list of inequality constraints.
    @param eq A list of equality constraints.
    @return A CasADi function that evaluates the constraints in the form v(x, p) >= 0 (see above).
    """
    con = [i(x, p) for i in ineq]
    for e in eq:
        con.append(e(x, p))
        con.append(-e(x, p))
    return cs.Function("v", [x, p], [cs.vertcat(*con)])


class Optimization:
    """! Base optimization class."""

    ## big number rather than np.inf
    inf = 1.0e10

    def __init__(
        self,
        decision_variables: MXContainer,
        parameters: MXContainer,
        cost_terms: MXContainer,
    ):
        """! Initializer for the Optimization class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms.
        @return Instance of the Optimization class.
        """
        # Set class attributes

        ## A list of the task and robot models (set during build method in the OptimizationBuilder class)
        self.models = None

        ## MXContainer containing decision variables.
        self.decision_variables = decision_variables

        ## MXContainer containing parameters.
        self.parameters = parameters

        ## MXContainer containing cost terms.
        self.cost_terms = cost_terms

        ## MXContainer containing linear equality constraints.
        self.lin_eq_constraints = {}

        ## MXContainer containing linear inequality constraints.
        self.lin_ineq_constraints = {}

        ## MXContainer containing equality constraints.
        self.eq_constraints = {}

        ## MXContainer containing inequality constraints.
        self.ineq_constraints = {}

        ## CasADi function that evaluates the P term in the cost function (note, this only applies to problems with a quadratic cost function).
        self.P = None

        ## CasADi function that evaluates the q term in the cost function (note, this only applies to problems with a quadratic cost function).
        self.q = None

        ## CasADi function that evaluates the linear equality constraints.
        self.k = None

        ## Number of linear inequality constraints.
        self.nk = 0

        ## Lower bound for the linear inequality constraints (i.e. zeros).
        self.lbk = None

        ## Upper bound for the linear inequality constraints (i.e. inf).
        self.ubk = None

        ## CasADi function that evaluates the M term in the linear inequality constraints.
        self.M = None

        ## CasADi function that evaluates the c term in the linear inequality constraints.
        self.c = None

        ## CasADi function that evaluates the linear equality constraints.
        self.a = None

        ## Number of linear equality constraints.
        self.na = 0

        ## Lower bound for the linear equality constraints (i.e. zeros).
        self.lba = None

        ## Upper bound for the linear equality constraints (i.e. zeros).
        self.uba = None

        ## CasADi function that evaluates the A term in the linear equality constraints.
        self.A = None

        ## CasADi function that evaluates the b term in the linear equality constraints.
        self.b = None

        ## CasADi function that evaluates the inequality constraints
        self.g = None

        ## Number of inequality constraints.
        self.ng = 0

        ## Lower bound for the inequality constraints (i.e. zeros).
        self.lbg = None

        ## Upper bound for the inequality constraints (i.e. inf).
        self.ubg = None

        ## CasADi function that evaluates the equality constraints
        self.h = None

        ## Number of equality constraints.
        self.nh = 0

        ## Lower bound for the equality constraints (i.e. zeros).
        self.lbh = None

        ## Upper bound for the equality constraints (i.e. zeros).
        self.ubh = None

        ## CasADi function that evaluates the constraints as a verticle column (set when specify_v is called), see vertcon.
        self.v = None

        ## Number of vectorized constraints (see vertcon).
        self.nv = 0

        ## Lower bound for the verticle constraints v (i.e. zeros).
        self.lbv = None

        ## Upper bound for the verticle constraints v (i.e. inf).
        self.ubv = None

        ## CasADi function that evaluates the Jacobian of the constraints v (set when specify_v is called).
        self.dv = None

        ## CasADi function that evaluates the Hessian of the constraints v (set when specify_v is called).
        self.ddv = None

        # Get symbolic variables

        ## Vectorized decision variables.
        self.x = decision_variables.vec()  # symbolic decision variables

        ## Vectorized parameters.
        self.p = parameters.vec()  # symbolic parameters

        # Setup objective function.
        f = cs.sum1(cost_terms.vec())

        ## CasADi function that evaluates the objective function.
        self.f = cs.Function("f", [self.x, self.p], [f])

        # Derive jocobian/hessian of objective function
        df, ddf = derive_jacobian_and_hessian_functions("f", self.f, self.x, self.p)

        ## Jacobian of the objective function.
        self.df = df

        ## Hessian of the objective function.
        self.ddf = ddf

        ## Number of decision variables.
        self.nx = decision_variables.numel()

        ## Number of parameters.
        self.np = parameters.numel()

    def set_models(self, models: List[Model]) -> None:
        """! Specify the models in the optimization problem.

        @param models A list of models (i.e. instance of a sub-class of Model).
        """
        self.models = models

    def specify_quadratic_cost(self) -> None:
        """! Specify the terms P and q of a quadratic cost function."""
        x_zero = cs.DM.zeros(self.nx)
        self.P = cs.Function("P", [self.p], [0.5 * self.ddf(self.x, self.p)])
        self.q = cs.Function("q", [self.p], [cs.vec(self.df(x_zero, self.p))])

    def specify_linear_constraints(
        self, lin_ineq_constraints, lin_eq_constraints
    ) -> None:
        """! Setup the constraints k(x, p) = M(p).x + c(p) >= 0, and a(x, p) = A(p).x + b(p) == 0.

        @param lin_ineq_constraints MXContainer containing the linear inequality constraints.
        @param lin_eq_constraints MXContainer containing the linear equality constraints.
        """

        self.lin_ineq_constraints = lin_ineq_constraints
        self.lin_eq_constraints = lin_eq_constraints

        # Setup k
        self.k = cs.Function("k", [self.x, self.p], [self.lin_ineq_constraints.vec()])
        self.nk = self.lin_ineq_constraints.numel()
        self.lbk = cs.DM.zeros(self.nk)
        self.ubk = self.inf * cs.DM.ones(self.nk)

        # Setup M and c
        x_zero = cs.DM.zeros(self.nx)
        self.M = cs.Function(
            "M", [self.p], [cs.jacobian(self.k(self.x, self.p), self.x)]
        )
        self.c = cs.Function("c", [self.p], [self.k(x_zero, self.p)])

        # Setup a
        self.a = cs.Function("a", [self.x, self.p], [self.lin_eq_constraints.vec()])
        self.na = self.lin_eq_constraints.numel()
        self.lba = cs.DM.zeros(self.na)
        self.uba = cs.DM.zeros(self.na)

        # Setup A and b
        self.A = cs.Function(
            "A", [self.p], [cs.jacobian(self.a(self.x, self.p), self.x)]
        )
        self.b = cs.Function("b", [self.p], [self.a(x_zero, self.p)])

    def specify_nonlinear_constraints(
        self, ineq_constraints: MXContainer, eq_constraints: MXContainer
    ) -> None:
        """! Setup the constraints g(x, p) >= 0, and h(x, p) == 0.

        @param ineq_constraints MXContainer containing the inequality constraints.
        @param eq_constraints MXContainer containing the equality constraints.
        """

        self.ineq_constraints = ineq_constraints
        self.eq_constraints = eq_constraints

        # Setup g
        self.g = cs.Function("g", [self.x, self.p], [self.ineq_constraints.vec()])
        self.ng = self.ineq_constraints.numel()
        self.lbg = cs.DM.zeros(self.ng)
        self.ubg = self.inf * cs.DM.ones(self.ng)
        self.dg, self.ddg = derive_jacobian_and_hessian_functions(
            "g", self.g, self.x, self.p
        )

        # Setup h
        self.h = cs.Function("g", [self.x, self.p], [self.eq_constraints.vec()])
        self.nh = self.eq_constraints.numel()
        self.lbh = cs.DM.zeros(self.nh)
        self.ubh = cs.DM.zeros(self.nh)
        self.dh, self.ddh = derive_jacobian_and_hessian_functions(
            "h", self.h, self.x, self.p
        )

    def specify_v(
        self, ineq: List[cs.Function] = [], eq: List[cs.Function] = []
    ) -> None:
        """! Specify the vertical constraints vector v. This is only called for optimization problems that have constraints.

        @param ineq List of CasADi functions that evalaute the (non)linear inequality constraints.
        @param ineq List of CasADi functions that evalaute the (non)linear equality constraints.
        """
        self.v = vertcon(self.x, self.p, ineq=ineq, eq=eq)
        self.nv = self.v.numel_out()
        self.lbv = cs.DM.zeros(self.nv)
        self.ubv = self.inf * cs.DM.ones(self.nv)
        self.dv, self.ddv = derive_jacobian_and_hessian_functions(
            "v", self.v, self.x, self.p
        )

    def has_discrete_variables(self):
        return self.decision_variables.has_discrete_variables()


class QuadraticCostUnconstrained(Optimization):
    """! Unconstrained Quadratic Program.

            min f(x, p) where f(x, p) = x'.P(p).x + x'.q(p)
             x

    The problem is unconstrained, and has quadratic cost function.
    """

    def __init__(
        self,
        decision_variables: MXContainer,  # MXContainer for decision variables
        parameters: MXContainer,  # MXContainer for parameters
        cost_terms: MXContainer,  # MXContainer for cost terms
    ):
        """! Initializer for the QuadraticCostUnconstrained class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms (must be quadratic).
        @return Instance of the QuadraticCostUnconstrained class.
        """
        super().__init__(decision_variables, parameters, cost_terms)
        self.specify_quadratic_cost()


class QuadraticCostLinearConstraints(Optimization):
    """! Linear constrained Quadratic Program.

            min f(x, p) where f(x, p) = x'.P(p).x + x'.q(p)
                 x

                subject to
                            k(x, p) = M(p).x + c(p) >= 0
                            a(x, p) = A(p).x + b(p) == 0

    The problem is constrained by only linear constraints and has a
    quadratic cost function.
    """

    def __init__(
        self,
        decision_variables: MXContainer,  # MXContainer for decision variables
        parameters: MXContainer,  # MXContainer for parameters
        cost_terms: MXContainer,  # MXContainer for cost terms
        lin_eq_constraints: MXContainer,  # MXContainer for linear equality constraints
        lin_ineq_constraints: MXContainer,  # MXContainer for linear inequality constraints
    ):
        """! Initializer for the QuadraticCostLinearConstraints class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms (must be quadratic).
        @param lin_eq_constraints MXContainer containing the linear equality constraints.
        @param lin_ineq_constraints MXContainer containing the linear inequality constraints.
        @return Instance of the QuadraticCostLinearConstraints class.
        """
        super().__init__(decision_variables, parameters, cost_terms)
        self.specify_quadratic_cost()
        self.specify_linear_constraints(lin_ineq_constraints, lin_eq_constraints)
        self.specify_v(ineq=[self.k], eq=[self.a])


class QuadraticCostNonlinearConstraints(Optimization):
    """! Nonlinear constrained optimization problem with quadratic cost function.

            min f(x, p) where f(x, p) = x'.P(p).x + x'.q(p)
                 x

            subject to

                k(x, p) = M(p).x + c(p) >= 0
                a(x, p) = A(p).x + b(p) == 0
                g(x) >= 0, and
                h(x) == 0

    The problem is constrained by nonlinear constraints and has a
    quadratic cost function.

    """

    def __init__(
        self,
        decision_variables: MXContainer,  # MXContainer for decision variables
        parameters: MXContainer,  # MXContainer for parameters
        cost_terms: MXContainer,  # MXContainer for cost terms
        lin_eq_constraints: MXContainer,  # MXContainer for linear equality constraints
        lin_ineq_constraints: MXContainer,  # MXContainer for linear inequality constraints
        eq_constraints: MXContainer,  # MXContainer for equality constraints
        ineq_constraints: MXContainer,  # MXContainer for inequality constraints
    ):
        """! Initializer for the QuadraticCostNonlinearConstraints class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms (must be quadratic).
        @param lin_eq_constraints MXContainer containing the linear equality constraints.
        @param lin_ineq_constraints MXContainer containing the linear inequality constraints.
        @param eq_constraints MXContainer containing the equality constraints.
        @param ineq_constraints MXContainer containing the inequality constraints.
        @return Instance of the QuadraticCostNonlinearConstraints class.
        """
        super().__init__(decision_variables, parameters, cost_terms)
        self.specify_quadratic_cost()
        self.specify_linear_constraints(lin_ineq_constraints, lin_eq_constraints)
        self.specify_nonlinear_constraints(ineq_constraints, eq_constraints)
        self.specify_v(ineq=[self.k, self.g], eq=[self.a, self.h])


class NonlinearCostUnconstrained(Optimization):
    """! Unconstrained optimization problem.

            min f(x, p)
             x

    The problem is unconstrained and the cost function is nonlinear.

    """

    def __init__(
        self,
        decision_variables: MXContainer,
        parameters: MXContainer,
        cost_terms: MXContainer,
    ):
        """! Initializer for the NonlinearCostUnconstrained class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms.
        @return Instance of the NonlinearCostUnconstrained class.
        """
        super().__init__(decision_variables, parameters, cost_terms)


class NonlinearCostLinearConstraints(Optimization):
    """! Linear constrained optimization problem.


            min f(x, p)
                 x

                subject to
                            k(x, p) = M(p).x + c(p) >= 0
                            a(x, p) = A(p).x + b(p) == 0

    The problem is constrained by only linear constraints and has a
    quadratic cost function.


    The problem is constrained with linear constraints and has a
    nonlinear cost function in x.

    """

    def __init__(
        self,
        decision_variables: MXContainer,  # MXContainer for decision variables
        parameters: MXContainer,  # MXContainer for parameters
        cost_terms: MXContainer,  # MXContainer for cost terms
        lin_eq_constraints: MXContainer,  # MXContainer for linear equality constraints
        lin_ineq_constraints: MXContainer,  # MXContainer for linear inequality constraints
    ):
        """! Initializer for the NonlinearCostLinearConstraints class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms.
        @param lin_eq_constraints MXContainer containing the linear equality constraints.
        @param lin_ineq_constraints MXContainer containing the linear inequality constraints.
        @return Instance of the NonlinearCostLinearConstraints class.
        """
        super().__init__(decision_variables, parameters, cost_terms)
        self.specify_linear_constraints(lin_ineq_constraints, lin_eq_constraints)
        self.specify_v(ineq=[self.k], eq=[self.a])


class NonlinearCostNonlinearConstraints(Optimization):
    """! Nonlinear constrained optimization problem.

            min f(x, p)
             x

                subject to

                    k(x, p) = M(p).x + c(p) >= 0
                    a(x, p) = A(p).x + b(p) == 0
                    g(x) >= 0, and
                    h(x) == 0

    The problem is constrained by nonlinear constraints and has a
    nonlinear cost function.
    """

    def __init__(
        self,
        decision_variables: MXContainer,  # MXContainer for decision variables
        parameters: MXContainer,  # MXContainer for parameters
        cost_terms: MXContainer,  # MXContainer for cost terms
        lin_eq_constraints: MXContainer,  # MXContainer for linear equality constraints
        lin_ineq_constraints: MXContainer,  # MXContainer for linear inequality constraints
        eq_constraints: MXContainer,  # MXContainer for equality constraints
        ineq_constraints: MXContainer,  # MXContainer for inequality constraints
    ):
        """! Initializer for the NonlinearCostNonlinearConstraints class.

        @param decision_variables MXContainer containing decision variables.
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms.
        @param lin_eq_constraints MXContainer containing the linear equality constraints.
        @param lin_ineq_constraints MXContainer containing the linear inequality constraints.
        @param eq_constraints MXContainer containing the equality constraints.
        @param ineq_constraints MXContainer containing the inequality constraints.
        @return Instance of the NonlinearCostNonlinearConstraints class.
        """
        super().__init__(decision_variables, parameters, cost_terms)
        self.specify_linear_constraints(lin_ineq_constraints, lin_eq_constraints)
        self.specify_nonlinear_constraints(ineq_constraints, eq_constraints)
        self.specify_v(ineq=[self.k, self.g], eq=[self.a, self.h])


class MixedIntegerNonlinearCostNonlinearConstrained(NonlinearCostNonlinearConstraints):
    """Nonlinear mixed-integer optimization problem.

    min f(x, z; p)
     x,z

        subject to

            k(x, z; p) = M(p).x + L(p).z + c(p) >= 0
            a(x, z; p) = A(p).x + G(p).z + b(p) == 0
            g(x, z; p) >= 0, and
            h(x, z; p) == 0
    """

    def __init__(
        self,
        decision_variables: MXContainer,  # MXContainer for decision variables
        parameters: MXContainer,  # MXContainer for parameters
        cost_terms: MXContainer,  # MXContainer for cost terms
        lin_eq_constraints: MXContainer,  # MXContainer for linear equality constraints
        lin_ineq_constraints: MXContainer,  # MXContainer for linear inequality constraints
        eq_constraints: MXContainer,  # MXContainer for equality constraints
        ineq_constraints: MXContainer,  # MXContainer for inequality constraints
    ):
        """! Initializer for the MixedIntegerNonlinearCostNonlinearConstrained class.

        @param decision_variables MXContainer containing decision variables (with at least one discrete variable).
        @param parameters MXContainer containing parameters.
        @param cost_terms MXContainer containing cost terms.
        @param lin_eq_constraints MXContainer containing the linear equality constraints.
        @param lin_ineq_constraints MXContainer containing the linear inequality constraints.
        @param eq_constraints MXContainer containing the equality constraints.
        @param ineq_constraints MXContainer containing the inequality constraints.
        @return Instance of the MixedIntegerNonlinearCostNonlinearConstrained class.
        """
        super().__init__(decision_variables, parameters, cost_terms, lin_eq_constraints,  lin_ineq_constraints, eq_constraints, ineq_constraints)
