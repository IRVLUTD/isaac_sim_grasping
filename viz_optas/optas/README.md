<p align="center">
  <img src="https://raw.githubusercontent.com/cmower/optas/master/doc/logo.png" width="60" align="right">
</p>

# OpTaS

[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Lint](https://github.com/cmower/optas/actions/workflows/black.yaml/badge.svg)](https://github.com/cmower/optas/actions/workflows/black.yaml)
[![Run tests](https://github.com/cmower/optas/actions/workflows/pytest.yaml/badge.svg)](https://github.com/cmower/optas/actions/workflows/pytest.yaml)
[![Build documentation](https://github.com/cmower/optas/actions/workflows/documentation.yaml/badge.svg)](https://github.com/cmower/optas/actions/workflows/documentation.yaml)


OpTaS is an OPtimization-based TAsk Specification library for trajectory optimization and model predictive control.

- Code: [https://github.com/cmower/optas](https://github.com/cmower/optas)
- Documentation: [https://cmower.github.io/optas/](https://cmower.github.io/optas/)
- PyPI: [https://pypi.org/project/pyoptas/](https://pypi.org/project/pyoptas/)
- Issues: [https://github.com/cmower/optas/issues](https://github.com/cmower/optas/issues)
- ICRA 2023 paper:
  - (arXiv) [https://arxiv.org/abs/2301.13512](https://arxiv.org/abs/2301.13512)
  - (ieee) [https://ieeexplore.ieee.org/document/10161272](https://ieeexplore.ieee.org/document/10161272)
- Video: [https://youtu.be/gCMNOenFngU](https://youtu.be/gCMNOenFngU)
- Presentation: [https://vimeo.com/824802366](https://vimeo.com/824802366)

In the past, OpTaS supported ROS from an internal module. This functionality, with additional updates, has now been moved to a dedicated repository: [optas_ros](https://github.com/cmower/optas_ros).

# Example

<p align="center">
  <img src="https://raw.githubusercontent.com/cmower/optas/master/doc/image/kuka_example.png" width="61.803398875%">
</p>


In this example we implement an optimization-based IK problem.
The problem computes an optimal joint configuration $q^*\in\mathbb{R}^n$ given by

$$
q^* = \underset{q}{\text{arg}\min}~\|\|q - q_N\|\|^2\quad\text{subject to}\quad p(q) = p_g, q^-\leq q \leq q^+
$$

where
* $q\in\mathbb{R}^n$ is the joint configuration for an $n$-dof robot (in our example, we use the KUKA LWR in the above figure with $n=7$),
* $q_N\in\mathbb{R}^n$ is a nominal joint configuration,
* $\|\|\cdot\|\|$ is the Euclidean norm,
* $p: \mathbb{R}^n\rightarrow\mathbb{R}^3$ computes the end-effector position via the forward kinematics,
* $p_g\in\mathbb{R}^3$ is a goal position, and
* $q^-, q^+\in\mathbb{R}^n$ is the lower and upper joint position limits respectively.

The example problem has a quadratic cost function with nonlinear constraints.
We use the nominal configuration $q_N$ as the initial seed for the problem.

The following example script showcases some of the main features of OpTaS:
creating a robot model,
building an optimization problem,
passing the problem to a solver,
computing an optimal solution, and
visualizing the robot in a given configuration.

```python
import os
import pathlib

import optas

# Specify URDF filename
cwd = pathlib.Path(__file__).parent.resolve()  # path to current working directory
urdf_filename = os.path.join(
    cwd, "robots", "kuka_lwr", "kuka_lwr.urdf"
)  # KUKA LWR, 7-DoF

# Setup robot model
robot = optas.RobotModel(urdf_filename=urdf_filename)
name = robot.get_name()

# Setup optimization builder
T = 1
builder = optas.OptimizationBuilder(T, robots=robot)

# Setup parameters
qn = builder.add_parameter("q_nominal", robot.ndof)
pg = builder.add_parameter("p_goal", 3)

# Constraint: end goal
q = builder.get_model_state(name, 0)
end_effector_name = "end_effector_ball"
p = robot.get_global_link_position(end_effector_name, q)
builder.add_equality_constraint("end_goal", p, pg)

# Cost: nominal configuration
builder.add_cost_term("nominal", optas.sumsqr(q - qn))

# Constraint: joint position limits
builder.enforce_model_limits(name)  # joint limits extracted from URDF

# Build optimization problem
optimization = builder.build()

# Interface optimization problem with a solver
solver = optas.CasADiSolver(optimization).setup("ipopt")
# solver = optas.ScipyMinimizeSolver(optimization).setup("SLSQP")

# Specify a nominal configuration
q_nominal = optas.deg2rad([0, 45, 0, -90, 0, -45, 0])

# Get end-effector position in nominal configuration
p_nominal = robot.get_global_link_position(end_effector_name, q_nominal)

# Specify a goal end-effector position
p_goal = p_nominal + optas.DM([0.0, 0.3, -0.2])

# Reset solver parameters
solver.reset_parameters({"q_nominal": q_nominal, "p_goal": p_goal})

# Reset initial seed
solver.reset_initial_seed({f"{name}/q": q_nominal})

# Compute a solution
solution = solver.solve()
q_solution = solution[f"{name}/q"]

# Visualize the robot
vis = optas.Visualizer(quit_after_delay=2.0)

# Draw goal position and start visualizer
vis.sphere(0.05, rgb=[0, 1, 0], position=p_goal.toarray().flatten().tolist())
# vis.robot(robot, q=q_nominal,display_link_names=True,show_links=True)   # nominal
vis.robot(robot, q=q_solution, display_link_names=True, show_links=True)  # solution

vis.start()
```

Run the example script [example.py](example/example.py).
Other examples, including dual-arm planning, Model Predictive Control, Trajectory Optimization, etc can be found in the [example/](example) directory.

# Support

The following operating systems and python versions are [officially supported](https://github.com/cmower/optas/blob/master/.github/workflows/pytest.yaml):

* Ubuntu 20.04 and 22.04
  * Python 3.7, 3.8, 3.9
* Windows
  * Python 3.8, 3.9
* Mac OS
  * Python 3.9

Note that OpTaS makes use of [dataclasses](https://docs.python.org/3/library/dataclasses.html) that was [introduced in Python 3.7](https://peps.python.org/pep-0557/), and so Python versions from 3.6 and lower are not supported on any operating system.
Other operating systems or higher Python versions will likely work.
If you experience problems, please [submit an issue](https://github.com/cmower/optas/issues/new/choose).

# Install

Make sure `pip` is up-to-date by running `$ python -m pip install --upgrade pip`.

## Via pip

```
$ pip install pyoptas
```

Alternatively, you can also install OpTaS using:

```
$ python -m pip install 'optas @ git+https://github.com/cmower/optas.git'
```

## From source
1. `$ git clone --recursive git@github.com:cmower/optas.git` (if you do not want to build the documentation then the `--recursive` flag is not necessary)
2. `$ cd optas`
4. `$ pip install .`
   - if you want to run the examples use: `$ pip install .[example]`
   - if you want to run the tests use: `$ pip install .[test]`

### Build documentation

1. `$ cd /path/to/optas/doc`
2. `$ sudo apt install doxygen graphviz`
3. `$ python gen_mainpage.py`
3. `$ doxygen`
4. Open the documentation in either HTML or PDF:
   - `html/index.html`
   - `latex/refman.pdf`

### Run tests

1. `$ cd /path/to/optas`
2. Each test can be run as follows
   - `$ pytest tests/test_builder.py`
   - `$ pytest tests/test_examples.py`
   - `$ pytest tests/test_models.py`
   - `$ pytest tests/test_optas_utils.py`
   - `$ pytest tests/test_optimization.py`
   - `$ pytest tests/test_solver.py`
   - `$ pytest tests/test_spatialmath.py`
   - `$ pytest tests/test_sx_container.py`

# Known Issues

- Loading robot models from xacro files is supported, however there can be issues if you are running this in a ROS agnositic environment. If you do not have ROS installed, then the xacro file should not contain ROS-specific features. For further details see [here](https://github.com/cmower/optas/issues/78).
- If NumPy ver 1.24 is installed, an `AttributeError` error is thrown when you try to solve an unconstrained problem with the OSQP interface. A temporary workaround is to add a constraint, e.g. `x >= -1e9` where `x` is a decision variable. See details on the issue [here](https://github.com/osqp/osqp-python/issues/104) and pull request [here](https://github.com/osqp/osqp-python/pull/105).

# Citation

If you use OpTaS in your work, please consider including the following citation.

```bibtex
@inproceedings{mower23optas,
  author={Mower, Christopher E. and Moura, João and Behabadi, Nazanin Zamani and Vijayakumar, Sethu and Vercauteren, Tom and Bergeles, Christos},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  title={OpTaS: An Optimization-based Task Specification Library for Trajectory Optimization and Model Predictive Control},
  year={2023},
  volume={},
  number={},
  pages={9118-9124},
  doi={10.1109/ICRA48891.2023.10161272}
}
```

The preprint can be found on [arXiv](https://arxiv.org/abs/2301.13512).

# Contributing

We welcome contributions from the community.
If you come across any issues or inacuracies in the documentation, please [submit an issue](https://github.com/cmower/optas/issues/new/choose).
If you would like to contribute any new features, please [fork the repository](https://github.com/cmower/optas/fork), and submit a pull request.

# Acknowledgement

This research received funding from the European Union’s Horizon 2020 research and innovation program under grant agreement No. 101016985 ([FAROS](https://h2020faros.eu/)).
Further, this work was supported by core funding from the Wellcome/EPSRC [WT203148/Z/16/Z; NS/A000049/1].
T. Vercauteren is supported by a Medtronic / RAEng Research Chair [RCSRF1819\7\34], and C. Bergeles by an ERC Starting Grant [714562].
This work has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No 101017008, Enhancing Healthcare with Assistive Robotic Mobile Manipulation ([HARMONY](https://harmony-eu.org/)).

<p align="center">
  <img src="https://raw.githubusercontent.com/cmower/optas/master/doc/image/eu.png" width="180" align="left">
</p>
