from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.franka import Franka
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.franka.controllers import PickPlaceController
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController
from omni.isaac.core.utils.stage import add_reference_to_stage

import numpy as np

class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return


    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = (command[0])
        joint_velocities[1] = (command[1])
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_positions=joint_velocities)


class Test(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return


    def setup_scene(self):

        world = self.get_world()
        world.scene.add_default_ground_plane(-1)
        usd_path = "/home/felipe/Documents/isaac_sim_grasping/grippers/fetch_gripper/fetch_gripper/fetch_gripper.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/gripper")
        world.scene.add(
            Robot(
                prim_path="/World/gripper",
                name="fancy_robot",
            )
        )
        
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/random_cube",
                name="fancy_cube",
                position=np.array([10.3, 10.3, 10.3]),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array([0, 0, 1.0]),
            )
        )
        return


    async def setup_post_load(self):
        self._world = self.get_world()
        self._robot = self._world.scene.get_object("fancy_robot")
        self._fancy_cube = self._world.scene.get_object("fancy_cube")
        # Initialize a pick and place controller
        self._controller = CoolController()
        self._controller.__init__()
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        # World has pause, stop, play..etc

        return


    # This function is called after Reset button is pressed
    # Resetting anything in the world should happen here
    async def setup_post_reset(self):
        self._controller.reset()

        return


    def physics_step(self, step_size):
        cube_position, _ = self._fancy_cube.get_world_pose()
        goal_position = np.array([-0.3, -0.3, 0.0515 / 2.0])
        current_joint_positions = self._robot.get_joint_positions()
        print(current_joint_positions)
        c1 = np.random.rand(2)*10000
        actions = self._controller.forward(command=c1)
        self._robot.apply_action(actions)
        # Only for the pick and place controller, indicating if the state
        # machine reached the final state.
        return