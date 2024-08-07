# Choosing the Right Controller
While developing the simulation a need arose for the optimal control of the grippers. For a 2 finger gripper this is a rudimentary task, since by activating/closing both DoFs of the gripper the grasp is performed correctly. Nonetheless, with a more complex gripper (a 3-5 fingers) the closing routine is not as straight forward. Consequently, a "closing mask" was implemented to be used in tandem with custom gripper controllers to control different grippers in the same simulation. First, the "close_dir" of every gripper was defined by the visualization of the grippers within the simulation. The close_dir is just a n-dimesional vector stating the direction a DoF of the gripper must move to in order to close the gripper.

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4e965198fb43f0d1335c656c1b60c6d3ea586c25/managers.py#L103-L115

For example, the fetch gripper, being a 2 finger gripper, must only move boths of its DoFs to the positive direction to perform the grasp. On the other hand, a more complex gripper, such as the Allegro gripper, has multiple DoFs that remain static and just a handfull of DoFs that move to close the gripper; we denoted the moving DoFs as "driving DoFs".  It is worth noting that this is a very subjective way of creating a closing routine. Many manufacturers provide different closing routines for the different types of objects the gripper may grasp, for example the [Robotiq 3-finger gripper](https://assets.robotiq.com/website-assets/support_documents/document/3-Finger_PDF_20190221.pdf) which has multiple grasping routines, such as the a fingertip grip or an encompassing grip. Nonetheless, a simple controller can be a good starting point for the grasping tests. In the future, more controllers could be developed without necessarily using the closing masks.

## Available controllers
For our simulation, 2 controllers were developed as starting points. One using the position control of the grippers and another using a force control approach. All controllers take as input the same parameters at initialization and during simulation (forward function):

Initialization:
- close_mask: close direction of the DoFs
- test_time: total time of the tests
- max_efforts: max effort(force/torque) of every DoF
- robots: ArticulationView of all the grippers within the simulation
Note: every controller must have is own name (Isaac Sim Convention) and the self.type with a description of the controller used (Saved to output file).

Forward function:
- action: Isaac Sim convention (string describing if it is a closing or opening action)
- time: current time since the test started.
- current_dofs: DoF values of the robot.
- close_position: Final DoF values that describe the closed position of the gripper. It is calculated within view.py view.post_reset() function using the close_mask.

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4e965198fb43f0d1335c656c1b60c6d3ea586c25/views.py#L97-L116

Note: For new controllers the reference must be added to the controller_dict.

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4e965198fb43f0d1335c656c1b60c6d3ea586c25/controllers.py#L164-L169


### Position-based Controller
https://github.com/IRVLUTD/isaac_sim_grasping/blob/4e965198fb43f0d1335c656c1b60c6d3ea586c25/controllers.py#L54-L100

The simplest of both controllers is the position based controller denominated as "PositionController" and accessible in the simulation with the "position" keyword. The controller simply gets the "close_position" and transforms it to an ArticulationActions object for the use in Isaac Sim. The resulting behavior is the movement of the gripper DoF to the final "closed position" denoted by the close_mask and the DoF range.

![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/PC.gif)

### Force-based Controller

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4e965198fb43f0d1335c656c1b60c6d3ea586c25/controllers.py#L10-L52

The force controller is denominated as "ForceController" and accessible in the simulation with the "force" keyword. The controller works by exerting forces in the driving DoFs of the gripper. The exerted forces are constantly decreasing from 100% of the max DoF effort (described in the robot .usd) to -100% of the max effort. In simple words, for the first half of the test the gripper is closing with a decreasing amount of effort and for the last half the gripper will be opening with an increasing amount of effort. Note that for the proper exertion of the forces in the DoFs, they first needed to be disabled by setting their max effort as 0 and then applying the ArticulationActions manually through the controller.

![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/FC.gif)

### Position-based Controller for Transferred Grasps

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4e965198fb43f0d1335c656c1b60c6d3ea586c25/controllers.py#L103-L157

Our simulation was able to evaluate the object fall-off time for a large amount of generated grasps. The successful grasps of one gripper can represent successful grasps in others and increase the overall amount of successful grasps in the dataset. To test this hypothesis, we implemented the grasp transfer of successful grasps from one gripper to others and evaluated the transferred grasps using Isaac Sim. We first utilize the alignment between grippers to transfer grasps.  We transform a source gripper pose to its aligned pose, and then transform the aligned pose to the target gripper. To transfer a grasp from a source gripper to a target gripper, the joint configuration of the source gripper cannot easily be mapped to the target gripper. Therefore, we first set the joint configuration of the target gripper to the open configuration. Then the controller closes all fingers until it comes into contact with the object. After that, the controller set the root joints of the fingers to reach their closing position. The gripper performs a closing motion. The joint values at the moment of contact between the gripper and object are recorded, and the object fall-off time is simulated.

<p align="center">
<img src='https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/transfer_close.gif' width='1000'>
</p>

