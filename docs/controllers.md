# Choosing the Correct Controller
While developing the simulation a need arose for the correct control of the grippers. For a 2 finger gripper this is a rudimentary task, since by activating/closing both DoFs of the gripper the grasp is performed correctly. Nonetheless, with a more complex gripper (a 3-5 fingers) the closing routine is not as straight forward. Consequently, a "closing mask" was implemented to be used in tandem with custom gripper controllers to control different grippers in the same simulation. First, the "close_dir" of every gripper was defined by the visualization of the grippers within the simulation. The close_dir is just a n-dimesional vector stating the direction a DoF of the gripper must move to in order to close the gripper.

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4d1695831defc6b71d90b0ea6d7a1d03f34c1346/manager.py#L104-L116

It is worth noting that this is a very subjective way of 
