# Choosing the Correct Controller
While developing the simulation a need arose for the correct control of the grippers. For a 2 finger gripper this is a rudimentary task, since by activating/closing both DoFs of the gripper the grasp is performed correctly. Nonetheless, with a more complex gripper (a 3-5 fingers) the closing routine is not as straight forward. Consequently, a "closing mask" was implemented to be used in tandem with custom gripper controllers to control different grippers in the same simulation. First, the "close_dir" of every gripper was defined by the visualization of the grippers within the simulation. The close_dir is just a n-dimesional vector stating the direction a DoF of the gripper must move to in order to close the gripper.

https://github.com/IRVLUTD/isaac_sim_grasping/blob/4d1695831defc6b71d90b0ea6d7a1d03f34c1346/manager.py#L104-L116

For example, the fetch gripper, being a 2 finger gripper, must only move boths of its DoFs to the positive direction to perform the grasp. On the other hand, a more complex gripper, such as the Allegro gripper, has multiple DoFs that remain static and just a handfull of DoFs that move to close the grasp, we denoted the moving DoFs as "driving DoFs".  It is worth noting that this is a very subjective way of creating a closing routine. Many manufacturers provide different closing routines for the different types of objects the gripper may grasp, for example the [Robotiq 3-finger gripper](https://assets.robotiq.com/website-assets/support_documents/document/3-Finger_PDF_20190221.pdf) which has multiple grasping routines, such as the a fingertip grip or an encompassing grip. Nonetheless, we believe a simple controller is a good starting point for the grasping tests. In the future, more controllers could be developed without necessarily using the closing masks.

## Available controllers
For our simulation 2 controllers were developed as starting points, one using the position control of the grippers and another using a force control approach.

### Position based controller
