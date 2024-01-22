# Adding Grippers
Import the grippers to Isaac Sim using the GUI and save them as .usd in the gripper directory exactly the same way as the other grippers in [the gripper folder](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/grippers). The .usd grippers must be tested and made ready for use within the simulation. To make things faster we recommend the use of a GUI user example as instructed in the [Isaac Sim Manual](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html).


## Steps to Add grippers:
1) Prepare Gripper .usd (Import .urdf and makes sure it has its correct max efforts, max joint velocities, colliders, etc.). Go to Isaac Utils > Workflows > URDF Importer.

![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/AG1.png)



3) Add the corresponding entries to the manager._init_gripper_dicts() function in [manager.py](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/manager.py).
https://github.com/IRVLUTD/isaac_sim_grasping/blob/6b43f3369395127aa0e532aed0129e3df6c7b422/manager.py#L71-L75
The dictionaries are as follows:
- EF_axis: Axis where the gripper end effector is pointing to.
- dts: physics_dt to run the gripper's simulations (for faster simulations we can increase this values, but it is limited by the gripper's complexity, for high complexity a small dt is required). Test and visualize the dts before running filters.
- close_dir: dofs and directions used to close the gripper
- contact_names: contact of gripper parts to filter collisions with (used for grasp set up probing) Must be exactly as specified by Isaac Sim.
- contact_ths: contacts needed to inialize grasp and activate gravity.



3) Visualize the filter with the gripper (reduce the amount of workstations to make iterating easier).
![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/AG2.png)
