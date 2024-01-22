# Adding Grippers
Import the grippers to Isaac Sim using the GUI and save them as .usd in the gripper directory exactly the same way as the other grippers in [the gripper folder](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/gripper). First, the .usd grippers must be tested and made ready for use within the simulation. To do so the user_examples folder was added to the repository. You should copy it to "~/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples" that way it can be loaded and used to test .usd files faster with the GUI. To load in the GUI go to the Isaac Examples tab >  ScratchPad > Testing.
## Steps to Add grippers:
1) Prepare Gripper .usd (Import .urdf and makes sure it has its correct max efforts, max joint velocities, colliders, etc.)
2) Add the corresponding entries to the manager._init_gripper_dicts() function in managerv2.py. The information is:
- EF_axis: Axis where the gripper end effector is pointing to.
- dts: physics_dt to run the gripper's simulations (for faster simulations we can increase this values, but it is limited by the gripper's complexity, for high complexity a small dt is required). Test and visualize the dts before running filters.
- controllers: controller to use with the gripper. Controllers are within the controllers.py file
- close_dir: dofs and directions used to close the gripper
- contact_names: contact of gripper parts to filter collisions with (used for grasp set up probing)
- contact_ths: contacts needed to inialize grasp and activate gravity.
3) Visualize the filter with the gripper (reduce the amount of workstations to make iterating easier).
