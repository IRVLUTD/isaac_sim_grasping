# Adding Grippers
Import the grippers to Isaac Sim using the GUI and save them as .usd in the gripper directory. It should be exactly the same way as the [other grippers](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/grippers). The .usd grippers must be tested and made ready for use within the simulation. To make things faster we recommend the use of a GUI user example as instructed in the [Isaac Sim Manual](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html).


## Steps to Add grippers:
1) Prepare Gripper .usd (Import the gripper into Isaac Sim and make sure it has the correct attributes, i.e. max efforts, max joint velocities, colliders, etc.). For .urdf files go to Isaac Utils > Workflows > URDF Importer.


![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/AG1.png)

2) (Recommended) Use the Isaac Sim GUI to ensure the gripper moves as intended. 
![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/SonyGripper.png)


3) Add the corresponding entries to the gripper information dictionaries  [gripper_isaac_info.json](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/grippers/gripper_isaac_info.json). The gripper ID must be unique and the corresponding .json grasp files must have the exact same ID.
https://github.com/IRVLUTD/isaac_sim_grasping/blob/6b43f3369395127aa0e532aed0129e3df6c7b422/manager.py#L71-L75

The entries are:
- "EF_axis": Gripper end effector axis (+/- 1,2,3) x,y, z respectively.
- "physics_frequency": Frequency in which the PhysiX engine will run when evaluating grasps.
- "close_dir": List with directions for gripper DoFs to close (1,-1,0) sign denotes direction, set to 0 for the DoFs that shouldn't move.
- "contact_names": List of joint names to check for contact collisions; they must be exactly as specified in the .usd of the gripper.
- "contact_th": Amount of contacts required for the grasp to be considered as ready.
- "transfer_close_dir": List encoding of DoF behavior when evaluating grasps, encoding:
    - 0: the dof won't move.
    - 1: dof will move only to set up grasp (sign denotes direction).
    - &gt;1: dof will move to set up grasp and to exert force on object after the grasp is ready(sign denotes direction).
- "transfer_contact_th": contact_th used on transferred grasps.
- "opened_dofs": List of DoF values for the gripper's opened position.
- "transfer_reference_pose": Reference Pose used for transferring grasps. The pose denotes a point on the gripper's palm. (x,y,z,qw,qx,qy,qz). For more information on how transfer grasps were calculated refer to [helpful_scripts folder.](https://github.com/IRVLUTD/isaac_sim_grasping/tree/d3a192e304ee7cdbeebd3c3ba1869e2d9cf5e057/helpful_scripts)



4) Visualize the simulation of the gripper to make sure everything is working correctly (reduce the amount of workstations to make iterating easier). Note: New gripper folders must contain the same relative path to their .usd as with the grippers in our repository; grippers/gripper_ID/gripper_ID/gripper_ID.usd


![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/AG2.png)