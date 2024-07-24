## The Grippers used with the MultiGripperGrasp Toolkit must include their relevant information in the  [gripper_isaac_info.json](gripper_isaac_info.json) file.

The file incudes:
- "EF_axis": Gripper end effector axis (+/- 1,2,3) x,y, z respectively.
- "physics_frequency": Frequency in which the PhysiX engine will run when evaluating grasps.
- "close_dir": List with directions for gripper DoFs to close (1,-1,0) sign denotes direction, set to 0 for the DoFs that won't move.
- "contact_names": List of joint names to check for collisions; they must be exactly as specified in the .usd of the gripper.
- "contact_th": Amount of contacts required for the grasp to be considered as ready.
- "transfer_close_dir": List encoding of DoF behavior when evaluating grasps, ecoding:
    - 0: the dof won't move.
    - 1: dof will move only to set up grasp (sign denotesdirection).
    - &gt;1: dof will move to set up grasp and to exert force on object (sign denotes direction).
- "transfer_contact_th": contact_th used on transferred grasps.
- "opened_dofs": List of DoF values for the gripper's opened positions.
- "trasnfer_reference_pose": Reference Pose used for transfering grasps. The pose denotes a point on the gripper's palm. (x,y,z,qw,qx,qy,qz)
