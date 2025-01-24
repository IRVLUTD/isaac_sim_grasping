# MultiGripperGrasp Grasp Data Format

All the grasps for one object-gripper pair should be in one .json file, the file must contain a dictionary with atleast the following information:
- 'gripper': Gripper ID to perform the grasps with.
- 'object_ID': Object ID to perform the grasps with.
- 'pose': List of sample poses (Pose of gripper with respect to Object Coordinate Frame)
- 'dofs': List of initial joint values to load in Isaac Sim. Optional if the --dof_given flag is not used.

After simulating the grasps a .json file is saved at the output directory with the following information in a dictionary:
- 'gripper': Gripper ID to perform the grasps with.
- 'object_ID': Object ID to perform the grasps with.
- 'pose': List of sample poses (Pose of gripper with respect to Object Coordinate Frame)
- 'dofs': List of initial joint values to load in Isaac Sim.
- 'test_type': Test class label used for grasp evaluation.
- 'controller': Controller class label used for grasp evaluation.
- 'test_duration': Test time of each grasp.
- 'result_time': Time it took to run the test of each grasp.
- 'result': Status of each test (PASSED = 1 or FAILED = 0)
- 'final_dofs': DoF or joint values at the moment that the setup_condition was fulfilled (refer to the [controller manual page](controllers.md))
- 'runtime': Runtime of entire object-gripper pair simulation.
- 'physics_dt': Inverse Physics engine frequency with which the simulation was runned. 
- 'timestamp': Timestamp at the moment of completion of evaluation. 

**Note**: The output will be a copy of the original dictionary file in the case that there is information the user wants to include. 