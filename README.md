# isaac_sim_grasping
![](https://github.com/IRVLUTD/isaac_sim_grasping/blob/main/media/robotiq_Clock.gif)
Simulation based grasp filter. This repository contains a grasp filter developed using Isaac Sim, it has the objective of testing generated grasps for a large amount of objects and grippers. In our case, the grasps tested were generated using GraspIt. These grasps were found to be of varied quality; upon close inspection many grasps could easily been classified as suboptimal grasps or failed grasps. Consequently, this simulation was created to evaluate the different grasps, creating multiple metrics with which they could be filtered and thus providing a large dataset of tested grasps that could be used for different purposes. Each grasp information consists of the relative pose between the object and gripper, as well as the Degree of Freedom (DoF) information of the gripper. 
   
## Simulation Behavior
The simulation can use any gripper and object provided they are prepared correctly and transformed to a .usd format for Isaac Sim. It loads the grasp information from the files specified and creates multiple "workstations" to test all the grasps in. Then, it tries to perform the grasps with the specified control routines. When the object falls or the testing time is up, the times are recorded and then saved to the output file. Any failed grasps will be recorded as a negative "fall time". Additionally, the slip metric was implemented by calculating the moment begins to slip from the grasp of the gripper.

### Parameters and Inputs
A standalone executable (standalone.py file) for the simulation is within the repository; a command to run the simulation is shown below. Note: for Isaac Sim standalone executables, the commands must be run from the isaac sim python.sh directory.


./python.sh (standalone folder)/standalone.py --json_dir=(json folder)/obj8 --gripper_dir=(repo directory)/grippers --objects_dir=(object directory) --output_dir=(output directory) --num_w=300 --test_time=6 --controller=position --headless --print_results

The standalone takes as input:
- grasp data directory (.json file)
- gripper directory (folder containing all the gripper .usd files)
- object directory (folder containing all the object .usd files)
- output directory (directory to save the outputed .json file)
- num_w: Number of Workstations to run the simulation with (gripper, object pair) (default: 150)
- test_time: total test time for each grasp test (default:6).
- controller: controller reference (within controllers.py)
- (Optional) print_results: Verbosity of standalone after finishing one .json file.
- (Optional) headless: Run the simulation headless

Note: To run the simulation without warning add the following parameters to the command: 
 --/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error

## Repository Structure
1) standalone.py: standalone executable
2) views.py: Simulation's behavioral code.
3) manager.py: contains grasp information and the reporting of results
4) controllers.py: Programmed gripper controllers to test with
5) utils.py: general utility functions
6) Helpful Scripts: Scripts found to be useful when developing simulations in Isaac Sim
7) grippers: gripper .usd files


# Adding Grippers
Import the grippers to Isaac Sim using the GUI and save them as .usd in the gripper directory exactly the same way as the other grippers. The .usd grippers must be tested and made ready for use within the simulation. To do so the user_examples folder was added to the repository. You should copy it to "~/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples" that way it can be loaded and used to test .usd files faster with the GUI. To load in the GUI go to the Isaac Examples tab >  ScratchPad > Testing.
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

# Adding Objects
Import object to Isaac Sim with the use of the GUI and save as .usd.
## Steps to Add objects:
1) Prepare the .usd objects (clean colliders, correct masses or densities, etc.)
2) Add the .json files describing the object grasps to the grasps directory.
3) Test the object's collider preferably by visualizing within the simulation.

# Isaac Sim Manual
https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html

## Helpful code for python standalones (code snippet samples)
https://docs.omniverse.nvidia.com/isaacsim/latest/reference_python_snippets.html

### Run standalone without warnings 
Add this arguments in command line:


## Install Packages Using isaac sim python.sh
 ./python.sh -m pip install name_of_package_here

 ## Notes: 
- You can change the command to the different set_up files we have.
- Need to run command from the issac sim directory, used 2023 Isaac Sim version
- Deactivate conda if you have an active environment, it may cause some errors when running isaac sim
- Always use complete paths, errors may occur otherwise
- The simulation supports a specific .json file format, if the format wants to be changed, the manager class must be editted.
- Readability is very fragile for examples in Isaac Sim, you should only modify Test.py.
- Don't use the stop button to reset example, instead use the reset below loading example.
- Be sure to run render = False and "headless" = True option for config as True whe running standalone.
- Many Isaac Sim API functions were found to give the incorrect data when called, if errors arise when implementing new code beware of this fact.
