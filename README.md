# isaac_sim_grasping
Graspit generated grasps filtered with simulations in Isaac Sim. Simulation standalone executable is the v2_standalone.py file, a command to run the simulation is shown below. The standalone takes in the directories of the .json files describing the GraspIt grasps, the directory for the gripper .usd files, the directory for the objects .usd files and the output where to save the simulation results. The simulation loads the grasps described within one .json file, loads the corresponding gripper-object pair and tries to grasp the object with the specified controller for the gripper (manager class). The simulation then gets the corresponding slip and fall times of the objects when realizing the grasps. Failed grasps will be given a negative fall time (-1). The Simulation iterates through all .json files and skips any already existing in the output_dir. 

## Files used for simulation
1) v2_standalone.py: standalone executable
2) views.py: Simulation's behavioral code
3) managerv2.py: contains gripper information and the reporting of results
4) controllersv2.py: gripper controllers
5) utils.py: general utility functions

## Simulation Hyperparameters (within standalone executable)
- num_w: Number of Workstations to run the simulation with (gripper, object pair)
- test_time: total test time for each grasp.
- fall_threshold: Verbosity fall threshold to consider the grasped as passed (doesn't affect output file)
- slip_threshold: Verbosity slip threshold to consider the grasped as passed (doesn't affect output file)

## Complete command to run v2_standalone.py 
 ./python.sh /home/felipe/Documents/isaac_sim_grasping/v2_standalone.py --json_dir=/home/felipe/Documents/GoogleScannedObjects_10Grippers-selected/obj8 --gripper_dir=/home/felipe/Documents/isaac_sim_grasping/grippers --objects_dir=/home/felipe/Documents/GoogleScannedObjects_USD --output_dir=/home/felipe/Documents/GoogleScannedObjects_10Grippers-selected/obj8_filtered

# Adding Grippers
Import the grippers to Isaac Sim using the GUI and save them as .usd in the gripper directory as the other grippers*. The .usd grippers must be tested and made ready for use within the simulation. To do so the user_examples folder was added to the repository. You should copy it to "~/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples" that way it can be loaded and used to test .usd files faster with the GUI. To load in the GUI go to the Isaac Examples tab >  ScratchPad > Testing.
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

Notes: 
- You can change the command to the different set_up files we have.
- Need to run command from the issac sim directory, used 2023 Isaac Sim version
- Deactivate conda if you have an active environment, it may cause some errors when running isaac sim
- Always use complete paths, errors may occur otherwise
- The simulation supports a specific .json file format, if the format wants to be changed, the manager class must be editted.
- Readability is very fragile for examples in Isaac Sim, you should only modify Test.py.
- Don't use the stop button to reset example, instead use the reset below loading example.
- Be sure to run render = False and "headless" option for config as True whe running standalone.
- Many Isaac Sim API functions were found to give the incorrect data when called, if errors arise when implementing new code beware of this fact.

## Isaac Sim Manual
https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html

## Helpful code for python standalones (code snippet samples)
https://docs.omniverse.nvidia.com/isaacsim/latest/reference_python_snippets.html

## Command to run standalone:
 ./python.sh ~/Documents/isaac_sim_grasping/v2_standalone.py

## Run without warnings 
Add this arguments in command line:
 --/log/level=error --/log/fileLogLevel=error --/log/outputStreamLevel=error

## Install Packages Using isaac sim python.sh
 ./python.sh -m pip install name_of_package_here
