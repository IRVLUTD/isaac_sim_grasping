# MultiGripperGrasp Toolkit 2.0
![](/media/F1_isaac.png)
**Overview**:
This repository hosts a grasp filter simulation crafted with Isaac Sim. It focuses on evaluating and visualizing contemporary robotic grasp datasets. Key updates in version 2.0 include:
  - **Consolidation**: All previous standalone tools have been integrated into a single standalone for improved usability.
  - **Simplified Testing**: Easier implementation of various grasp tests.
  - **Code Optimization**: The codebase has been streamlined, reducing both size and complexity for better performance.

### Citing MultiGripperGrasp Toolkit

This toolkit was developed to evaluate synthetically generated grasps for our MultiGripperGrasp (MGG) dataset. It was featured in a research paper at IROS 2024, with the original repository available at the [iros_2024 branch](https://github.com/IRVLUTD/isaac_sim_grasping/tree/iros_2024). Paper details:

**MultiGripperGrasp: A Dataset for Robotic Grasping from Parallel Jaw Grippers to Dexterous Hands**

Luis Felipe Casas, Ninad Khargonkar, Balakrishnan Prabhakaran, Yu Xiang

[[paper](https://arxiv.org/pdf/2403.09841.pdf)] [[video](https://www.youtube.com/watch?v=pm1K6wbc830)] [[arXiv](https://arxiv.org/abs/2403.09841)] [[project site](https://irvlutd.github.io/MultiGripperGrasp)] [[dataset folder](https://utdallas.box.com/v/multi-gripper-grasp-data)]

If the MultiGripperGrasp Toolkit has been useful for your research, we'd really appreciate if you could cite it as it will help our ongoing work.

    @misc{casas2024multigrippergraspdatasetroboticgrasping,
      title={MultiGripperGrasp: A Dataset for Robotic Grasping from Parallel Jaw Grippers to Dexterous Hands}, 
      author={Luis Felipe Casas and Ninad Khargonkar and Balakrishnan Prabhakaran and Yu Xiang},
      year={2024},
      eprint={2403.09841},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2403.09841}, 
     }

### License

MultiGripperGrasp Toolkit is released under the [GNU General Public License v3.0](LICENSE).

### Table of Contents

- [MultiGripperGrasp Toolkit 2.0](#multigrippergrasp-toolkit-20)
    - [Citing MultiGripperGrasp Toolkit](#citing-multigrippergrasp-toolkit)
    - [License](#license)
  - [Installation](#installation)
  - [Running the Simulation](#running-the-simulation)
      - [Running MGG Data](#running-mgg-data)
      - [Running Alternative Tests.](#running-alternative-tests)
      - [Testing grasps without joint values data](#testing-grasps-without-joint-values-data)
      - [Visualizing Grasps](#visualizing-grasps)
  - [Documentation](#documentation)
    - [Helpful Links](#helpful-links)
    - [Notes](#notes)


## Installation
This repository was tested using Isaac Sim 4.2.0 on Ubuntu

1. Clone the repo:

    ```Shell
    git clone git@github.com:IRVLUTD/isaac_sim_grasping.git
    ```

2. Install the Isaac Sim 4.2.0 and Navigate to its Python directory:
   Follow the instructions in the [Isaac Sim Website](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) for proper installation. 
   Normally the python.sh is on the directory `~/.local/share/ov/pkg/(isaac-sim version)`. You can access it directly by opening Isaac Sim in terminal using the Omniverse Launcher.

3. Install the required libraries using the Isaac Sim `python.sh` environment. :
   ```Shell
    ./python.sh -m pip install tqdm
    ```
    - Note: Only tqdm must be installed, all other libraries should come with the Isaac Sim version. If you would like to use other libraries you would need to use the same method.


## Running the Simulation
The simulation can use any gripper and object, provided they are prepared correctly. We offer ready to use objects and grippers from our lab and the MGG dataset; the grippers are included in the repository, and the objects must be downloaded externally ([object_usd](https://utdallas.box.com/v/multi-gripper-grasp-data)). Refer to the documentation for guidance on how to:

- Import your own grippers [import_grippers.md](docs/import_grippers.md)
- Import your own objects [import_objects.md](docs/import_objects.md)


**Simulation Process:**

- **Load Grasp Information**: The simulation starts by loading the necessary grasp data from a correctly formatted .json file. ([grasp data format](docs/grasp_data_format.md))
- **Create Workstations**: Multiple "workstations" are set up to simulate multiple grasps simultaneously.
- **Perform Grasps**: Using the specified gripper controller, the simulation attempts to execute the grasps while conducting the specified test on the object.
- **Records Results**: Once a test is completed, the result (including time and status) is saved to an output file with the same name as the grasps.json file. The status is either passed or failed.
- **Asynchronous Operation**: Grasp tests are performed asynchronously across all workstations. After completing a test, each workstation resets and loads the next grasp for testing.
- **Completion and Reset**: When all grasps are tested, the output file is finalized, the entire simulation environment is reset, and a new simulation file is loaded to start the process anew.

<p align="center">
<img src='media/transfer_far.gif'>
</p>

### Important Files/Folders description
1) standalone.py: Standalone executable of the simulation
2) views.py: Simulation's behavioral code
3) managers.py: Contains the class responsible for loading grasp information and the reporting of results.
4) controllers.py: Developed gripper controllers
5) tests.py: Developed tests.
6) utils_local.py: General utility functions
7) sim_utils.py: Simulation utility functions
8) grippers: Gripper .usd files folder
9) grasps: Sample .json file with grasp information

### Parameters and Inputs
The new MGG Toolkit has been overhauled to only use one standalone executable `standalone.py`. With it you are able to run the simulation with complete grasp data, without specified joint values for the grasps or in visualization mode. 

The standalone.py takes as input:
- json_dir: Directory of grasp data to filter. The program does not read .json files within folders, therefore, a valid directory would be the Allegro folder within the [MGG dataset](https://utdallas.box.com/v/multi-gripper-grasp-data). Defaults to the repository /grasps folder. 

- gripper_dir: Gripper directory (folder containing all the gripper .usd files). Defaults to the repository /grippers folder.
  
- objects_dir: Object directory (folder containing all the object .usd folder). Defaults to the repository /objects folder. **Note**: the objects are not included into the repo, you must download them externally ([object_usd](https://utdallas.box.com/v/multi-gripper-grasp-data)) and may add the folder to the repository directory if you would like to omit this input. 
  
- output_dir: Output directory where the results will be stored in .json files. The format information can be found in the [grasp data format](docs/grasp_data_format.md) page. Defaults to a /output folder in the repository, it will create one if it doesn't exits.
  
- num_w: Number of Workstations to run grasps simultaneously (default: 100).
  
- test_time: Total test time for each grasp test in seconds (default:3).

- controller: Controller reference (within [controllers.py](controllers.py) controller_dict)(default: position). For more information on how to use the different controllers and how to create a new refer to [controllers.md](docs/controllers.md).
  
- test_type: Test reference (within [tests.py](tests.py) test_dict)(default: gravity). For more information on how to use the different tests and how to create a new one refer to [tests.md](docs/tests.md).

- (Optional) dof_given: Boolean parameter to signal the simulation if the joint value information or Degrees of Freedom (DoF) information is given. If so the simulation will try to load the values, else it will start each grasp at the gripper's specified opened position.
- (Optional) print_results: Verbosity of standalone after finishing one .json file. Prints results...
- (Optional) view_mode: launches the simulation in view mode, showing the grasps directly without performing any test, every grasp will be shown for the total test time.
- (Optional) force_reset: Relaunches Isaac Sim after completing the simulation of 1 .json file, this can be handy when the simulation takes to long to reset on its own.
- (Optional) device: GPU device to use for Isaac Sim (default: 0)
- (Optional) headless: Run the simulation headless 

Note: To run the simulation without warnings printed in the terminal add the following parameter to the command: 
```Shell
 --/log/outputStreamLevel=error
```

Sample commands to run the simulation are shown below for multiple configurations. Note: for Isaac Sim standalone executables, the commands must be run from the isaac sim python.sh directory. We  assume you have added the objects folder to the repository. 

#### Running MGG Data 

The original MGG dataset was tested using the repository's position controller and the gravity test. The simulation consisted on actively closing the gripper joints while applying gravity to the object and recording the time it took for the object to fall. Refer to the original [paper](https://arxiv.org/pdf/2403.09841.pdf) for more information. 

Command:
```Shell
./python.sh (repo directory)/standalone.py --json_dir=(path to grasp data) --num_w=25 --test_time=3 --test_type=gravity --dof_given --/log/outputStreamLevel=error 
```
This command will create 25 workstations and will perform the gravity test for 3 seconds on each grasp. We also include the --dof_given flag to signal the simulation to load the grasp data from the grasps. 

<p align="center">
<img src='media/robotiq_Clock.gif'>
</p>

#### Running Alternative Tests.

State of the Art (SotA) datasets use different tests to evaluate grasps in simulation. One popular test used in both the [Multidex](https://sites.google.com/view/gendexgrasp/multidex) and the [DexGraspNet](https://pku-epic.github.io/DexGraspNet/) datasets is to apply forces on the objects in the +/- direction of the x, y, z axes of the world coordinate frame while fixing the grippers' joint values. The new MGG Toolkit makes it easy to implement these tests by creating new controller and test classes. In the case of running the previous configuration you would use the subsequent command.

Command:
```Shell
./python.sh /home/felipe/Documents/RL_MGG/isaac_sim_grasping/standalone.py --num_w=25 --test_time=6 --test_type=axes_forces --controller=static --dof_given --/log/outputStreamLevel=error 
```

This command would create 25 workstations, use the new test of "axes_forces" and the static controller to fix the joint values throughout the test. Since the json_dir is not specified it will use the files on the [/grasps](/grasps) folder.

<p align="center">
<img src='media/axes_forces.gif'>
</p>


#### Testing grasps without joint values data
Furthermore, for the implementation of transferred grasps from our original MGG [paper](https://arxiv.org/pdf/2403.09841.pdf), we have added the option to use predetermined initial joint values for all the grasps. You can observe the opened position of all grippers and command to run our transferred grasps experiments below:

<p align="center">
<img  src='media/transfer_grasp.png' width='1000'>
</p>

Command:
``` Shell
./python.sh /home/felipe/Documents/RL_MGG/isaac_sim_grasping/standalone.py --num_w=25 --test_time=3 --test_type=gravity --controller=transfer_position --/log/outputStreamLevel=error 
```

This command will run the gravity test using our transferred grasp gripper controller "transfer_position". Additionally, the simulation can now filter grasps where the object and grippers initialize in an overlapped position. 

<p align="center">
<img  src='media/transfer_close.gif' width=1000>
</p>


#### Visualizing Grasps
Finally, to visualize grasps from the .json files without performing the grasps the standalone can be executed in viewer mode. 

Command:
``` Shell
./python.sh /home/felipe/Documents/RL_MGG/isaac_sim_grasping/standalone.py --num_w=10 --test_time=1  --view_mode --dof_given --/log/outputStreamLevel=error
```

The grasps will be visualized for 1 sec (test_time) at every workstation. An alternative visualization tool can be found on the [viz_optas](viz_optas) folder. 

<p align="center">
<img  src='media/visualize.gif'>
</p>

## Documentation
- [Importing a new Gripper](docs/import_grippers.md)
- [Importing a new Object](docs/import_objects.md)
- [Gripper Controllers](docs/controllers.md)
- [Custom Tests](docs/tests.md)
- [Grasp Data Format](docs/grasp_data_format.md)

### Helpful Links
- MultiGripperGrasp Shared Folder [here](https://utdallas.box.com/v/multi-gripper-grasp-data). This folder has the following data along with a README for the dataset:
  - Google Scanned Objects and YCB objects .usd files (`Object_Models` folder)
  - Filtered GraspIt Grasps Dataset (.zip file)
  - Transferred Grasps Dataset (.zip file)
- Isaac Sim Manual: https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html
- Helpful code for python standalones (code snippet samples): https://docs.omniverse.nvidia.com/isaacsim/latest/reference_python_snippets.html
- Installing Packages for use with isaac sim python.sh: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html

### Notes
- Deactivate conda if you have an active environment, it may cause some errors when running isaac sim.
- Always use complete paths for the directories, errors may occur otherwise.
- For proper testing a hypertuning of the gripper .usds may be required (e.g joint velocities)
