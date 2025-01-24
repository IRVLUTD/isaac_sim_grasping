# Gripper Controllers

When testing robotic grasps, you might want a different approach to control the gripper's joint values. Our controller class allows you to program various gripper behaviors as you see fit, ranging from actively managing the position, velocity, and force of the gripper joints, to keeping the joint values constant. Below, you'll find the base controller class named 'GenericController', including the necessary functions and inputs used in the simulation. Additionally, it loads the [controller_info.json](../grippers/controller_info.json) file, which contains relevant control data for each gripper.

- When creating custom controllers, add your class to the [controllers.py](../controllers.py) file and add an entry in [controller_dict](../controllers.py). To use your controller, just call the standalone with your controller key `--controller=(new controller key)`.

### GenericController Class

``` python
from omni.isaac.core.controllers import BaseController
import json 
import os

class GenericController(BaseController):
    """
    Base class for all controller types. This class provides common functionality 
    shared across different controllers. It can also be used 
    """

    def __init__(self, gripper_name='', grippers= None, test_time = 0):
        """
        Initialize the generic controller using Isaac Sim BaseController. 

        Parameters: 
            gripper_name: Gripper ID.
            grippers: ArticulationView class containing all the grippers to control.
            test_time: Time of test to perform
        """
        self.name = 'Controller_' + gripper_name
        super().__init__(name=self.name)
        #Load controller info.json
        c_path = os.path.dirname(os.path.abspath(__file__))
        controller_json =  os.path.join(c_path,"grippers/controller_info.json")
        with open(controller_json) as fd:
            self.gripper_dict = json.load(fd)
        self.label = 'Unknown'  # A label to identify the controller version or type

    def forward(self, time):
        """
        Method to be overridden by subclasses. This method computes actions 
        for the grippers based on the current time.

        Arguments:
        - time: Current clock time of every workstation in the simulation (W x 1 numpy array)
        """
        raise NotImplementedError("Subclass must implement abstract method")
```
**Note**: When creating a new Controller be sure that the functions have the same input. For the code of example controllers refer to [controllers.py](../controllers.py). A visual demonstration of how this controllers work is show below.

#### Position-based Controller
Uses the 'close_dir' mask from [controller_info.json](../grippers/controller_info.json) to actionate specific joints and close the grippers.

<p align="center">
<img src='../media/robotiq_Clock.gif'>
</p>

#### Transfer Position-based Controller
Uses the 'transfer_close_dir' mask from [controller_info.json](../grippers/controller_info.json) to activate specific joints and close the grippers. Unlike the standard position-based controller, this one is tailored for grippers that start in the open position and operates in two distinct phases: one phase before setting up the grasp, and another phase after."

<p align="center">
<img src='../media/transfer_close.gif'>
</p>

#### Static Controller

It was created for the implementation of the axes forces test from the[Multidex](https://sites.google.com/view/gendexgrasp/multidex) and the[DexGraspNet](https://pku-epic.github.io/DexGraspNet/) datasets. It fixes the grippers' joints for the entirety of the test.

<p align="center">
<img src='../media/axes_forces.gif'>
</p>