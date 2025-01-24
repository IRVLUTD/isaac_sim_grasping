# Tests

The State of the Art robotic grasp datasets use perform different tests to evaluate their robotic grasps in simulation. Similar to the gripper controller, we have added a 'Test' class that simplifies the creation of new tests. This class is responsible for applying the necessary force, acceleration, or velocity to the objects being grasped during these tests. Below, you'll find the base test class named 'GenericTest', which includes the required functions and inputs used in the simulation.

- When creating custom tests, add your class to the [tests.py](../tests.py) file and add an entry in [tests_dict](../tests.py). To use your test, just call the standalone with your test key `--test_type=(new test key)`.


### Generic Test Class

``` python
class GenericTest():
    """
    Base class for all test types. This class provides common functionality shared across different tests.
    """

    def __init__(self):
        """ Initialize the test class. It must always contain a label with its tag
        """
        self.label = "Unknown"
        return
    
    def failure_condition(self, init_pos, init_rot, indices):
        ''' Method to be overridden by subclasses. Function to evaluate if the test has failed. 
        W = number of workstations to check

        Arguments:
        - init_pos: Initial positions of objects (numpy W x 3 array)
        - init_rot: Initial rotation of objects (numpy W x 4 array)
        - indices: numpy array with ID indices of objects to check 

        Return:
        - finish_ind: array with all the indices within indices that failed the test
        '''
        return NotImplementedError("Subclass must implement abstract method")

    def setup_condition(self, init_pos, init_rot, indices):
        ''' Function to test if the objects advance from the setup phase. Default lets every object pass the setup phase, use ifthere is no desire for such a phase. During the the setup phase a workstations, current time will remain 0.

        Arguments:
        - init_pos: Initial positions of objects (numpy W x 3 array)
        - init_rot: Initial rotation of objects (numpy W x 4 array)
        - indices: numpy array with ID indices of objects to check 

        Return:
        - setup_ind: array with all the indices within indices that passed the setup phase
        '''
        setup_ind = indices
        return setup_ind
    
    def test_step(self, current_times):
        ''' Method to be overridden by subclasses. Step function to run at every physics step. During the the setup phase a workstations, current time will remain 0.

        Arguments:
        - current_times: Current clock time of every workstation in the simulation (W x 1 numpy array)
        '''

        return NotImplementedError("Subclass must implement abstract method")
```

**Note**: When creating a new Test be sure that the functions have the same input. For the code of example tests refer to [tests.py](../tests.py). A visual demonstration of how this tests work is show below.

#### Gravity
Applies gravity for the entire test time after the gripper has secured enough contacts with the object. The amount of contacts varies with each gripper (contact_th in [gripper_isaac_info.json](../grippers/gripper_isaac_info.json)). 

<p align="center">
<img src='../media/robotiq_Clock.gif'>
</p>

#### Axes Forces
Applies forces in the positive and negative direction of the world reference frame x, y and z axes. It is the test performed by the [Multidex](https://sites.google.com/view/gendexgrasp/multidex) and the [DexGraspNet](https://pku-epic.github.io/DexGraspNet/) datasets.

<p align="center">
<img src='../media/axes_forces.gif'>
</p>

