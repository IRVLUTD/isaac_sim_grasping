from omni.isaac.core.controllers import BaseController, BaseGripperController
from omni.isaac.core.utils.types import ArticulationActions
import numpy as np

#Dynamic control API
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()

class ForceController(BaseGripperController):
    def __init__(self, close_mask, test_time, max_efforts):
        name = "Controller"
        super().__init__(name=name)
        self.close_mask = close_mask
        self.type = 'force_control_v0'
        self.total_time = test_time
        self.max_efforts = max_efforts
        return 

    def close(self,time):
        joint_command = ((self.total_time-2*time)/self.total_time)*self.max_efforts
        #print(joint_command)
        return joint_command
    
    def open(self,time):
        joint_command =  ((self.total_time-2*time)/self.total_time)*self.max_efforts
        return joint_command

    def forward(self, action, time, current_dofs, close_position):
        command = self.close(time)

        new_pos = np.zeros_like(close_position)
        for i in range(len(self.close_mask)):
            if self.close_mask[i] == 0:
                new_pos[:,i] = close_position[:,i]
            else:
                new_pos[:,i] = current_dofs[:,i]
        
        actions = ArticulationActions(joint_positions = new_pos, joint_efforts=command)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        return actions


class PositionController(BaseGripperController):
    def __init__(self, close_mask, test_time, max_efforts):
        name = "Controller"
        super().__init__(name=name)
        self.close_mask = close_mask
        self.type = 'position_controller_v0'
        self.total_time = test_time
        return 

    def close(self,time):
        joint_command = 3*((self.total_time-2*time)/self.total_time)*self.max_efforts
        #print(joint_command)
        return joint_command
    
    def open(self,time):
        joint_command =  3*((self.total_time-2*time)/self.total_time)*self.max_efforts
        return joint_command

    def forward(self, action, time, current_dofs, close_position):
        actions = ArticulationActions(joint_positions = close_position)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        return actions
