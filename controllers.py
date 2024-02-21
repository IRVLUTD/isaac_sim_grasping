# Every Controller must have same inputs
from omni.isaac.core.controllers import BaseController, BaseGripperController
from omni.isaac.core.utils.types import ArticulationActions
import numpy as np

#Dynamic control API
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()

class ForceController(BaseGripperController):
    """ Force Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """
    def __init__(self, close_mask, test_time, max_efforts,robots):
        name = "Controller"
        super().__init__(name=name)
        self.close_mask = close_mask
        self.type = 'force_control_v0'
        self.total_time = test_time
        ind = np.squeeze(np.argwhere(max_efforts[0]!=0))

        robots.set_max_efforts(np.zeros_like(ind),joint_indices=ind.reshape((1,len(ind))))
        self.max_efforts = max_efforts
        return 

    def close(self,time):
        joint_command = ((self.total_time-2*time)/self.total_time)*self.max_efforts
        #print(joint_command)
        return joint_command
    
    def open(self,time):
        return 

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
    """ Position Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """

    def __init__(self, close_mask, test_time, max_efforts, robots):
        name = "Controller"
        super().__init__(name=name)
        self.type = 'position_controller_v0'
        self.effort = max_efforts
        self.close_mask = close_mask
        return 

    def close(self,time):
        return 
    
    def open(self,time):
        return 

    def forward(self, action, time, grippers, close_position):
        current_dofs = grippers.get_joint_positions()
        pos = np.zeros_like(close_position)
        for i in range(len(self.close_mask)):
            if (self.close_mask[i]==0):
                pos[:,i] =  close_position[:,i]
            else:
                pos[:,i] = close_position[:,i] * abs(self.close_mask[i])


        if action == "h5_hand":
            pos[:,2]= -1*current_dofs[:,0]
            pos[:,3]= -1*current_dofs[:,1]
            tmp= pos[:,2:]

            grippers.set_joint_positions(tmp, joint_indices = [2,3])


        actions = ArticulationActions(joint_positions = pos)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        #print(pos)
        return actions


class TransferPositionController(BaseGripperController):
    """ Transfer Position Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """

    def __init__(self, close_mask, test_time, max_efforts, robots):
        name = "Controller"
        super().__init__(name=name)
        self.type = 'transfer_position_controller_v0'
        self.effort = max_efforts
        self.close_mask = close_mask
        self.touch_dofs = np.zeros_like(max_efforts)
        return 

    def close(self,time):
        return 
    
    def open(self,time):
        return 

    def forward(self, action, time, grippers, close_position):
        current_dofs = grippers.get_joint_positions()
        pos = np.zeros_like(close_position)

        uninit = np.argwhere(time==0)
        init = np.argwhere(time!=0)

        #2 behaviors ready and not ready grasps (view.py gives 0 as time for each workstation that is not set up)
        for i in range(len(self.close_mask)):
            if (self.close_mask[i]== 0):
                pos[:,i] = close_position[:,i]
            elif ((abs(self.close_mask[i])-1)==0):
                if(len(init)>0):
                    pos[init,i] =  self.touch_dofs[init,i]
                if(len(uninit)>0):
                    pos[uninit,i] = close_position[uninit,i]
                    self.touch_dofs[uninit]= current_dofs[uninit]  
            else:
                pos[:,i] = close_position[:,i] * (abs(self.close_mask[i])-1)
        
        # update dofs  for untouched
                  

        if action == "h5_hand":
            pos[:,2]= -1*current_dofs[:,0]
            pos[:,3]= -1*current_dofs[:,1]
            grippers.set_joint_positions(pos[:,2:], joint_indices = [2,3])

        #print("close", close_position[0])
        #print("pos", pos[0])
        #print("current", current_dofs[0])
        #print('mask', self.close_mask)
        #input()
        actions = ArticulationActions(joint_positions = pos)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        return actions



""" LIST OF CONTROLLERS: 
They are the references used in command line to determine the controller to use
"""
controller_dict = {
    'default': PositionController,
    'force' : ForceController,
    'position': PositionController,
    'transfer_default': TransferPositionController
}
 