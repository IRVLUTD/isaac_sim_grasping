# Every Controller must have same inputs
from omni.isaac.core.controllers import BaseController, BaseGripperController
from omni.isaac.core.utils.types import ArticulationActions, ArticulationAction
import numpy as np
import json
import os

class PositionController(BaseController):
    """ Position Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """

    def __init__(self, gripper_name, grippers, test_time):
        name = "Controller" #Stage name of controller
        super().__init__(name=name)
        self.type = 'position_controller_v0'
        
        self.grippers = grippers
        self.gripper_name = gripper_name
        self.total_test_time = test_time

        gripper_info = self.grippers.get_dof_limits()
        dof_limits = gripper_info[0]
        self.close_positions  = np.zeros((gripper_info.shape[0],len(dof_limits)))
        #Load controlle info.json
        c_path = os.path.dirname(os.path.abspath(__file__))
        controller_json =  os.path.join(c_path,"grippers/controller_info.json")
        with open(controller_json) as fd:
            gripper_dict = json.load(fd)
        self.close_mask = gripper_dict[gripper_name]["close_dir"]

        for i in range(len(dof_limits)):
            if (self.close_mask[i]==0):
                self.close_positions[:,i]=(self.grippers.get_joint_positions()[0][i])
            elif (self.close_mask[i]>0):
                self.close_positions[:,i]=(dof_limits[i][1])
            elif (self.close_mask[i]<0):
                self.close_positions[:,i]=(dof_limits[i][0]) 
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")
        

    def forward(self, time):
        current_dofs = self.grippers.get_joint_positions()
        pos = np.zeros_like(self.close_positions)
        for i in range(len(self.close_mask)):
            if (self.close_mask[i]==0):
                pos[:,i] =  self.close_positions[:,i]
            else:
                pos[:,i] = self.close_positions[:,i] * abs(self.close_mask[i])


        if self.gripper_name == "h5_hand":
            pos[:,2]= -1*current_dofs[:,0]
            pos[:,3]= -1*current_dofs[:,1]
            tmp= pos[:,2:]

            self.grippers.set_joint_positions(tmp, joint_indices = [2,3])


        actions = ArticulationActions(joint_positions = pos)
        
        # Save last action and apply to stage view
        self.last_actions = actions
        self.grippers.apply_action(actions)
        return actions


class TransferPositionController(BaseController):
    """ Transfer Position Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """
    

    def __init__(self, gripper_name, grippers, test_time):
        name = "Controller" #Stage name
        super().__init__(name=name)
        self.type = 'transfer_position_controller_v0'

        self.grippers = grippers
        self.gripper_name = gripper_name
        self.total_test_time = test_time

        gripper_info = self.grippers.get_dof_limits()
        dof_limits = gripper_info[0]
        self.close_positions  = np.zeros((gripper_info.shape[0],len(dof_limits)))
        #Load controlle info.json
        c_path = os.path.dirname(os.path.abspath(__file__))
        controller_json =  os.path.join(c_path,"grippers/controller_info.json")
        with open(controller_json) as fd:
            gripper_dict = json.load(fd)
        self.close_mask = gripper_dict[gripper_name]["transfer_close_dir"]

        for i in range(len(dof_limits)):
            if (self.close_mask[i]==0):
                self.close_positions[:,i]=(self.grippers.get_joint_positions()[0][i])
            elif (self.close_mask[i]>0):
                self.close_positions[:,i]=(dof_limits[i][1])
            elif (self.close_mask[i]<0):
                self.close_positions[:,i]=(dof_limits[i][0]) 
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")
        
        self.touch_dofs = np.zeros_like(self.close_positions)
        return 
    

    def forward(self, time):
        current_dofs = self.grippers.get_joint_positions()
        pos = np.zeros_like(self.close_positions)
        time = np.squeeze(time)
        uninit = np.argwhere(time==0)
        init = np.argwhere(time!=0)
        self.touch_dofs[uninit]= current_dofs[uninit]  
        
        #2 behaviors ready and not ready grasps (view.py gives 0 as time for each workstation that is not set up)
        for i in range(len(self.close_mask)):
            if (self.close_mask[i]== 0):
                pos[:,i] = self.close_positions[:,i]
            elif ((abs(self.close_mask[i])-1)!=0):
                pos[:,i] = self.close_positions[:,i] * (abs(self.close_mask[i])-1)
            else:
                if(len(init)>0):
                    pos[init,i] =  self.touch_dofs[init,i]
                if(len(uninit)>0):
                    pos[uninit,i] = self.close_positions[uninit,i]

        if self.gripper_name == "h5_hand":
            pos[:,2]= -1*current_dofs[:,0]
            pos[:,3]= -1*current_dofs[:,1]
            self.grippers.set_joint_positions(pos[:,2:], joint_indices = [2,3])

        #print("action", pos[0])
        actions = ArticulationActions(joint_positions = pos)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        self.grippers.apply_action(actions)
        return actions

class StaticController(BaseController):
    """ Static Control
    """

    def __init__(self, gripper_name, grippers, test_time):
        name = "Controller" #Stage name of controller
        super().__init__(name=name)
        self.type = 'static_controller_v0'
        
        self.grippers = grippers
        self.gripper_name = gripper_name
        self.total_test_time = test_time

        #Load controlle info.json

    def forward(self, time):
        current_dofs = self.grippers.get_joint_positions()

        actions = ArticulationActions(joint_positions = current_dofs)
        
        # Save last action and apply to stage view
        self.last_actions = actions
        self.grippers.apply_action(actions)
        return actions

class PositionSphereController(BaseController):
    """ Position sphere controller
    UGCS based controller for the movement of grippers using a unified geometric space.


    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """

    def __init__(self, close_mask, test_time, max_efforts, robots):
        name = "Controller"
        super().__init__(name=name)

        # Initialize vectors on unit sphere
        self.vectors = np.zeros((14,3))
        self.vectors[0] = [1, 0, -100] # -100 is key for sphere pole 
        self.vectors[1] = [1, 45, 0]
        self.vectors[2] = [1, 45, 90]
        self.vectors[3] = [1, 45, 180]
        self.vectors[4] = [1, 45, 270]
        self.vectors[5] = [1, 90, 0]
        self.vectors[6] = [1, 90, 90]
        self.vectors[7] = [1, 90, 180]
        self.vectors[8] = [1, 90, 270]        
        self.vectors[9] = [1, 135, 0]
        self.vectors[10] = [1, 135, 90]
        self.vectors[11] = [1, 135, 180]
        self.vectors[12] = [1, 135, 270]
        self.vectors[13] = [1, 180, -100] # -100 is key for sphere pole 

        # Load gripper correspondences (3D points + spherical value in max sphere)
        
        # Use an external .json file
        
        
        self.type = 'sphere_position_controller'
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
        time = np.squeeze(time)
        uninit = np.argwhere(time==0)
        init = np.argwhere(time!=0)
        self.touch_dofs[uninit]= current_dofs[uninit]  
        return 



""" LIST OF CONTROLLERS: 
They are the references used in command line to determine the controller to use
"""
controller_dict = {
    'default': PositionController,
    'position': PositionController,
    'transfer_position': TransferPositionController,
    'static': StaticController
}
 