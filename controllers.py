from omni.isaac.core.controllers import BaseController, BaseGripperController
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

#Dynamic control API
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()

class ForceController(BaseGripperController):
    def __init__(self, path, ID, close_mask, init_dofs, test_time):
        name = "Controller_"+str(ID)
        super().__init__(name=name)
        self.close_mask = close_mask
        self.init_dofs = init_dofs
        self.type = 'force_control_v0'
        self.hot_init = False #Need to initialize dynamic control when the physics simulation is running 
        self.path = path
        self.ID = ID
        self.total_time = test_time
        self.last_action = np.zeros_like(init_dofs)
        return 

    def close(self,time):
        joint_command = 0.5*(self.total_time-2*time)*np.multiply(self.close_mask,self.max_efforts)
        #print(joint_command)
        return ArticulationAction(joint_efforts=joint_command)
    
    def open(self,time):
        joint_command =  0.5*(self.total_time-2*time)*np.multiply(self.close_mask,self.max_efforts)
        return ArticulationAction(joint_efforts=joint_command)

    def forward(self, action, time):
        if (self.hot_init== False):
            self.hot_init = True
            return self.reset(self.close_mask, self.init_dofs)
            

        if action == 'close':
            actions = self.close(time)
        if action == 'open':
            actions = self.open(time)
        if action == 'any':
            if time < self.total_time/2:
                actions = self.close(time)
            else:
                actions = self.open(time)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        return actions
    
    def reset(self, close_mask ,init_dofs):
        self.close_mask = close_mask
        articulation = dc.get_articulation(self.path+"/gripper_"+str(self.ID))
        dof_props = dc.get_articulation_dof_properties(articulation)
        self.max_efforts = []
        for i in dof_props:
            self.max_efforts.append(i[6])
        self.max_efforts=np.asarray(self.max_efforts)
        return ArticulationAction(joint_positions=init_dofs, joint_efforts =np.zeros_like(init_dofs))

    def get_last_action(self):
        return self.last_actions




class PositionController(BaseGripperController):
    def __init__(self, path, ID, close_mask, init_dofs, test_time):
        name = "Controller_"+str(ID)
        super().__init__(name=name)
        self.close_mask = close_mask
        self.init_dofs = init_dofs
        self.type = 'position_control_v0'
        self.hot_init = False #Need to initialize dynamic control when the physics simulation is running 
        self.path = path
        self.ID = ID
        self.total_time = test_time
        self.last_action = np.zeros_like(init_dofs)
        return 

    def close(self,time):
        joint_command = self.close_position
        #print(joint_command)
        return ArticulationAction(joint_positions=joint_command)
    
    def open(self,time):
        joint_command =  self.open_position
        return ArticulationAction(joint_positions=joint_command)

    def forward(self, action, time):
        if (self.hot_init== False):
            self.hot_init = True
            return self.reset(self.close_mask, self.init_dofs)
            
        actions = self.close(time)
        self.last_actions = actions
        return actions
    
    def reset(self, close_mask ,init_dofs):
        self.close_mask = close_mask
        articulation = dc.get_articulation(self.path+"/gripper_"+str(self.ID))
        dof_props = dc.get_articulation_dof_properties(articulation)
        self.close_position = []
        self.open_position = []
        for i in range(len(dof_props)):
            if (close_mask[i]==0):
                self.open_position.append(init_dofs[i])
                self.close_position.append(init_dofs[i])
            elif (close_mask[i]==1):
                self.open_position.append(dof_props[i][2])
                self.close_position.append(dof_props[i][3])
            elif (close_mask[i]==-1):
                self.open_position.append(dof_props[i][3])
                self.close_position.append(dof_props[i][2])
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")
        self.open_position=np.asarray(self.open_position)
        self.close_position=np.asarray(self.close_position)
        #print(self.open_position)
        #print(self.close_position)
        #print(dof_props)
        return ArticulationAction(joint_positions=init_dofs, joint_efforts =np.zeros_like(init_dofs), joint_velocities= np.zeros_like(init_dofs))

    def get_last_action(self):
        return self.last_action
