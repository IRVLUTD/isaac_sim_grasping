from omni.isaac.core.controllers import BaseController, BaseGripperController
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

#Dynamic control API
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()

class ForceController(BaseGripperController):
    def __init__(self, path, ID, close_mask, robot_pos, test_time, translator):
        name = "Controller_"+str(ID)
        super().__init__(name=name)
        self.close_mask = close_mask
        self.raw_dofs = robot_pos #Untranslated
        self.type = 'force_control_v0'
        self.hot_init = False #Need to initialize dynamic control when the physics simulation is running 
        self.path = path
        self.translator = translator
        self.ID = ID
        self.total_time = test_time
        return 

    def close(self,time):
        joint_command = 3*((self.total_time-2*time)/self.total_time)*self.max_efforts
        #print(joint_command)
        return joint_command
    
    def open(self,time):
        joint_command =  3*((self.total_time-2*time)/self.total_time)*self.max_efforts
        return joint_command

    def forward(self, action, time, current_dofs):
        if (self.hot_init== False):
            self.hot_init = True
            return self.reset(self.close_mask, self.raw_dofs)
            
        if action == 'close':
            command = self.close(time)
        if action == 'open':
            command = self.open(time)
        if action == 'any':
            command = self.close(time)
        new_pos = np.zeros_like(self.close_position)
        for i in range(len(self.close_mask)):
            if self.close_mask[i] == 0:
                new_pos[i] = self.close_position[i]
            else:
                new_pos[i] = current_dofs[i]
        actions = ArticulationAction(joint_positions = new_pos, joint_efforts=command)
        # A controller has to return an ArticulationAction
        self.last_actions = actions
        return actions
    
    def reset(self, close_mask ,robot_pos):
        # Hot reset and initialization 
        self.init_dofs= np.squeeze(self.translator(robot_pos))
        self.last_action = np.zeros_like(self.init_dofs)
        self.close_position = np.zeros_like(self.init_dofs)
        self.open_position = np.zeros_like(self.init_dofs)
        self.max_efforts = np.zeros_like(self.init_dofs)
        self.close_mask = close_mask
        articulation = dc.get_articulation(self.path+"/gripper_"+str(self.ID))
        dof_props = dc.get_articulation_dof_properties(articulation)

        for i in range(len(dof_props)):
            if (close_mask[i]==0):
                self.max_efforts[i] = 0
                self.open_position[i] =(self.init_dofs[i])
                self.close_position[i]=(self.init_dofs[i])
            elif (close_mask[i]==1):
                self.max_efforts[i]= dof_props[i][6]
                self.open_position[i]=(dof_props[i][2])
                self.close_position[i]=(dof_props[i][3])
            elif (close_mask[i]==-1):
                self.max_efforts[i]= -dof_props[i][6]
                self.open_position[i]=(dof_props[i][3])
                self.close_position[i]=(dof_props[i][2])
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")
            
        return ArticulationAction(joint_efforts =self.init_dofs)

    def get_last_action(self):
        return self.last_actions















class PositionController(BaseGripperController):
    def __init__(self, path, ID, close_mask, robot_pos, test_time, translator):
        name = "Controller_"+str(ID)
        super().__init__(name=name)
        self.close_mask = close_mask
        self.raw_dofs = robot_pos
        self.type = 'position_control_v0'
        self.hot_init = False #Need to initialize dynamic control when the physics simulation is running 
        self.path = path
        self.ID = ID
        self.translator = translator
        self.total_time = test_time
        
        return 

    def close(self,time):
        joint_command = self.close_position
        #print(joint_command)
        return ArticulationAction(joint_positions=joint_command)
    
    def open(self,time):
        joint_command =  self.open_position
        return ArticulationAction(joint_positions=joint_command)

    def forward(self, action, time, current_dofs):
        if (self.hot_init== False):
            self.hot_init = True
            return self.reset(self.close_mask, self.raw_dofs)
            
        actions = self.close(time)
        self.last_actions = actions
        return actions
    
    def reset(self, close_mask ,robot_pos):
        self.init_dofs= np.squeeze(self.translator(robot_pos))
        self.last_action = np.zeros_like(self.init_dofs)
        self.close_position = np.zeros_like(self.init_dofs)
        self.open_position = np.zeros_like(self.init_dofs)
        self.close_mask = close_mask
        articulation = dc.get_articulation(self.path+"/gripper_"+str(self.ID))
        dof_props = dc.get_articulation_dof_properties(articulation)

        for i in range(len(dof_props)):
            if (close_mask[i]==0):
                self.open_position[i] =(self.init_dofs[i])
                self.close_position[i]=(self.init_dofs[i])
            elif (close_mask[i]==1):
                self.open_position[i]=(dof_props[i][2])
                self.close_position[i]=(dof_props[i][3])
            elif (close_mask[i]==-1):
                self.open_position[i]=(dof_props[i][3])
                self.close_position[i]=(dof_props[i][2])
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")

        return ArticulationAction(joint_efforts =np.zeros_like(self.init_dofs), joint_velocities= np.zeros_like(self.init_dofs))

    def get_last_action(self):
        return self.last_action

