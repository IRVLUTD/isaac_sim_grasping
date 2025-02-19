# Every Controller must have same inputs

from omni.isaac.core.utils.types import ArticulationActions
import numpy as np
import json
import os
import tf.transformations as tf
from sim_utils import GenericController
from utils_local import load_pickle, save_pickle, get_joint_transformations, get_inverse_joint_transformations

class PositionController(GenericController):
    """ Position Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """

    def __init__(self, gripper_name, grippers, test_time):
        super().__init__(gripper_name, grippers, test_time)
        self.label = 'position_controller_v0'
        
        self.grippers = grippers
        self.gripper_name = gripper_name
        self.total_test_time = test_time

        gripper_info = self.grippers.get_dof_limits()
        dof_limits = gripper_info[0]
        self.close_positions  = np.zeros((gripper_info.shape[0],len(dof_limits)))
        self.close_mask = self.gripper_dict[gripper_name]["close_dir"]

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

class TransferPositionController(GenericController):
    """ Transfer Position Controller Version 0

    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """
    

    def __init__(self, gripper_name, grippers, test_time):
        super().__init__(gripper_name, grippers, test_time)
        self.label = 'transfer_position_controller_v0'

        self.grippers = grippers
        self.gripper_name = gripper_name
        self.total_test_time = test_time

        gripper_info = self.grippers.get_dof_limits()
        dof_limits = gripper_info[0]
        self.close_positions  = np.zeros((gripper_info.shape[0],len(dof_limits)))
        self.close_mask = self.gripper_dict[gripper_name]["transfer_close_dir"]

        for i in range(len(dof_limits)):
            if (self.close_mask[i]==0):
                self.close_positions[:,i]=(self.grippers.get_joint_positions()[0][i])
            elif (self.close_mask[i]>0):
                self.close_positions[:,i]=(dof_limits[i][1])
            elif (self.close_mask[i]<0):
                self.close_positions[:,i]=(dof_limits[i][0]) 
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")
        
        self.touch_dofs = self.grippers.get_joint_positions()
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

class StaticController(GenericController):
    """ Static Control
    """

    def __init__(self, gripper_name, grippers, test_time):
        super().__init__(gripper_name, grippers, test_time)
        self.label = 'static_controller_v0'
        self.grippers = grippers
        self.gripper_name = gripper_name
        self.total_test_time = test_time

    def forward(self, time):
        current_dofs = self.grippers.get_joint_positions()

        actions = ArticulationActions(joint_positions = current_dofs)
        
        # Save last action and apply to stage view
        self.last_actions = actions
        self.grippers.apply_action(actions)
        return actions

class SphereController(GenericController):
    """ Position sphere controller
    UGCS based controller for the movement of grippers using a unified geometric space.


    Args:
        close_mask: dofs directions to close grippers 
        test_time: total test time
        max_effors: max_effort for Gripper "driving dofs"
        robots: ArticulationView for Robots in simulation
    """

    def __init__(self, gripper_name, grippers, test_time):
        super().__init__(gripper_name, grippers, test_time)

        # Initialize vectors on unit sphere
        self.vectors = np.zeros((14,3))
        self.vectors[0] = [1, 0, 0] # -100 is key for sphere pole 
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
        self.vectors[13] = [1, 180, 0] # -100 is key for sphere pole 

        self.label = 'sphere_position_controller' 

        # Get gripper data
        self.sphere_info = load_pickle("/home/felipe/Documents/isaac_sim_grasping/grippers/Barrett/sphere_info.pk")
        self.fixed_joints = ['a_palm_link1_joint', 'b_palm_link1_joint']
        self.fixed_joints_value = [0.96, 0.96]
        self.ks = self.sphere_info["kinematics"]
        self.finger_info = self.sphere_info["finger_chains"]

        # Get JOint Transformation matrices
        self.dof_names = self.grippers.dof_names
        self.joint_Ts = get_joint_transformations(self.ks, self.dof_names)
        self.inv_joint_Ts = get_inverse_joint_transformations(self.ks, self.dof_names)

        self.init_pos = np.zeros_like(self.dof_names, dtype = float)
        for i, item in enumerate(self.fixed_joints):
            idx = self.grippers.get_dof_index(item)
            self.init_pos[idx] = self.fixed_joints_value[i]
        
        # Precompute kinematic chains
        self.forward_kins = []
        self.inv_kins = []
        self.fingers = []
        self.finger_widths = []
        self.finger_plane_joint_indices = []
        self.finger_root_indices =[]
        self.finger_tip_indices =[]

        # Sphere T
        sphere_t = self.ks["sphere_frame"][2]
        sphere_quat = self.ks["sphere_frame"][3]
        self.sphere_T = np.eye(4)
        self.sphere_T[:3,:3] = tf.quaternion_matrix(sphere_quat)[:3,:3]
        self.sphere_T[:3,3] = sphere_t
        self.inv_sphere_T = np.linalg.inv(self.sphere_T)
        self.sphere_radius = self.ks["sphere_frame"][8]

        for finger in self.finger_info.keys():
            fi = self.finger_info[finger]
            kc = []
            inv_kc = []
            root_indices = []
            tip_indices = []
            self.fingers += [fi["kc"]]
            self.finger_widths += [fi["finger_width"]]
            self.finger_plane_joint_indices += [fi["finger_plane_joint_index"]]# Add sphere frame offset
            for i, joint in enumerate(fi["kc"]):
                if i > 0: 
                    if i < fi["finger_plane_joint_index"]:
                        root_indices+= [self.grippers.get_joint_index(joint)]
                        print("root: ", joint)
                    elif i<(len(fi["kc"])-1):
                        tip_indices += [self.grippers.get_joint_index(joint)]
                        print("tip: ", joint)
                t = self.ks[joint][2]
                q = self.ks[joint][3]
                T = np.eye(4)
                T[:3,:3]= tf.quaternion_matrix(q)[:3,:3]
                T[:3,3] = t
                kc += [T]
                inv_kc+= [np.linalg.inv(T)]
            kc = np.array(kc)
            inv_kc = np.array(inv_kc)
            self.forward_kins.append(kc)
            self.inv_kins.append(inv_kc)
            self.finger_root_indices += [np.array(root_indices)]
            self.finger_tip_indices += [np.array(tip_indices)]

        return 

    def forward(self, time):
        current_dofs = self.grippers.get_joint_positions()
        time = np.squeeze(time)

        # Solve for every finger
        for i, finger in enumerate(self.fingers):
            fpji = self.finger_plane_joint_indices[i]
            width = self.finger_widths[i]

            # Calculate finger plane transformation (jTs)
            if len(self.finger_root_indices[i]) > 0:
                # Get root angle 
                root_status = current_dofs[:,self.finger_root_indices[i]]
                bTj = np.matmul(self.forward_kins[0])
                #final_transform = np.reduce(np.matmul, transformations, axis=0) # Use similar method
                """# Helper function for einsum matrix multiplication
                    def einsum_matmul(a, b):
                        return np.einsum('...ij,...jk->...ik', a, b)

                    # Using reduce with einsum for matrix multiplication
                    final_transforms = np.reduce(einsum_matmul, transformations, axis=1)"""
            else:
                sTf = np.matmul(self.inv_sphere_T,self.forward_kins[i][0])
                #print("Root Status: ", root_status)
            if len(self.finger_tip_indices[i])>0:
                tip_status = current_dofs[:,self.finger_tip_indices[i]]
                #print("Tip Status: ", tip_status)
        #print(self.dof_names)
        #print(current_dofs)
        print(self.finger_root_indices)
        print(self.finger_tip_indices)

        actions = ArticulationActions(joint_positions = self.init_pos)
        # A controller has to return an ArticulationAction
        self.grippers.apply_action(actions)
        return 

    def get_finger_plane(self):

        return



""" LIST OF CONTROLLERS: 
They are the references used in command line to determine the controller to use
"""
controller_dict = {
    'default': PositionController,
    'position': PositionController,
    'transfer_position': TransferPositionController,
    'sphere': SphereController,
    'static': StaticController
}
 