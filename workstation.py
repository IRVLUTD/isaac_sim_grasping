#External Libraries
import numpy as np
import pandas as pd

#Custom Classes and utils
from manager import Manager
from utils import InverseT

#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative
from omni.isaac.core.utils.prims import create_prim, delete_prim
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.controllers import BaseController, BaseGripperController

#Dynamic control API
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()

class CustomController(BaseGripperController):
    def __init__(self, ID, close_dir, effort_mask):
        name = "Controller_"+str(ID)
        super().__init__(name=name)
        self.close_dir = close_dir
        self.effort_mask = effort_mask
        return 

    def close(self,command):
        joint_command = np.multiply(self.close_dir,self.effort_mask)
        joint_command = np.multiply(joint_command,command)
        #print(joint_command)
        return ArticulationAction(joint_efforts=joint_command)
    
    def open(self,command):
        joint_command = -1 * np.multiply(self.close_dir,self.effort_mask)
        joint_command = np.multiply(joint_command,command)
        return ArticulationAction(joint_efforts=joint_command)

    def forward(self, action, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).

        if action == 'close':
            actions = self.close(command)
        if action == 'open':
            actions = self.open(command)
        # A controller has to return an ArticulationAction
        return actions


class Workstation():

    def __init__(self, ID, manager, path, world, effort_mask = None, effort_step = 0.05):
        #External Objects Information
        self.world = world
        self.manager = manager
        
        #Important info for Workstations
        self.job = manager.request_job(ID)
        self.ID = ID
        self.path = path # Addition to make absolute path (Used by Isaac Sim)

        # Positional information for workstations
        self.prim = get_prim_at_path(path)
        self.pose = get_world_pose_from_relative(self.prim,[0,0,0],[1,0,0,0])
        self.worldT = tf_matrix_from_pose(self.pose[0],self.pose[1])

        # Initialize gripper and object
        self.robot = self._import_gripper()
        self.payload = self._import_object()

        #Controller Information
        self.current_effort = 1.1 #percentage
        close_dir = manager.close_dir[self.job['gripper']]
        close_dir = np.asarray(close_dir)
        if (effort_mask == None):
            effort_mask = np.ones_like(close_dir)
        self.controller = CustomController(ID,close_dir, effort_mask)
        self.effort_step = effort_step
        

    def job_step(self):
        self.job = self.manager.request_job()
        return self.job
    
    def _import_gripper(self):
        # Pose loading
        # Pointing gripper Downward
        EF_axis = self.manager.EF_axis[self.job['gripper']]
        # No transform needed EF_axis == -3 (default self.T_EF)
        self.T_EF = np.array([[1,0,0,0],
                             [0,1,0,0],
                             [0,0,1,0],
                             [0,0,0,1]])
        if (EF_axis == 1):
            self.T_EF = np.array([[ 0,0,1,0],
                             [ 0,1,0,0],
                             [-1,0,0,0],
                             [0,0,0,1]])
        elif (EF_axis == 2):
            self.T_EF = np.array([[1, 0,0,0],
                             [0, 0,1,0],
                             [0,-1,0,0],
                             [0, 0,0,1]])
        elif (EF_axis == 3):
            self.T_EF = np.array([[1, 0, 0,0],
                             [0,-1, 0,0],
                             [0, 0,-1,0],
                             [0, 0, 0,1]])
        elif (EF_axis == -1):
            self.T_EF = np.array([[0,0,-1,0],
                             [0,1, 0,0],
                             [1,0, 0,0],
                             [0,0, 0,1]])
        elif (EF_axis == -2):
            self.T_EF = np.array([[1,0, 0,0],
                             [0,0,-1,0],
                             [0,1, 0,0],
                             [0,0, 0,1]])

        
        # Robot Pose
        T = np.matmul(self.worldT, self.T_EF)
        self.gripper_pose= pose_from_tf_matrix(T)

        # Adding Robot usd
        usd_path = self.manager.gripper_dict[self.job['gripper']] 
        add_reference_to_stage(usd_path=usd_path, prim_path=self.path+"/gripper_"+str(self.ID))
        robot = self.world.scene.add(Robot(prim_path = self.path+"/gripper_"+str(self.ID), name="gripper_"+str(self.ID),
                                           position = self.gripper_pose[0], orientation = self.gripper_pose[1]))
        return robot
    
    def _import_object(self):
        # Pose Calculation
        T = np.matmul(self.worldT, self.T_EF)
        pos =  self.job['grasps']['pose'][:3]
        q = self.job['grasps']['pose'][3:]
        reorder = [3, 0, 1, 2]
        q = [q[i] for i in reorder]
        tmp = tf_matrix_from_pose(pos, q)
        tmp = InverseT(tmp)
        T_final = np.matmul(T, tmp)        
        self.object_init_pose= pose_from_tf_matrix(T_final)

        #Adding Object usd
        usd_path = self.manager.object_dict[self.job["object_id"]]
        add_reference_to_stage(usd_path=usd_path, prim_path=self.path+"/object_"+str(self.ID))
        payload = self.world.scene.add(GeometryPrim(prim_path = self.path+"/object_"+str(self.ID), name="object_"+str(self.ID),
                                                 position = self.object_init_pose[0], orientation = self.object_init_pose[1]))
        return payload

    def print_robot_info(self):
        '''Print Robot Information to Terminal'''
        print("Workstation " + str(self.ID) + " Info: ")
        print("Gripper: " + str(self.job["gripper"]) )
        print("# DoF = " + str(self.robot.num_dof)) # prints 2
        print("DoF names: "+ str(self.robot.dof_names))
        print("Joint Positions for Workstation " + str(self.ID) + ": " + str(self.robot.get_joint_positions()))

    def reset_robot(self):
        '''Reset Robot to Initial Position'''
        self.robot.initialize()
        robot_pos = np.array(self.job["grasps"]['dofs'])
        if(self.job["gripper"]=="fetch_gripper"):
            robot_pos = robot_pos/-100
        self.current_effort = 1
        #print(robot_pos)
        self.robot.set_joint_positions(robot_pos)

    def physics_step(self,step_size):
        """ Physics step performed by physics API of Isaac Sim

        Args: 
            step_size: time difference between physics steps (Default frquency is 60 Hz)
        """

        #Initialize articulation Run on first simulation step (needed for functions to work)
        if (self.current_effort==1): 
            articulation = dc.get_articulation(self.path+"/gripper_"+str(self.ID))
            dof_props = dc.get_articulation_dof_properties(articulation)
            self.max_efforts = []
            for i in dof_props:
                self.max_efforts.append(i[6])
            self.max_efforts=np.asarray(self.max_efforts)
                
        effort = self.max_efforts*self.current_effort

        #Percent per second you will reduce
        self.current_effort -= self.effort_step*step_size
        #print(step_size)
        actions = self.controller.forward('close', effort)
        self.robot.apply_action(actions)
        return


if __name__ == "__main__":
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/gripper_usd"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    G_manager = Manager(json_path, grippers_path, objects_path)
    w1 = Workstation(G_manager)
    w2 = Workstation(G_manager)

    print(w1.step())
    print(w2.step())
    q = w1.job['grasps']['pose'][3:]
    print(q)
    reorder = [3, 0, 1, 2]
    print(q)
    q = [q[i] for i in reorder]
    print(w2.step())
    print(w2.step())
    print(G_manager.task_pointer)
