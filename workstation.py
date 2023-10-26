#External Libraries
import numpy as np
import pandas as pd

#Custom Classes and utils
from manager import Manager
from utils import InverseT
from controllers import ForceController

#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative
from omni.isaac.core.utils.prims import create_prim, delete_prim



#Dynamic control API
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()


class Workstation():

    def __init__(self, ID, manager, path, world, controller, test_time):
        #External Objects Information
        self.world = world
        self.manager = manager
        
        #Important info for Workstations
        self.job = manager.request_job()
        self.ID = ID
        self.path = path # Addition to make absolute path (Used by Isaac Sim)

        # Positional information for workstations
        self.prim = get_prim_at_path(path)
        self.pose = get_world_pose_from_relative(self.prim,[0,0,0],[1,0,0,0])
        self.worldT = tf_matrix_from_pose(self.pose[0],self.pose[1])

        #Controller Information
        self.initial_time = 0
        self.current_time = self.initial_time
        self.test_time = test_time
        close_dir = manager.close_dir[self.job['gripper']]
        close_dir = np.asarray(close_dir)


        robot_pos = np.array(self.job["grasps"]['dofs'])
        init_dofs = self.manager.translate_dofs(self.job["gripper"], robot_pos)
        self.controller = controller(path,ID,close_dir,init_dofs, test_time)

        # Initialize gripper and object
        self.robot = self._import_gripper()
        self.object_parent_prim = self._import_object()

        
    
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
        robot = self.world.scene.add(Articulation(prim_path = self.path+"/gripper_"+str(self.ID), name="gripper_"+str(self.ID),
                                           position = self.gripper_pose[0], orientation = self.gripper_pose[1], enable_dof_force_sensors = True))
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
        payload = self.world.scene.add(GeometryPrim(prim_path = self.path+"/object_"+str(self.ID), name="object_"+str(self.ID)))
        self.object_prim = XFormPrim(prim_path= self.path +"/object_"+str(self.ID) + "/baseLink",
                                                 position = self.object_init_pose[0], orientation = self.object_init_pose[1])
        
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
        robot_pos = self.manager.translate_dofs(self.job["gripper"], robot_pos)
        self.current_time = self.initial_time
        #print(robot_pos)
        self.robot.set_joint_positions(robot_pos)

    def physics_step(self,step_size):
        """ Physics step performed by physics API of Isaac Sim

        Args: 
            step_size: time difference between physics steps (Default frquency is 60 Hz)
        """
        # Check object Pose
        object_pose = self.object_prim.get_world_pose()
        if ((object_pose[0][2]<-0.5 and self.current_time!=self.initial_time) or self.current_time>=self.test_time): #If object fell
            self.test_finish()
            return
        self.current_time += step_size
        #print(self.current_time)

        actions = self.controller.forward('any', self.current_time)
        self.robot.apply_action(actions)
        return
    
    def test_finish(self): 
        ## Code for Logging Results
        #- Time and actions for: Slip, Fall and Reset
        #- Controller Type
        ###

        #Get new job
        self.job = self.manager.request_job()

        #Reset controller
        close_dir = self.manager.close_dir[self.job['gripper']]
        close_dir = np.asarray(close_dir)
        robot_pos = np.array(self.job["grasps"]['dofs'])
        init_dofs = self.manager.translate_dofs(self.job["gripper"], robot_pos)
        actions = self.controller.reset(close_dir, init_dofs)

        #Reset Robot
        self.reset_robot()
        self.robot.apply_action(actions)

        # Reset Object
        T = np.matmul(self.worldT, self.T_EF)
        pos =  self.job['grasps']['pose'][:3]
        q = self.job['grasps']['pose'][3:]
        reorder = [3, 0, 1, 2]
        q = [q[i] for i in reorder]
        tmp = tf_matrix_from_pose(pos, q)
        tmp = InverseT(tmp)
        T_final = np.matmul(T, tmp)        
        self.object_init_pose= pose_from_tf_matrix(T_final)
        self.object_prim.set_world_pose(position = self.object_init_pose[0], orientation = self.object_init_pose[1])

        #Reset Control var
        self.current_time = self.initial_time
        
        return


if __name__ == "__main__":
    # Scratch pad for workstation.py
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
