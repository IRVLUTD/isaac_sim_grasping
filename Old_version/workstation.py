#External Libraries
import numpy as np
import pandas as pd

#Custom Classes and utils
from manager import Manager
from utils import InverseT, re, te, R_t_from_tf
from controllers import ForceController

#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim, RigidPrimView    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative
from omni.isaac.core.utils.prims import create_prim, delete_prim

from controllers import ForceController, PositionController

class Workstation():

    def __init__(self, ID, manager, path, world, test_time):
        #External Objects Information
        self.world = world
        self.manager = manager
        self.reported_slip = 0

        #Important info for Workstations
        self.job, self.job_ID = manager.request_job()
        if(self.job_ID==-1): return
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

        # Initialize gripper and object
        self.robot = self._import_gripper()
        self.object_parent_prim = self._import_object()
        world.add_physics_callback("physics_step_ws_"+ str(ID), callback_fn=self.physics_step)
        self.grasp_set_up = False

        controller = manager.controllers[self.job["gripper"]]
        #controller = ForceController
        robot_pos = np.array(self.job["grasps"]['dofs'])
        self.controller = controller(path,ID,close_dir,robot_pos, test_time, self.translator)

        
    
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
        robot.set_enabled_self_collisions(False)
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
        self.init_T = np.matmul(T, tmp)        
        self.object_init_pose= pose_from_tf_matrix(self.init_T)

        #Adding Object usd
        usd_path = self.manager.object_dict[self.job["object_id"]]
        add_reference_to_stage(usd_path=usd_path, prim_path=self.path+"/object_"+str(self.ID))

        payload = self.world.scene.add(GeometryPrim(prim_path = self.path+"/object_"+str(self.ID), name="object_"+str(self.ID)))
        self.object_prim = RigidPrim(prim_path= self.path +"/object_"+str(self.ID) + "/base_link",
                                                 position = self.object_init_pose[0], orientation = self.object_init_pose[1])
        
        self.contact_th = self.manager.contact_th[self.job['gripper']]
        c_names = self.manager.contact_names[self.job['gripper']]
        contact_names = []
        for i in c_names:
            contact_names.append(self.path+"/gripper_"+str(self.ID)+"/"+i)
        self.rigid_view = RigidPrimView(prim_paths_expr= self.path +"/object_"+str(self.ID) + "/base_link", track_contact_forces= True,prepare_contact_sensors = True,
                                        contact_filter_prim_paths_expr  = contact_names, disable_stablization = False)
        #self.object_prim.disable_rigid_body_physics() 
        # Counter intuitive this disables the gravity
        self.rigid_view.enable_gravities()
        
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
        self.rigid_view.initialize()
        robot_pos = np.array(self.job["grasps"]['dofs'])
        robot_pos = self.manager.translate_dofs(self.job["gripper"], robot_pos, self.robot.dof_names)
        self.current_time = self.initial_time
        self.robot.set_joint_positions(robot_pos)
        return
        
    def physics_step(self,step_size):
        """ Physics step performed by physics API of Isaac Sim

        Args: 
            step_size: time difference between physics steps (Default frquency is 60 Hz)
        """
        if self.job_ID < 0:
            return

        if self.grasp_set_up == True:
            self.rigid_view.apply_forces(np.asarray([0,0, -9.81 ]))
              
        # Check object Pose
        object_pose = self.object_prim.get_world_pose()
        object_T = tf_matrix_from_pose(object_pose[0], object_pose[1])
        # Calculate height and orientation changes
        object_R, object_t = R_t_from_tf(object_T)
        init_R, init_t = R_t_from_tf(self.init_T)
        t_error = abs(te(init_t,object_t))      
        r_error = abs(re(init_R, object_R))
        #print("T_error: " +str(t_error))
        #print("R_error: " +str(r_error))

        #Report Slip
        if ((self.grasp_set_up == True) and ((self.reported_slip == 0) and (t_error > 0.02 or r_error > 5))):
            self.manager.report_slip(self.job_ID, self.current_time)
            self.reported_slip = 1
            #print("Slip reported WS: " + str(self.ID))

        if ((t_error>0.3 and self.current_time!=self.initial_time) or self.current_time>=self.test_time): #If object fell
            self.test_finish()
            return
        #print(self.job["grasps"]['dofs'])
        self.current_time += step_size
        
        #Apply action
        actions = self.controller.forward('any', self.current_time, self.robot.get_joint_positions())
        self.robot.apply_action(actions)

        if(self.grasp_set_up == False): # CHECK if the grasp has been set up
            #print(self.robot.dof_names)
            if(self.current_time >= 1):
                #print("GRASP FAILURE")
                self.test_finish()
                return
            cfs = self.rigid_view.get_contact_force_matrix()
            #print(cfs.shape)
            cfs = np.reshape(cfs, (cfs.shape[1], cfs.shape[2]))
            tcfs = np.sum(cfs,axis = 1)
            num_c = np.count_nonzero(tcfs)
            if  (num_c >= self.contact_th):
                
                self.grasp_set_up = True       
                self.current_time = self.initial_time
                #print(num_c)
                
        return
    
    def test_finish(self): 
        """
        Reports Test and gets new one
        """
        ## Code for Logging Results
        #- Time and actions for: Slip, Fall and Reset

        #Gripper closed with no collisions
        if(self.grasp_set_up == False):
            self.current_time = -1 # IF grasp failed the fall time will be reported as negative

        # Report Fall
        #print("Finished Test: ", self.ID)
        self.manager.report_fall(self.job_ID,self.current_time,self.controller.type, self.test_time)

        self.job, self.job_ID = self.manager.request_job()
        if (self.job_ID < 0): 
            self.robot.set_visibility(False)
            self.object_parent_prim.set_visibility(False)
            return
        
        #Reset controller
        close_dir = self.manager.close_dir[self.job['gripper']]
        close_dir = np.asarray(close_dir)
        robot_pos = np.array(self.job["grasps"]['dofs'])
        actions = self.controller.reset(close_dir, robot_pos)

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
        self.init_T = np.matmul(T, tmp)       
        self.rigid_view.set_velocities([0,0,0,0,0,0]) 
        self.object_init_pose= pose_from_tf_matrix(self.init_T)
        self.object_prim.set_world_pose(position = self.object_init_pose[0], orientation = self.object_init_pose[1])

        #Reset Control var
        self.current_time = self.initial_time
        
        #Slip var
        self.reported_slip = 0

        #Reset grasp set up var 
        self.grasp_set_up = False
        return
    
    def translator(self, init_dofs):
        dofs = self.manager.translate_dofs(self.job["gripper"], init_dofs, self.robot.dof_names)
        return dofs


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
