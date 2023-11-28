#External Libraries
import numpy as np
import pandas as pd

#Custom Classes and utils
from managerv2 import Manager
from utils import InverseT, re, te_batch, R_t_from_tf,re_batch
from controllersv2 import ForceController

#Omni Libraries
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim, RigidPrimView    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation, ArticulationView
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrices_from_poses, get_world_pose_from_relative
from omni.isaac.core.utils.prims import create_prim, delete_prim


class View():

    def __init__(self, work_path, contact_names_expr, num_w,  manager, world, test_time, mass):
        self.grippers = ArticulationView(prim_paths_expr = work_path[:-1]+"*"+"/gripper",reset_xform_properties = False)
        self.objects = RigidPrimView(prim_paths_expr= work_path[:-1]+"*"+"/object/base_link", track_contact_forces = True, prepare_contact_sensors = True, 
                                        contact_filter_prim_paths_expr  = contact_names_expr, reset_xform_properties = False)
        #print("GRIPPER PATHS",self.grippers.prim_paths)
        #print(work_path[:-1]+"*"+"/gripper")
        #print("Object PATHS",self.objects.prim_paths)
        #print(work_path[:-1]+"*"+"/object/base_link")
        # Initialize
        #print(self.objects.count)
        #print(self.grippers.count)
        self.num_w = num_w
        self.test_time = test_time
        self.work_path = work_path
        self.world = world
        ws_poses = self.grippers.get_world_poses()
        self.ws_Ts = tf_matrices_from_poses(ws_poses[0],ws_poses[1])
        #print(self.ws_Ts)
        self.current_times = np.zeros((num_w,1))
        self.grasp_set_up = np.zeros((num_w,1))
        self.last_step_gsetup = np.zeros_like(self.grasp_set_up)
        self.reported_slips = np.zeros((num_w,1))
        self.gravity = np.asarray([0,0,-9.81]) * mass 
        self.gravities = np.zeros((num_w,3))
        
        self.manager = manager
        self.current_poses = []
        self.current_job_IDs=[]
        self.dofs = []
        world.add_physics_callback("physics_steps", callback_fn=self.physics_step)
        
    def get_jobs(self,n):
        dofs, poses, job_IDs = self.manager.request_jobs(n)
        return dofs, poses, job_IDs
    
    def post_reset(self):
        # Set dofs
        #print(self.dofs)
       # print(self.grippers.get_joint_positions())
        self.grippers.set_joint_positions(self.dofs)

        # Set object positions
        object_Ts = tf_matrices_from_poses(self.current_poses[:,:3], self.current_poses[:,3:])
        object_Ts = np.linalg.inv(object_Ts)
        #print(object_Ts.shape)
        object_Ts = np.matmul(self.ws_Ts,object_Ts)
        self.init_positions=np.zeros((object_Ts.shape[0],3))
        self.init_rotations =np.zeros((object_Ts.shape[0],4))
        for i in range(object_Ts.shape[0]):
            self.init_positions[i], self.init_rotations[i] = pose_from_tf_matrix(object_Ts[i].astype(float))
        
        self.objects.set_velocities([0,0,0,0,0,0]) 
        self.objects.set_world_poses(self.init_positions, self.init_rotations)
       # print("RUNNSSS")
        # Get max efforts and dofs
        dc = self.world.dc_interface
        articulation = dc.get_articulation(self.work_path+"/gripper")
        self.dof_props = dc.get_articulation_dof_properties(articulation)
        #print(dof_props)
        self.close_positions = np.zeros_like(self.dofs)
        max_efforts = np.zeros_like(self.dofs)
        #print(max_efforts.shape)
        close_mask = self.manager.close_mask

        #print("Gravity disabled")
        #print(self.dofs,self.close_positions)

        for i in range(len(self.dof_props)):
            if (close_mask[i]==0):
                max_efforts[:,i] = 0
                self.close_positions[:,i]=(self.dofs[:,i])
            elif (close_mask[i]==1):
                max_efforts[:,i]= self.dof_props[i][6]
                self.close_positions[:,i]=(self.dof_props[i][3])
            elif (close_mask[i]==-1):
                max_efforts[:,i]= -self.dof_props[i][6]
                self.close_positions[:,i]=(self.dof_props[i][2])
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")

        #print(max_efforts)
        #print(self.close_positions)
        self.controller= self.manager.controller(close_mask, self.test_time, max_efforts)
        self.test_type = self.controller.type
        return
    
    def physics_step(self,step_size):
        #self.current_job_IDs[0]=-1
        #Check active workstations
        active_ind = np.argwhere(self.current_job_IDs>=0) #ws indices
        if(len(active_ind)>0):
            #print("active", active_ind)
            current_positions, current_rotations = self.objects.get_world_poses(active_ind)
            t_error = abs(te_batch(self.init_positions[active_ind], current_positions))
            #print(t_error)

            finish_ind = active_ind[t_error>0.3]
            if(len(finish_ind)>0):
                self.test_finish(finish_ind)

            # Implement slip
            te_slip = np.squeeze(t_error>0.02)
            tmp = np.squeeze(self.reported_slips[active_ind] == 0 )
            s_ind = np.squeeze(active_ind[np.multiply(te_slip,tmp)])
            #print("s_ind", s_ind)
            s_ind=np.atleast_1d(s_ind)
            if(len(s_ind)>0):
                
                #print(self.current_job_IDs[s_ind])
                #print(self.current_times[s_ind])
                self.manager.report_slip(self.current_job_IDs[s_ind],self.current_times[s_ind])
                self.reported_slips[s_ind] = 1
            


            

            
        tmp_active = np.squeeze(self.current_job_IDs>=0)

        # Apply forces to seted up grasp
        tmp = np.squeeze((self.grasp_set_up==1))
        
        tmp = np.multiply(tmp,tmp_active)
        g_ind = np.argwhere(tmp ==1)[:,0]
        #print("g", g_ind)
        if (len(g_ind)>0):
            
            self.gravities[g_ind] = self.gravity
        self.objects.apply_forces(self.gravities)

        # Rigid Body Probing
        tmp = np.squeeze(self.grasp_set_up==0 )
        tmp = np.multiply(tmp,tmp_active)
        rb_ind = np.argwhere(tmp==1)[:,0]
        #print("rb_ind", rb_ind)
        if (len(rb_ind)>0):
            self.objects.set_velocities([0,0,0,0,0,0],rb_ind) 
            self.objects.set_world_poses(self.init_positions[rb_ind], self.init_rotations[rb_ind],rb_ind)
            tmp = np.sum(self.objects.get_contact_force_matrix(rb_ind),axis =2)
            #print(tmp)
            tmp = np.count_nonzero(tmp,axis=1)
            #Update grasp_setup
            self.current_times[rb_ind[tmp>=self.manager.contact_th]]=0
            self.grasp_set_up[rb_ind[tmp>=self.manager.contact_th]]=1
        

        #Apply actions
        current_dofs = self.grippers.get_joint_positions()
        actions = self.controller.forward('any', self.current_times, current_dofs, self.close_positions)
        self.grippers.apply_action(actions)
        
        #Update time
        self.current_times += step_size

        #End of time
        tmp = np.squeeze((self.current_times>self.test_time))
        tmp = np.multiply(tmp,tmp_active)
        time_ind = np.argwhere(tmp)[:,0]
        #print("time", time_ind)
        if (len(time_ind)>0):
            
            self.test_finish(time_ind)

        #Failed grasps 
        tmp = np.squeeze(self.current_times>=1)
        tmp = np.multiply(tmp, np.squeeze(self.grasp_set_up==0))
        tmp = np.multiply(tmp,tmp_active)
        failed_ind = np.argwhere((tmp==1))[:,0]
        #print("failed", failed_ind)  
        if(len(failed_ind)>0):
                     
            self.current_times[failed_ind] = -1
            self.test_finish(failed_ind)
 
        return
    
    def test_finish(self, finish_ind):
        finish_ind=np.atleast_1d(np.squeeze(finish_ind))

        #print("Finished", finish_ind)
        #print(self.current_job_IDs[finish_ind])
        #print(self.manager.completed)
        #Report Fall
        self.manager.report_fall(self.current_job_IDs[finish_ind], self.current_times[finish_ind],self.test_type,self.test_time)
        
        #Report
        self.dofs[finish_ind], self.current_poses[finish_ind], self.current_job_IDs[finish_ind] =self.get_jobs(len(finish_ind))
        #print("Runs")
        self.current_times[finish_ind] = 0
        self.grasp_set_up[finish_ind] = 0
        self.reported_slips[finish_ind] = 0
        tmp = np.zeros((len(finish_ind),3))
        self.gravities[finish_ind] = tmp

        #print("Runs")

        self.grippers.set_joint_positions(self.dofs[finish_ind], finish_ind)
        
        object_Ts = tf_matrices_from_poses(self.current_poses[finish_ind,:3], self.current_poses[finish_ind,3:])
        object_Ts = np.linalg.inv(object_Ts)
        object_Ts = np.matmul(self.ws_Ts[finish_ind],object_Ts)

        for i in range(object_Ts.shape[0]):
            self.init_positions[finish_ind[i]], self.init_rotations[finish_ind[i]] = pose_from_tf_matrix(object_Ts[i].astype(float))

        self.objects.set_velocities([0,0,0,0,0,0],finish_ind) 
        self.objects.set_world_poses(self.init_positions[finish_ind], self.init_rotations[finish_ind],finish_ind)


        for i in range(len(self.dof_props)):
            if (self.manager.close_mask[i]==0):
                self.close_positions[:,i]=(self.dofs[:,i])

        return
