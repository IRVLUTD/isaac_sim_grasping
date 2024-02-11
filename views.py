#External Libraries
import numpy as np
import pandas as pd
import time


#Custom Classes and utils
from utils import te_batch,re_batch

#Omni Libraries
from omni.isaac.core.utils.numpy.rotations import quats_to_rot_matrices

from omni.isaac.core.prims.rigid_prim import RigidPrimView    
from omni.isaac.core.articulations import  ArticulationView
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrices_from_poses
from omni.isaac.core.prims import XFormPrimView

class View():
    """ISAAC SIM VIEWS Class 
    Facilitates the probing and the programming of the simulation. In this class you will find all the code of the simulation behavior.

    Args:
        work_path: prim_path of workstation prim
        contact_names_expr: Names of gripper meshes to filter collisions from in the Isaac Sim format
        num_w: Total number of workstations
        manager: Manager class containing grasp information
        world: Isaac Sim World object
        test_time: Total test time of each test
        mass: Mass of the object to test
    """
    def __init__(self, work_path, contact_names_expr, num_w,  manager, world, test_time, mass):
        #Create Views
        self.objects = world.scene.add(
            RigidPrimView(
                prim_paths_expr= work_path[:-1]+"*"+"/object/base_link", 
                track_contact_forces = True, 
                prepare_contact_sensors = True, 
                contact_filter_prim_paths_expr  = contact_names_expr, 
                reset_xform_properties = False,
                disable_stablization = False))
        self.grippers = world.scene.add(
            ArticulationView(
                prim_paths_expr = work_path[:-1]+"*"+"/gripper",
                reset_xform_properties = False))

        # Initialize Variables
        self.num_w = num_w
        self.test_time = test_time
        self.work_path = work_path
        self.world = world
        ws_poses = self.grippers.get_world_poses()
        self.ws_Ts = tf_matrices_from_poses(ws_poses[0],ws_poses[1])
        self.current_times = np.zeros((num_w,1))
        self.grasp_set_up = np.zeros((num_w,1))
        self.last_step_gsetup = np.zeros_like(self.grasp_set_up)
        self.reported_slips = np.zeros((num_w,1))
        self.gravity = np.asarray([0,0,-9.81]) * mass 
        self.gravities = np.zeros((num_w,3))

        self.objects.disable_gravities()

        self.manager = manager
        self.current_poses = []
        self.current_job_IDs=[]
        self.dofs = []
        self.t = time.time()
        #Add physics Step
        world.add_physics_callback("physics_steps", callback_fn=self.physics_step)
        
    def get_jobs(self,n):
        """Request jobs from manager class
        
        Args:
            n: number of jobs to request
        """
        dofs, poses, job_IDs = self.manager.request_jobs(n)
        return dofs, poses, job_IDs
    
    def post_reset(self):
        """ Code that needs to run after the reset of the world (dependent on the existence of physics context object)"""
        # Set grippers dofs
        self.grippers.set_joint_positions(self.dofs)

        # Calculate objects positions
        object_Ts = tf_matrices_from_poses(self.current_poses[:,:3], self.current_poses[:,3:])
        object_Ts = np.linalg.inv(object_Ts)
        object_Ts = np.matmul(self.ws_Ts,object_Ts)
        self.init_positions=np.zeros((object_Ts.shape[0],3))
        self.init_rotations =np.zeros((object_Ts.shape[0],4))
        for i in range(object_Ts.shape[0]):
            self.init_positions[i], self.init_rotations[i] = pose_from_tf_matrix(object_Ts[i].astype(float))
        
        # Set object position and velocities
        self.objects.set_velocities([0,0,0,0,0,0]) 
        #self.objects_parents.set_world_poses(self.init_positions, self.init_rotations)
        self.objects.set_world_poses(self.init_positions, self.init_rotations)

        # Get max efforts and dofs
        dc = self.world.dc_interface
        articulation = dc.get_articulation(self.work_path+"/gripper")
        self.dof_props = dc.get_articulation_dof_properties(articulation)
        self.close_positions = np.zeros_like(self.dofs)
        max_efforts = np.zeros_like(self.dofs)
        close_mask = self.manager.close_mask
        for i in range(len(self.dof_props)):
            if (close_mask[i]==0):
                max_efforts[:,i] = self.dof_props[i][6]
                self.close_positions[:,i]=(self.dofs[:,i])
            elif (close_mask[i]>0):
                max_efforts[:,i]= self.dof_props[i][6]
                self.close_positions[:,i]=(self.dof_props[i][3])
            elif (close_mask[i]<0):
                max_efforts[:,i]= -self.dof_props[i][6]
                self.close_positions[:,i]=(self.dof_props[i][2]) 
            else: 
                raise ValueError("clos_dir arrays for grippers can only have 1,-1 and 0 values indicating closing direction")

        # Initialize controller
        self.controller= self.manager.controller(close_mask, self.test_time, max_efforts, self.grippers)
        self.test_type = self.controller.type
        return
    
    def physics_step(self,step_size):
        """ Function runs before every physics frame

        step_size: time since last physics step. Depends on physics_dt
        """
        #Check  for active workstations
        #print("Outside step", time.time()-self.t)
        #self.t= time.time()
        active_ind = np.argwhere(self.current_job_IDs>=0) #ws indices
        if(len(active_ind)>0):
            # Calculate falls
            current_positions, current_rotations = self.objects.get_world_poses(active_ind)
            t_error = abs(te_batch(self.init_positions[active_ind], current_positions))
            
            # Object that fell
            finish_ind = active_ind[t_error>0.6]
            if(len(finish_ind)>0):
                self.test_finish(finish_ind)
            
            
            # Calculate slips
            te_slip = np.squeeze(t_error>0.02)
            R_current = quats_to_rot_matrices(np.squeeze(current_rotations))
            R_init = quats_to_rot_matrices(np.squeeze(self.init_rotations[active_ind]))
            re_slip = re_batch(R_current, R_init) > 5
            slip = np.logical_or(te_slip, re_slip)
            tmp = np.squeeze(self.reported_slips[active_ind] == 0 )
            s_ind = np.squeeze(active_ind[np.multiply(slip,tmp)])

            #Objects that slipped
            s_ind=np.atleast_1d(s_ind)
            if(len(s_ind)>0):
                self.manager.report_slip(self.current_job_IDs[s_ind],self.current_times[s_ind])
                self.reported_slips[s_ind] = 1
            
        # Apply gravity to ready grasps
        tmp_active = np.squeeze(self.current_job_IDs>=0)
        g_ind = np.argwhere(np.multiply(np.squeeze((self.grasp_set_up==1)),tmp_active) ==1)[:,0] # optimizable
        if (len(g_ind)>0):
            self.objects.enable_gravities(g_ind)
            self.gravities[g_ind] = self.gravity
            #print("SETTED UP ", g_ind)
        #self.objects.apply_forces(self.gravities)

        # Rigid Body Probing, mark grasps as ready
        rb_ind = np.argwhere(np.multiply(np.squeeze(self.grasp_set_up==0 ),tmp_active)==1)[:,0]
        if (len(rb_ind)>0):
            self.objects.set_velocities([0,0,0,0,0,0],rb_ind) #
            #self.objects_parents.set_world_poses(self.init_positions[rb_ind], self.init_rotations[rb_ind],rb_ind)
            self.objects.set_world_poses(self.init_positions[rb_ind], self.init_rotations[rb_ind],rb_ind)
            tmp = np.count_nonzero(np.sum(self.objects.get_contact_force_matrix(rb_ind),axis =2),axis=1)
            
            
            #Update grasp_setup
            self.current_times[rb_ind[tmp>=self.manager.contact_th]]=0
            self.grasp_set_up[rb_ind[tmp>=self.manager.contact_th]]=1

        # Apply gripper actions
        current_dofs = self.grippers.get_joint_positions()
        set_up_timers = np.zeros_like(self.current_times)
        set_up_timers[g_ind]= self.current_times[g_ind]
        actions = self.controller.forward('any', set_up_timers, current_dofs, self.close_positions)
        self.grippers.apply_action(actions)
        
        # Update time
        self.current_times += step_size

        # End of testing time
        time_ind = np.argwhere(np.multiply(np.squeeze((self.current_times>self.test_time)),tmp_active))[:,0]
        if (len(time_ind)>0):
            self.test_finish(time_ind)

        # Failed grasps; gripper never touched object
        failed_ind = np.argwhere((np.multiply(np.multiply(np.squeeze(self.current_times>=0.7), np.squeeze(self.grasp_set_up==0)),tmp_active)==1))[:,0]
        if(len(failed_ind)>0): 
            self.current_times[failed_ind] = -1
            self.test_finish(failed_ind)

        #print("physics_step", time.time()-self.t)
        #self.t= time.time()
        return
    
    def test_finish(self, finish_ind):
        """ Function to reset workstations after tests are finished
        
        Args:
            finished_ind: IDs of Workstations that completed the test.
        """
        finish_ind=np.atleast_1d(np.squeeze(finish_ind))

        #Report Fall
        self.manager.report_fall(self.current_job_IDs[finish_ind], self.current_times[finish_ind],self.test_type,self.test_time)
        
        # Get new jobs
        self.dofs[finish_ind], self.current_poses[finish_ind], self.current_job_IDs[finish_ind] =self.get_jobs(len(finish_ind))
        self.current_times[finish_ind] = 0
        self.grasp_set_up[finish_ind] = 0
        self.reported_slips[finish_ind] = 0
        self.gravities[finish_ind] = np.zeros((len(finish_ind),3))
        self.objects.disable_gravities(finish_ind)

        # Reset Workstations
        self.grippers.set_joint_positions(self.dofs[finish_ind], finish_ind)
        object_Ts = tf_matrices_from_poses(self.current_poses[finish_ind,:3], self.current_poses[finish_ind,3:])
        object_Ts = np.linalg.inv(object_Ts)
        object_Ts = np.matmul(self.ws_Ts[finish_ind],object_Ts)
        for i in range(object_Ts.shape[0]):
            self.init_positions[finish_ind[i]], self.init_rotations[finish_ind[i]] = pose_from_tf_matrix(object_Ts[i].astype(float))
        self.objects.set_velocities([0,0,0,0,0,0],finish_ind) 
        #self.objects_parents.set_world_poses(self.init_positions[finish_ind], self.init_rotations[finish_ind],finish_ind)
        self.objects.set_world_poses(self.init_positions[finish_ind], self.init_rotations[finish_ind],finish_ind)

        # Update close positions to new dofs (already placed on gripper initialization dofs)
        for i in range(len(self.dof_props)):
            if (self.manager.close_mask[i]==0):
                self.close_positions[:,i]=(self.dofs[:,i])
        return
