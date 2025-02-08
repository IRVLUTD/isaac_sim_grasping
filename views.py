#External Libraries
import numpy as np
import time


#Custom Classes and utils
from utils_local import te_batch,re_batch
from sim_utils import check_mesh_overlap
from controllers import controller_dict
from tests import test_dict

#Omni Libraries
from omni.isaac.core.prims.rigid_prim import RigidPrimView    
from omni.isaac.core.articulations import  ArticulationView
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrices_from_poses

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
    def __init__(self, work_path, contact_names_expr, num_w,  manager,
                  world, test_time, controller, test_type, dof_given, view_mode = False):
        #Create Views
        self.objects = world.scene.add(
            RigidPrimView(
                prim_paths_expr= work_path[:-1]+"*"+"/object/base_link", 
                track_contact_forces = True, 
                prepare_contact_sensors = True, 
                contact_filter_prim_paths_expr  = contact_names_expr))
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
        self.manager = manager
        self.dof_given = dof_given
        self.view_mode = view_mode

        self.current_times = np.zeros((num_w,1))
        self.set_up_timers = np.zeros_like(self.current_times)
        self.grasp_set_up = np.zeros((num_w,1))
        self.initial_check = np.zeros((self.num_w,))
        self.current_poses = []
        self.current_job_IDs=[]
        self.dofs = []
        self.t = time.time()
        self.init_step=0

        # Controller and test init
        self.controller_class = controller_dict[controller]
        self.test_class = test_dict[test_type]

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
        self.objects.set_world_poses(self.init_positions, self.init_rotations)

        # Get max efforts and dofs
        self.controller = self.controller_class(self.manager.gripper, self.grippers, self.test_time)
        self.controller_type = self.controller.label # Save info of controller used (for results)
        
        self.new_dofs = np.zeros_like(self.dofs)

        self.test = self.test_class(self.objects, self.manager.gripper_dict, self.test_time)
        self.test_type = self.test.label
        #self.outside_start = time.time()
        return
    
    def physics_step(self,step_size):
        """ Function runs before every physics frame

        step_size: time since last physics step. Depends on physics_dt
        """
        #outside_end = time.time()
        #start_time = time.time()
        finish_ind = np.array([],dtype=int)
        tmp_active = np.squeeze(self.current_job_IDs>=0)

        # Check for overlap
        if (self.dof_given == False and self.view_mode==False): # Don't filter overlap for grasps with dof given (Assume they are correct)
            initial_ind = np.argwhere(np.multiply(tmp_active,
                np.squeeze(self.initial_check==0))==1)[:,0] #ws indices
            if(len(initial_ind)>0):
                # Get the spheres which are overlapped
                tmp = np.count_nonzero(np.sum(self.objects.get_contact_force_matrix(initial_ind),axis =2),axis=1)
                col_ind = initial_ind[tmp>=1]
                self.current_times[col_ind] = -1
                self.initial_check[initial_ind]= 1
                finish_ind = np.concatenate([finish_ind, col_ind])
        

        #Check  for active workstations
        active_ind = np.setdiff1d(np.argwhere(self.current_job_IDs>=0),finish_ind) #ws indices
        if(len(active_ind)>0):
            # Calculate workstations which have failed the test
            failed_ind = self.test.failure_condition(self.init_positions, 
                                        self.init_rotations, active_ind)
            finish_ind = np.concatenate([finish_ind, failed_ind])    

        # Rigid Body Probing, mark grasps as ready
        rb_ind = np.setdiff1d(np.argwhere(
            np.multiply(
                np.squeeze(self.grasp_set_up==0 ),tmp_active)==1)[:,0], finish_ind)
        if (len(rb_ind)>0):
            self.set_up_timers[rb_ind] +=step_size

            # Test setup_condition
            setup_ind = self.test.setup_condition(
                self.init_positions, self.init_rotations,
                rb_ind
            )
            
            nonsetup_ind = np.setdiff1d(rb_ind, setup_ind)

            #Update grasp_setup
            self.current_times[nonsetup_ind]=0
            self.grasp_set_up[setup_ind]=1
            self.new_dofs[setup_ind] = self.grippers.get_joint_positions(indices= setup_ind)

        # Apply gripper actions
        if not self.view_mode:
            self.controller.forward(self.current_times)
            self.test.test_step(self.current_times)
        else:
            self.objects.set_velocities([0,0,0,0,0,0])

        # Update time
        update_ind = np.setdiff1d(active_ind, np.argwhere(self.current_times== -1))
        self.current_times[update_ind] += step_size

        # Failed grasps; gripper never touched object
        failed_ind = np.argwhere(np.squeeze(self.set_up_timers>self.test_time))[:,0]
        if(len(failed_ind)>0): 
            self.current_times[failed_ind] = -1
            finish_ind = np.concatenate([finish_ind, failed_ind])

        # Reset workstations with failed grasps
        finish_ind = np.unique(finish_ind)
        if (len(finish_ind)>0):
            self.test_finish(finish_ind)

        # End of testing time - Reset workstations with successful Grasps
        time_ind = np.argwhere(np.squeeze(self.current_times>self.test_time))[:,0]
        if (len(time_ind)>0):
            self.test_finish(time_ind, status=1)

        
        #end_time = time.time()
        #print(f"Total time of physics_step: {end_time - start_time:.6f} seconds")
        #print(f"Total time outside: {outside_end - self.outside_start:.6f} seconds")
        #self.outside_start = time.time()
        return
    
    def test_finish(self, finish_ind, status = 0):
        """ Function to reset workstations after tests are finished
        
        Args:
            finished_ind: IDs of Workstations that completed the test.
        """
        finish_ind=np.atleast_1d(np.squeeze(finish_ind))

        #Report Fall
        self.manager.report_result(self.current_job_IDs[finish_ind], 
                                 self.current_times[finish_ind],
                                 self.new_dofs[finish_ind],
                                 status)

        # Get new jobs
        self.dofs[finish_ind], self.current_poses[finish_ind], self.current_job_IDs[finish_ind] =self.get_jobs(len(finish_ind))
        self.current_times[finish_ind] = 0
        self.set_up_timers[finish_ind] = 0
        self.initial_check[finish_ind] = 0
        self.grasp_set_up[finish_ind] = 0
        self.new_dofs[finish_ind] = np.zeros_like(self.dofs[finish_ind])

        # Reset Workstations
        self.grippers.set_joint_positions(self.dofs[finish_ind], finish_ind)
        object_Ts = tf_matrices_from_poses(self.current_poses[finish_ind,:3], self.current_poses[finish_ind,3:])
        object_Ts = np.linalg.inv(object_Ts)
        object_Ts = np.matmul(self.ws_Ts[finish_ind],object_Ts)
        for i in range(object_Ts.shape[0]):
            self.init_positions[finish_ind[i]], self.init_rotations[finish_ind[i]] = pose_from_tf_matrix(object_Ts[i].astype(float))
        #self.objects.set_velocities([0,0,0,0,0,0],finish_ind) 
        self.objects.set_world_poses(self.init_positions[finish_ind], self.init_rotations[finish_ind],finish_ind)
        return
