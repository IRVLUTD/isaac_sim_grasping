from utils_local import te_batch, re_batch
import numpy as np

class SimpleGravity:
    def __init__(self, objects, gripper_dict, total_test_time):
        self.label = "gravity"
        self.objects = objects
        self.contact_th = gripper_dict['contact_th'] 
        self.test_time = total_test_time
        return
    
    def failure_condition(self, init_pos, init_rot, indices):
        ''' Function to test if the test has failed

        Arguments:
        - init_pos: Initial positions of objects
        - init_rot: Initial rotation of objects
        - indices: array with indices of objects to check
        '''
        current_positions, current_rotations = self.objects.get_world_poses(indices)
        t_error = abs(te_batch(init_pos[indices], current_positions))

        finish_ind = indices[t_error>0.3]
        return finish_ind
    
    def setup_condition(self, init_pos, init_rot, indices):
        ''' Function to test if the objects advance from the setup phase

        Arguments:
        - init_pos: Initial positions of objects
        - init_rot: Initial rotation of objects
        - indices: array with indices of objects to check
        '''
        # Fix velocities and check for set up criteria
        self.objects.set_velocities([0,0,0,0,0,0],indices) 
        self.objects.set_world_poses(init_pos[indices], init_rot[indices],indices)

        # Setup Condition
        tmp = np.count_nonzero(np.sum(self.objects.get_contact_force_matrix(indices),axis =2),axis=1)
        
        # Update grasp_setup for the ones that fulfill criteria
        setup_ind = indices[tmp>=self.contact_th]
        
        return setup_ind
    
    def test_step(self, current_times):
        gravity = np.tile(np.array([0,0,-9.81]), (len(current_times),1))*self.objects.get_masses()[:, np.newaxis]
        self.objects.apply_forces(gravity)
        return
    
class SixAxisForce:
    def __init__(self, objects, gripper_dict, total_test_time):
        self.objects = objects
        self.contact_th = gripper_dict['contact_th'] 
        self.test_time = total_test_time
        return
    
    def failure_condition(self, init_pos, init_rot, indices):
        ''' Function to test if the test has failed

        Arguments:
        - init_pos: Initial positions of objects
        - init_rot: Initial rotation of objects
        - indices: array with indices of objects to check
        '''
        current_positions, current_rotations = self.objects.get_world_poses(indices)
        t_error = abs(te_batch(init_pos[indices], current_positions))

        finish_ind = indices[t_error>2]
        return finish_ind
    
    def setup_condition(self, init_pos, init_rot, indices):
        ''' Function to test if the objects advance from the setup phase

        Arguments:
        - init_pos: Initial positions of objects
        - init_rot: Initial rotation of objects
        - indices: array with indices of objects to check
        '''
        # Fix velocities and check for set up criteria   
        return indices
    
    def test_step(self, current_times):
        acc = np.zeros((len(current_times),3))
        masses = self.objects.get_masses()[:, np.newaxis]

        # Axis forces
        acc[:] = [0, 0, 0.5]
        tmp_ind = np.argwhere(current_times<5*self.test_time/6)
        acc[tmp_ind] = [0, 0, -0.5]
        tmp_ind = np.argwhere(current_times<4*self.test_time/6)
        acc[tmp_ind] = [0, 0.5, 0]
        tmp_ind = np.argwhere(current_times<3*self.test_time/6)
        acc[tmp_ind] = [0, -0.5, 0]
        tmp_ind = np.argwhere(current_times<2*self.test_time/6)
        acc[tmp_ind] = [0.5, 0, 0]
        tmp_ind = np.argwhere(current_times<self.test_time/6)
        acc[tmp_ind] = [-0.5, 0, 0]

        forces = acc*masses
        self.objects.apply_forces(forces)
        return


""" LIST OF Tests to Perform: 
They are the references used in command line to determine the controller to use
"""
test_dict = {
    'default': SimpleGravity,
    'gravity' : SimpleGravity,
    'six_axis': SixAxisForce,
}
