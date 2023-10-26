import numpy as np
import pandas as pd
import os

class Manager:
    """ Grasp Data Manager:
    Makes sure everythin needed by Isaac Sim is present within the local directory and ensures that every grasp is tested.

    Args:
        graps_path: .json file containing the grasp information
        grippers_path: folder path for all the gripper .usd folders
        objects_path: folder path for all the object .urdf files
    """
    def __init__(self, grasps_path, grippers_path, objects_path,
                 world=None):
        # Loading files and urdfs
        self.world = world
        self.grasps = pd.read_json(grasps_path)
        self.gripper_names = [] #List of gripper names
        self.object_names =[] #List of object names 
        for i, row in self.grasps.iterrows():
            if row['gripper'] not in self.gripper_names:
                self.gripper_names.append(row['gripper'])
            if row['object_id'] not in self.gripper_names:
                self.object_names.append(row['object_id'])
        
        # Initialize dictionaries (paths to objects and grippers)
        self.gripper_dict = {}
        self._check_gripper_usd(grippers_path)
        self.object_dict = {}
        self._check_object_usd(objects_path)
        # Current task
        self.task_pointer = 0
        self.n_jobs = self.grasps.shape[0]
        #End effector axis (+/- 1,2,3) x,y, z respectively
        self.EF_axis = {
            "fetch_gripper" : 1,
            "sawyer": 3,
            "robotiq_3finger":2
        }
        #Controllers Information
        self.close_dir= {
            "fetch_gripper" : [-1,-1],
            "sawyer": [1,1]
        }
        self.job_pointer = 0


    def _check_gripper_usd(self,grippers_path):
        """ Check if the gripper usds are readable by program

            grippers_path: Path to directory containing all gripper files. Make sure every gripper.urdf is within a folder with the same name
        """
        for i in self.gripper_names :
            abs_path = os.path.join(grippers_path, i, i, i+".usd")
            if (os.path.exists(abs_path)) : 
                self.gripper_dict[i] = abs_path
            else: 
                raise LookupError("Couldn't find gripper .usd file at " + abs_path)
        
        #print(self.gripper_dict)

    def _check_object_usd(self,objects_path):
        """ Check if the object usds are readable by program
        
        Args:
            objects_path: Path to directory containing all object files. Make sure every object.urdf is within a folder with the same name
        """
        for i in self.object_names :
            abs_path = os.path.join(objects_path, i, i, i+".usd")
            if (os.path.exists(abs_path)) : 
                self.object_dict[i] = abs_path
            else: 
                raise LookupError("Couldn't find object .usd file for " + abs_path)
        
        #print(self.object_dict)

    def request_job(self):
        """ Function used by workstations to request job
        """
        if(self.job_pointer<=self.n_jobs):
            job = self.grasps.iloc[self.job_pointer]
            self.job_pointer +=1
            print("Current Job:" + str(self.job_pointer))
        else:
            return None
        #self.task_pointer = self.task_pointer+1
        return job
    
    def translate_dofs(self, gripper, dofs):
        """ Function to translate the GraspIt dofs to Isaac Sim dofs
        
        Args: 
            gripper: name of the gripper to translate dofs
            dofs: np array of dofs to translate
        """

        if(gripper=="fetch_gripper"):
            robot_pos = dofs/-100
        if(gripper=="sawyer"):
            robot_pos = dofs/1000
        return robot_pos



if __name__ == "__main__":
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    man = Manager(json_path, grippers_path, objects_path)
    
    print(man.gripper_names)
    print(man.grasps.iloc[0])
