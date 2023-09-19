import numpy as np
import pandas as pd
import os

class Manager:
    """ Grasp Data Manager:

    Args:
        graps_path: .json file containing the grasp information
        grippers_path: folder path for all the gripper .urdf files
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
        self._check_gripper_urdf(grippers_path)
        self.object_dict = {}
        self._check_object_urdf(objects_path)
        # Current task
        self.task_pointer = 0


    def _check_gripper_urdf(self,grippers_path):
        """ Check if the gripper urdfs are readable by program
        """
        for i in self.gripper_names :
            abs_path = os.path.join(grippers_path, i, i+".urdf")
            if (os.path.exists(abs_path)) : 
                self.gripper_dict[i] = abs_path
            else: 
                raise LookupError("Couldn't find gripper .urdf file for " + i)
        
        #print(self.gripper_dict)

    def _check_object_urdf(self,objects_path):
        """ Check if the gripper urdfs are readable by program
        """
        for i in self.object_names :
            abs_path = os.path.join(objects_path, i, i+".urdf")
            if (os.path.exists(abs_path)) : 
                self.object_dict[i] = abs_path
            else: 
                raise LookupError("Couldn't find object .urdf file for " + i)
        
        #print(self.object_dict)

    def request_job(self):
        """ Function used by workstations to request job
        """
        job = self.grasps.iloc[self.task_pointer]
        self.task_pointer = self.task_pointer+1
        return job


if __name__ == "__main__":
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    man = Manager(json_path, grippers_path, objects_path)

    print(man.gripper_names)
    print(man.grasps.iloc[0])