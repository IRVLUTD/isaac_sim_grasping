import numpy as np
import os
import json
import time

class Manager:
    """ Grasp Data Manager:
    Manages the information of all grippers, grasps and the reporting of results (Verbosity and saving)
    This class takes in a specific .json structure, if it is desired to change the .json format, this 
    is the place to make the simulation compatible to the new format.

    Args:
        grasps_path: .json file containing the grasp information.
        grippers_path: folder path for all the gripper .usd folders
        objects_path: folder path for all the object .urdf files
        controller: specified controller to use in grippers
        world: Isaac Sim World object
    """
    def __init__(self, grasps_path, grippers_path, objects_path, dofs_given):
        # Loading files and urdfs
        print("Loading File: ", grasps_path)
        with open(grasps_path) as fd:
            self.json = json.load(fd)
        self.gripper = self.json['gripper']
        self.object = self.json['object_id']

        # Extract grasps 
        self.grasps = np.asarray(self.json['pose'])
        
        # Check for usds Object's and Gripper's
        self._check_gripper_usd(grippers_path)
        self._check_object_usd(objects_path)

        # Verbosity of json data loading
        self.n_jobs = self.grasps.shape[0]
        print("Number of Grasps: " + str(self.n_jobs))

        # GRIPPER SPECIFIC DATA
        abs_dir = os.path.join(grippers_path,"gripper_isaac_info.json")
        with open(abs_dir) as fd:
            self.gripper_dict = json.load(fd)[self.gripper]
        if dofs_given:        
            self.dofs = self.json['dofs']
            self.dofs = np.asarray(self.dofs) #graspit_dofs
        else:
            self.dofs = np.tile(self.gripper_dict['opened_dofs'],(len(self.grasps),1))
        self.grasps = np.asarray(self.grasps) #graspit_pose
        
        # Extract info from dictionaries external and internal
        self.physics_dt = 1/self.gripper_dict["physics_frequency"]
        self.c_names = self.gripper_dict["contact_names"]
        self.EF_axis = self.gripper_dict["EF_axis"]
        self.init_time = time.time()

        #Pointer and result vars
        self.job_pointer = 0 # Start to 0
        self.controller = None #
        self.test_type = None #
        self.total_test_time = None #
        self.result_time = np.zeros(len(self.grasps))
        self.result = np.zeros(len(self.grasps))
        self.completed = np.zeros(len(self.grasps))
        self.final_dofs = np.zeros_like(self.dofs)


    def _check_gripper_usd(self,gripper_path):
        """ Check if the gripper usd exist

            grippers_path: Path to directory containing all gripper files. Make sure every gripper.urdf is within a folder with the same name
        """
        i = self.gripper
        self.gripper_path = os.path.join(gripper_path, i, i, i+".usd")
        if (os.path.exists(self.gripper_path)) : 
            print("Found Gripper")
        else: 
            raise LookupError("Couldn't find gripper .usd file at " + self.gripper_path )
        return
    
    def _check_object_usd(self,object_path):
        """ Check if the object usds are readable by program
        
        Args:
            objects_path: Path to directory containing all object files. Make sure every object.urdf is within a folder with the same name
        """
        i = self.object
        self.object_path = os.path.join(object_path, i, i+".usd")
        if (os.path.exists(self.object_path)) : 
            print("Found Object")
        else: 
            raise LookupError("Couldn't find object .usd file for " + self.object_path )
        return

    def request_jobs(self, n):
        """ Function used by workstations class to request jobs

        Args: 
            n: number of new jobs required
        """
        job_IDs = []
        tmp = []
        poses = []
        dofs = []
        for i in range(n):
            if(self.job_pointer<self.n_jobs):
                job_IDs.append(self.job_pointer)
                tmp.append(self.job_pointer)
                self.job_pointer +=1
            else:
                tmp.append(0)
                job_IDs.append(-1)

        poses = np.asarray(self.grasps[tmp,:])
        dofs = np.asarray(self.dofs[tmp,:])
        job_IDs = np.asarray(job_IDs)
        #print('Jobs given ', job_IDs)
        return dofs, poses, job_IDs

    def report_result(self, job_ID, value, new_dofs, status):
        """ Reports falls of objects in grasp tests
        
        Args:
            job_ID: IDs of workstations where objects fell
            value: total test time for each test
            new_dofs: Dofs recorded at the end of setup phase
            status: Failed or passed status
        """

        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)
        
        active = (self.completed[job_ID]==0)
        job_ID = job_ID[active]
        
        if len(job_ID):
            self.result_time[job_ID] = value[active]
            self.final_dofs[job_ID] = new_dofs[active]
            self.completed[job_ID] = 1
            self.result[job_ID] = status
        return
    
    def save_json(self,output_path):
        """ Saves json on disk.

        Args: 
            out_path: path to save json at
        
        """
        print("Saving File at: ",output_path)
        new_json = self.json
        
        # Single elements
        new_json['gripper'] = self.gripper
        new_json['object_id'] = self.object
        new_json["test_type"] = self.test_type
        new_json['controller'] = self.controller
        new_json['test_duration'] = self.total_test_time

        # Lists
        new_json['pose'] = self.grasps.tolist()
        new_json['dofs'] = self.dofs.tolist()
        new_json["result_time"] = self.result_time.tolist()
        new_json["result"] = self.result.tolist()
        new_json['final_dofs'] = self.final_dofs.tolist()
        
        # Runtime info
        new_json['runtime'] = time.time()-self.init_time
        new_json['physics_dt'] = self.physics_dt
        new_json['timestamp'] = time.time()

        with open(output_path,'w') as outfile:
            json.dump(new_json,outfile)
        return

    def report_results(self):
        """ Verbosity for results of .json file filter

        Args:
            ft: fall_threshold
            st: slip_threshold
        """
        print("Completed " + str(round(np.sum(self.completed),0))+ " out of " + str(len(self.completed)))
        print("Total Test Time: " +str(self.total_test_time))
        print("Result Tests Passed " + str(np.sum(self.result,0)))
        print("Mean: " +str(round(np.mean(self.result_time[self.result_time>0]),3)) + "-- Std: " +str(round(np.std(self.result_time[self.result_time>0]),3)) + "-- Variance: " +str(round(np.var(self.result_time[self.result_time>0]),3)))
        print("Max: "+ str(round(np.max(self.result_time),3)) + " -- Min: " + str(round(np.min(self.result_time[self.result_time>0]),3)))
        return