import numpy as np
import os
from controllers import controller_dict
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
    def __init__(self, grasps_path, grippers_path, objects_path, controller= 'default',
                 world=None):
        # Loading files and urdfs
        self.world = world
        print("Loading File: ", grasps_path)
        with open(grasps_path) as fd:
            self.json = json.load(fd)
        self.gripper = self.json['gripper']
        self.object = self.json['object_id']

        # Extract grasps 
        self.grasps = self.json['pose']        
        self.dofs = self.json['dofs']
        self.dofs = np.asarray(self.dofs) #graspit_dofs
        self.grasps = np.asarray(self.grasps) #graspit_pose

        # Check for usds Object's and Gripper's
        self._check_gripper_usd(grippers_path)
        self._check_object_usd(objects_path)

        # Verbosity of json data loading
        self.n_jobs = self.grasps.shape[0]
        print("Number of Grasps: " + str(self.n_jobs))

        # GRIPPER SPECIFIC DATA
        self._init_gripper_dicts(grippers_path)
        
        # Extract info from dictionaries external and internal
        self.controller = controller_dict[controller]
        self.close_mask = self.gripper_dict["close_dir"]
        self.contact_th = self.gripper_dict["contact_th"]
        self.physics_dt = 1/self.gripper_dict["physics_frequency"]
        self.c_names = self.gripper_dict["contact_names"]
        self.EF_axis = self.gripper_dict["EF_axis"]
        self.init_time = time.time()

        #Pointer and result vars
        self.job_pointer = 0 # Start to 0
        self.test_type = None #!!!
        self.total_test_time = None #!!!
        self.fall_time = np.zeros(len(self.grasps))
        self.slip_time = np.ones(len(self.grasps))*-1
        self.completed = np.zeros(len(self.grasps))
        self.reported_slips = np.zeros(len(self.grasps))
        self.final_dofs = np.zeros_like(self.dofs)


    def _init_gripper_dicts(self,dict_dir):
        """ GRIPPER INFORMATION INITIALIZATION
        Every gripper should have its information within the gripper_isaac_info.json file 
        
        """
        abs_dir = os.path.join(dict_dir,"gripper_isaac_info.json")
        with open(abs_dir) as fd:
            self.gripper_dict = json.load(fd)[self.gripper]
        return

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

    def report_fall(self, job_ID, value,test_type, test_time, new_dofs):
        """ Reports falls of objects in grasp tests
        
        Args:
            job_ID: IDs of workstations where objects fell
            value: time of fall
            test_type: controller information
            test_time: total test time
        """

        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)

        if(self.completed[job_ID].any()):
            pass
        else:
            self.fall_time[job_ID] = value
            self.final_dofs[job_ID] = new_dofs
            
            if self.test_type == None:
                self.test_type = test_type  
                self.total_test_time = test_time
            self.completed[job_ID]= 1
        return
    
    def report_slip(self, job_ID, value):
        """ Reports slip of objects (used in view.py)
        
        Args:
            job_ID: IDs of workstations where objects slipped
            value: time of slip        
        """        
        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)
        self.slip_time[job_ID] = value
        self.reported_slips[job_ID] = 1

        return
    
    def save_json(self,output_path):
        """ Saves json on disk.

        Args: 
            out_path: path to save json at
        
        """
        print("Saving File at: ",output_path)
        new_json = self.json
        #Single elements
        new_json['standalone'] = "standalone"
        new_json['gripper'] = self.gripper
        new_json['object_id'] = self.object
        new_json["test_type"] = self.test_type
        new_json['test_duration'] = self.total_test_time
        tmp =np.logical_and(self.slip_time==-1,self.fall_time!=-1)
        self.slip_time[tmp] = self.total_test_time #Didn't even slip

        #Lists
        new_json['pose'] = self.grasps.tolist()
        new_json['dofs'] = self.dofs.tolist()
        new_json["fall_time"] = self.fall_time.tolist()
        new_json["slip_time"] = self.slip_time.tolist()
        
        #NEW
        new_json['runtime'] = time.time()-self.init_time
        new_json['physics_dt'] = self.physics_dt
        new_json['final_dofs'] = self.final_dofs.tolist()

        with open(output_path,'w') as outfile:
            json.dump(new_json,outfile)
        return

    def report_results(self,ft=1, st=1):
        """ Verbosity for results of .json file filter

        Args:
            ft: fall_threshold
            st: slip_threshold
        """
        print("Completed " + str(round(np.sum(self.completed),0))+ " out of " + str(len(self.completed)))
        passed = (self.fall_time > ft).sum()
        print("Total Test Time: " +str(self.total_test_time))
        print("Fall Tests Passed (th = " +str(ft)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.fall_time[self.fall_time>0]),3)) + "-- Std: " +str(round(np.std(self.fall_time[self.fall_time>0]),3)) + "-- Variance: " +str(round(np.var(self.fall_time[self.fall_time>0]),3)))
        print("Max: "+ str(round(np.max(self.fall_time),3)) + " -- Min: " + str(round(np.min(self.fall_time[self.fall_time>0]),3)))
        passed = (self.slip_time > st).sum()
        print("Slip Tests Passed (th = " +str(st)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.slip_time),3)) + "-- Std: " +str(round(np.std(self.slip_time),3)) + "-- Variance: " +str(round(np.var(self.slip_time),3)))
        print("Max: "+ str(round(np.max(self.slip_time),3)) + " -- Min: " + str(round(np.min(self.slip_time),3)))
        return



class T_Manager:
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
    def __init__(self, grasps_path, grippers_path, objects_path, controller= 'default',
                 world=None):
        # Loading files and urdfs
        self.world = world
        print("Loading File: ", grasps_path)
        with open(grasps_path) as fd:
            self.json = json.load(fd)
        self.gripper = self.json['gripper']
        self.object = self.json['object_id']

        # Extract grasps and reorder quaternions
        self.grasps = self.json['pose']
        self.grasps = np.asarray(self.grasps)         

        # Check for usds Object's and Gripper's
        self._check_gripper_usd(grippers_path)
        self._check_object_usd(objects_path)

        # Verbosity of json data loading
        self.n_jobs = self.grasps.shape[0]
        print("Number of Grasps: " + str(self.n_jobs))

        # GRIPPER SPECIFIC DATA
        self._init_gripper_dicts(grippers_path)
        
        # Extract info from dictionaries external and internal
        self.controller = controller_dict[controller]
        self.close_mask = self.gripper_dict["transfer_close_dir"]
        self.contact_th = self.gripper_dict["transfer_contact_th"]
        self.physics_dt = 1/self.gripper_dict["physics_frequency"]
        self.c_names = self.gripper_dict["contact_names"]
        self.EF_axis = self.gripper_dict["EF_axis"]
        self.init_time = time.time()


        #Pointer and result vars
        self.job_pointer = 0 # Start to 0
        self.test_type = None #!!!
        self.total_test_time = None #!!!
        self.fall_time = np.zeros(len(self.grasps))
        self.slip_time = np.ones(len(self.grasps))*-1
        self.completed = np.zeros(len(self.grasps))
        self.reported_slips = np.zeros(len(self.grasps))
        self.final_dofs = np.zeros((len(self.fall_time),len(self.close_mask)))

    def _init_gripper_dicts(self,dict_dir):
        """ GRIPPER INFORMATION INITIALIZATION
        Every gripper should have its information within the gripper_isaac_info.json file 
        
        """
        abs_dir = os.path.join(dict_dir,"gripper_isaac_info.json")
        with open(abs_dir) as fd:
            self.gripper_dict = json.load(fd)[self.gripper]

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
        #Verbosity
        #print("JOBS GIVEN: ", np.asarray(self.json['og_gripper'])[tmp])
        #print('Jobs given ', job_IDs)
        return dofs, poses, job_IDs
    
    def opened_dofs(self, robot_idx):
        """ Sets starting DoFs of all grasps to the gripper opened position.

        Args: 
            robot_idx: List of dofs indices names of the grippers given by Isaac Sim
        """

        robot_pos= np.ones((self.n_jobs,len(robot_idx)))

        # Opened dof for gripper
        robot_pos= robot_pos * np.asarray(self.gripper_dict["opened_dofs"])
        
        self.dofs = robot_pos
        self.final_dofs = np.zeros_like(self.dofs)
        return robot_pos

    def report_fall(self, job_ID, value,test_type, test_time, new_dofs):
        """ Reports falls of objects in grasp tests
        
        Args:
            job_ID: IDs of workstations where objects fell
            value: time of fall
            test_type: controller information
            test_time: total test time
        """

        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)

        if(self.completed[job_ID].any()):
            pass
        else:
            self.fall_time[job_ID] = value
            self.final_dofs[job_ID] = new_dofs
            if self.test_type == None:
                self.test_type = test_type  
                self.total_test_time = test_time
            self.completed[job_ID]= 1

        return
    
    def report_slip(self, job_ID, value):
        """ Reports slip of objects (used in view.py)
        
        Args:
            job_ID: IDs of workstations where objects slipped
            value: time of slip        
        """        
        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)
        self.slip_time[job_ID] = value
        self.reported_slips[job_ID] = 1

        return
    
    def save_json(self,output_path):
        """ Saves json on disk.

        Args: 
            out_path: path to save json at
        
        """
        print("Saving File at: ",output_path)
        new_json = self.json
        #Single elements
        new_json['gripper'] = self.gripper
        new_json['object_id'] = self.object
        new_json["test_type"] = self.test_type
        new_json['test_duration'] = self.total_test_time
        tmp =np.logical_and(self.slip_time==-1,self.fall_time!=-1)
        self.slip_time[tmp] = self.total_test_time #Didn't even slip

        #Lists
        new_json['pose'] = self.grasps.tolist()
        new_json["fall_time"] = self.fall_time.tolist()
        new_json["slip_time"] = self.slip_time.tolist()

        #NEW
        new_json['physics_dt'] = self.physics_dt
        new_json['runtime'] = time.time()-self.init_time
        new_json['final_dofs'] = self.final_dofs.tolist()

        with open(output_path,'w') as outfile:
            json.dump(new_json,outfile)
        return

    def report_results(self,ft=1, st=1):
        """ Verbosity for results of .json file filter

        Args:
            ft: fall_threshold
            st: slip_threshold
        """
        print("Completed " + str(round(np.sum(self.completed),0))+ " out of " + str(len(self.completed)))
        passed = (self.fall_time > ft).sum()
        print("Total Test Time: " +str(self.total_test_time))
        print("Fall Tests Passed (th = " +str(ft)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.fall_time[self.fall_time>0]),3)) + "-- Std: " +str(round(np.std(self.fall_time[self.fall_time>0]),3)) + "-- Variance: " +str(round(np.var(self.fall_time[self.fall_time>0]),3)))
        print("Max: "+ str(round(np.max(self.fall_time),3)) + " -- Min: " + str(round(np.min(self.fall_time[self.fall_time>0]),3)))
        passed = (self.slip_time > st).sum()
        print("Slip Tests Passed (th = " +str(st)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.slip_time),3)) + "-- Std: " +str(round(np.std(self.slip_time),3)) + "-- Variance: " +str(round(np.var(self.slip_time),3)))
        print("Max: "+ str(round(np.max(self.slip_time),3)) + " -- Min: " + str(round(np.min(self.slip_time),3)))
        return



class V_Manager:
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
    def __init__(self, grasps_path, grippers_path, objects_path, controller= 'default', transfer = False,
                 world=None, ub= None, lb= None):
        # Loading files and urdfs
        self.world = world
        print("Loading File: ", grasps_path)
        with open(grasps_path) as fd:
            self.json = json.load(fd)
        self.gripper = self.json['gripper']
        self.object = self.json['object_id']

        # Extract grasps and reorder quaternions
        self.grasps = self.json['pose']
        self.grasps = np.asarray(self.grasps)         

        # Check for usds Object's and Gripper's
        self._check_gripper_usd(grippers_path)
        self._check_object_usd(objects_path)
        self.dofs = np.asarray(self.json['final_dofs'])
        fall_time = np.asarray(self.json["fall_time"])

        if ub!=None:
            tmp = fall_time<=ub
            self.dofs = self.dofs[tmp]
            self.grasps = self.grasps[tmp]
            fall_time = fall_time[tmp]
        if lb!=None:
            tmp = fall_time>=lb
            self.dofs = self.dofs[tmp]
            self.grasps = self.grasps[tmp]
            fall_time = fall_time[tmp]
        
        tmp = np.squeeze(np.argwhere(fall_time == -1))
        if transfer:
            self.dofs[tmp] = np.zeros_like(self.dofs[0])
        else:
            self.dofs[tmp] = np.asarray(self.json['dofs'])[tmp]

        # Verbosity of json data loading
        self.n_jobs = self.grasps.shape[0]
        print("Number of Grasps: " + str(self.n_jobs))

        # GRIPPER SPECIFIC DATA
        self._init_gripper_dicts(grippers_path)

        # Extract info from dictionaries external and internal
        if transfer: 
            self.close_mask = self.gripper_dict["transfer_close_dir"]
            self.contact_th = self.gripper_dict["transfer_contact_th"]  
        else:
            self.close_mask = self.gripper_dict["close_dir"]
            self.contact_th = self.gripper_dict["contact_th"]  
        self.controller = controller_dict[controller]
        self.physics_dt = 1/self.gripper_dict["physics_frequency"]
        self.c_names = self.gripper_dict["contact_names"]
        self.EF_axis = self.gripper_dict["EF_axis"]
        self.init_time = time.time()


        #Pointer and result vars
        self.job_pointer = 0 # Start to 0
        self.test_type = None #!!!
        self.total_test_time = None #!!!
        self.fall_time = np.zeros(len(self.grasps))
        self.slip_time = np.ones(len(self.grasps))*-1
        self.completed = np.zeros(len(self.grasps))
        self.reported_slips = np.zeros(len(self.grasps))
        self.final_dofs = np.zeros((len(self.fall_time),len(self.close_mask)))


    def _init_gripper_dicts(self,dict_dir):
        """ GRIPPER INFORMATION INITIALIZATION
        Every gripper should have its information within the gripper_isaac_info.json file 
        
        """
        abs_dir = os.path.join(dict_dir,"gripper_isaac_info.json")
        with open(abs_dir) as fd:
            self.gripper_dict = json.load(fd)[self.gripper]

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
        #Verbosity
        #print("JOBS GIVEN: ", np.asarray(self.json['og_gripper'])[tmp])
        #print('Jobs given ', job_IDs)
        return dofs, poses, job_IDs

    def report_fall(self, job_ID, value,test_type, test_time, new_dofs):
        """ Reports falls of objects in grasp tests
        
        Args:
            job_ID: IDs of workstations where objects fell
            value: time of fall
            test_type: controller information
            test_time: total test time
        """

        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)

        if(self.completed[job_ID].any()):
            pass
        else:
            #self.fall_time[job_ID] = value
            #self.final_dofs[job_ID] = new_dofs
            if self.test_type == None:
                self.test_type = test_type  
                self.total_test_time = test_time
            self.completed[job_ID]= 1

        return
    
    def report_slip(self, job_ID, value):
        """ Reports slip of objects (used in view.py)
        
        Args:
            job_ID: IDs of workstations where objects slipped
            value: time of slip        
        """        
        job_ID = np.squeeze(job_ID).astype(int)
        value = np.squeeze(value)
        self.slip_time[job_ID] = value
        self.reported_slips[job_ID] = 1

        return
    