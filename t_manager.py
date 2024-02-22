import numpy as np
import pandas as pd
import os
from controllers import controller_dict
import utils
import json

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
        fd = open(grasps_path)
        self.json = json.load(fd)
        self.gripper = self.json['gripper']
        self.object = self.json['object_id']

        #Translate GraspIt DoFs Information
        self.pickle_file_data = utils.load_pickle(os.path.join(grippers_path, "gripper_pyb_info.pk"))

        # Extract grasps and reorder quaternions
        self.grasps = self.json['pose']
        self.grasps = np.asarray(self.grasps) 

        #self.grasps[:,[3,4,5,6]]= self.grasps[:,[6,3,4,5]]
        

        # Check for usds Object's and Gripper's
        self._check_gripper_usd(grippers_path)
        self._check_object_usd(objects_path)

        # Verbosity of json data loading
        self.n_jobs = self.grasps.shape[0]
        print("Number of Grasps: " + str(self.n_jobs))

        # GRIPPER SPECIFIC DATA
        self._init_gripper_dicts()
        
        # Extract info from dictionaries external and internal
        self.controller = controller_dict[controller]
        self.close_mask = self.close_dir[self.gripper]
        self.contact_th = self.contact_ths[self.gripper]
        self.physics_dt = self.dts[self.gripper]
        self.c_names = self.contact_names[self.gripper]
        self.EF_axis = self.EF_axes[self.gripper]

        #Pointer and result vars
        self.job_pointer = 0 # Start to 0
        self.test_type = None #!!!
        self.total_test_time = None #!!!
        self.fall_time = np.zeros(len(self.grasps))
        self.slip_time = np.ones(len(self.grasps))*-1
        self.completed = np.zeros(len(self.grasps))
        self.reported_slips = np.zeros(len(self.grasps))

    def _init_gripper_dicts(self):
        """ GRIPPER INFORMATION INITIALIZATION
        Every gripper should have its information added here
        
        """
        #End effector axis (+/- 1,2,3) x,y, z respectively
        self.EF_axes = {
            "fetch_gripper": 1,
            "franka_panda": 3,
            "sawyer": 3,
            "wsg_50": 3,
            "Barrett": 3,
            "robotiq_3finger": 2,
            "jaco_robot": -1,
            "Allegro": 1,
            "shadow_hand": -2,
            "HumanHand": -2,
            "h5_hand": -3
        }

        #Custom Physics dts (increase filtering speed)
        self.dts = {
            "fetch_gripper": 1/120,
            "franka_panda": 1/120,
            "sawyer": 1/120,
            "wsg_50": 1/120,
            "Barrett": 1/120,
            "robotiq_3finger": 1/120,
            "jaco_robot": 1/120,
            "Allegro": 1/120,
            "shadow_hand": 1/120,
            "HumanHand": 1/120,
            "h5_hand": 1/120
        }

        #Array like structure for DoFs to close gripper for 0 the dof won't move 
        # 1 dof will move only to set up grasp
        # >1 dof will move to set up grasp and to exert force on object
        # Sign determines direction
        self.close_dir= {
            "fetch_gripper" : [2,2],
            "franka_panda": [-2, -2], # NOTE, franka_panda gripper by default is closed, so need to open before, Opendir = [1, 1]
            "sawyer": [2, 2],
            "wsg_50": [2, -2], # NOTE, wsg_50 gripper by default is closed, so need to open before, Opendir = [-1, 1]
            "Barrett": [0, 0, 2, 2, 2, 0, 0, 0],
            "jaco_robot": [2, 2, 2],
            "robotiq_3finger": [0, 0, 2, 2, 2, 2, 2, 2, 0, 0, 0], #[0, 0, 2, 2, 2, 2, 2, 2, 0, 0, 0]
            "h5_hand": [2,-2,0,0],
            "Allegro": [0, 1.5, 0, 0, 2, 0, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0], #
            "HumanHand":[0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0 , 0, 0, 0 , 0],
            "shadow_hand": [0 , 0, 0, 0, 2, 2, 0, 2, 2, 0, 2, 2, 2, 2, 0, 0, 2, 0, 0, 2, 0, 2]
        }

        #List of names of joints to check for collisions; it must be as specified in the .usd of the gripper
        self.contact_names= { 
            "fetch_gripper" : ["l_gripper_finger_link_joint","r_gripper_finger_link_joint"],
            "franka_panda": ["panda_hand", "panda_leftfinger", "panda_rightfinger"], 
            "sawyer": ["leftfinger", "rightfinger"],
            "wsg_50": ["gripper_left", "gripper_right"], 
            "Barrett": ["a_link2_joint","a_link3_joint","b_link2_joint","b_link3_joint","c_link2_joint_0","c_link3_joint"],
            "jaco_robot": ["jaco_8_finger_index", "jaco_8_finger_thumb", "jaco_8_finger_pinkie"],
            "robotiq_3finger": ["RIQ_link_1_joint_a", "RIQ_link_1_joint_b", "RIQ_link_1_joint_c",
                             "RIQ_link_2_joint_a", "RIQ_link_2_joint_b", "RIQ_link_2_joint_c",
                             "RIQ_link_3_joint_a", "RIQ_link_3_joint_b", "RIQ_link_3_joint_c"],
            "Allegro": ["link_0_0", "link_2_0","link_3_0", "link_4_0", "link_5_0","link_6_0","link_7_0","link_8_0","link_9_0","link_10_0",
                        "link_11_0","link_12_0","link_13_0","link_14_0","link_15_0"],
            "HumanHand": ["index1_joint","index2_joint","index3_joint", "mid1_joint","mid2_joint","mid3_joint", "pinky1_joint","pinky2_joint","pinky3_joint",
                           "ring1_joint","ring2_joint","ring3_joint", "thumb1_joint","thumb2_joint","thumb3_joint" ],
            "shadow_hand": [ "index_finger_proximal", "index_finger_middle", "index_finger_distal",
                             "little_finger_proximal", "little_finger_middle", "little_finger_distal",
                             "middle_finger_proximal", "middle_finger_middle", "middle_finger_distal",
                             "ring_finger_proximal", "ring_finger_middle", "ring_finger_distal",
                            "thumb_proximal", "thumb_middle", "thumb_distal"],
            "h5_hand" : ["left_tip_link", "right_tip_link"]
        }
        
        #Amount of contacts required for the grasp to be considered as ready
        self.contact_ths = { 
            "fetch_gripper" : 2,
            "franka_panda": 2, 
            "sawyer": 2,
            "wsg_50": 2, 
            "Barrett": 2,
            "jaco_robot": 2,
            "robotiq_3finger": 2,
            "Allegro": 2,
            "HumanHand": 2,
            "shadow_hand": 2,
            "h5_hand": 2
        }

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
    
    def translate_dofs(self, robot_idx):
        """ Function to translate the GraspIt dofs to Isaac Sim dofs. It overwrites manager.dofs
        
        Args: 
            robot_idx: List of dofs indices names of the grippers given by Isaac Sim
        """

        robot_pos= np.ones((self.n_jobs,len(robot_idx)))

        # Open grippers
        if self.gripper== "fetch_gripper":
            robot_pos = robot_pos *np.asarray([-0.025, -0.025])
        elif self.gripper == "franka_panda":
            robot_pos = robot_pos * np.asarray([0.04, 0.04])
        elif self.gripper == "wsg_50":
            robot_pos = robot_pos * np.asarray([-0.055, 0.055])
        elif self.gripper == "sawyer":
            robot_pos = robot_pos * np.asarray([0, 0])
        elif self.gripper == "h5_hand":
            robot_pos = robot_pos * np.asarray([np.radians(-79), np.radians(79),np.radians(79),np.radians(-79)])
        elif self.gripper == "Barrett":
            robot_pos = robot_pos * np.asarray([np.radians(30),np.radians(30),0,0,0,0,0,0])
        elif self.gripper == "jaco_robot":
            robot_pos = robot_pos * np.asarray([0, 0, 0])
        elif self.gripper == "robotiq_3finger":
            robot_pos = robot_pos * np.asarray([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        elif self.gripper == "Allegro":
            robot_pos = robot_pos * np.asarray([0, 0, 0, 0, np.radians(-11), np.radians(67), np.radians(-11),
                                                 np.radians(-11), np.radians(-10), 0, np.radians(-9),
                                                 np.radians(-10), np.radians(10), np.radians(10), np.radians(10), 
                                                 np.radians(10)])
        elif self.gripper == "HumanHand":
            robot_pos = robot_pos * np.asarray([np.radians(10), 0, np.radians(-15), np.radians(-5), np.radians(-10), 
                                                np.radians(-10), np.radians(-10), np.radians(-10), np.radians(-10),
                                                    np.radians(-50), np.radians(-10), np.radians(-10),
                                                      np.radians(-10), np.radians(-10), np.radians(-10),
                                                        np.radians(20), np.radians(20), np.radians(20),
                                                          np.radians(20), np.radians(20)])
        elif self.gripper == "shadow_hand":
            robot_pos = robot_pos * np.asarray([np.radians(-10), np.radians(15), np.radians(-5), 0, np.radians(-45),
                                                 0, np.radians(-5), 0, 0, np.radians(69), 0, 0, 0, 0, 0,np.radians(20), 0, 
                                                      np.radians(20), np.radians(20), np.radians(-20),
                                                        np.radians(20), np.radians(20)])
        else:            
            raise(LookupError("No dof translation for gripper"))


        self.dofs = robot_pos
        return robot_pos

    def report_fall(self, job_ID, value,test_type, test_time):
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
        new_json = {}
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
        new_json["og_gripper"] = self.json["og_gripper"]

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
