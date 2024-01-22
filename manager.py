import numpy as np
import pandas as pd
import os
from controllers import controller_dict
import utils
import json

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
        self.json = pd.read_json(grasps_path)
        self.gripper = self.json.iloc[0]['gripper']
        self.object = self.json.iloc[0]['object_id']

        #Translate GraspIt DoFs Information
        self.pickle_file_data = utils.load_pickle(os.path.join(grippers_path, "gripper_pyb_info.pk"))

        # Extract grasps and reorder quaternions
        self.grasps = []
        self.dofs = []
        for i, r in self.json.iterrows(): 
            self.grasps.append(r['grasps']['pose'])
            self.dofs.append(r['grasps']['dofs'])
        self.dofs = np.asarray(self.dofs)
        self.grasps = np.asarray(self.grasps)
        self.grasps[:,[3,4,5,6]]= self.grasps[:,[6,3,4,5]]

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
        self.test_type = np.asarray([None] * len(self.grasps))
        self.total_test_time = np.zeros(len(self.grasps))
        self.fall_time = np.zeros(len(self.grasps))
        self.slip_time = np.zeros(len(self.grasps))
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
            "HumanHand": -2
        }

        #Custom Physics dts (increase filtering speed)
        self.dts = {
            "fetch_gripper": 1/30,
            "franka_panda": 1/30,
            "sawyer": 1/40,
            "wsg_50": 1/30,
            "Barrett": 1/50,
            "robotiq_3finger": 1/50,
            "jaco_robot": 1/50,
            "Allegro": 1/80,
            "shadow_hand": 1/80,
            "HumanHand": 1/80
        }

        # Direction for DoFs to close gripper
        self.close_dir= {
            "fetch_gripper" : [1,1],
            "franka_panda": [-1, -1], # NOTE, franka_panda gripper by default is closed, so need to open before, Opendir = [1, 1]
            "sawyer": [1, 1],
            "wsg_50": [1, -1], # NOTE, wsg_50 gripper by default is closed, so need to open before, Opendir = [-1, 1]
            "Barrett": [0, 0, 1, 1, 1, 0, 0, 0],
            "jaco_robot": [1, 1, 1],
            "robotiq_3finger": [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
            "Allegro": [0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            "HumanHand": [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "shadow_hand": [0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        }

        #List of names of joints to check for collisions; it must be as specified in the .usd of the gripper
        self.contact_names= { 
            "fetch_gripper" : ["l_gripper_finger_link_joint","r_gripper_finger_link_joint"],
            "franka_panda": ["panda_hand", "panda_leftfinger", "panda_rightfinger"], 
            "sawyer": ["base_link", "leftfinger", "rightfinger"],
            "wsg_50": ["gripper_left", "gripper_right"], 
            "Barrett": ["base_link","a_link1_joint","a_link2_joint","a_link3_joint","b_link1_joint","b_link2_joint","b_link3_joint","c_link2_joint_0","c_link3_joint"],
            "jaco_robot": ["base_link", "jaco_8_finger_index", "jaco_8_finger_thumb", "jaco_8_finger_pinkie"],
            "robotiq_3finger": ["RIQ_palm", "RIQ_link_1_joint_a", "RIQ_link_1_joint_b", "RIQ_link_1_joint_c",
                             "RIQ_link_2_joint_a", "RIQ_link_2_joint_b", "RIQ_link_2_joint_c",
                             "RIQ_link_3_joint_a", "RIQ_link_3_joint_b", "RIQ_link_3_joint_c"],
            "Allegro": ["base_link", "link_0_0", "link_2_0","link_3_0", "link_4_0", "link_5_0","link_6_0","link_7_0","link_8_0","link_9_0","link_10_0",
                        "link_11_0","link_12_0","link_13_0","link_14_0","link_15_0"],
            "HumanHand": ["base_link","index1_joint","index2_joint","index3_joint", "mid1_joint","mid2_joint","mid3_joint", "pinky1_joint","pinky2_joint","pinky3_joint",
                           "ring1_joint","ring2_joint","ring3_joint", "thumb1_joint","thumb2_joint","thumb3_joint" ],
            "shadow_hand": ["base_link", "index_finger_knuckle", "index_finger_proximal", "index_finger_middle", "index_finger_distal",
                            "little_finger_knuckle", "little_finger_proximal", "little_finger_middle", "little_finger_distal",
                            "middle_finger_knuckle", "middle_finger_proximal", "middle_finger_middle", "middle_finger_distal",
                            "ring_finger_knuckle", "ring_finger_proximal", "ring_finger_middle", "ring_finger_distal",
                            "thumb_proximal", "thumb_middle", "thumb_distal"]
        }
        
        #Amount of contacts required for the grasp to be considered as ready
        self.contact_ths = { 
            "fetch_gripper" : 1,
            "franka_panda": 1, 
            "sawyer": 2,
            "wsg_50": 1, 
            "Barrett": 2,
            "jaco_robot": 2,
            "robotiq_3finger": 2,
            "Allegro": 2,
            "HumanHand": 2,
            "shadow_hand": 2
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
        #print('Jobs given ', job_IDs)
        return dofs, poses, job_IDs
    
    def translate_dofs(self, robot_idx):
        """ Function to translate the GraspIt dofs to Isaac Sim dofs
        
        Args: 
            robot_idx: List of dofs indices names of the grippers given by Isaac Sim
        """
        json_idx = self.pickle_file_data[self.gripper][1]

        if self.gripper == "fetch_gripper":
            robot_pos = np.asarray([(self.dofs / 1000.0) * 10.0, (self.dofs / 1000.0) * 10.0])
            robot_pos = np.reshape(robot_pos,(robot_pos.shape[1],robot_pos.shape[0]))
        elif self.gripper == "franka_panda":
            robot_pos = np.asarray([self.dofs/ 1000,self.dofs/ 1000])
            robot_pos = np.reshape(robot_pos,(robot_pos.shape[1],robot_pos.shape[0]))
        elif self.gripper == "Barrett":
            robot_pos = np.asarray([ 
                self.dofs[:,0],
                self.dofs[:,1],
                self.dofs[:,1] / 3.0,
                self.dofs[:,0],
                self.dofs[:,2],
                self.dofs[:,2] / 3.0,
                self.dofs[:,3],
                self.dofs[:,3] / 3.0,
            ])
            robot_pos = np.reshape(robot_pos,(robot_pos.shape[1],robot_pos.shape[0]))
        elif self.gripper == "Allegro":
            robot_pos = self.dofs
            for i in range(len(json_idx)):
                    json_idx[i]= json_idx[i].replace(".","_")
        elif self.gripper in {
            "HumanHand", 
            "robotiq_3finger",
            "jaco_robot",
            "shadow_hand",
        }:
            robot_pos = self.dofs
        elif self.gripper in {"wsg_50", "sawyer"}:
            robot_pos = self.dofs/1000.0
        else:
            raise(LookupError("No dof translation for gripper"))

        dof_dict = {json_idx[i]: robot_pos[:,i] for i in range(robot_pos.shape[1])}
        tmp = np.zeros_like(robot_pos)
        c = 0

        for i in robot_idx:
            tmp[:,c] = dof_dict[i] 
            c +=1

        robot_pos = np.squeeze(tmp)
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
            self.test_type[job_ID] = test_type
            self.total_test_time[job_ID] = test_time
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
        self.new_json = pd.DataFrame()
        self.new_json['pose'] = self.grasps.tolist()
        self.new_json['dofs'] = self.dofs.tolist()
        self.new_json["test_type"] = self.test_type
        self.new_json["total_test_time"] = self.total_test_time
        self.new_json["fall_time"] = self.fall_time
        self.new_json["slip_time"] = self.slip_time
        self.new_json.to_json(output_path)
        return

    def report_results(self,ft=2, st=1):
        """ Verbosity for results of .json file filter

        Args:
            ft: fall_threshold
            st: slip_threshold
        """
        print("Completed " + str(round(np.sum(self.completed),0))+ " out of " + str(len(self.completed)))
        passed = (self.fall_time > ft).sum()
        print("Total Test Time: " +str(self.total_test_time[0]))
        print("Fall Tests Passed (th = " +str(ft)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.fall_time[self.fall_time>0]),3)) + "-- Std: " +str(round(np.std(self.fall_time[self.fall_time>0]),3)) + "-- Variance: " +str(round(np.var(self.fall_time[self.fall_time>0]),3)))
        print("Max: "+ str(round(np.max(self.fall_time),3)) + " -- Min: " + str(round(np.min(self.fall_time[self.fall_time>0]),3)))
        passed = (self.slip_time > st).sum()
        print("Slip Tests Passed (th = " +str(st)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.slip_time),3)) + "-- Std: " +str(round(np.std(self.slip_time),3)) + "-- Variance: " +str(round(np.var(self.slip_time),3)))
        print("Max: "+ str(round(np.max(self.slip_time),3)) + " -- Min: " + str(round(np.min(self.slip_time),3)))
        return