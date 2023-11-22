import numpy as np
import pandas as pd
import os
from controllers import ForceController, PositionController
import utils
import json

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
        print("Loading File: ", grasps_path)
        self.grasps = pd.read_json(grasps_path)
        self.gripper_names = [] #List of gripper names
        self.object_names =[] #List of object names 
        self.pickle_file_data = utils.load_pickle(os.path.join(grippers_path, "gripper_pyb_info.pk"))
        #print(self.pickle_file_data)
        for i, row in self.grasps.iterrows():
            if row['gripper'] not in self.gripper_names:
                self.gripper_names.append(row['gripper'])
            if row['object_id'] not in self.object_names:
                self.object_names.append(row['object_id'])
        #print(self.object_names)
        # Initialize dictionaries (paths to objects and grippers)
        self.gripper_dict = {}
        self._check_gripper_usd(grippers_path)
        self.object_dict = {}
        self._check_object_usd(objects_path)
        # Current task
        self.task_pointer = 0
        self.n_jobs = self.grasps.shape[0]
        print("Number of Grasps: " + str(self.n_jobs))
        #End effector axis (+/- 1,2,3) x,y, z respectively
        self.EF_axis = {
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
        #Controllers Information
        self.controllers= {
            "fetch_gripper" : ForceController,
            "franka_panda": ForceController, 
            "sawyer": ForceController,
            "wsg_50": ForceController, 
            "Barrett": ForceController,
            "jaco_robot": ForceController,
            "robotiq_3finger": ForceController,
            "Allegro": PositionController,
            "HumanHand": ForceController,
            "shadow_hand": ForceController
        }
        self.close_dir= {
            "fetch_gripper" : [1,1],
            "franka_panda": [-1, -1], # NOTE, franka_panda gripper by default is closed, so need to open before, Opendir = [1, 1]
            "sawyer": [1, 1],
            "wsg_50": [1, -1], # NOTE, wsg_50 gripper by default is closed, so need to open before, Opendir = [-1, 1]
            "Barrett": [0, 0, 1, 1, 1, 0, 0, 0],
            "jaco_robot": [1, 1, 1],
            "robotiq_3finger": [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
            "Allegro": [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            "HumanHand": [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            "shadow_hand": [0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        }     
        self.contact_names= { #List of names of joints to check for collisions ** USE BASE XFORM OF MESHES
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
            "shadow_hand": []
        }
        self.contact_th = { # Amount of contacts needed to count for grasp set up
            "fetch_gripper" : 2,
            "franka_panda": 2, 
            "sawyer": 2,
            "wsg_50": 2, 
            "Barrett": 2,
            "jaco_robot": 2,
            "robotiq_3finger": 2,
            "Allegro": 2,
            "HumanHand": 2,
            "shadow_hand": 2
        }
        self.job_pointer = 0
        self.test_type = [None] * len(self.grasps)
        self.total_test_time = np.zeros(len(self.grasps))
        self.fall_time = np.zeros(len(self.grasps))
        self.slip_time = np.zeros(len(self.grasps))
        self.completed = np.zeros(len(self.grasps))
        self.reported_slips = np.zeros(len(self.grasps))
        self.rigid_view = 0


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
            abs_path = os.path.join(objects_path, i, i+".usd")
            if (os.path.exists(abs_path)) : 
                print("adding path " + abs_path)
                self.object_dict[i] = abs_path
            else: 
                raise LookupError("Couldn't find object .usd file for " + abs_path)
        
        #print(self.object_dict)

    def request_job(self):
        """ Function used by workstations to request job
        """
        job_ID = -1
        job = None
        if(self.job_pointer<self.n_jobs):
            job_ID = self.job_pointer
            job = self.grasps.iloc[self.job_pointer]
            self.job_pointer +=1
            #print("Current  Job:" + str(self.job_pointer))
        else:
            job = self.grasps.iloc[self.n_jobs-1]
            job_ID = -1
        #self.task_pointer = self.task_pointer+1
        return job, job_ID
    
    def translate_dofs(self, gripper, dofs, robot_idx):
        """ Function to translate the GraspIt dofs to Isaac Sim dofs
        
        Args: 
            gripper: name of the gripper to translate dofs
            dofs: np array of dofs to translate
        """
        #print(robot_idx)
        json_idx = self.pickle_file_data[gripper][1]

        if gripper == "fetch_gripper":
            robot_pos = [(dofs / 1000.0) * 10.0, (dofs / 1000.0) * 10.0]
            #robot_pos = [0.25,0.25]
        elif gripper == "franka_panda":
            robot_pos = [dofs/ 1000,dofs/ 1000]
        elif gripper == "Barrett":
            # assert len(dofs) == len(joint_idxs)
            robot_pos = [ 
                dofs[0],
                dofs[1],
                dofs[1] / 3.0,
                dofs[0],
                dofs[2],
                dofs[2] / 3.0,
                dofs[3],
                dofs[3] / 3.0,
            ]
        elif gripper == "Allegro":
            robot_pos = dofs
            for i in range(len(json_idx)):
                    json_idx[i]= json_idx[i].replace(".","_")
        elif gripper in {
            "HumanHand", 
            "robotiq_3finger",
            "jaco_robot",
            "shadow_hand",
        }:
            robot_pos = dofs
        elif gripper in {"wsg_50", "sawyer"}:
            robot_pos = dofs/1000.0
        else:
            raise(LookupError("No dof translation for gripper"))

        #print(json_idx)
        dof_dict = {json_idx[i]: robot_pos[i] for i in range(len(robot_pos))}
        tmp = np.zeros_like(robot_pos)
        c = 0
        #print(dof_dict)
        for i in robot_idx:
            tmp[c] = dof_dict[i] 
            c +=1
        #print(robot_pos)
        #print(tmp)
        robot_pos = np.squeeze(tmp)
        
       # if gripper == "robotiq_3finger":
            #robot_pos=[0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0]
        return robot_pos

    def report_fall(self, job_ID, value, test_type, test_time):
        """ Reports fall
        
        """
        
        if(self.completed[job_ID]== 0):
            self.fall_time[job_ID] = value
            self.test_type[job_ID] = test_type
            self.total_test_time[job_ID] = test_time
            self.completed[job_ID]= 1
        else: 
            raise IndexError("Two workstations performed the same task")
        return
    
    def report_slip(self, job_ID, value):
        """ Reports slip
        
        """        
        if(self.reported_slips[job_ID]== 0):
            self.slip_time[job_ID] = value
            self.reported_slips[job_ID] = 1
        else: 
            raise IndexError("Slip was reported twice")
        return
    
    def save_json(self,output_path):
        print("Saving File at: ",output_path)
        self.grasps["test_type"] = self.test_type
        self.grasps["total_test_time"] = self.total_test_time
        self.grasps["fall_time"] = self.fall_time
        self.grasps["slip_time"] = self.slip_time
        self.grasps.to_json(output_path)
        return

    def report_results(self,ft, st):
        print("Completed " + str(round(np.sum(self.completed),0))+ " out of " + str(len(self.completed)))
        passed = (self.fall_time > ft).sum()
        print("Total Test Time: " +str(self.total_test_time[0]))
        print("Fall Tests Passed (th = " +str(ft)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.fall_time),3)) + "-- Std: " +str(round(np.std(self.fall_time),3)) + "-- Variance: " +str(round(np.var(self.fall_time),3)))
        passed = (self.slip_time > st).sum()
        print("Slip Tests Passed (th = " +str(st)+ "): "+ str(passed))
        print("Mean: " +str(round(np.mean(self.slip_time),3)) + "-- Std: " +str(round(np.std(self.slip_time),3)) + "-- Variance: " +str(round(np.var(self.slip_time),3)))
        return
    
    



if __name__ == "__main__":
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    man = Manager(json_path, grippers_path, objects_path)
    
    print(man.gripper_names)
    print(man.grasps.iloc[0])
