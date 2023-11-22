#External Libraries
import numpy as np
import pandas as pd

#Custom Classes and utils
from manager import Manager
from utils import InverseT, re, te, R_t_from_tf
from controllers import ForceController

#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.prims.rigid_prim import RigidPrim, RigidPrimView    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation, ArticulationView
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative
from omni.isaac.core.utils.prims import create_prim, delete_prim

from controllers import ForceController, PositionController

class View():

    def __init__(self, work_path, contact_names_expr, num_w,  manager, world):
        self.grippers = ArticulationView(prim_paths_expr = work_path[:-1]+"[0-"+str(num_w)+"]"+"/gripper")
        self.objects = RigidPrimView(prim_paths_expr= work_path[:-1]+"[0-"+str(num_w)+"]"+"/object",track_contact_forces= True,prepare_contact_sensors = True,
                                        contact_filter_prim_paths_expr  = contact_names_expr, disable_stablization = False, reset_xform_properties = False)
        
        print(self.grippers.get_world_poses([1, 2, 0]))
        #self.objects.enable_gravities()


    def get_job(self):
        rigidView = []
        return rigidView
