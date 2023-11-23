#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
config= {
    "headless": False,
    'max_bounces':0,
    'max_specular_transmission_bounces':0,
}
simulation_app = SimulationApp(config) # we can also run as headless.

#External Libraries
import numpy as np
from tqdm import tqdm
import os

#World Imports
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import Cloner interface
from pxr import Gf, Sdf, UsdPhysics
from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage, save_stage

# Custom Classes
from managerv2 import Manager
from workstationv2 import Workstation
from controllersv2 import ForceController, PositionController
from views import View
#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims.rigid_prim import RigidPrim, RigidPrimView    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import print_stage_prim_paths, traverse_stage
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative


if __name__ == "__main__":
    # Directories
    objects_directory = "/home/felipe/Documents/GoogleScannedObjects_USD"

    #Debugging
    render = True

    #Load json files 
    objs = [pos_json for pos_json in os.listdir(objects_directory)]
    print(objs)
    
    world = World()
    with tqdm(total=len(objs)) as pbar:
        for j in objs:      
            path = os.path.join(objects_directory,j,j + '.usd')
            open_stage(path)

            object = get_prim_at_path(j)
            stage = world.stage
            it = traverse_stage()
            j = 0 
            for i in it:
                if j == 0:
                    stage.SetDefaultPrim(i)
                elif j ==1:
                    pass
                j+=1
                
            #Reset World and create set first robot positions
            #world.reset()
            #viewer.grippers.initialize()
            #viewer.objects.initialize()
            
            #world.pause()
            save_stage(path)
            pbar.update(1)

    simulation_app.close() # close Isaac Sim
