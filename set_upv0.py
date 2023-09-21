#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

#External Libraries
import numpy as np

#World Imports
from omni.isaac.core import World

# Object Initiation file
from omni.isaac.core.utils.prims import define_prim

from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import Cloner interface

from manager import Manager
from workstation import Workstation

#Initialize world
def init_world(num_w):
    world = World()
    world.scene.add_default_ground_plane(-1)
    #Create initial Workstation
    work = define_prim("/World/Workstation")
    cloner = GridCloner(spacing = 1)
    target_paths = cloner.generate_paths('World/Workstation', num_w)
    cloner.clone(source_prim_path = "/World/Workstation", prim_paths = target_paths)
    
    return world, target_paths



if __name__ == "__main__":
    # Initialize Manager 
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"
    manager = Manager(json_path, grippers_path, objects_path)    

    num_w= manager.n_jobs # Number of Workstations to create
    #initialize World
    world, workstation_paths = init_world(num_w)
    #print(workstation_paths)
    workstations = []
    #Initialize Workstations
    for i in range(len(workstation_paths)):
        tmp = Workstation(i, manager, workstation_paths[i], world)
        workstations.append(tmp)
    
    
    world.reset()
    
    
    
    while simulation_app.is_running():
        world.step(render=True) # execute one physics step and one rendering step
        if world.is_playing():
            if world.current_time_step_index == 0:
                #world.reset()
                pass

        


    simulation_app.close() # close Isaac Sim