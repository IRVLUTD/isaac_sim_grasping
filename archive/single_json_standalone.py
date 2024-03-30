#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
config= {
    "headless": False,
    'max_bounces':1,
    'max_specular_transmission_bounces':1,
}
simulation_app = SimulationApp(config) # we can also run as headless.

#External Libraries
import numpy as np
from tqdm import tqdm

#World Imports
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import Cloner interface
from pxr import Gf, Sdf, UsdPhysics

# Custom Classes
from managers import Manager
from workstation import Workstation
from controllers import ForceController, PositionController


def init_world(num_w):
    """ Initialize World function
    Initializes the Isaac Sim world and the workstations' parent coordinate frame of num_w workstations.

    Args:
        num_w: Number of Workstation coordinate frames to create

    Returns:
        world: Isaac Sim world object
        target_paths: prim paths for the Workstation coordinate frames
    """
    world = World()
    #world.scene.add_default_ground_plane(-1)
    #Create initial Workstation
    work = define_prim("/World/Workstation")
    cloner = GridCloner(spacing = 1)
    target_paths = cloner.generate_paths('World/Workstation', num_w)
    cloner.clone(source_prim_path = "/World/Workstation", prim_paths = target_paths)
    
    return world, target_paths



if __name__ == "__main__":
    # Local Directories, use complete paths****
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"
    output_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/out.json"
    num_w = 4
    physics_dt = 1/60

    # Initialize Manager 
    manager = Manager(json_path, grippers_path, objects_path)   

    #initialize World with Workstations Coordinate frames
    world, workstation_paths = init_world(num_w)

    #Initialize Workstations
    workstations = []
    for i in range(len(workstation_paths)):
        tmp = Workstation(i, manager, "/" + workstation_paths[i], world, ForceController, test_time=10)
        workstations.append(tmp)

    #Set desired physics_dt
    physicsContext = world.get_physics_context()
    physicsContext.set_physics_dt(physics_dt)

    #Reset World and create set up first robot positions
    world.reset()
    for i in workstations:
        i.reset_robot()
        
    #Code that runs in simulation

    with tqdm(total=len(manager.completed)) as pbar:
        while not all(manager.completed):
            world.step(render=True) # execute one physics step and one rendering step if not headless
            #pbar.reset(total = len(manager.completed))
            if pbar.n != np.sum(manager.completed):
                pbar.update(np.sum(manager.completed)-pbar.n)
    manager.save_json(output_path)

    simulation_app.close() # close Isaac Sim