#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

#External Libraries
import numpy as np

#World Imports
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import Cloner interface
from pxr import Gf, Sdf, UsdPhysics

# Custom Classes
from manager import Manager
from workstation import Workstation


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
    world.scene.add_default_ground_plane(-1)
    #Create initial Workstation
    work = define_prim("/World/Workstation")
    cloner = GridCloner(spacing = 1)
    target_paths = cloner.generate_paths('World/Workstation', num_w)
    cloner.clone(source_prim_path = "/World/Workstation", prim_paths = target_paths)
    
    return world, target_paths



if __name__ == "__main__":
    # Local Directories, use complete paths****
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/fetch_gripper-Nestle_Nips_Hard_Candy_Peanut_Butter.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    # Initialize Manager 
    manager = Manager(json_path, grippers_path, objects_path)   

    #Get Stage
    stage = omni.usd.get_context().get_stage()
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.8)


    num_w= manager.n_jobs # Number of Workstations to create
    num_w =70 # Reduce for debugging

    #initialize World with Workstations Coordinate frames
    world, workstation_paths = init_world(num_w)
    #print(workstation_paths)

    #initialize Workstations
    workstations = []
    for i in range(len(workstation_paths)):
        tmp = Workstation(i, manager, "/" + workstation_paths[i], world)
        workstations.append(tmp)
    
    #Reset World and create set up first robot positions
    world.reset()
    for i in workstations:
        i.reset_robot()
        world.add_physics_callback("Step_Robot_"+ str(i), callback_fn=i.physics_step)


    
    #Code that runs in simulation
    while simulation_app.is_running():
        world.step(render=True) # execute one physics step and one rendering step
        if world.is_playing():

            if world.current_time_step_index == 0:
                world.reset()
                for i in workstations:
                    i.reset_robot()
                    #i.print_robot_info()

        


    simulation_app.close() # close Isaac Sim