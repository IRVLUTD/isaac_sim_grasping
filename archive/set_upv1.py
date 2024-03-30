#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

#External Libraries
import numpy as np
import math

#World Imports
from omni.isaac.core import World

# Object Initiation file
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from managers import Manager
from workstation import Workstation
from omni.isaac.core.utils.prims import get_prim_at_path

import omni
from pxr import Gf, Sdf, UsdPhysics


#Initialize world
def init_world(num_w):
    world = World()
    world.scene.add_default_ground_plane(-1)
    #Create initial Workstation
    workstation_name = "/World/Workstation"
    spacing = 1
    rows = math.floor(math.sqrt(num_w))
    workstations_paths = []
    #workstation_prims = []
    j = -1
    print("ROOOOOWWWWS " + str(rows))
    
    for i in range(num_w):
        if (i%rows == 0):j = j+1
        position = [(i%rows)* spacing, j * spacing, 0]
        prim = create_prim(workstation_name + "_" + str(i), 'Xform', position)
        prim = get_prim_at_path(workstation_name + "_" + str(i))
        path = workstation_name + "_" + str(i)
        workstations_paths.append(path)
        #workstation_prims.append(prim)
        #print(type(prim))

    world.reset()
    
    return world, workstations_paths



if __name__ == "__main__":
    # Initialize Manager 
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/fetch_gripper-Nestle_Nips_Hard_Candy_Peanut_Butter.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"
    manager = Manager(json_path, grippers_path, objects_path)   

    #Set Gravity to 0
    stage = omni.usd.get_context().get_stage()
    # Add a physics scene prim to stage
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

    # Set gravity vector
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.8)


    num_w= manager.n_jobs # Number of Workstations to create
    #initialize World
    print(num_w)
    world, workstation_paths = init_world(1000)
    #print(workstation_paths)
    workstations = []
    #Initialize Workstations
    for i in range(len(workstation_paths)):
        tmp = Workstation(i, manager, workstation_paths[i], world)
        workstations.append(tmp)
        #print(i)
    
    world.reset()
    for i in workstations:
                    #i.robot.initialize()
                    i.set_robot_pos()
                    #i.print_robot_info()

    
    while simulation_app.is_running():
        world.step(render=True) # execute one physics step and one rendering step
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
                for i in workstations:
                    #i.robot.initialize()
                    i.set_robot_pos()
                    #i.print_robot_info()

        


    simulation_app.close() # close Isaac Sim