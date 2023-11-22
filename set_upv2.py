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
from omni.isaac.core.utils.stage import add_reference_to_stage

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
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative



def import_gripper(work_path,usd_path, EF_axis):
        T_EF = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])
        if (EF_axis == 1):
            T_EF = np.array([[ 0,0,1,0],
                            [ 0,1,0,0],
                            [-1,0,0,0],
                            [0,0,0,1]])
        elif (EF_axis == 2):
            T_EF = np.array([[1, 0,0,0],
                                [0, 0,1,0],
                                [0,-1,0,0],
                                [0, 0,0,1]])
        elif (EF_axis == 3):
            T_EF = np.array([[1, 0, 0,0],
                                [0,-1, 0,0],
                                [0, 0,-1,0],
                                [0, 0, 0,1]])
        elif (EF_axis == -1):
            T_EF = np.array([[0,0,-1,0],
                                [0,1, 0,0],
                                [1,0, 0,0],
                                [0,0, 0,1]])
        elif (EF_axis == -2):
            T_EF = np.array([[1,0, 0,0],
                                [0,0,-1,0],
                                [0,1, 0,0],
                                [0,0, 0,1]])
        #Robot Pose
        #T_EF.astype(float)
        gripper_pose= pose_from_tf_matrix(T_EF.astype(float))
        
        # Adding Robot usd
        add_reference_to_stage(usd_path=usd_path, prim_path=work_path+"/gripper")
        robot = world.scene.add(Articulation(prim_path = work_path+"/gripper", name="gripper",
                            position = gripper_pose[0], orientation = gripper_pose[1], enable_dof_force_sensors = True))
        robot.set_enabled_self_collisions(False)
        return robot, T_EF

def import_object(work_path, usd_path):
    print(usd_path)
    print(work_path)
    add_reference_to_stage(usd_path=usd_path, prim_path=work_path+"/object")
    object_parent = world.scene.add(GeometryPrim(prim_path = work_path+"/object", name="object"))
    object_prim = []
    object_prim = RigidPrim(prim_path= work_path +"/object")
    return object_parent, object_prim

    

if __name__ == "__main__":
    # Directories
    json_directory = "/home/felipe/Documents/isaac_sim_grasping/grasp_data"
    grippers_directory = "/home/felipe/Documents/isaac_sim_grasping/grippers"
    objects_directory = "/home/felipe/Documents/isaac_sim_grasping/objects"
    output_directory = "/home/felipe/Documents/isaac_sim_grasping/Outputs"

    # Hyperparameters
    num_w = 4
    physics_dt = 1/60
    test_time = 5
    fall_threshold = 5 #Just for final print (Not in json)
    slip_threshold = 2 #Just for final print (Not in json)

    #Debugging
    render = True

    #Load json files 
    json_files = [pos_json for pos_json in os.listdir(json_directory) if pos_json.endswith('.json')]
    
    for j in json_files:
        # Initialize Manager (first)
        manager = Manager(os.path.join(json_directory,j), grippers_directory, objects_directory)   

        #initialize World with Workstations Coordinate frames
        world = World()
        #world.scene.add_default_ground_plane(-1)
        #Create initial Workstation
        work_path = "/World/Workstation_0"
        work_prim = define_prim(work_path)
        EF_axis = manager.EF_axis[manager.gripper]

        #Initialize Workstation
        robot, T_EF = import_gripper(work_path, manager.gripper_path,EF_axis)
        object_parent, object_prim = import_object(work_path, manager.object_path)
        contact_th = manager.contact_th[manager.gripper]

        c_names = manager.contact_names[manager.gripper]
        contact_names = []
        for i in c_names:
            contact_names.append(work_path[:-1]+"[0-"+str(num_w)+"]"+"/gripper/" +  i)
        
        cloner = GridCloner(spacing = 1)
        print(contact_names)
        target_paths = []
        for i in range(num_w):
             target_paths.append(work_path[:-1]+str(i+1))
        cloner.clone(source_prim_path = "/World/Workstation_0", prim_paths = target_paths)


        # Create prim view ** Juge liability in code Needs to be the same in workstation and manage class, be careful when changing
        #viewer = View(work_path,contact_names,num_w, manager,world)

        

        #Set desired physics_dt
        physicsContext = world.get_physics_context()
        physicsContext.set_physics_dt(physics_dt)
        physicsContext.enable_gpu_dynamics(True)
        #world.set_simulation_dt(physics_dt,2*physics_dt)

        #Reset World and create set first robot positions
        world.reset()
        #viewer.grippers.initialize()
        #viewer.objects.initialize()
        
            
        #Run Sim
        with tqdm(total=len(manager.completed)) as pbar:
            while not all(manager.completed):
                world.step(render=render) # execute one physics step and one rendering step if not headless
                #pbar.reset(total = len(manager.completed))
                if pbar.n != np.sum(manager.completed):
                    pbar.update(np.sum(manager.completed)-pbar.n)

        #Save new json with results
        manager.save_json(output_directory+"/simulated-"+j)
        manager.report_results(fall_threshold,slip_threshold)
        world.pause()
        world.clear_all_callbacks()
        world.clear()

    simulation_app.close() # close Isaac Sim
