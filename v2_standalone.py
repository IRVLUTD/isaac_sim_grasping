#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
config= {
    "headless": True,
    'max_bounces':0,
    'max_specular_transmission_bounces':0,
}
simulation_app = SimulationApp(config) # we can also run as headless.

#External Libraries
import numpy as np
from tqdm import tqdm
import os
import sys

#World Imports
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import Cloner interface
from pxr import Gf, Sdf, UsdPhysics
from omni.isaac.core.utils.stage import add_reference_to_stage

# Custom Classes
from managerv2 import Manager
#from workstationv2 import Workstation
from controllersv2 import ForceController
from views import View
#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims.rigid_prim import RigidPrim 
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path, get_prim_children, get_prim_path, delete_prim
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
    add_reference_to_stage(usd_path=usd_path, prim_path=work_path+"/object")
    object_parent = world.scene.add(GeometryPrim(prim_path = work_path+"/object", name="object"))
    l = get_prim_children(object_parent.prim)
    
    object_prim = RigidPrim(prim_path= get_prim_path(l[0]))
    mass= object_prim.get_mass()

    return object_parent, mass

    

if __name__ == "__main__":
    n = len(sys.argv)
    
    # Directories
    json_directory = []
    grippers_directory = []
    objects_directory = []
    output_directory = []
    
    for i in range(1,n):
        tmp = sys.argv[i].split('=')
        if tmp[0] == "json":
            json_directory = tmp[1]
        if tmp[0] == "grippers":
            grippers_directory = tmp[1]
        if tmp[0] == "objects":
            objects_directory = tmp[1]
        if tmp[0] == "output":
            output_directory = tmp[1]

    if len(json_directory)== 0 or not os.path.exists(json_directory):
        raise ValueError("Json directory not given correctly")
    elif len(grippers_directory)== 0 or not os.path.exists(grippers_directory):
        raise ValueError("Grippers directory not given correctly")
    elif len(objects_directory)== 0 or not os.path.exists(objects_directory):
        raise ValueError("Objects directory not given correctly")
    elif len(output_directory)== 0 or not os.path.exists(output_directory): 
        raise ValueError("Output directory not given correctly")

    # Hyperparameters
    num_w = 500
    test_time = 5
    fall_threshold = 2 #Just for final print (Not in json)
    slip_threshold = 1 #Just for final print (Not in json)

    #Debugging
    render = False

    #Load json files 
    json_files = [pos_json for pos_json in os.listdir(json_directory) if pos_json.endswith('.json')]
    
    for j in json_files:
        out_path = os.path.join(output_directory,j)

        if(os.path.exists(out_path)):
            continue
        # Initialize Manager
        manager = Manager(os.path.join(json_directory,j), grippers_directory, objects_directory)   
        if manager.gripper == "Allegro":
            continue
        elif manager.gripper=="shadow_hand":
            continue
        #initialize World 
        world = World(set_defaults = False)
        #world.scene.add_default_ground_plane(-1)

        #Create initial Workstation
        work_path = "/World/Workstation_0"
        work_prim = define_prim(work_path)

        #Contact names for collisions
        contact_names = []
        for i in manager.c_names:
            contact_names.append(work_path[:-1]+"*"+"/gripper/" +  i)

        #Initialize Workstation
        robot, T_EF = import_gripper(work_path, manager.gripper_path,manager.EF_axis)
        object_parent, mass = import_object(work_path, manager.object_path)
        
        #Clone
        offsets = np.asarray([0,0,-1])
        #offsets = np.zeros((num_w-1,3)) *offsets
        cloner = GridCloner(spacing = 1)
        target_paths = []
        for i in range(num_w):
             target_paths.append(work_path[:-1]+str(i))
        #print(target_paths)
        cloner.clone(source_prim_path = "/World/Workstation_0", prim_paths = target_paths,
                     copy_from_source = True, replicate_physics = True, base_env_path = "/World",
                     root_path = "/World/Workstation_")

        viewer = View(work_path,contact_names,num_w, manager,world, test_time, mass)

        

        #Reset World and create set first robot positions
        
        #world.scene.add(viewer.grippers)
        world.reset()
        #world.initialize_physics()

        # Translate DoFs and get new jobs (must be done after reset)
        manager.translate_dofs(robot.dof_names)
        viewer.dofs, viewer.current_poses, viewer.current_job_IDs = viewer.get_jobs(num_w)

        # Set desired physics_dt
        physicsContext = world.get_physics_context()
        world.reset()
        physicsContext.set_physics_dt(manager.physics_dt)
        physicsContext.enable_gpu_dynamics(True)
        physicsContext.set_gravity(0)

        viewer.grippers.initialize(world.physics_sim_view)
        viewer.objects.initialize(world.physics_sim_view)
        viewer.post_reset()

            
        #Run Sim
        with tqdm(total=len(manager.completed)) as pbar:
            while not all(manager.completed):
                world.step(render=render) # execute one physics step and one rendering step if not headless

                if pbar.n != np.sum(manager.completed):
                    pbar.update(np.sum(manager.completed)-pbar.n)
        
        world.pause()
        #Save new json with results
        manager.save_json(out_path)
        world.clear_all_callbacks()
        world.clear()
        manager.report_results(fall_threshold,slip_threshold)

    simulation_app.close() # close Isaac Sim
