#External Libraries
import numpy as np
from tqdm import tqdm
import os
import argparse
import sys
import time

st_path = os.path.dirname(os.path.abspath(__file__))

def make_parser():
    """ Input Parser """
    parser = argparse.ArgumentParser(description='Standalone script for grasp filtering.')
    parser.add_argument('--headless', type=bool, help='Running Program in headless mode',
                        default=False, action = argparse.BooleanOptionalAction)
    parser.add_argument('--dof_given', type=bool, help='DoFs of grasps are given in .json files',
                        default=False, action = argparse.BooleanOptionalAction)
    parser.add_argument('--view_mode', type=bool, help='DoFs of grasps are given in .json files',
                        default=False, action = argparse.BooleanOptionalAction)
    parser.add_argument('--force_reset', type=bool, help='Force Reset of Isaac Sim',
                        default=False, action = argparse.BooleanOptionalAction)
    parser.add_argument('--json_dir', type=str, help='Directory of Grasp Information', default=os.path.join(st_path,'grasps'))
    parser.add_argument('--gripper_dir', type=str, help='Directory of Gripper urdf/usd', default=os.path.join(st_path,'grippers'))
    parser.add_argument('--objects_dir', type=str, help='Directory of Object usd', default=os.path.join(st_path,'objects'))
    parser.add_argument('--output_dir', type=str, help='Output directroy for filterd grasps', default='')
    parser.add_argument('--num_w', type=int, help='Number of Workstations used in the simulation', default=150)
    parser.add_argument('--device', type=int, help='Gpu to use', default=0)
    parser.add_argument('--test_time', type=int, help='Total time for each grasp test', default=3)
    parser.add_argument('--print_results', type=bool, help='Enable printing of grasp statistics after filtering a document',
                         default=False, action = argparse.BooleanOptionalAction)
    parser.add_argument('--controller', type=str,
                        help='Gripper Controller to use while testing, should match the controller dictionary in the Manager Class',
                        default='default')
    parser.add_argument('--test_type', type=str, help='Test type to perform', default='default')
    parser.add_argument('--/log/outputStreamLevel', type=str, help='isaac sim logging arguments', default='', required=False)
    return parser

def import_gripper(work_path,usd_path, EF_axis):
        """ Imports Gripper to World

        Args:
            work_path: prim_path of workstation
            usd_path: path to .usd file of gripper
            EF_axis: End effector axis needed for proper positioning of gripper
        
        """
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
        gripper_pose= pose_from_tf_matrix(T_EF.astype(float))
        
        # Adding Robot usd
        add_reference_to_stage(usd_path=usd_path, prim_path=work_path+"/gripper")
        robot = world.scene.add(Articulation(prim_path = work_path+"/gripper", name="gripper",
                            position = gripper_pose[0], orientation = gripper_pose[1]))
        robot.set_enabled_self_collisions(False)
        return robot, T_EF

def import_object(work_path, usd_path):
    """ Import Object .usd to World

    Args:
        work_path: prim_path to workstation
        usd_path: path to .usd file of object
    """
    add_reference_to_stage(usd_path=usd_path, prim_path=work_path+"/object")
    object_parent = world.scene.add(GeometryPrim(prim_path = work_path+"/object", name="object"))
    l = get_prim_children(object_parent.prim)
    #print(l)

    prim = get_prim_at_path(work_path+"/object"+ '/base_link/collisions/mesh_0')
    '''
    MassAPI = UsdPhysics.MassAPI.Get(world.stage, prim.GetPath())
    try: 
        og_mass = MassAPI.GetMassAttr().Get()
        if og_mass ==0:
            og_mass = 1
            print("Failure reading object mass, setting to default value of 1 kg.")
    except:
        og_mass = 1
        print("Failure reading object mass, setting to default value of 1 kg.")

    # Create Rigid Body attribute
    og_mass = 1
    '''

    object_prim = RigidPrim(prim_path= get_prim_path(l[0]))

    mass= 1 #Deprecated use of mass for gravity 

    return object_parent, mass


if __name__ == "__main__":
    #Parser
    parser = make_parser()
    args = parser.parse_args()
    head = args.headless
    force_reset = args.force_reset
    dof_flag = args.dof_given
    num_w = args.num_w
    test_time = args.test_time
    verbose = args.print_results
    controller = args.controller
    test_type = args.test_type
    view_mode = args.view_mode

    #launch Isaac Sim before any other imports
    from omni.isaac.kit import SimulationApp
    config= {
        "headless": head,
        'max_bounces':0,
        'fast_shutdown': True,
        'max_specular_transmission_bounces':0,
        'physics_gpu': args.device,
        'active_gpu': args.device
        }
    simulation_app = SimulationApp(config) # we can also run as headless.

    # Omniverse imports
    from omni.isaac.core import World
    from omni.isaac.core.utils.prims import define_prim
    from omni.isaac.cloner import GridCloner    # import Cloner interface
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.core.utils.stage import add_reference_to_stage,open_stage, save_stage
    from omni.isaac.core.prims.rigid_prim import RigidPrim 
    from omni.isaac.core.prims.geometry_prim import GeometryPrim
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.core.utils.prims import get_prim_children, get_prim_path, get_prim_at_path
    from omni.isaac.core.utils.transformations import pose_from_tf_matrix


    # Custom Classes
    from managers import Manager
    from views import View
    from sim_utils import add_light

    # Directories
    json_directory = args.json_dir
    grippers_directory = args.gripper_dir
    objects_directory = args.objects_dir
    output_directory = args.output_dir
    
    if not os.path.exists(json_directory):
        raise ValueError("Json directory not given correctly")
    if not os.path.exists(grippers_directory):
        raise ValueError("Grippers directory not given correctly")
    if not os.path.exists(objects_directory):
        raise ValueError("Objects directory not given correctly")
    if output_directory == '':
        output_directory = os.path.join(st_path,"output")
        print("No output directory provided, using default at: ", output_directory)
        if not os.path.exists(output_directory):
            os.makedirs(output_directory)

    world = World(set_defaults = False)
    render = not head

    #Load json files 
    json_files = [pos_json for pos_json in os.listdir(json_directory) if pos_json.endswith('.json')]

    for j in json_files:
        #path to output .json file
        out_path = os.path.join(output_directory,j)

        if(os.path.exists(out_path)): #Skip completed
            continue

        # Initialize Manager
        manager = Manager(os.path.join(json_directory,j), grippers_directory, objects_directory, dof_flag)   
        

        #Create initial Workstation Prim
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
        cloner = GridCloner(spacing = 1)
        target_paths = []
        for i in range(num_w):
             target_paths.append(work_path[:-1]+str(i))
        cloner.clone(source_prim_path = "/World/Workstation_0", prim_paths = target_paths,
                     copy_from_source = True, replicate_physics = True, base_env_path = "/World",
                     root_path = "/World/Workstation_")

        # ISAAC SIM views initialization
        viewer = View(work_path, contact_names, num_w, manager,
                      world, test_time, controller, test_type, 
                      dof_flag, view_mode)

        
        #Reset World and create set first robot positions
        world.reset()

        # Print Robot DoFs
        print(robot.dof_names)
        viewer.dofs, viewer.current_poses, viewer.current_job_IDs = viewer.get_jobs(num_w)
        
        #Debug
        add_light()

        # Set desired physics Context options
        world.reset()
        physicsContext = world.get_physics_context()
        #physicsContext.set_solver_type("PGS")
        physicsContext.set_physics_dt(manager.physics_dt)
        physicsContext.enable_gpu_dynamics(True)
        physicsContext.enable_stablization(True)
        physicsContext.set_gravity(0)
        world.reset()
        
        #Initialize views
        viewer.grippers.initialize(world.physics_sim_view)
        viewer.objects.initialize(world.physics_sim_view)
        viewer.post_reset()

        # Sim info added to manager
        manager.test_type = viewer.test_type
        manager.total_test_time = test_time
        manager.controller = viewer.controller_type

        #world.pause()
        #Run Sim
        with tqdm(total=len(manager.completed)) as pbar:
            while not all(manager.completed):
                #print(mass)
                
                world.step(render=render) # execute one physics step and one rendering step if not headless
                #world.pause()
                if pbar.n != np.sum(manager.completed): #Progress bar
                    pbar.update(np.sum(manager.completed)-pbar.n)
    

        #Save new json with results
        if not view_mode:
            manager.save_json(out_path)
        if (verbose):
            manager.report_results()
        #print("Reseting Environment")

        #Reset World    
        if not force_reset:
            print('Reseting Environment')
            t = time.time()
            world.stop()
            world.clear_physics_callbacks()
            world.clear()
            t = time.time() -t
            print('Reseted, time in seconds: ', t)

        if force_reset:
            os.execl(sys.executable, sys.executable, *sys.argv)
            pass
    
    simulation_app.close() # close Isaac Sim
        
