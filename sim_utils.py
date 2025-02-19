import numpy as np
from omni.isaac.core.controllers import BaseController
import os
import json

class GenericController(BaseController):
    """
    Base class for all controller types. This class provides common functionality 
    shared across different controllers. It can also be used 
    """

    def __init__(self, gripper_name='', grippers= None, test_time = 0):
        """
        Initialize the generic controller using Isaac Sim BaseController. 

        Parameters: 
            gripper_name: Gripper ID.
            grippers: ArticulationView class containing all the grippers to control.
            test_time: Time of test to perform
        """
        self.name = 'Controller_' + gripper_name
        super().__init__(name=self.name)
        #Load controller info.json
        c_path = os.path.dirname(os.path.abspath(__file__))
        controller_json =  os.path.join(c_path,"grippers/controller_info.json")
        with open(controller_json) as fd:
            self.gripper_dict = json.load(fd)
        self.label = 'Unknown'  # A label to identify the controller version or type
        self.grippers = grippers
        self.total_test_time = test_time

    def forward(self, time):
        """
        Method to be overridden by subclasses. This method computes actions 
        for the grippers based on the current time.

        Arguments:
        - time: Current clock time of every workstation in the simulation (W x 1 numpy array)
        """
        raise NotImplementedError("Subclass must implement abstract method")
    
class GenericTest():
    """
    Base class for all test types. This class provides common functionality 
    shared across different tests.
    """

    def __init__(self):
        """ Initialize the test controller. It must always contain a label with its tag
        """
        self.label = "Unknown"
        return
    
    def failure_condition(self, init_pos, init_rot, indices):
        ''' Method to be overridden by subclasses. Function to evaluate if the test has failed. 
        W = number of workstations to check

        Arguments:
        - init_pos: Initial positions of objects (numpy W x 3 array)
        - init_rot: Initial rotation of objects (numpy W x 4 array)
        - indices: numpy array with ID indices of objects to check 

        Return:
        - finish_ind: array with all the indices within indices that failed the test
        '''
        return NotImplementedError("Subclass must implement abstract method")
    
    def setup_condition(self, init_pos, init_rot, indices):
        ''' Function to test if the objects advance from the setup phase. Default lets every object pass the setup phase, use ifthere is no desire for such a phase.

        Arguments:
        - init_pos: Initial positions of objects (numpy W x 3 array)
        - init_rot: Initial rotation of objects (numpy W x 4 array)
        - indices: numpy array with ID indices of objects to check 

        Return:
        - setup_ind: array with all the indices within indices that passed the setup phase
        '''
        setup_ind = indices
        return setup_ind
    
    def test_step(self, current_times):
        ''' Method to be overridden by subclasses. Step function to run at every physics step. 

        Arguments:
        - current_times: Current clock time of every workstation in the simulation (W x 1 numpy array)
        '''

        return NotImplementedError("Subclass must implement abstract method")

def import_urdf(manager, job):
    """ Import URDF from manager.grippers dictionary

    Args: -- DEPRICATED
    """
    import omni.kit.commands

    #Adding a .urdf file
    urdf_interface = _urdf.acquire_urdf_interface()
    # Set the settings in the import config
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = True
    import_config.import_inertia_tensor = True
    import_config.default_drive_strength = 1047.19751
    import_config.default_position_drive_damping = 52.35988
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0

    urdf_path = manager.object_dict[job['object_id']]
    
    result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=urdf_path,import_config=import_config,)
    return

def spherical_raycasts(origin, num_rays=100, max_distance=1000.0, mesh_dirs = None):
    all_hits = []
    
    # Generate directions using spherical coordinates
    latitude = np.linspace(0, np.pi, num_rays//2)  # angle from z-axis
    longitude = np.linspace(0, 2 * np.pi, num_rays)  # angle in x-y plane
    
    for p in latitude:
        for t in longitude:
            direction = np.array([
                np.sin(p) * np.cos(t),
                np.sin(p) * np.sin(t),
                np.cos(p)
            ])
            
            hits = raycast_all(origin, direction, max_distance, mesh_dirs)
            if hits:
                all_hits.append(hits)
                if p in [0, np.pi]:
                    break
    return all_hits

def raycast_all(origin, direction, max_distance, mesh_dirs = None):
    import omni.physx
    from pxr import Gf
    hits = []
    # Add collision meshes filters here and conditional stop
    def report_raycast(hit):
        if mesh_dirs is not None:
            if hit.rigid_body in mesh_dirs:
                hits.append({
                    'rigid_body': hit.rigid_body,
                    'direction' : direction,
                    'position': hit.position
                })
                return False
            return True  # Continue to report more
        else: 
            hits.append({
                    'rigid_body': hit.rigid_body,
                    'direction' : direction,
                    'position': hit.position
                })
            return True

    omni.physx.get_physx_scene_query_interface().raycast_all(
        Gf.Vec3d(*origin), 
        Gf.Vec3d(*direction), 
        max_distance,
        report_raycast
    )
    
    return hits

def create_sphere(name, position, radius):
    from omni.isaac.core.objects import DynamicSphere
    sphere = DynamicSphere(prim_path= f"/World/{name}",
                            color=np.array([1.0, 1.0, 0.0]), 
                            mass=0.01,radius = radius,
                            name = name , translation = position)
    return sphere

def add_light():
    import omni.isaac.core.utils.prims as prim_utils
    light_1 = prim_utils.create_prim(
        f"/World/Light",
        "DomeLight",
        attributes={
            "inputs:intensity": 1000
        }
    )
    return light_1


# Function to check for mesh overlap
def check_mesh_overlap(mesh_path):
    from pxr import PhysicsSchemaTools
    from omni.physx import get_physx_scene_query_interface
    # Encode the SdfPath to integers for PhysX query
    encoded_path = PhysicsSchemaTools.encodeSdfPath(mesh_path)

    # Overlap detection function
    def report_overlap(hit):
        if hit.rigid_body != mesh_path:
            #print(f"Overlap detected between {mesh_path} and {hit.rigid_body}")
            return False
        return True  

    # Perform the overlap query
    num_hits = get_physx_scene_query_interface().overlap_shape(encoded_path[0], encoded_path[1], report_overlap)
    #print("Hits: ", num_hits)
    return num_hits > 1 # Always counts itself

def overlap_check_thread(mesh_path, result, timings):
    start_time = time.time()
    result[mesh_path] = check_mesh_overlap(mesh_path)
    end_time = time.time()
    timings[mesh_path] = end_time - start_time


if __name__ == "__main__":
    # utilities tester using simulator
    from omni.isaac.kit import SimulationApp
    config= {
        "headless": False,
        'max_bounces':0,
        'fast_shutdown': True,
        'max_specular_transmission_bounces':0
        }
    simulation_app = SimulationApp(config) # we can also run as headless.
    import omni.usd
    from omni.isaac.core import World
    from pxr import Gf
    import omni.physx
    import omni.isaac.core.utils.prims as prim_utils
    from omni.isaac.core.objects import DynamicSphere, GroundPlane, VisualSphere
    import carb
    import threading
    from pxr import PhysicsSchemaTools
    from omni.physx import get_physx_scene_query_interface
    import time
    import random

    # Get the world
    world = World()
    # Load a scene or add objects
    stage = omni.usd.get_context().get_stage()
    # Wait for the world to be ready
    world.reset()

    light_1 = add_light()
    physicsContext = world.get_physics_context()
    #physicsContext.set_solver_type("PGS")
    physicsContext.enable_gpu_dynamics(True)
    physicsContext.enable_stablization(True)
    physicsContext.set_gravity(0)

    
    # Example: Add a ground plane for the rays to potentially hit
    # Create three spheres (treated as meshes for this example)
    # Create 100 spheres with random positions
    spheres = []
    for i in range(100):
        x = random.uniform(-1, 1)  # Random x position
        y = random.uniform(-1, 1)  # Random y position
        z = random.uniform(-1, 1)    # Random z position, above ground to avoid immediate collision
        sphere = create_sphere(f"Sphere{i}",  [x, y, z], 0.5)
        spheres.append(sphere)

    
    world.reset()

    for sphere in spheres:
        sphere.initialize(world.physics_sim_view)


    body_found = False

    # Simulation loop (optional for visualization or further simulation)
    while(True):  # Simulate for 100 frames
        world.step(render=True)
        if world.is_playing():
            start_time = time.time()
            # Check overlaps between each pair of spheres
            meshes = [f"/World/Sphere{i}" for i in range(100)]
            threads = []
            results = {}
            timings = {}

            for i in range(len(meshes)):
                check_mesh_overlap(meshes[i])

            end_time = time.time()

            for mesh, overlap in results.items():
                #print(f"Overlap for {mesh}: {'Detected' if overlap else 'Not detected'}")
                #print(f"Time taken for {mesh}: {timings[mesh]:.6f} seconds")
                break

            print(f"Total time for all checks: {end_time - start_time:.6f} seconds")
        #world.pause()

    # Clean up
    simulation_app.close()
