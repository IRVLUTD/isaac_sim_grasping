import numpy as np


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
    sphere1 = create_sphere("Sphere1", [0, 0, 0], 1.0)
    sphere2 = create_sphere("Sphere2", [0.1, 0, 0], 1.0)
    sphere3 = create_sphere("Sphere3", [0, 0.1, 0], 1.0)

    world.reset()

    sphere1.initialize(world.physics_sim_view)
    sphere2.initialize(world.physics_sim_view)
    sphere3.initialize(world.physics_sim_view)

    body_found = False

    # Simulation loop (optional for visualization or further simulation)
    while(True):  # Simulate for 100 frames
        world.step(render=True)
        
        if world.is_playing():
            # Check overlaps between each pair of spheres
            meshes = ["/World/Sphere1", "/World/Sphere2", "/World/Sphere3"]
            for i in range(len(meshes)):
                check_mesh_overlap(meshes[i])
        #world.pause()

    # Clean up
    simulation_app.close()
