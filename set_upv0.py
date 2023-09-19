#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

#External Libraries
import numpy as np

#World Imports
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import omni.kit.commands

# URDF imports  
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.urdf import _urdf

# Object Initiation file
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim





#Initialize world
def init_world(num_w):
    world = World(stage_units_in_meters=1.0)
    #Create initial Workstation
    workstation = create_prim("/World/Workstation", "Xform")
    return world, workstation

def add_urdf():
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
    import_config.import_inertia_tensor = False
    import_config.default_drive_strength = 1047.19751
    import_config.default_position_drive_damping = 52.35988
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0
    import_config.make_instanceable = False

    urdf_path = "/home/felipe/Documents/urdf/fetch_gripper/fetch_gripper.urdf"
    result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=urdf_path,import_config=import_config,)

# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly

def add_usd(usd_path):
    #omni.usd.get_context().open_stage(usd_path)
    usd_added = add_reference_to_stage(usd_path= usd_path, prim_path = "/World/Stuff")



if __name__ == "__main__":
    num_w= 4 # Number of Workstations
    world, workstation = init_world(4)

    #add_urdf()
    world.reset()
    while simulation_app.is_running():
        world.step(render=True) # execute one physics step and one rendering step
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()


    simulation_app.close() # close Isaac Sim