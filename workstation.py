import numpy as np
import pandas as pd
from manager import Manager
from omni.isaac.urdf import _urdf
import omni.kit.commands
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot

class Workstation():

    def __init__(self, ID, manager, path, world):
        self.manager = manager
        self.job = manager.grasps.iloc[ID]
        self.ID = ID
        self.path = "/" + path # Addition to make absolute path (Used by Isaac Sim)
        self.world = world
        
        self.robot = self._import_gripper()
        self.payload = self._import_object()
        #print(self.path, self.robot.num_dof)
        #self._import_urdf() # Import .urdf file directly\

    def step(self):
        self.job = self.manager.request_job()
        return self.job
    
    def _import_urdf(self):
        """ Import URDF from manager.grippers dictionary

        Args: None
        """
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

        urdf_path = self.manager.object_dict[self.job['object_id']]
        result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=urdf_path,import_config=import_config,)

    def _import_gripper(self):
        usd_path = self.manager.gripper_dict[self.job['gripper']] 
        add_reference_to_stage(usd_path=usd_path, prim_path=self.path+"/gripper_"+str(self.ID))
        robot = self.world.scene.add(Robot(prim_path = self.path+"/gripper_"+str(self.ID), name="gripper_"+str(self.ID)))
        q = self.job['grasps']['pose'][3:]
        reorder = [3, 0, 1, 2]
        q = [q[i] for i in reorder]
        robot.set_local_pose(self.job['grasps']['pose'][:3],q)
        return robot
    
    def _import_object(self):
        usd_path = self.manager.object_dict[self.job["object_id"]]
        add_reference_to_stage(usd_path=usd_path, prim_path=self.path+"/object_"+str(self.ID))
        payload = self.world.scene.add(XFormPrim(prim_path = self.path+"/object_"+str(self.ID), name="object_"+str(self.ID)))
        return payload




if __name__ == "__main__":
    json_path = "/home/felipe/Documents/isaac_sim_grasping/grasp_data/Grasps_dataset.json"
    grippers_path = "/home/felipe/Documents/isaac_sim_grasping/gripper_usd"
    objects_path = "/home/felipe/Documents/isaac_sim_grasping/objects"

    G_manager = Manager(json_path, grippers_path, objects_path)
    w1 = Workstation(G_manager)
    w2 = Workstation(G_manager)

    print(w1.step())
    print(w2.step())
    q = w1.job['grasps']['pose'][3:]
    print(q)
    reorder = [3, 0, 1, 2]
    print(q)
    q = [q[i] for i in reorder]
    print(w2.step())
    print(w2.step())
    print(G_manager.task_pointer)
