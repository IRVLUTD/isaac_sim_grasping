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

#World Imports
from omni.isaac.core import World
from omni.isaac.core.utils.prims import define_prim
from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import Cloner interface
from pxr import Gf, Sdf, UsdPhysics
from omni.isaac.core.utils.stage import add_reference_to_stage, open_stage, save_stage


#Omni Libraries
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims.rigid_prim import RigidPrim, RigidPrimView    
from omni.isaac.core.prims.geometry_prim import GeometryPrim
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import print_stage_prim_paths, traverse_stage
from omni.isaac.core.utils.transformations import pose_from_tf_matrix, tf_matrix_from_pose, get_world_pose_from_relative
from omni.isaac.core.utils.bounds import compute_obb, create_bbox_cache, compute_aabb
import omni.kit.commands
from omni.isaac.core.materials import PhysicsMaterial
from pxr import Gf, Sdf


if __name__ == "__main__":
    # Directories
    objects_directory = "/home/felipe/Downloads/YCB_usds"

    #Debugging
    render = False

    #Load json files 
    objs =  os.listdir(objects_directory)
    print(objs)

    density = 1000
    
    world = World()
    with tqdm(total=len(objs)) as pbar:
        for j in objs:      
            #print(omni.kit.commands.get_command_doc('AddPhysicsComponent'))
            #print(omni.kit.commands.get_command_doc('CreateAndBindMdlMaterialFromLibrary'))
            
            
            #Instanceable mesh
            path = os.path.join(objects_directory,j,'instanceable_meshes.usd')
            open_stage(path)

            object = get_prim_at_path(j)
            it = traverse_stage()

            for i in it:
                p2 = str(i.GetPath())
                
                print(i.GetPath())
                if str(i.GetPath())[-6:] == "Shader":
                    print("SHADER")
                    omni.kit.commands.execute('ChangeProperty',
                        prop_path=str(i.GetPath())+".inputs:diffuse_texture",
                        value='./materials/texture_map.png',
                        prev=' ')


                if str(i.GetPath())[-17:] == "collisions/mesh_0":
                    '''Collision Mesh '''
                    print("Collision Mesh")
                    omni.kit.commands.execute('SetStaticCollider',
                        path=str(i.GetPath()),
                        approximationShape = 'convexDecomposition')
                    
                    omni.kit.commands.execute('ChangeProperty',
                        prop_path=str(i.GetPath())+".physxConvexDecompositionCollision:maxConvexHulls",
                        value=128,
                        prev=32)
                    
                    MeshAPI= UsdPhysics.MeshCollisionAPI.Get(world.stage,p2)
                    x = MeshAPI.GetApproximationAttr().Get()
                    print('Approx', x)

                    omni.kit.commands.execute('AddPhysicsComponent',
                          usd_prim=i,
                          component='PhysicsMassAPI')
                    
                    omni.kit.commands.execute('ChangeProperty',
                        prop_path=str(i.GetPath())+".physics:density",
                        value=100.0,
                        prev=0.0)
                    

                    MassAPI = UsdPhysics.MassAPI.Get(world.stage, p2) 
                    #MassAPI.CreateMassAttr(mass)
                    og_mass = MassAPI.GetMassAttr().Get()
            save_stage(path)

            #Object.usd
            path = os.path.join(objects_directory,j, j +'.usd')
            open_stage(path)
            
            object = get_prim_at_path(j)
            base = get_prim_at_path(j +'/base_link')

            it = traverse_stage()
            c=0 
            for i in it:
                print(i)
                if c == 0:
                    world.stage.SetDefaultPrim(i)
                    omni.kit.commands.execute('RemovePhysicsComponent',
                                usd_prim=i,
                                component='PhysicsArticulationRootAPI')
                if c == 1:
                    omni.kit.commands.execute('RemovePhysicsComponent',
                          usd_prim=i,
                          component='PhysicsMassAPI')
                c+=1


            
            save_stage(path)
            pbar.update(1)
            

    simulation_app.close() # close Isaac Sim
