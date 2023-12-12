import argparse
import os
import trimesh
from functools import reduce
parser = argparse.ArgumentParser(description='Prepare Google Scan-3D objects URDFs for Pybullet/IsaacSim',
    usage="\
    python google_prepare_urdf.py -f /PATH/TO/GOOGLE_DATA_FOLDER/ -m model_list.txt'")

parser.add_argument('-m', '--models_file', type=str, required=True, default='model_list.txt', help="List of object names to export to URDF")
parser.add_argument('-f', '--google_dir', type=str, required=True, default='/mnt/Data/GoogleScannedObjects/SampleObjects/', help='Path to GoogleScan3d Models Directory.')


def get_urdf_file(ycb_model: str, mesh_path: str, mass: float) -> str:
    URDF_TEMPLATE = '''<?xml version='1.0' ?>
<robot name="{ycb_model_name}">
    <link name="base_link">
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{ycb_model_mesh_path}" scale="1.0 1.0 1.0"/>
            </geometry>
            <material name="texture">
                <color rgba="1.0 1.0 1.0 1.0" />
            </material>
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <geometry>
                <mesh filename="{ycb_model_mesh_path}" scale="1.0 1.0 1.0"/>
            </geometry>
        </collision>
        <inertial>
            <!-- <mass value="{mass_value}"/> -->
            <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
        </inertial>
    </link>
</robot>'''
    return URDF_TEMPLATE.format(ycb_model_name=ycb_model, ycb_model_mesh_path=mesh_path, mass_value=mass)


def main(args):

    if args.models_file and not os.path.isfile(args.models_file):
        print(f"File does not exist: {args.models_file}")
        exit(0)

    if not args.google_dir:
        print(f"models list not specified")
        exit(0)

    if not os.path.isdir(args.google_dir):
        print(f"models directory (containing meshes): {args.google_dir} is incorrect")
        exit(0)

    with open(args.models_file) as f:
        model_names = f.read().splitlines()

    DENSITY = 500
    for model in model_names:
        model_dir = os.path.join(args.google_dir, model) 
        mesh_p =  os.path.join("meshes", "model.obj")
        obj = trimesh.load(os.path.join(model_dir, mesh_p))
        _, extents = trimesh.bounds.oriented_bounds(obj)
        volume = reduce(lambda x, y: x*y, extents)
        mass = volume * DENSITY
        print(f"{model}: BBox {extents} ; VOL={volume} ; MASS = {volume * DENSITY} Kg")
        urdf_content = get_urdf_file(model, mesh_p, mass)
        with open(os.path.join(model_dir, f"{model}.urdf"), "w") as outf:
            outf.write(urdf_content)
        # NOTE: We write another file with same content, but its titled model.urdf similar to the model.sdf in each object's folder
        with open(os.path.join(model_dir, "model.urdf"), "w") as outf:
            outf.write(urdf_content)


if __name__ == '__main__':
    main(parser.parse_args())