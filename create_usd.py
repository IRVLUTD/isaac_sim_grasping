import os
import argparse
from tqdm import tqdm

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})
import omni.kit.commands


def import_urdf(urdf_path, dest_path):
    """ Import URDF from manager.grippers dictionary

    Args: None
    """
    _status, _config = omni.kit.commands.execute("URDFCreateImportConfig")
       
    _config.merge_fixed_joints = True
    _config.convex_decomp = True
    _config.import_inertia_tensor = False
    _config.fix_base = False
    _config.distance_scale = 1
    _config.make_instanceable = True
    _config.density = 1.0

    result = omni.kit.commands.execute(
            "URDFParseAndImportFile", 
            urdf_path=urdf_path, 
            import_config=_config, 
            dest_path=dest_path
    )
    return


def make_parser():
    parser = argparse.ArgumentParser(description='Prepare Google Scan-3D objects USDs for IsaacSim. Need to run it with IsaacSim Python Env',
    usage="\
    ./python.sh ~/PATH_TO_SCRIPT/save_object_usd.py -f Path_To_GoogleData -m model_list.txt'")

    parser.add_argument('-m', '--models_file', type=str, required=True, help="List of object names to export to URDF")
    parser.add_argument('-f', '--object_dir', type=str, required=True, help='Path to GoogleScan3d or other Models Directory.')
    parser.add_argument('-o', '--output_dir', type=str, required=True, help='Path to Output Directory with USDs.')
    return parser


if __name__ == "__main__":
    parser = make_parser()
    args = parser.parse_args()

    if args.models_file and not os.path.isfile(args.models_file):
        print(f"File does not exist: {args.models_file}")
        exit(0)

    if not os.path.isdir(args.object_dir):
        print(f"models directory (containing meshes): {args.object_dir} is incorrect")
        exit(0)

    if not os.path.isdir(args.output_dir):
        print(f"models directory (containing meshes): {args.output_dir} is incorrect or does not exist")
        exit(0)

    with open(args.models_file) as f:
        model_names = f.read().splitlines()
   
    for model in tqdm(model_names):
        # print(model)
        # model = model_names[0]
        model_p = os.path.join(args.object_dir, model)
        usd_dir = os.path.join(args.output_dir, model)
        os.makedirs(usd_dir, exist_ok=True)

        org_texture_p = os.path.join(model_p, "materials", "textures", "texture.png")
        # This line is needed for the GoogleScannedObjects models dir, might not be needed for others!
        symlink_new_texture_p = os.path.join(model_p, "meshes", "texture.png")
        if not os.path.exists(symlink_new_texture_p):
            os.symlink(org_texture_p, symlink_new_texture_p)

        usd_p = os.path.join(usd_dir, f"{model}.usd")
        urdf_p  = os.path.join(model_p, f"{model}.urdf")
        
        import_urdf(urdf_path=urdf_p, dest_path=usd_p)

    simulation_app.close()
