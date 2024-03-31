import argparse
import numpy as np

from renderer import Renderer


def make_args():
    parser = argparse.ArgumentParser(
        description="Generate grid and spawn objects", add_help=True
    )
    parser.add_argument(
        "-m",
        "--model_dir",
        type=str,
        help="3D models data directory",
        required=True,
    )
    parser.add_argument(
        "-g",
        "--graspit_grasp_dir",
        type=str,
        help="Original Graspit Grasp json datadir",
        required=True,
    )
    parser.add_argument(
        "-f",
        "--filtered_grasp_dir",
        type=str,
        help="Filtered Grasp json datadir",
        required=True,
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = make_args()
    model_dir = args.model_dir
    graspit_grasp_dir = args.graspit_grasp_dir
    filtered_grasp_dir = args.filtered_grasp_dir

    num_grasps = 5
    num_grippers_first_row = 6
    grid_delta = 0.6
    resolution = [1920, 1080]

    gripper_model_pairs = {
        "sawyer": "Remington_TStudio_Hair_Dryer",
        "h5_hand": "DPC_Handmade_Hat_Brown",
        "franka_panda": "ACE_Coffee_Mug_Kristen_16_oz_cup",
        "wsg_50": "006_mustard_bottle",
        "fetch_gripper": "Markings_Letter_Holder",
        "robotiq_3finger": "Room_Essentials_Fabric_Cube_Lavender",
        "Barrett": "Elephant",
        "jaco_robot": "Chefmate_8_Frypan",
        "Allegro": "Mens_ASV_Billfish_Boat_Shoe_in_Dark_Brown_Leather_zdHVHXueI3w",
        "shadow_hand": "010_potted_meat_can",
        "HumanHand": "Crosley_Alarm_Clock_Vintage_Metal",
    }

    num_rows = len(gripper_model_pairs)
    num_cols = num_grasps
    x_grid_delta = grid_delta - 0.2

    camera_focus = [
        (num_rows - 1) / 2 * x_grid_delta,
        (num_cols - 1) / 3 * grid_delta,
        0.1,
    ]
    camera_postn = [(num_rows - 1) / 2 * x_grid_delta - 0.1, -4.7, 2]

    print("Cam Position", camera_postn)
    print("Cam Focus:", camera_focus)

    vizzer = Renderer(
        model_dir,
        graspit_grasp_dir,
        filtered_grasp_dir,
        num_grasps,
        using_filtered=True,
        resolution=resolution,
        grid_delta=grid_delta,
        cam_pos=camera_postn,
        cam_focus=camera_focus,
    )

    start_pos = np.array([0, 0, 0])
    counter = 0

    for gripper, model in gripper_model_pairs.items():
        x_offset = counter * x_grid_delta
        y_offset = 0
        curr_pos = start_pos + np.array([x_offset, y_offset, 0])
        vizzer.render(gripper, model, curr_pos)
        counter += 1

    print("Rendered the grasps....Close the window to exit!")
    vizzer.vis.start()
