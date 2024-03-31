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
        help="Models data directory",
        required=True,
    )
    parser.add_argument(
        "-f",
        "--filtered_grasp_dir",
        type=str,
        help="Filtered Grasp json data directory",
        required=True,
    )
    parser.add_argument(
        "-g",
        "--gripper",
        type=str,
        help="Gripper name",
        required=True,
    )
    parser.add_argument(
        "--model",
        type=str,
        help="Model name",
        required=True,
    )


    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = make_args()
    model_dir = args.model_dir
    filtered_grasp_dir = args.filtered_grasp_dir
    gripper = args.gripper
    model = args.model
    num_grasps = 5
    grid_delta = 0.5
    resolution = [1280, 720]

    num_rows = 1
    num_cols = num_grasps

    camera_focus = [0, 0.1, 0.05]
    camera_postn = [0, -3, 1.5]

    print("Cam Position", camera_postn)
    print("Cam Focus:", camera_focus)

    vizzer = Renderer(
        model_dir,
        filtered_grasp_dir,
        filtered_grasp_dir,
        num_grasps,
        using_filtered=True,
        resolution=resolution,
        grid_delta=grid_delta,
        cam_pos=camera_postn,
        cam_focus=camera_focus,
    )

    curr_pos = np.array([0, 0, 0])
    vizzer.render(gripper, model, curr_pos)
    print("Rendered the grasps....Close the window to exit!")
    vizzer.vis.start()
