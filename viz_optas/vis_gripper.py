import os
import argparse
import numpy as np
import optas
from optas.visualize import Visualizer

from utils_viz import (
    get_urdf_path,
    convert_aligned_to_gripper_pose,
)


def show_frame(vis, RT, alpha: float = 1.0, line_width: float = 1.0):

    origin = RT[:3, 3]
    frame = np.eye(3)
    frame_new = RT[:3, :3] @ frame + origin.reshape((3, 1))

    vis.line(
        start=origin,
        end=frame_new[:, 0],
        rgb=[1.0, 0.0, 0.0],
        alpha=alpha,
        linewidth=line_width,
    )
    vis.line(
        start=origin,
        end=frame_new[:, 1],
        rgb=[0.0, 1.0, 0.0],
        alpha=alpha,
        linewidth=line_width,
    )
    vis.line(
        start=origin,
        end=frame_new[:, 2],
        rgb=[0.0, 0.0, 1.0],
        alpha=alpha,
        linewidth=line_width,
    )


def make_args():
    parser = argparse.ArgumentParser(
        description="Generate viz of aligned grippers with axes", add_help=True
    )
    parser.add_argument(
        "-g",
        "--gripper",
        type=str,
        required=True,
        help="Gripper name",
    )
    parser.add_argument(
        "-l",
        "--linewidth",
        type=float,
        default=3,
        help="Line width for showing the gripper axes",
    )
    args = parser.parse_args()
    return args


if __name__ == "__main__":
    args = make_args()
    gripper_name = args.gripper
    line_width = args.linewidth
    assert line_width > 0
    gripper_urdf_root = "./data/grippers"
    gripper_urdf_path = os.path.join(gripper_urdf_root, get_urdf_path(gripper_name))
    robot_model = optas.RobotModel(urdf_filename=gripper_urdf_path)

    vis = Visualizer(
        background_color=(0.9, 0.9, 0.9), camera_position=[-0.5, -0.5, 0.75]
    )

    # specify pose as (xyz) position and (xyzw) quaternion
    aligned_pose = [0, 0, 0, 0, 0, 0, 1]
    target_gripper_pose = convert_aligned_to_gripper_pose(
        aligned_pose, target_gripper=gripper_name
    )

    # Custom gripper width for better viz of Axes
    if gripper_name == "franka_panda":
        # qval = robot_model.get_random_joint_positions()
        qval = [0.03, 0.03]
    elif gripper_name == "wsg_50":
        qval = [-0.03, 0.03]
    else:
        qval = None

    vis.robot(
        robot_model,
        base_position=target_gripper_pose[:3],
        base_orientation=target_gripper_pose[3:],
        q=qval,
    )

    show_frame(vis, np.eye(4) * 0.3, line_width=line_width)
    print("Rendered....close window to exit.")
    vis.start()
