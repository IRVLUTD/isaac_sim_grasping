import os
import pathlib
import numpy as np

import optas
from optas.visualize import Visualizer
from transforms3d.quaternions import mat2quat

from utils_viz import (
    parse_grasps,
    parse_grasps_filtered,
    get_urdf_path,
    adjust_dofs,
)


class Renderer:

    def __init__(
        self,
        model_dir,
        graspit_grasp_dir,
        filtered_grasp_dir,
        num_grasps=3,
        using_filtered=True,
        grid_delta=0.6,
        bg_color=(0.8, 0.8, 0.8),
        resolution=[3840, 2160],
        cam_pos=(-1, -1, 1),
        cam_focus=(0, 0, 0),
    ) -> None:
        self.model_dir = model_dir
        self.graspit_grasp_dir = graspit_grasp_dir
        self.filtered_grasp_dir = filtered_grasp_dir
        self.num_grasps = num_grasps
        self.using_filtered = using_filtered
        self.grid_delta = grid_delta  # both in x and y

        self.vis = Visualizer(
            window_size=resolution,
            background_color=bg_color,
            camera_position=cam_pos,
            camera_focal_point=cam_focus,
        )
        self.gripper_urdf_root = "./data/grippers"

    def _get_urdf_path(self, gripper_name):
        return os.path.join(self.gripper_urdf_root, get_urdf_path(gripper_name))

    def _get_graspit_grasps_file(self, gripper_name, model_name):
        return os.path.join(
            self.graspit_grasp_dir, gripper_name, f"{gripper_name}-{model_name}.json"
        )

    def _get_filtered_grasps_file(self, gripper_name, model_name):
        return os.path.join(
            self.filtered_grasp_dir, gripper_name, f"{gripper_name}-{model_name}.json"
        )

    def _get_object_files(self, model_name):
        model_data_path = os.path.join(self.model_dir, model_name)

        if model_name in {
            "003_cracker_box",
            "004_sugar_box",
            "005_tomato_soup_can",
            "006_mustard_bottle",
            "007_tuna_fish_can",
            "008_pudding_box",
            "009_gelatin_box",
            "010_potted_meat_can",
            "011_banana",
            "021_bleach_cleanser",
            "024_bowl",
            "025_mug",
            "035_power_drill",
            "037_scissors",
            "052_extra_large_clamp",
        }:
            obj_mesh_f = os.path.join(model_data_path, "textured_simple.obj")
            texture_f = os.path.join(model_data_path, "texture_map.png")
        else:
            obj_mesh_f = os.path.join(model_data_path, "meshes", "model.obj")
            texture_f = os.path.join(
                model_data_path, "materials", "textures", "texture.png"
            )
        return obj_mesh_f, texture_f

    def render(self, gripper_name, model_name, start_position):
        num_grasps = self.num_grasps
        graspit_grasp_file = self._get_graspit_grasps_file(gripper_name, model_name)
        obj_mesh_f, texture_f = self._get_object_files(model_name)
        gripper_urdf_path = self._get_urdf_path(gripper_name)
        robot_model = optas.RobotModel(urdf_filename=gripper_urdf_path)

        # parse grasps
        if self.using_filtered:
            filtered_grasp_file = self._get_filtered_grasps_file(
                gripper_name, model_name
            )
            RT_grasps, dofs = parse_grasps_filtered(
                filtered_grasp_file,
                num_grasps,
                sort_key="fall_time",
            )
        else:
            RT_grasps, dofs = parse_grasps(graspit_grasp_file)

        start_position = np.asarray(start_position)
        start_y = start_position[1]
        obj_positions_y = np.linspace(
            start=start_y,
            stop=start_y + num_grasps * self.grid_delta,
            num=num_grasps,
            endpoint=False,
        )
        for i in range(num_grasps):
            offset = np.array([0, obj_positions_y[i], 0])
            obj_position = start_position + offset
            self.vis.obj(
                obj_mesh_f,
                png_texture_filename=texture_f,
                position=obj_position,
                orientation=[0, 0, 0],
                euler_degrees=True,
            )
            # draw robot
            q = adjust_dofs(gripper_name, dofs[i])
            RT_g = RT_grasps[i]
            position = RT_g[:3, 3] + obj_position
            quat = mat2quat(RT_g[:3, :3])
            orientation = [quat[1], quat[2], quat[3], quat[0]]
            self.vis.robot(
                robot_model, base_position=position, base_orientation=orientation, q=q
            )
