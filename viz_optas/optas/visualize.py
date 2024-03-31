"""! @brief Visualizer class implementation."""

import os
import open3d as o3d

import vtk
from vtkmodules.vtkFiltersSources import (
    vtkCylinderSource,
    vtkSphereSource,
    vtkCubeSource,
)

import casadi as cs
from casadi import DM
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from urdf_parser_py.urdf import Mesh, Cylinder, Sphere

from .models import RobotModel

from .spatialmath import *

from typing import List, Union, Dict
import random


class ActorList:
    def __init__(self):
        self.add_actors = True
        self.actors = []

    def start_adding_actors(self):
        self.add_actors = True

    def stop_adding_actors(self):
        self.add_actors = False

    def append(self, actor):
        if self.add_actors:
            self.actors.append(actor)


class AnimationCallback:
    def __init__(self, traj, duration, iren, ren):
        # Setup class attributes
        self.nsteps = len(traj)
        self.dt = duration / float(self.nsteps - 1)
        self.dt_ms = int(round(self.dt * 1e3))
        self.step = 0
        self.iren = iren
        self.traj = traj
        self.prev = None
        self.ren = ren

    def start(self):
        self.iren.AddObserver("TimerEvent", self.callback)
        self.timer_id = self.iren.CreateRepeatingTimer(self.dt_ms)

    def callback(self, *args):
        # Remove previous actors
        if self.prev is not None:
            for actor in self.prev:
                self.ren.RemoveActor(actor)

        # Create new robot actors
        current = self.traj[self.step]
        for actor in current:
            self.ren.AddActor(actor)

        # Render to window
        self.iren.GetRenderWindow().Render()

        # Reset
        self.step = (self.step + 1) % self.nsteps
        self.prev = current


class VtkPointCloud:

    def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=1e9):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.ScalarVisibilityOff()
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)


class Visualizer:
    """! This class defines the Visualizer for simple visualization."""

    def __init__(
        self,
        window_size: List[int] = [1440, 810],
        background_color: ArrayType = [0.0, 0.0, 0.0],
        camera_position: ArrayType = [2, 2, 2],
        camera_focal_point: ArrayType = [0, 0, 0],
        camera_view_up: ArrayType = [0, 0, 1],
        quit_after_delay: Union[None, float] = None,
    ):
        """! Initializer for the Visualizer class.

        @param window_size The window size [W, H] for the visualizer.
        @param background_color The color of the background (RGB), default is black.
        @param camera_position The position of the camera in the global frame.
        @param camera_focal_point The focus point of the camera.
        @param camera_view_up Direction of the up direction for the camera.
        @param quit_after_delay Number of seconds to keep visualizer running, if None then the visualizer is run indefinitely until the user quits the window.
        @return An instance of the Visualizer class.
        """

        self.ren = vtk.vtkRenderer()
        self.ren.SetBackground(*background_color)
        self.renWin = vtk.vtkRenderWindow()
        self.renWin.SetOffScreenRendering(0)
        self.renWin.AddRenderer(self.ren)
        self.renWin.SetSize(int(window_size[0]), int(window_size[1]))

        self.camera = self.ren.GetActiveCamera()
        self.camera.SetPosition(*camera_position)
        self.camera.SetFocalPoint(*camera_focal_point)
        self.camera.SetViewUp(*camera_view_up)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.renWin)

        style = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(style)

        self.actors = ActorList()

        self.animate_callbacks = []

        if isinstance(quit_after_delay, float):
            assert quit_after_delay > 0.0, "the delay must be positive"
            quit_after_delay_ms = int(quit_after_delay * 1000.0)
            timer_id = self.iren.CreateOneShotTimer(quit_after_delay_ms)
            self.iren.AddObserver("TimerEvent", self.close, timer_id)

    # def append_actors(self, *actors):
    #     for actor in actors:
    #         if isinstance(actor, list):
    #             self.actors += actor  # assume actor is a list of actors
    #         else:
    #             self.actors.append(actor)  # assume actor is a single actor

    def reset_camera(self, position, view_dir, view_up):
        """! Reset the camera pose.

        @param position The position of the camera.
        @param view_dir The direction that the camera should view.
        @param view_up The up direction for the image.
        """
        focal_point = (np.asarray(position) + np.asarray(view_dir)).tolist()
        self.camera.SetPosition(*position)
        self.camera.SetFocalPoint(*focal_point)
        self.camera.SetViewUp(*view_up)

    #
    # Set/convert helper methods
    #

    @staticmethod
    @arrayify_args
    def cvt_orientation_to_rotation_matrix(
        orientation: ArrayType, euler_seq: str, euler_degrees: bool
    ) -> DM:
        """! Convert an orientation input to a rotation matrix.

        @param orientation Either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return Rotation matrix.
        """
        orientation = orientation.toarray().flatten().tolist()
        if len(orientation) == 4:
            R = Rot.from_quat(orientation)
        elif len(orientation) == 3:
            R = Rot.from_euler(euler_seq, orientation, degrees=euler_degrees)
        else:
            raise ValueError(
                f"length for orientation is incorrect, expected 3 or 4, got {len(orientation)}"
            )
        return cs.DM(R.as_matrix())

    @staticmethod
    @arrayify_args
    def set_tf(
        actor: vtk.vtkActor,
        position: ArrayType,
        orientation: ArrayType,
        euler_seq: str,
        euler_degrees: bool,
    ) -> None:
        """! Sets a transform to a vtk Actor.

        @param actor A vtk actor object.
        @param position The position of the transformation.
        @param orientation Orientation of the transformation, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        """
        R = Visualizer.cvt_orientation_to_rotation_matrix(
            orientation, euler_seq, euler_degrees
        )
        tf = rt2tr(R, position)
        Visualizer.set_transformation(actor, tf)

    @staticmethod
    @arrayify_args
    def set_transformation(actor: vtk.vtkActor, tf: ArrayType) -> None:
        """! Sets a transformation to a vtk Actor.

        @param actor A vtk actor object.
        @param tf A homogenous transformation matrix.
        """
        tf = tf.toarray().flatten().tolist()
        transform = vtk.vtkTransform()
        transform.SetMatrix(tf)
        actor.SetUserTransform(transform)

    @staticmethod
    @arrayify_args
    def set_rgba(actor: vtk.vtkActor, rgb: ArrayType, alpha: float) -> None:
        """! Set the RGB and alpha channels for an actor.

        @param actor A vtk actor object.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        """
        if rgb is not None:
            rgb = rgb.toarray().flatten().tolist()
            assert len(rgb) == 3, f"rgb is incorrect length, got {len(rgb)} expected 3"
            actor.GetProperty().SetColor(*rgb)

        alpha = alpha.toarray().flatten()[0]
        assert (
            0.0 <= alpha <= 1.0
        ), f"the scalar alpha must be in the range [0, 1], got {alpha}"
        actor.GetProperty().SetOpacity(alpha)

    @staticmethod
    @arrayify_args
    def set_translation(actor: vtk.vtkActor, translation: ArrayType) -> None:
        """! Set translation for an actor.

        @param actor A vtk actor object.
        @param translation The translation in the global frame.
        """
        translation = translation.toarray().flatten().tolist()
        transform = vtk.vtkTransform()
        transform.Translate(translation)
        actor.SetUserTransform(transform)

    #
    # Drawing methods
    #

    @arrayify_args
    def line(
        self,
        start: ArrayType = [0.0, 0.0, 0.0],
        end: ArrayType = [1.0, 0.0, 0.0],
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
        linewidth: float = 1.0,
    ) -> vtk.vtkActor:
        """! Draw a line.

        @param start The start of the line.
        @param end The end of the line.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param linewidth The width of the line.
        @return The line actor.
        """
        start = start.toarray().flatten().tolist()
        end = end.toarray().flatten().tolist()

        points = vtk.vtkPoints()
        points.InsertNextPoint(*start)
        points.InsertNextPoint(*end)

        line = vtk.vtkLine()
        line.GetPointIds().SetId(0, 0)
        line.GetPointIds().SetId(0, 1)

        lines = vtk.vtkCellArray()
        lines.InsertNextCell(line)

        polydata = vtk.vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetLines(lines)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(polydata)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        actor.GetProperty().SetLineWidth(linewidth)

        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor
    
    @arrayify_args
    def points(
        self,
        data: ArrayType,
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
        size=1.0,
    ) -> vtk.vtkActor:
        """! Draw 3D points.

        @param data The 3D points with shape [N, 3]
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @return The points actor.
        """

        pointCloud = VtkPointCloud()
        data = data.toarray().flatten().tolist()
        num = int(len(data) / 3)
        for i in range(num):
            pointCloud.addPoint(data[3*i:3*i+3])

        actor = pointCloud.vtkActor
        actor.GetProperty().SetPointSize(size)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor    

    @arrayify_args
    def sphere(
        self,
        radius: float = 1.0,
        position: ArrayType = [0.0, 0.0, 0.0],
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
        theta_resolution: int = 20,
        phi_resolution: int = 20,
    ) -> vtk.vtkActor:
        """! Draw a sphere.

        @param radius The radius of the sphere.
        @param position The position of the sphere.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param theta_resolution The number of points in the longitude direction.
        @param phi_resolution The number of points in the latitude direction.
        @return The sphere actor.
        """

        sphere = vtkSphereSource()
        sphere.SetRadius(radius)
        sphere.SetThetaResolution(theta_resolution)
        sphere.SetPhiResolution(phi_resolution)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        self.set_translation(actor, position)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor

    @arrayify_args
    def sphere_traj(
        self,
        position_traj: ArrayType,
        radius: float = 1.0,
        rgb: Union[None, ArrayType] = None,
        theta_resolution: int = 20,
        phi_resolution: int = 20,
        alpha_spec: Union[None, Dict] = None,
        animate=False,
        duration=5.0,
    ) -> List[vtk.vtkActor]:
        """! Draw a sphere trajectory.

        @param position_traj The position trajectory for the spheres.
        @param radius The radius of the spheres.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param theta_resolution The number of points in the longitude direction.
        @param phi_resolution The number of points in the latitude direction.
        @param The alpha specification.
        @return The sphere actors.
        """

        default_alpha_spec = {"style": "none"}
        if alpha_spec is None:
            alpha_spec = default_alpha_spec.copy()

        n = position_traj.shape[1]

        if alpha_spec["style"] == "none":
            alphas = [1.0] * n
        elif alpha_spec["style"] == "A":
            alpha_min = alpha_spec.get("alpha_min", 0.1)
            alpha_max = alpha_spec.get("alpha_max", 1.0)
            alphas = np.linspace(alpha_min, alpha_max, n).tolist()
        elif alpha_spec["style"] == "B":
            alpha_min = alpha_spec.get("alpha_min", 0.1)
            alpha_max = alpha_spec.get("alpha_max", 1.0)
            alphas = [alpha_min] * (n - 1) + [alpha_max]
        elif alpha_spec["style"] == "C":
            alpha_start = alpha_spec.get("alpha_start", 1.0)
            alpha_mid = alpha_spec.get("alpha_mid", 0.1)
            alpha_end = alpha_spec.get("alpha_end", 1.0)
            alphas = [alpha_start] + [alpha_mid] * (n - 2) + [alpha_end]
        elif alpha_spec['style'] == 'const':
            alphas = [alpha_spec.get("alpha", 1.0)]*n

        sphere_traj = []
        if animate:
            self.actors.stop_adding_actors()

        actors = []
        for i, alpha in enumerate(alphas):
            sphere = self.sphere(
                radius=radius,
                position=position_traj[:, i],
                rgb=rgb,
                alpha=alpha,
                theta_resolution=theta_resolution,
                phi_resolution=phi_resolution,
            )

            if animate:
                sphere_traj.append([sphere])

            actors.append(sphere)

        if animate:
            self.animate_callbacks.append(
                AnimationCallback(
                    sphere_traj, duration.toarray().flatten()[0], self.iren, self.ren
                )
            )
            self.actors.start_adding_actors()

        return actors

    @arrayify_args
    def box(
        self,
        scale: ArrayType = [1, 1, 1],
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
        position: ArrayType = [0.0, 0.0, 0.0],
        orientation: ArrayType = [0.0, 0.0, 0.0],
        euler_seq: str = "xyz",
        euler_degrees: bool = False,
    ) -> vtk.vtkActor:
        """! Draw a box.

        @param scale The length, width, and height for the box [L, W, H].
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param position The position of the transformation.
        @param orientation Orientation of the transformation, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return The box actor.
        """

        scale = scale.toarray().flatten().tolist()

        cube = vtk.vtkCubeSource()
        cube.SetBounds(
            -0.5 * scale[0],
            0.5 * scale[0],
            -0.5 * scale[1],
            0.5 * scale[1],
            -0.5 * scale[2],
            0.5 * scale[2],
        )

        # Create a vtkPolyDataMapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())

        # Create a vtkActor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        self.set_tf(actor, position, orientation, euler_seq, euler_degrees)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor

    @arrayify_args
    def cylinder(
        self,
        radius: float = 1.0,
        height: float = 1.0,
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
        resolution: int = 20,
        position: ArrayType = [0.0, 0.0, 0.0],
        orientation: ArrayType = [0.0, 0.0, 0.0],
        euler_seq: str = "xyz",
        euler_degrees: bool = False,
    ) -> vtk.vtkActor:
        """! Draw a cylinder (with main axis along y-axis).

        @param radius The cylinder radius.
        @param height The height of the cylinder.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param resolution The number of facets used to define cylinder.
        @param position The position of the object.
        @param orientation Orientation of the object, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return The cylinder actor.
        """
        cylinder = vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        cylinder.SetResolution(resolution)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cylinder.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        self.set_tf(actor, position, orientation, euler_seq, euler_degrees)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor

    @arrayify_args
    def cylinder_urdf(
        self,
        radius=1.0,
        height=1.0,
        rgb=None,
        alpha=1.0,
        resolution=20,
        position=[0.0, 0.0, 0.0],
        orientation=[0.0, 0.0, 0.0],
        euler_seq="xyz",
        euler_degrees=False,
    ) -> vtk.vtkActor:
        """! Draw a cylinder (with main axis along the z-axis, i.e. similar to URDF).

        @param radius The cylinder radius.
        @param height The height of the cylinder.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param resolution The number of facets used to define cylinder.
        @param position The position of the object.
        @param orientation Orientation of the object, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return The cylinder actor.
        """

        cylinder = vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        cylinder.SetResolution(resolution)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cylinder.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        R = self.cvt_orientation_to_rotation_matrix(
            orientation, euler_seq, euler_degrees
        )
        tf_0 = r2t(Rot.from_euler("x", 90, degrees=True).as_matrix())
        tf_1 = rt2tr(R, position)
        tf = tf_1 @ tf_0

        self.set_transformation(actor, tf)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor

    @arrayify_args
    def text(
        self,
        msg: str = "Hello, world!",
        position: ArrayType = [0.0, 0.0, 0.0],
        scale: ArrayType = [1.0, 1.0, 1.0],
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
    ) -> vtk.vtkFollower:
        """! Draw text.

        @param msg The text to draw.
        @param position The position to draw the text in the global frame.
        @param scale The size of the text.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @return The text actor.
        """
        position = position.toarray().flatten().tolist()
        scale = scale.toarray().flatten().tolist()

        # Create a text source to generate the text
        textSource = vtk.vtkTextSource()
        textSource.SetText(msg)
        textSource.Update()

        # Create a mapper to map the text source to graphics primitives
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(textSource.GetOutputPort())

        # Create a follower to position the text in 3D space
        follower = vtk.vtkFollower()
        follower.SetMapper(mapper)
        follower.SetPosition(*position)
        follower.SetScale(*scale)
        follower.SetCamera(self.camera)

        self.set_rgba(follower, rgb, alpha)

        self.actors.append(follower)

        return follower

    @arrayify_args
    def link(
        self,
        T: ArrayType = None,
        axis_scale: float = 0.1,
        axis_linewidth: float = 1.0,
        center_radius: float = 0.01,
        center_rgb: Union[None, ArrayType] = None,
        center_alpha: float = 1.0,
        center_theta_resolution: int = 20,
        center_phi_resolution: int = 20,
        axis_alpha: float = 1.0,
    ) -> List[vtk.vtkActor]:
        """! Draw a link.

        @param T Homogeneous transformation matrix.
        @param axis_scale The axis length.
        @param axis_linewidth Width of the axis lines.
        @param center_radius The radius of the central sphere.
        @param center_rgb The Red-Green-Blue values in range [0, 1].
        @param center_alpha Transparency of the centeral sphere in range [0, 1].
        @param center_theta_resolution The number of points in the longitude direction.
        @param center_phi_resolution The number of points in the latitude direction.
        @param axis_alpha Transparency of the axes in range [0, 1].
        @return The link actors.
        """
        if T is None:
            T = I4()

        p = transl(T)
        x = T[:3, 0]
        y = T[:3, 1]
        z = T[:3, 2]

        actors = []

        actors.append(
            self.line(
                start=p,
                end=p + axis_scale * x,
                rgb=[1, 0, 0],
                alpha=axis_alpha,
                linewidth=axis_linewidth,
            )
        )

        actors.append(
            self.line(
                start=p,
                end=p + axis_scale * y,
                rgb=[0, 1, 0],
                alpha=axis_alpha,
                linewidth=axis_linewidth,
            )
        )

        actors.append(
            self.line(
                start=p,
                end=p + axis_scale * z,
                rgb=[0, 0, 1],
                alpha=axis_alpha,
                linewidth=axis_linewidth,
            )
        )

        actors.append(
            self.sphere(
                radius=center_radius,
                position=p,
                rgb=center_rgb,
                alpha=center_alpha,
                theta_resolution=center_theta_resolution,
                phi_resolution=center_phi_resolution,
            )
        )

        return actors

    @arrayify_args
    def grid_floor(
        self,
        num_cells: int = 10,
        rgb: Union[None, ArrayType] = None,
        alpha: float = 1.0,
        linewidth: float = 3.0,
        inner_rgb: Union[None, ArrayType] = None,
        inner_alpha: Union[None, float] = None,
        inner_linewidth: float = 1.0,
        stride: float = 1.0,
        euler: ArrayType = [0, 0, 0],
        euler_seq: str = "xyz",
        euler_degrees: bool = True,
    ) -> List[vtk.vtkActor]:
        """! Draw a grid floor.

        @param num_cells The number of cells for the grid floor.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param linewidth The line width for the grid.
        @param inner_rgb The Red-Green-Blue values in range [0, 1] for the inner lines at half the stride length.
        @param inner_alpha Transparency of the actor in range [0, 1] for the inner lines at half the stride length.
        @param inner_linewidth The line width for the grid for the inner lines at half the stride length.
        @param stride The length of each main grid cell.
        @param euler The Euler angles that defines the orientation of the plane the grid is defined on.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return The grid actors.
        """
        num_cells = num_cells.toarray().flatten().astype(int)[0]
        stride = stride.toarray().flatten()[0]

        assert num_cells > 0, "num_cells must be a positive number!"
        assert num_cells % 2 == 0, "num_cells must be even!"

        actors = []

        num_lines = num_cells + 1

        tf_0 = rt2tr(
            I3(),
            [
                -0.5 * float(num_cells) * stride,
                -0.5 * float(num_cells) * stride,
                0.0,
            ],
        )
        tf_1 = r2t(
            Rot.from_euler(
                euler_seq, euler.toarray().flatten(), degrees=euler_degrees
            ).as_matrix()
        )
        tf = tf_1 @ tf_0

        offset = vtk.vtkTransform()
        offset.SetMatrix(tf.toarray().flatten().tolist())

        if inner_rgb is None:
            inner_rgb = rgb

        if inner_alpha is None:
            inner_alpha = alpha

        for i in range(num_cells):
            actor = self.line(
                start=[(float(i) + 0.5) * stride, 0, 0],
                end=[(float(i) + 0.5) * stride, float(num_cells) * stride, 0],
                linewidth=inner_linewidth,
                alpha=inner_alpha,
                rgb=inner_rgb,
            )
            actor.SetUserTransform(offset)
            actors.append(actor)

            actor = self.line(
                start=[0, (float(i) + 0.5) * stride, 0],
                end=[float(num_cells) * stride, (float(i) + 0.5) * stride, 0],
                linewidth=inner_linewidth,
                alpha=inner_alpha,
                rgb=inner_rgb,
            )
            actor.SetUserTransform(offset)
            actors.append(actor)

        for i in range(num_lines):
            actor = self.line(
                start=[float(i) * stride, 0, 0],
                end=[float(i) * stride, float(num_cells) * stride, 0],
                rgb=rgb,
                alpha=alpha,
                linewidth=linewidth,
            )
            actor.SetUserTransform(offset)
            actors.append(actor)

            actor = self.line(
                start=[0, float(i) * stride, 0],
                end=[float(num_cells) * stride, float(i) * stride, 0],
                rgb=rgb,
                alpha=alpha,
                linewidth=linewidth,
            )
            actor.SetUserTransform(offset)
            actors.append(actor)

        return actors

    @arrayify_args
    def obj(
        self,
        obj_filename: str,
        rgb=None,
        alpha=1.0,
        png_texture_filename: str = None,
        position: ArrayType = [0.0, 0.0, 0.0],
        orientation: ArrayType = [0.0, 0.0, 0.0],
        euler_seq: str = "xyz",
        euler_degrees: bool = False,
    ):
        """! Load .obj file.

        @param obj_filename The filename for the .obj file.
        @param png_texture_filename The texture filename.
        @param position The position of the object.
        @param orientation Orientation of the object, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return The actor representing the .obj file.
        """

        # Read the .obj file
        reader = vtk.vtkOBJReader()
        reader.SetFileName(obj_filename)
        reader.Update()

        # Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        if isinstance(png_texture_filename, str):
            # Create a texture from the png file
            texture = vtk.vtkTexture()
            texture_image = vtk.vtkPNGReader()
            texture_image.SetFileName(png_texture_filename)
            texture_image.Update()
            texture.SetInputConnection(texture_image.GetOutputPort())

            actor.SetTexture(texture)

        self.set_tf(actor, position, orientation, euler_seq, euler_degrees)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor

    @arrayify_args
    def stl(
        self,
        filename,
        scale=None,
        rgb=None,
        alpha=1.0,
        position=[0.0, 0.0, 0.0],
        orientation=[0.0, 0.0, 0.0],
        euler_seq="xyz",
        euler_degrees=False,
    ) -> vtk.vtkActor:
        """! Load .stl file.

        @param filename The filename for the .stl file.
        @param scale Scale applied to the mesh file in the xyz directions.
        @param rgb The Red-Green-Blue values in range [0, 1].
        @param alpha Transparency of the actor in range [0, 1].
        @param position The position of the object.
        @param orientation Orientation of the object, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.
        @return The actor representing the .stl file.
        """

        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)

        if scale is None:
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(reader.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
        else:
            scale = scale.toarray().flatten().tolist()

            transform = vtk.vtkTransform()
            transform.Scale(*scale)

            transformFilter = vtk.vtkTransformPolyDataFilter()
            transformFilter.SetTransform(transform)
            transformFilter.SetInputConnection(reader.GetOutputPort())

            # Visualize the object
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(transformFilter.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)

        self.set_tf(actor, position, orientation, euler_seq, euler_degrees)
        self.set_rgba(actor, rgb, alpha)
        self.actors.append(actor)

        return actor

    def robot(
        self,
        robot_model: RobotModel,
        base_position: ArrayType = [0.0, 0.0, 0.0],
        base_orientation: ArrayType = [0.0, 0.0, 0.0],
        euler_seq: str = "xyz",
        euler_degrees: bool = False,        
        q: ArrayType = None,
        alpha: float = 1.0,
        show_links: bool = False,
        link_axis_scale: float = 0.2,
        link_axis_linewidth: float = 1.0,
        link_center_radius: float = 0.01,
        link_center_rgb: Union[None, ArrayType] = None,
        link_center_alpha: Union[None, ArrayType] = None,
        display_link_names: bool = False,
        link_names_scale: ArrayType = [0.005, 0.005, 0.005],
        link_names_rgb: ArrayType = [1, 1, 1],
        link_names_alpha: float = 1.0,
    ) -> List[vtk.vtkActor]:
        """! Draw a robot.

        @param robot_model The robot model defining the robot kinematics and visuals.
        @param base_position The position of the robot.
        @param base_orientation Orientation of the robot, either a quaternion or Euler angles.
        @param euler_seq When orientation are Euler angles, specifies sequence of axes for rotations. Up to 3 characters belonging to the set {'X', 'Y', 'Z'} for intrinsic rotations, or {'x', 'y', 'z'} for extrinsic rotations. When orientation are Euler angles, extrinsic and intrinsic rotations cannot be mixed in one function call.
        @param euler_degrees If True, then the given angles are assumed to be in degrees. Default is False.        
        @param q Joint configuration to draw robot.
        @param alpha Transparency of the actor in range [0, 1].
        @param show_links When true, the robot links are shown.
        @param link_axis_scale The scale for the link axes when shown.
        @param link_axis_linewidth The linewidth for the axes of the robot links when shown.
        @param link_center_radius The radius of the axis central sphere when shown.
        @param link_center_rgb The RGB values for the central sphere in range [0, 1] when shown.
        @param link_center_alpha The transparency for the central sphere in the range [0, 1] when shown.
        @param display_link_names When true, the names of the robot link are shown.
        @param link_names_scale The size of the link names text when shown.
        @param link_names_rgb The RGB values in range [0, 1] for the robot link names text when shown.
        @param link_names_alpha The transparency in range [0, 1] for the robot link names when shown.
        @return The actors representing the robot.
        """
        if link_center_alpha is None:
            link_center_alpha = alpha

        # compute robot base tf
        R = Visualizer.cvt_orientation_to_rotation_matrix(
            base_orientation, euler_seq, euler_degrees
        )
        tf_base = rt2tr(R, base_position)            

        actors = []

        urdf = robot_model.get_urdf()

        material_names = [m.name for m in urdf.materials]

        def get_material_rgba(name):
            try:
                idx = material_names.index(name)
            except ValueError:
                return None
            material = urdf.materials[idx]
            if material.color is not None:
                return material.color.rgba

        if q is None:
            q_user_input = [0.0] * robot_model.ndof
        else:
            q_user_input = cs.vec(q)

        # Setup functions to compute visual origins in global frame
        q = cs.SX.sym("q", robot_model.ndof)
        link_tf = {}
        visual_tf = {}
        for urdf_link in urdf.links:
            name = urdf_link.name

            lnk_tf = robot_model.get_global_link_transform(urdf_link.name, q)
            link_tf[name] = cs.Function(f"link_tf_{name}", [q], [lnk_tf])

            xyz, rpy = robot_model.get_link_visual_origin(urdf_link)
            visl_tf = rt2tr(rpy2r(rpy), xyz)

            # move robot base as well
            tf = tf_base @ lnk_tf @ visl_tf
            visual_tf[name] = cs.Function(f"visual_tf_{name}", [q], [tf])

        for urdf_link in urdf.links:
            tf = visual_tf[urdf_link.name](q_user_input).toarray()
            position = tf[:3, 3].flatten().tolist()
            orientation = Rot.from_matrix(tf[:3, :3]).as_quat().tolist()

            if show_links:
                actors += self.link(
                    tf,
                    axis_scale=link_axis_scale,
                    axis_linewidth=link_axis_linewidth,
                    center_radius=link_center_radius,
                    center_rgb=link_center_rgb,
                    center_alpha=link_center_alpha,
                )

            if display_link_names:
                actors.append(
                    self.text(
                        msg=urdf_link.name,
                        position=position,
                        scale=link_names_scale,
                        rgb=link_names_rgb,
                        alpha=link_names_alpha,
                    )
                )

            for visual in urdf_link.visuals:
                if visual is None:
                    continue

                geometry = visual.geometry
                T = tf
                position = transl(T).toarray().flatten()
                orientation = Rot.from_matrix(T[:3, :3]).as_quat().tolist()

                material = visual.material
                rgb = None
                if material is not None and isinstance(material.name, str) and material.name in material_names:
                    rgba = get_material_rgba(visual.material.name)
                    rgb = rgba[:3]

                if isinstance(geometry, Mesh):
                    if geometry.filename.lower().endswith(".stl"):
                        filename = geometry.filename

                        if not os.path.exists(filename):
                            relpath = robot_model.get_urdf_dirname()
                            filename = os.path.join(relpath, filename)

                        scale = None
                        if geometry.scale is not None:
                            scale = geometry.scale

                        actors.append(
                            self.stl(
                                filename,
                                scale=scale,
                                rgb=rgb,
                                alpha=alpha,
                                position=position,
                                orientation=orientation,
                                euler_seq="xyz",
                                euler_degrees=True,
                            )
                        )
                    elif geometry.filename.lower().endswith(".obj"):
                        filename = geometry.filename

                        if not os.path.exists(filename):
                            relpath = robot_model.get_urdf_dirname()
                            filename = os.path.join(relpath, filename)

                        scale = None
                        if geometry.scale is not None:
                            scale = geometry.scale

                        mesh = o3d.io.read_triangle_mesh(filename, enable_post_processing=True)
                        if mesh.has_textures:
                            png_texture_filename = filename.replace('.obj', '_uv.png')
                            if not os.path.exists(png_texture_filename):
                                im = mesh.textures[0]
                                if not im.is_empty():
                                    o3d.io.write_image(png_texture_filename, im)
                                else:
                                    png_texture_filename = None
                        else:
                            png_texture_filename = None

                        actors.append(
                            self.obj(
                                filename,
                                png_texture_filename=png_texture_filename,
                                alpha=alpha,
                                position=position,
                                orientation=orientation,
                                euler_seq="xyz",
                                euler_degrees=True,
                            )
                        )                        

                elif isinstance(geometry, Sphere):
                    actors.append(
                        self.sphere(
                            radius=geometry.radius,
                            position=position,
                            rgb=rgb,
                            alpha=alpha,
                            theta_resolution=20,
                            phi_resolution=20,
                        )
                    )

                elif isinstance(geometry, Cylinder):
                    actors.append(
                        self.cylinder_urdf(
                            radius=geometry.radius,
                            height=geometry.length,
                            position=position,
                            orientation=orientation,
                            rgb=rgb,
                            alpha=alpha,
                        )
                    )

        return actors

    def robot_traj(
        self,
        robot_model,
        Q,
        base_position: ArrayType = [0.0, 0.0, 0.0],
        base_orientation: ArrayType = [0.0, 0.0, 0.0],
        euler_seq: str = "xyz",
        euler_degrees: bool = False,
        alpha_spec=None,
        show_links=False,
        link_axis_scale=0.2,
        link_axis_linewidth=1.0,
        link_center_rgb=1,
        link_center_alpha=None,
        link_center_radius=0.01,
        display_link_names=False,
        link_names_scale=[0.01, 0.01, 0.01],
        link_names_rgb=[1, 1, 1],
        link_names_alpha=1.0,
        animate=False,
        duration=5.0,
    ):
        """! Draw a robot through a trajectory.

        @param robot_model The robot model defining the robot kinematics and visuals.
        @param Q Joint configuration trajectory to draw robot.
        @param alpha_spec Transparency specification of the robots in motion.
        @param show_links When true, the robot links are shown.
        @param link_axis_scale The scale for the link axes when shown.
        @param link_axis_linewidth The linewidth for the axes of the robot links when shown.
        @param link_center_radius The radius of the axis central sphere when shown.
        @param link_center_rgb The RGB values for the central sphere in range [0, 1] when shown.
        @param link_center_alpha The transparency for the central sphere in the range [0, 1] when shown.
        @param display_link_names When true, the names of the robot link are shown.
        @param link_names_scale The size of the link names text when shown.
        @param link_names_rgb The RGB values in range [0, 1] for the robot link names text when shown.
        @param link_names_alpha The transparency in range [0, 1] for the robot link names when shown.
        @return The actors representing the robot.
        """
        default_alpha_spec = {"style": "none"}
        if alpha_spec is None:
            alpha_spec = default_alpha_spec.copy()

        actors = []
        n = Q.shape[1]

        if alpha_spec["style"] == "none":
            alphas = np.ones(n).tolist()
        elif alpha_spec["style"] == "A":
            alpha_min = alpha_spec.get("alpha_min", 0.1)
            alpha_max = alpha_spec.get("alpha_max", 1.0)
            alphas = np.linspace(alpha_min, alpha_max, n).tolist()
        elif alpha_spec["style"] == "B":
            alpha_min = alpha_spec.get("alpha_min", 0.1)
            alpha_max = alpha_spec.get("alpha_max", 1.0)
            alphas = [alpha_min] * (n - 1) + [alpha_max]
        elif alpha_spec["style"] == "C":
            alpha_start = alpha_spec.get("alpha_start", 1.0)
            alpha_mid = alpha_spec.get("alpha_mid", 0.1)
            alpha_end = alpha_spec.get("alpha_end", 1.0)
            alphas = [alpha_start] + [alpha_mid] * (n - 2) + [alpha_end]
        elif alpha_spec["style"] == "D":
            alphas = np.ones(n) * 0.2
            alphas = alphas.tolist()

        robot_traj = []
        if animate:
            self.actors.stop_adding_actors()

        for i, alpha in enumerate(alphas):
            robot = self.robot(
                robot_model,
                base_position=base_position,
                base_orientation=base_orientation,
                euler_seq=euler_seq,
                euler_degrees=euler_degrees,                  
                q=Q[:, i],
                alpha=alpha,
                show_links=show_links,
                link_axis_scale=link_axis_scale,
                link_axis_linewidth=link_axis_linewidth,
                link_center_rgb=link_center_rgb,
                link_center_alpha=link_center_alpha,
                link_center_radius=link_center_radius,
                display_link_names=display_link_names,
                link_names_scale=link_names_scale,
                link_names_rgb=link_names_rgb,
                link_names_alpha=link_names_alpha,
            )

            if animate:
                robot_traj.append(robot)

            actors += robot

        if animate:
            self.animate_callbacks.append(
                AnimationCallback(robot_traj, duration, self.iren, self.ren)
            )
            self.actors.start_adding_actors()

        return actors

    def save(self, file_name):
        """! Save the visualizer window as png image. Note, saving animations is currently not supported."""
        assert (
            len(self.animate_callbacks) == 0
        ), "saving animations is currently not supported"

        if not file_name.endswith(".png"):
            file_name += ".png"

        for actor in self.actors.actors:
            self.ren.AddActor(actor)
        self.iren.Initialize()

        self.renWin.Render()

        # Capture the contents of the render window
        window_to_image_filter = vtk.vtkWindowToImageFilter()
        window_to_image_filter.SetInput(self.renWin)
        window_to_image_filter.Update()

        # Write the captured image to a PNG file
        png_writer = vtk.vtkPNGWriter()
        png_writer.SetFileName(file_name)
        png_writer.SetInputConnection(window_to_image_filter.GetOutputPort())
        png_writer.Write()
        print(f"Saved {file_name}")

        # rendering offscreen when saving!
        # self.iren.Start()

    def start(self) -> None:
        """! Start the visualizer."""
        for actor in self.actors.actors:
            self.ren.AddActor(actor)
        self.iren.Initialize()

        for callback in self.animate_callbacks:
            callback.start()

        self.renWin.Render()
        self.iren.Start()

    def close(self, obj, event) -> None:
        """! Close the visualizer."""
        self.renWin.Finalize()
        self.iren.TerminateApp()
