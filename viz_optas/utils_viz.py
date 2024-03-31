import json
import random
import math

import numpy as np
import quaternion
from transforms3d.quaternions import quat2mat
from transforms3d.euler import mat2euler


def get_urdf_path(gripper_name):
    if gripper_name == "fetch_gripper":
        return "fetch_gripper/fetch_gripper.urdf"
    elif gripper_name == "Barrett":
        return "Barrett/Barrett.urdf"
    elif gripper_name == "HumanHand":
        return "HumanHand/HumanHand.urdf"
    elif gripper_name == "Allegro":
        return "Allegro/allegro_hand_description_right.urdf"
    elif gripper_name == "franka_panda":
        return "franka_panda/franka_panda.urdf"
    elif gripper_name == "jaco_robot":
        return "jaco_robot/jaco_robot.urdf"
    elif gripper_name == "robotiq_3finger":
        return "robotiq_3finger/robotiq_3finger.urdf"
    elif gripper_name == "wsg_50":
        return "wsg_50/wsg_50.urdf"
    elif gripper_name == "shadow_hand":
        return "shadow_hand/shadow_hand.urdf"
    elif gripper_name == "sawyer":
        return "sawyer/sawyer.urdf"
    elif gripper_name == "h5_hand":
        return "h5_hand/h5_hand.urdf"
    else:
        print("[ERROR]: INVALID Gripper name. Returning empty string!!!")
        return ""


def adjust_dofs(gripper, dofs):
    # Note: We are setting both joints to the SAME dofvalue for panda and fetch
    if gripper == "fetch_gripper":
        dofvalue = (dofs[0] / 1000.0) * 10.0
        return [dofvalue, dofvalue]
    elif gripper == "franka_panda":
        dofvalue = dofs[0] / 1000
        return [dofvalue, dofvalue]
    elif gripper == "Barrett":
        j_states = [
            dofs[0],
            dofs[1],
            dofs[1] / 3.0,
            dofs[0],
            dofs[2],
            dofs[2] / 3.0,
            dofs[3],
            dofs[3] / 3.0,
        ]
        return j_states
    elif gripper in {
        "HumanHand",
        "Allegro",
        "robotiq_3finger",
        "jaco_robot",
        "shadow_hand",
    }:
        return dofs
    elif gripper in {"wsg_50", "sawyer"}:
        return [d / 1000.0 for d in dofs]
    elif gripper == "h5_hand":
        dofs_to_use = [dofs[0], -dofs[0], dofs[1], -dofs[1]]
        return dofs_to_use


def rotX(rotx):
    RotX = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(rotx), -np.sin(rotx), 0],
            [0, np.sin(rotx), np.cos(rotx), 0],
            [0, 0, 0, 1],
        ]
    )
    return RotX


def rotY(roty):
    RotY = np.array(
        [
            [np.cos(roty), 0, np.sin(roty), 0],
            [0, 1, 0, 0],
            [-np.sin(roty), 0, np.cos(roty), 0],
            [0, 0, 0, 1],
        ]
    )
    return RotY


def rotZ(rotz):
    RotZ = np.array(
        [
            [np.cos(rotz), -np.sin(rotz), 0, 0],
            [np.sin(rotz), np.cos(rotz), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    return RotZ


def unpack_pose(pose, rot_first=False):
    unpacked = np.eye(4)
    if rot_first:
        unpacked[:3, :3] = quat2mat(pose[:4])
        unpacked[:3, 3] = pose[4:]
    else:
        unpacked[:3, :3] = quat2mat(pose[3:])
        unpacked[:3, 3] = pose[:3]
    return unpacked


def se3_inverse(RT):
    R = RT[:3, :3]
    T = RT[:3, 3].reshape((3, 1))
    RT_new = np.eye(4, dtype=np.float32)
    RT_new[:3, :3] = R.transpose()
    RT_new[:3, 3] = -1 * np.dot(R.transpose(), T).reshape((3))
    return RT_new


def ros_qt_to_rt(rot, trans):
    qt = np.zeros((4,), dtype=np.float32)
    qt[0] = rot[3]
    qt[1] = rot[0]
    qt[2] = rot[1]
    qt[3] = rot[2]
    obj_T = np.eye(4)
    obj_T[:3, :3] = quat2mat(qt)
    obj_T[:3, 3] = trans

    return obj_T


def projection_to_intrinsics(mat, width=224, height=224):
    intrinsic_matrix = np.eye(3)
    mat = np.array(mat).reshape([4, 4]).T
    fv = width / 2 * mat[0, 0]
    fu = height / 2 * mat[1, 1]
    u0 = width / 2
    v0 = height / 2

    intrinsic_matrix[0, 0] = fu
    intrinsic_matrix[1, 1] = fv
    intrinsic_matrix[0, 2] = u0
    intrinsic_matrix[1, 2] = v0
    return intrinsic_matrix


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def argsort(seq):
    return sorted(range(len(seq)), key=seq.__getitem__, reverse=True)


def parse_grasps_filtered(
    graspit_filename,
    num_grasps,
    filtered_filename=None,
    sort_key="fall_time",
):
    with open(graspit_filename, "r") as f:
        data = json.load(f)
    grasps = data["grasps"]

    if filtered_filename is not None:
        with open(filtered_filename, "r") as f:
            filtered_grasps = json.load(f)
        sorted_index = argsort(filtered_grasps[sort_key])
    else:
        sorted_index = range(len(grasps))  # i.e no ordering
    NUM = 2 * num_grasps  # for some randomization!
    dofs_grasp = []
    poses_grasp = np.zeros((num_grasps, 4, 4), dtype=np.float32)
    idxs_to_iterate = random.sample(sorted_index[:NUM], k=num_grasps)
    for i, idx in enumerate(idxs_to_iterate):
        pose = grasps[idx]["pose"]
        curr_dofs = filtered_grasps["graspit_dofs"][idx]
        dofs_grasp.append(curr_dofs)
        rot = pose[3:]
        trans = pose[:3]
        RT = ros_qt_to_rt(rot, trans)
        poses_grasp[i, :, :] = RT
    return poses_grasp, np.asarray(dofs_grasp)


def parse_grasps(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    grasps = data["grasps"]

    n = len(grasps)
    dofs_grasp = []
    poses_grasp = np.zeros((n, 4, 4), dtype=np.float32)
    for i in range(n):
        pose = grasps[i]["pose"]
        dofs_grasp.append(grasps[i]["dofs"])
        rot = pose[3:]
        trans = pose[:3]
        RT = ros_qt_to_rt(rot, trans)
        poses_grasp[i, :, :] = RT
    return poses_grasp, np.asarray(dofs_grasp)


def get_quat_pyb(quat_np):
    """
    quat_np: np.quaternion in (w,x,y,z) format
    Return: quaternion list in [x,y,z,w] format (for pybullet)
    """
    return [quat_np.x, quat_np.y, quat_np.z, quat_np.w]


def get_quat_np(quat_pyb):
    """
    quat_pyb: pybullet quaternion in (x,y,z,w) format (as a List)
    Return: np.quaternion [w,x,y,z] format
    """
    return np.quaternion(quat_pyb[-1], quat_pyb[0], quat_pyb[1], quat_pyb[2])


def get_gripper_common_alignment(gname: str):
    """
    Bring a given gripper's default (urdf) orientation to the common orientation.
    Common orientation: Palm (base link) normal = Z axis, Major axis of Palm = Y axis.

    Input:
        gname: gripper name. Has to be one of: [fetch_gripper, sawyer, franka_panda, wsg_50,
               Barrett, robotiq_3finger, jaco_robot, HumanHand, Allegro, shadow_hand]

    Returns:
        quaternion (w,x,y,z) transformation which when applied on the gripper default urdf, brings it
        into the common alignment orientation described above.
    """
    if gname == "fetch_gripper":
        return quaternion.from_euler_angles([0, -math.pi / 2.0, 0])
    elif gname == "sawyer":
        return np.quaternion(1, 0, 0, 0)  # unit quaternion, i.e NO CHANGE NEEDED
    elif gname == "franka_panda":
        return np.quaternion(1, 0, 0, 0)  # unit quaternion, i.e NO CHANGE NEEDED
    elif gname == "wsg_50":
        return quaternion.from_euler_angles([0, 0, math.pi / 2.0])
    elif gname == "Barrett":
        return np.quaternion(1, 0, 0, 0)  # unit quaternion, i.e NO CHANGE NEEDED
    elif gname == "robotiq_3finger":
        return quaternion.from_euler_angles([0, -math.pi / 2.0, -math.pi / 2.0])
    elif gname == "jaco_robot":
        return quaternion.from_euler_angles([0, math.pi / 2.0, 0])
    elif gname == "HumanHand":
        return quaternion.from_euler_angles([0, math.pi / 2.0, -math.pi / 2.0])
    elif gname == "Allegro":
        return quaternion.from_euler_angles([-math.pi / 2.0, -math.pi / 2.0, 0])
    elif gname == "shadow_hand":
        return quaternion.from_euler_angles([math.pi / 2, math.pi / 2, -math.pi / 2])
    elif gname == "h5_hand":
        return quaternion.from_euler_angles([0, math.pi, 0])
    else:
        print("Invalid gripper name. Returning None!")
        return None


def get_gripper_palm_position(gname: str):
    """
    This function returns the position vector of a point on the gripper's palm surface
    and center of the palm (so think of it like centroid along 2 axes and just on the
    surface for the 3rd axis) -- in the gripper's default urdf frame.

    We are using such a palm point to unify the notion of "position for gripper". The
    position vectors (or offsets) were all computed in the respective gripper URDF's
    base frame (i.e no alignment). Notice the multiplication by -1 here, this means that
    the returned value is indeed the position vector (instead of the offset for the base
    link's position -- this would be true if we didn't have -1)

    Input:
        gname: gripper name. Has to be one of: [fetch_gripper, sawyer, franka_panda, wsg_50,
               Barrett, robotiq_3finger, jaco_robot, HumanHand, Allegro, shadow_hand]

    Returns:
        position vector/offset (x,y,z) for a palm point
    """
    if gname == "fetch_gripper":
        return -1 * np.array([-0.135, 0, 0])
    elif gname == "sawyer":
        return -1 * np.array([0, 0, -0.05])
    elif gname == "franka_panda":
        return -1 * np.array([0, 0, -0.06])
    elif gname == "wsg_50":
        return -1 * np.array([0, 0, -0.072])
    elif gname == "Barrett":
        return -1 * np.array([0, 0, 0])
    elif gname == "robotiq_3finger":
        return -1 * np.array([0, -0.05, 0])
    elif gname == "jaco_robot":
        return -1 * np.array([0.102, 0, 0])
    elif gname == "HumanHand":
        return -1 * np.array([0.1, 0.02, 0])
    elif gname == "Allegro":
        return -1 * np.array([0, 0, 0.03])  # or [-0.01, 0, 0.03]
    elif gname == "shadow_hand":
        return -1 * np.array([0, 0, -0.05])
    elif gname == "h5_hand":
        return -1 * np.array([0, 0, 0.045])
    else:
        return None


def convert_gripper_to_aligned_pose(grasp_pose, source_gripper):
    """
    Function to bring the grasp pose (position, orientation) 7D vector for a gripper
    into the common alignment space. We transform both the position and orientation
    and return a new "aligned" pose for the gripper

    NOTE: Might be slow since it operates on 1 grasp at a time!

    Input:
        grasp_pose: length 7 numpy array containing (x,y,z) location coordinates
        and the values [x,y,z,w] for the orientation quaternion

        gripper: string containing the gripper's name

    Output:
        Single 7-D Numpy Array for Pose with: [position, orientation_quaternion]
        - position_common: length 3 array containing the aligned position
        - orientation_common: length 3 array containing the aligned orientation (x,y,z,w) for pybullet
    """
    # Get the posn vector (offset) for palm point and convert to np.quaternion (w,x,y,z)

    curr_orn = grasp_pose[3:]
    q_curr_orn = get_quat_np(curr_orn)
    q_align_grp = get_gripper_common_alignment(source_gripper)
    q_orn_common = q_curr_orn * q_align_grp.inverse()
    orientation_common = get_quat_pyb(q_orn_common)

    curr_pos = grasp_pose[:3]
    pv_grp = get_gripper_palm_position(source_gripper)
    q_pv_grp = np.quaternion(
        0, pv_grp[0], pv_grp[1], pv_grp[2]
    )  # Note w = 0 since posn vec
    q_offset_com = q_curr_orn * q_pv_grp * q_curr_orn.inverse()
    offset_common = np.array([q_offset_com.x, q_offset_com.y, q_offset_com.z])
    # offset_common = quaternion.as_vector_part() can also use this
    position_common = curr_pos + offset_common

    # return position_common, orientation_common
    return np.concatenate((position_common, orientation_common))


def convert_aligned_to_gripper_pose(grasp_pose_aligned, target_gripper):
    """
    Function to obtain the base link's (for gripper URDF) position and orientation given
    a grasp pose in the common "aligned" space. Basically, it figures out how to use the
    common aligned pose for a specific (given) gripper.

    NOTE: Might be slow since it operates on 1 grasp at a time!

    Input:
        grasp_pose_aligned: length 7 numpy array containing (x,y,z) location coordinates
        and the values [x,y,z,w] for the orientation quaternion in the "aligned" way i.e
        what the network should ideally predict.

        target_gripper: string containing the target gripper's name -- to which we want to
        apply the aligned grasp pose

    Output:
        Single 7-D Numpy Array for Pose with: [position, orientation_quaternion]
        - position_base: length 3 array for the base link's position
        - orientation_base: length 3 array for the base link's orientation (x,y,z,w)
    """

    orn_com = grasp_pose_aligned[3:]
    q_orn_com = get_quat_np(orn_com)
    q_orn_base = q_orn_com * get_gripper_common_alignment(target_gripper)
    orientation_base = get_quat_pyb(q_orn_base)

    pos_com = grasp_pose_aligned[:3]
    pv_grp = get_gripper_palm_position(target_gripper)
    q_pv_grp = np.quaternion(
        0, pv_grp[0], pv_grp[1], pv_grp[2]
    )  # Note w = 0 since we are representing a position vec using quaternion
    q_offset_base = q_orn_base * q_pv_grp * q_orn_base.inverse()
    position_base = pos_com - np.array(
        [q_offset_base.x, q_offset_base.y, q_offset_base.z]
    )

    # return position_base, orientation_base
    return np.concatenate((position_base, orientation_base))
