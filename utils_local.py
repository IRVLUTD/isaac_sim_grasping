import numpy as np
import math
import pickle
#from omni.isaac.urdf import _urdf
from scipy.interpolate import LinearNDInterpolator, griddata

import rospy
from urdf_parser_py.urdf import URDF
import tf.transformations as tf
from collections.abc import Iterable

def get_unfixed_kin_chains(robot_dir, base_link="base_link"):
    robot = load_urdf(robot_dir, False)
    joint_info = dict()
    kin_chains = []
    transform = np.eye(4)
    kc = [base_link]

    def get_child_joints(parent_name, transform, kin_chain):
        childs = []
        for joint in robot.joints:
            wkc = kin_chain.copy()
            if joint.parent == parent_name:
                childs.append(joint.name) # add to childs
                nt = joint.origin
                translation = np.array(nt.xyz)
                rotation = np.array(nt.rpy)
                rotation_matrix = tf.euler_matrix(rotation[0], rotation[1], rotation[2], axes='sxyz')
                
                full_transform = np.eye(4)
                full_transform[:3, :3] = rotation_matrix[:3, :3]  # Rotation
                full_transform[:3, 3] = translation
                merged_transform = np.matmul(transform, full_transform)
                xyz = merged_transform[:3, 3]
                check = full_transform
                check[np.abs(check)<0.001] = 0
                #print(joint.name)
                #print(check)
                #print(nt.rpy)
                
                if joint.type != "fixed":
                    # Save joint as part of the kin_chain
                    wkc.append(joint.name)
                    xyz = merged_transform[:3, 3]
                    quat = tf.quaternion_from_matrix(merged_transform)
                    # Ensure axis_in_parent is a unit vector
                    check = merged_transform
                    check[np.abs(check)<0.001] = 0
                    #print("Joint")
                    #print(joint.axis)
                    #print(check)

                    joint_info[joint.name] = (
                        joint.parent,
                        joint.child,
                        xyz,
                        quat,
                        joint.type,
                        joint.axis,
                        joint.limit.lower,
                        joint.limit.upper,
                        np.linalg.norm(xyz)
                    )
                    merged_transform = np.eye(4)
                
                c = get_child_joints(joint.child, merged_transform, wkc)
                if c == []:
                    #Save finger and add fixed joint as kin_chain (use link child name as ref.)
                    if joint.type == 'fixed':
                        # In case it didn't save the joint information for the finger tip
                        wkc.append(joint.name)
                        xyz = merged_transform[:3, 3]
                        rotation_matrix = merged_transform[:3, :3]
                        quat = tf.quaternion_from_matrix(merged_transform)
                        joint_info[joint.name] = (
                            joint.parent, # Parent  
                            joint.child,    #Child
                            xyz, # Translation
                            quat, # Rotation
                            joint.type, # Type of joint
                            joint.axis, # Axis of Joint
                            None, # Lower Limit
                            None, # Upper Limit 
                            np.linalg.norm(xyz) #Length of link
                        )
                    kin_chains.append(wkc)

        return childs
    
    T = np.eye(4)
    q = tf.quaternion_from_matrix(T)

    joint_info[base_link] = (
        None,
        None,
        np.array([0,0,0]),
        q,
        'fixed',
        None,
        None,
        None,
        None
    )
    get_child_joints(base_link, transform, kc)

    return joint_info, kin_chains

def load_urdf(robot_dir, verbose = False):
    """ Load urdf using urdf parser"""
    # Load the URDF
    robot = URDF.from_xml_file(robot_dir)

    # Iterating over joints to get their initial transforms
    if (verbose):
        for joint in robot.joints:
            parent = joint.parent
            child = joint.child
            transform = joint.origin
            
            print(f"Joint {joint.name}:")
            print(f"  Parent: {parent}")
            print(f"  Child: {child}")
            print(f"  Transform:\n{transform}")
            #print(robot.joint_map)
            print()
    return robot

def load_pickle(fname):
    """Loads pickle file from disk
    
    Args:
        fname: path to file
    """
    with open(fname, 'rb') as pf:
        data = pickle.load(pf)
    return data

def save_pickle(fname, d):
    """Save pickle file from disk
    
    Args:
        fname: path to file
        d: data to store
    """
    with open(fname, 'wb') as pf:
        pickle.dump(d, pf)
    return 

def InverseT(x):
    """Returns the inverse of a Transformation Matrix (4x4)
    
    Args:
        x: T matrix
    """
    R = x[:3,:3]
    p = x[:,3][:3]
    RT = R.T
    ip = np.matmul(-1 * RT, p)
    y = np.array([[R.T[0,0], R.T[0,1], R.T[0,2], ip[0]],
                  [R.T[1,0], R.T[1,1], R.T[1,2], ip[1]],
                  [R.T[2,0], R.T[2,1], R.T[2,2], ip[2]],
                  [       0,        0,        0,     1]])
    return y

def re(R_est, R_gt):
    """
    Rotational Error.

    :param R_est: Rotational element of the estimated pose (3x1 vector).
    :param R_gt: Rotational element of the ground truth pose (3x1 vector).
    :return: Error of t_est w.r.t. t_gt.
    """
    assert(R_est.shape == R_gt.shape == (3, 3))
    error_cos = 0.5 * (np.trace(R_est.dot(np.linalg.inv(R_gt))) - 1.0)
    error_cos = min(1.0, max(-1.0, error_cos)) # Avoid invalid values due to numerical errors
    error = math.acos(error_cos)
    error = 180.0 * error / np.pi # [rad] -> [deg]
    return error

def re_batch(R_est, R_gt):
    """
    Rotational Error.

    :param R_est: Rotational element of the estimated pose (3x1 vector).
    :param R_gt: Rotational element of the ground truth pose (3x1 vector).
    :return: Error of t_est w.r.t. t_gt.
    """
    if R_est.ndim <3:
        R_est = np.expand_dims(R_est, axis=0)
        R_gt = np.expand_dims(R_gt, axis=0)
    assert(R_est.shape[0] == R_gt.shape[0])

    # Calculate the dot product of the two sets of rotation matrices
    dot_products = 0.5 * (np.einsum('ijk,ikj->i', R_est, np.linalg.inv(R_gt))-1)

    # Ensure dot products are within the valid range [-1, 1]
    dot_products = np.clip(dot_products, -1.0, 1.0)

    # Calculate the angles using arccosine and convert to degrees
    angles = np.degrees(np.arccos((dot_products)))

    return angles

def te(t_est, t_gt):
    """
    Translational Error.

    :param t_est: Translation element of the estimated pose (3x1 vector).
    :param t_gt: Translation element of the ground truth pose (3x1 vector).
    :return: Error of t_est w.r.t. t_gt.
    """
    assert(t_est.size == t_gt.size == 3)
    error = np.linalg.norm(t_gt - t_est)
    return error

def R_t_from_tf(T):
    "Returns the Rotational matrix and the translation element of T matrix"
    R = T[:3,:3]
    t = T[:,3][:3]
    return R,t

def te_batch(t_est, t_gt):
    """
    Translational Error.

    :param t_est: Translation element of the estimated pose (3x1 vector).
    :param t_gt: Translation element of the ground truth pose (3x1 vector).
    :return: Error of t_est w.r.t. t_gt.
    """
    #print(t_est.shape, t_gt.shape)
    if(len(t_est.shape)>2):
        t_est = np.squeeze(t_est,axis=1)
        t_gt = np.squeeze(t_gt,axis=1)
    #assert(t_est.shape[1] == t_gt.shape[1]== 3)
    error = np.linalg.norm(t_gt - t_est,axis=1)
    return error

def spherical_coord_interpolation(base_points, query_points, poles= None):
    """ #!! Changee to lambda (longitude) and phi (latitude)
    Interpolate radii for points on a sphere using spherical coordinates in radians, 
    accounting for periodicity in both theta and phi. 

    Parameters:
    - base_points: numpy array of shape (n, 3) where each row is (r, theta_rad, phi_rad) 
                   in radians for a base point. 
    - poles: numpy array of shape (p, 2) where each row is (r, theta_rad) 
                in radians for poles we what to use to interpolate.
    - query_points: numpy array of shape (m, 2) where each row is (theta_rad, phi_rad) 
                    in radians for points where we want to interpolate.

    Returns:
    - interpolated_radii: numpy array of shape (m,) with interpolated radii for query points.
    """
    
    # Repeat poles for consistent near pole interpolation
    if poles is not None:
        n_poles = poles.shape[0]
        pole_phi = np.unique(base_points[:,2])
        poles = np.repeat(poles, pole_phi.size, axis=0)
        pole_phi = np.tile(pole_phi, n_poles)
        poles = np.column_stack([poles,pole_phi])
        #print(poles)

        base_points = np.vstack([
            base_points,
            poles
        ])

    # Extract values
    base_radii, base_theta, base_phi = base_points[:,0], base_points[:, 1], base_points[:, 2]
    query_theta, query_phi = query_points[:, 0], query_points[:, 1]

    # Create extended sets of points for wrapping

    # For phi: wrap around by adding and subtracting 2 pi
    base_points_extended = np.vstack([
        base_points[:,1:],
        np.column_stack([base_theta, base_phi + 2*np.pi]),  # Wrapping theta up
        np.column_stack([base_theta, base_phi - 2*np.pi])   # Wrapping theta down
    ])

    # Extend radii to match extended points 
    base_radii_extended = np.tile(base_radii, 3)  
    print(base_points_extended, base_radii_extended)
    
    # For phi: wrap around, note phi becomes starts to be redundant at 0 and pi
    base_points_extended = np.vstack([
        base_points_extended,
        np.column_stack([-base_points_extended[:, 0] + 2*np.pi, base_points_extended[:, 1] + np.pi]),
        np.column_stack([-base_points_extended[:, 0] + 2*np.pi, base_points_extended[:, 1] - np.pi]),
        np.column_stack([-base_points_extended[:, 0], base_points_extended[:, 1] + np.pi]),
        np.column_stack([-base_points_extended[:, 0], base_points_extended[:, 1] - np.pi])
    ])
    

    base_radii_extended = np.tile(base_radii_extended, 5)  

    # Perform interpolation
    interpolated_radii = griddata(base_points_extended, base_radii_extended, 
                                  np.column_stack([query_theta, query_phi]), 
                                  method='linear', fill_value=np.nan)
    
    return interpolated_radii

def spherical_to_cartesian(spherical_coords):
    """ #!! Changee to lambda (longitude) and phi (latitude)
    Convert from spherical coordinates to Cartesian coordinates for arrays of shape (n, 3).

    Parameters:
    - spherical_coords: NumPy array of shape (n, 3) where each row is [r, theta, phi].
        - r: Radius from the origin.
        - theta: Polar angle from the positive z-axis (0 to pi).
        - phi: Azimuthal angle in the x-y plane from the positive x-axis (0 to 2*pi).

    Returns:
    - cartesian_coords: NumPy array of shape (n, 3) where each row is [x, y, z].

    Note:
    - All angles should be in radians.
    """
    # Ensure input is a numpy array
    spherical_coords = np.asarray(spherical_coords)
    
    if spherical_coords.shape[1] != 3:
        raise ValueError("Input array must have 3 columns corresponding to [r, theta, phi]")

    r, theta, phi = spherical_coords[:, 0], spherical_coords[:, 1], spherical_coords[:, 2]
    
    # Compute Cartesian coordinates
    x = r * np.sin(theta) * np.sin(phi)
    y = r * np.sin(theta) * np.cos(phi)
    z = r * np.cos(theta)

    # Stack the results into one array
    cartesian_coords = np.column_stack((x, y, z))
    
    return cartesian_coords

def get_joint_transformations(joint_info, dof_names):
    """ Transformation functions given the types

    Args:
        - joint_info: Kinematic information from urdf
        - dof_names: dof names that will be used in Isaac Sim order 
    """
    def RevoluteX(a):
        T = np.array([
            [1, 0, 0, 0], 
            [0, np.cos(a), np.sin(-a), 0], 
            [0, np.sin(a), np.cos(a), 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def RevoluteY(a):
        T = np.array([
            [np.cos(a), 0, np.sin(a), 0], 
            [0, 1, 0, 0], 
            [np.sin(-a), 0, np.cos(a), 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def RevoluteZ(a):
        T = np.array([
            [np.cos(a), np.sin(-a), 0, 0], 
            [np.sin(a), np.cos(a), 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def PrismaticX(a):
        T = np.array([
            [1, 0, 0, a], 
            [0, 1, 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def PrismaticY(a):
        T = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, a], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def PrismaticZ(a):
        T = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, 0], 
            [0, 0, 1, a], 
            [0, 0, 0, 1], 
        ])
        return T

    T_funcs = []
    for dof in dof_names:
        axis = np.argmax(joint_info[dof][5])
        if joint_info[dof][4] == "revolute":
            if axis == 0:
                T_funcs.append(RevoluteX)
            elif axis == 1:
                T_funcs.append(RevoluteY)
            elif axis == 2:
                T_funcs.append(RevoluteZ)
        elif joint_info[dof][4] == "prismatic":
            if axis == 0:
                T_funcs.append(PrismaticX)
            elif axis == 1:
                T_funcs.append(PrismaticY)
            elif axis == 2:
                T_funcs.append(PrismaticZ)
    return T_funcs

def get_inverse_joint_transformations(joint_info, dof_names):
    """ Transformation functions given the types

    Args:
        - joint_info: Kinematic information from urdf
        - dof_names: dof names that will be used in Isaac Sim order 
    """
    def RevoluteX(a):
        T = np.array([
            [1, 0, 0, 0], 
            [0, np.cos(a), np.sin(a), 0], 
            [0, np.sin(-a), np.cos(a), 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def RevoluteY(a):
        T = np.array([
            [np.cos(a), 0, np.sin(-a), 0], 
            [0, 1, 0, 0], 
            [np.sin(a), 0, np.cos(a), 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def RevoluteZ(a):
        T = np.array([
            [np.cos(a), np.sin(a), 0, 0], 
            [np.sin(-a), np.cos(a), 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def PrismaticX(a):
        T = np.array([
            [1, 0, 0, -a], 
            [0, 1, 0, 0], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def PrismaticY(a):
        T = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, -a], 
            [0, 0, 1, 0], 
            [0, 0, 0, 1], 
        ])
        return T
    def PrismaticZ(a):
        T = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, 0], 
            [0, 0, 1, -a], 
            [0, 0, 0, 1], 
        ])
        return T

    T_funcs = []
    for dof in dof_names:
        axis = np.argmax(joint_info[dof][5])
        if joint_info[dof][4] == "revolute":
            if axis == 0:
                T_funcs.append(RevoluteX)
            elif axis == 1:
                T_funcs.append(RevoluteY)
            elif axis == 2:
                T_funcs.append(RevoluteZ)
        elif joint_info[dof][4] == "prismatic":
            if axis == 0:
                T_funcs.append(PrismaticX)
            elif axis == 1:
                T_funcs.append(PrismaticY)
            elif axis == 2:
                T_funcs.append(PrismaticZ)
    return T_funcs

if __name__ == "__main__":
    robot_dir = "/home/felipe/Documents/isaac_sim_grasping/grippers/Barrett/Barrett.urdf"
    info_dir = "/home/felipe/Documents/isaac_sim_grasping/grippers/Barrett/sphere_info.pk"
    joint_info, kin_chains = get_unfixed_kin_chains(robot_dir)
    
    # Sphere Frame Calculation
    t = np.array([0,0,0.055])
    R = np.array([
        [1, 0, 0],
        [0,-1, 0],
        [0, 0,-1]
    ],dtype=np.float64)
    T = np.eye(4)
    T[:3,:3]= R

    #print(T)
    quat = tf.quaternion_from_matrix(T)
    # Ensure axis_in_parent is a unit vector
    joint_info["sphere_frame"] = (
        kin_chains[0][0],
        None,
        t,
        quat,
        "fixed",
        None,
        None,
        None,
        np.linalg.norm(t)
    )
    merged_transform = np.eye(4)
    
    finger_info = dict()
    for joint in joint_info.keys():
        print(joint, ": ", joint_info[joint])
    print()

    base_joints = [
        'a_link1_link2_joint',
        'b_link1_link2_joint',
        'c_palm_link2_joint_0'
    ]
    
    for i, kc in enumerate(kin_chains):
        index = 0
        finger_info[str(i)] = dict()
        finger_info[str(i)]["kc"] = kc
        for j, item in enumerate(kc):
            if item in base_joints:
                index = j
                break
        if index == 0:
            raise NameError("No base finger joint found")
        finger_info[str(i)]["finger_plane_joint_index"] = j
        #Add finger width
        finger_info[str(i)]["finger_width"] = 0.01
    
    for finger in finger_info.keys():
        print(finger, ": ", finger_info[finger])
    print()
    
    info = dict()
    info["kinematics"] = joint_info
    info["finger_chains"] = finger_info
    save_pickle(info_dir, info)
    
    data = load_pickle(info_dir)
    print(data)