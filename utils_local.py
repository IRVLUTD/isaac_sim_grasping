import numpy as np
import math
import pickle
#from omni.isaac.urdf import _urdf

def load_pickle(fname):
    """Loads pickle file from disk
    
    Args:
        fname: path to file
    """
    with open(fname, 'rb') as pf:
        data = pickle.load(pf)
    return data


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

def spherical_to_cartesian(spherical_coords, z_offset=0):
    """ #!! Changee to lambda (longitude) and phi (latitude)
    Convert from spherical coordinates to Cartesian coordinates for arrays of shape (n, 3).

    Parameters:
    - spherical_coords: NumPy array of shape (n, 3) where each row is [r, theta, phi].
        - r: Radius from the origin.
        - theta: Polar angle from the positive z-axis (0 to pi).
        - phi: Azimuthal angle in the x-y plane from the positive x-axis (0 to 2*pi).
    - z_offset: Useful for when the sphere center is moved in the UGCS controller (top pole),
        must be correctly calculated beforehand.

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
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta) + z_offset

    # Stack the results into one array
    cartesian_coords = np.column_stack((x, y, z))
    
    return cartesian_coords
