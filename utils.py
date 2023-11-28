import numpy as np
import omni.kit.commands
import math
import pickle
#from omni.isaac.urdf import _urdf

def load_pickle(fname):
    with open(fname, 'rb') as pf:
        data = pickle.load(pf)
    return data


def InverseT(x):
    R = x[:3,:3]
    p = x[:,3][:3]
    RT = R.T
    ip = np.matmul(-1 * RT, p)
    y = np.array([[R.T[0,0], R.T[0,1], R.T[0,2], ip[0]],
                  [R.T[1,0], R.T[1,1], R.T[1,2], ip[1]],
                  [R.T[2,0], R.T[2,1], R.T[2,2], ip[2]],
                  [       0,        0,        0,     1]])
    return y

def import_urdf(manager, job):
    """ Import URDF from manager.grippers dictionary

    Args: None
    """
    #Adding a .urdf file
    urdf_interface = _urdf.acquire_urdf_interface()
    # Set the settings in the import config
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True
    import_config.make_default_prim = True
    import_config.self_collision = False
    import_config.create_physics_scene = True
    import_config.import_inertia_tensor = True
    import_config.default_drive_strength = 1047.19751
    import_config.default_position_drive_damping = 52.35988
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.distance_scale = 1
    import_config.density = 0.0

    urdf_path = manager.object_dict[job['object_id']]
    result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path=urdf_path,import_config=import_config,)
    return

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
    assert(R_est.shape[0] == R_gt.shape[0])
    R= R_est[0].dot(np.linalg.inv(R_gt[0]))
    print("R", R)
    error_cos = 0.5 * (np.trace(R) - 1.0)

    print("error_cos", error_cos)
    error_cos = min(1.0, max(-1.0, error_cos)) # Avoid invalid values due to numerical errors
    print("error_cos", error_cos)

    error = math.acos(error_cos)
    print("error", error)
    error = 180.0 * error / np.pi # [rad] -> [deg]
    print(np.linalg.inv(R_gt).shape)
    R = np.tensordot(R_est, np.linalg.inv(R_gt), axes = ([1,2],[2,1]))
    print("R", R)
    print(R.shape)
    error_cos = 0.5 * (np.trace(R) - 1.0)
    print("error_cos", error_cos)
    error_cos = np.minimum(1.0, np.maximum(-1.0, error_cos)) # Avoid invalid values due to numerical errors
    print("error_cos", error_cos)
    error = np.arccos(error_cos)
    print("error", error)
    error = 180.0 * error / np.pi # [rad] -> [deg]


    return error


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