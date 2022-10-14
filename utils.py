import numpy as np
import quaternion as npq

def get_trans_quat_hpp(matrix, shift=None):
    """
    Convert 4 x 4 numpy matrix to list of translation and quaternion in (x, y, z, w) format used by hpp

    :param: matrix: 4 x 4 numpy array
    :param shift: optional shift
    :return: list of [translation, quaternion]
    """
    if shift is not None:
        t = matrix[:3, 3] + np.array(shift)
    else:
        t = matrix[:3, 3]
    q = npq.as_float_array(npq.from_rotation_matrix(matrix[:3, :3]))
    return list(t) + [q[1], q[2], q[3], q[0]]

def get_trans_quat_pyphysx(matrix):
    """
    Convert 4 x 4 numpy matrix to tuple of translation and quaternion in (w, x, y, z) format
    :param matrix: 4 x 4 numpy array
    :return: tuple of translation and quaternion
    """
    t = matrix[:3, 3]
    q = npq.as_float_array(npq.from_rotation_matrix(matrix[:3, :3]))
    return t, q

def generate_joint_bounds_unlimited_rot(min_pos=None, max_pos=None):
    min_pos = -np.ones(3) if min_pos is None else min_pos
    max_pos = np.ones(3) if max_pos is None else max_pos
    pos_bound = np.concatenate([*zip(min_pos, max_pos)])
    rot_bound = np.array([-1.0001, 1.0001] * 4)

    return np.concatenate([pos_bound, rot_bound]).tolist()