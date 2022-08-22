import numpy as np
from scipy.spatial.transform import Rotation as R
import math
# Converts a 3x3 matrix into corresponding 4x4 affine matrix, with given translation vector
def to_44(mat_33, t = [0, 0, 0]):
    return np.concatenate(
                [np.concatenate(
                    [mat_33, [[0, 0, 0]]], 0
                ), [[t[0]], [t[1]], [t[2]], [1]]], 1
            )

# Converts a 4x4 matrix into corresponding 3x3 matrix
def to_33(mat_44):
    return mat_44[:3, :3]

# Converts a dope output from DopeReader into corresponding affine matrix
def dope_to_affine(dope, scale=1):
    return to_44(R.from_quat(dope[3:7]).as_dcm(), np.array(dope[:3]) * scale)

# Converts an ur5 tcp(6-dimensional) into corresponding affine matrix(L_B_W)
def tcp_to_affine(tcp):
    affine = R.from_rotvec(tcp[3:]).as_dcm()
    return to_44(affine, tcp[:3])

# Converts an affine matrix(L_B_W) into corresponding ur5 tcp
def affine_to_tcp(affine=None, dcm=None, t=None):
    if affine is not None:
        pos, rot = affine[:3, 3], affine[:3, :3]
    if dcm is not None and t is not None:
        pos, rot = t, dcm

    return np.concatenate([pos, R.from_dcm(rot).as_rotvec()])

# T : 4x4 matrix, or scipy.spatial.transform.Rotation
# position : 3d vector
# direction : 3d vector
# scipy_R : scipy.spatial.transform.Rotation
# dcm : 3x3 matrix
# affine : 4x4 matrix
# Applies transform T to a given position, direction, rotation or affine transformation
def transform(T, position=None, direction=None, scipy_R=None, dcm=None, affine=None):
    if isinstance(T, R):
        T = to_44(T.as_dcm())

    if position is not None:
        p = [position[0], position[1], position[2], 1]
        return np.matmul(T, p)[:3]
    if direction is not None:
        return np.matmul(T[:3, :3], direction)
    if scipy_R is not None:
        return np.matmul(T[:3, :3], scipy_R.as_dcm())
    if dcm is not None:
        return np.matmul(T[:3, :3], dcm[:3, :3])
    if affine is not None:
        return np.matmul(T, affine)

# Checks if a matrix is a valid rotation matrix.
def is_rotation_matrix(mat) :
    Rt = np.transpose(mat)
    shouldBeIdentity = np.dot(Rt, mat)
    I = np.identity(3, dtype = mat.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotation_matrix_to_euler_angles(rot_mat):
    assert(is_rotation_matrix(rot_mat))
    sy = math.sqrt(rot_mat[0,0] * rot_mat[0,0] +  rot_mat[1,0] * rot_mat[1,0])
    singular = sy < 1e-6

    if not singular :
        x = math.atan2(rot_mat[2, 1], rot_mat[2, 2])
        y = math.atan2(-rot_mat[2, 0], sy)
        z = math.atan2(rot_mat[1, 0], rot_mat[0, 0])
    else:
        x = math.atan2(-rot_mat[1, 2], rot_mat[1, 1])
        y = math.atan2(-rot_mat[2, 0], sy)
        z = 0

    return np.array([x, y, z])