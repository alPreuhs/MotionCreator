import math
import numpy as np
from Math import ProjectiveGeometry as pg


def create_default_projection_matrix(rao_lao_ang = 0, cran_caud_ang = 0, img_rot_ang = 0, pixel_spacing =0.308, offset_u=512, offset_v = 512, sid = 1200, sisod = 750):
    rao = math.radians(rao_lao_ang)
    cran = math.radians(cran_caud_ang)
    img_rot = math.radians(img_rot_ang)
    K = get_K(sid,pixel_spacing, offset_u, offset_v)
    R_z_180 = get_Pinhole_correction()
    R_x_90 = get_rx90()
    T = get_translation(sisod)
    R = get_rotation(rao,cran,imgRot=img_rot)

    R_horizontal = np.matrix(np.zeros(shape=(4, 4)))
    R_horizontal[0, 0] = 1
    R_horizontal[1, 1] = -1
    R_horizontal[2, 2] = 1
    R_horizontal[3, 3] = 1
    P = K  * R_z_180 * R_x_90 * T * R
    return P


def get_uv_point_in_xyz(point_uv,projMat,sid):
    p_i = np.linalg.pinv(projMat)
    point_bp_p = point_uv.backproject(p_i)
    line = point_bp_p.join(get_source_position(projMat))
    det_plane = p3(projMat).get_plane_at_distance(sid)
    return line.meet(det_plane)


def get_K(sid,pixel_spacing, offset_u, offset_v):
    d = sid
    p = pixel_spacing
    ou = offset_u
    ov = offset_v
    K = np.matrix(np.zeros(shape=(3,4)))
    K[0,0] = d/p
    K[1,1] = d/p
    K[0,2] = ou
    K[1,2] = ov
    K[2,2] = 1
    return K


def p1(p):
    return pg.plane_p3(p[0, :])


def p2(p):
    return pg.plane_p3(p[1, :])


def p3(p):
    return pg.plane_p3(p[2, :])


def sp(p):
    return get_source_position(p)


def get_source_position(p):
    return (p1(p).meet(p2(p))).meet(p3(p))


def get_Pinhole_correction():
    R = np.matrix(np.zeros(shape=(4,4)),dtype=np.float64)
    R[0,0] = -1
    R[1,1] = -1
    R[2,2] = 1
    R[3,3] = 1
    return R


def get_rx90():
    R = np.matrix(np.zeros(shape=(4,4)),dtype=np.float64)
    R[0,0] = 1
    R[1,2] = 1
    R[2,1] = -1
    R[3,3] = 1
    return R


def get_horizontal_flip():
    R_horizontal = np.matrix(np.zeros(shape=(4, 4)),dtype=np.float64)
    R_horizontal[0, 0] = 1
    R_horizontal[1, 1] = -1
    R_horizontal[2, 2] = 1
    R_horizontal[3, 3] = 1
    return R_horizontal


def get_translation(sisod):
    d = sisod
    T = np.matrix(np.zeros(shape=(4,4)),dtype=np.float64)
    T[0,0] = 1
    T[1,1] = 1
    T[2,2] = 1
    T[3,3] = 1
    T[1,3] = -d
    return T


def get_rotation(alpha, beta, imgRot = 0):
    def s(x):
        return math.sin(x)

    def c(x):
        return math.cos(x)


    ##Rz
    rP = np.matrix(np.zeros(shape=(4, 4)),dtype=np.float64)
    rP[0, 0] = c(alpha)
    rP[0, 1] = s(alpha)
    rP[1, 0] = -s(alpha)
    rP[1, 1] = c(alpha)
    rP[2, 2] = 1.0
    rP[3, 3] = 1.0


    ##Rx
    rS = np.matrix(np.zeros(shape=(4, 4)),dtype=np.float64)
    rS[0, 0] = 1.0
    rS[1, 1] = c(-beta)
    rS[1, 2] = s(-beta)
    rS[2, 1] = -s(-beta)
    rS[2, 2] = c(-beta)
    rS[3, 3] = 1.0

    ##Ry
    rT = np.matrix(np.zeros(shape=(4, 4)),dtype=np.float64)
    rT[1, 1] = 1.0
    rT[0, 0] = c(imgRot)
    rT[0, 2] = s(imgRot)
    rT[2, 0] = -s(imgRot)
    rT[2, 2] = c(imgRot)
    rT[3, 3] = 1.0
    return rT*rS*rP


def rodriguez(axis : np.matrix, angle_radians, make_matrix_homogen = False):
    return get_rotation_matrix_by_axis_and_angle(axis, math.degrees(angle_radians), make_matrix_homogen=make_matrix_homogen)

def get_rotation_matrix_by_axis_and_angle(axis, angle_deg, make_matrix_homogen = False):
    """
    This method is kind of depricated...we do not want to give angles as function input. We always expect radians.
    However, this function takes angles...
    Use rodriguez instead
    :param axis:
    :param angle_deg:
    :param make_matrix_homogen:
    :return:
    """
    angle_rad = angle_deg*math.pi/180
    cos_theta = math.cos(angle_rad)
    sin_theta = math.sin(angle_rad)
    axis = axis/np.linalg.norm(axis)

    ux = axis[0,0]
    uy = axis[1,0]
    uz = axis[2,0]
    u_skew = np.matrix([[0,-uz,uy],[uz,0,-ux],[-uy ,ux ,0]])
    u_tensor = np.matrix([[ux*ux, ux*uy, ux*uz],[ ux*uy, uy*uy, uy*uz],[ux*uz,uy*uz,uz*uz]])
    id = np.matrix(np.diag(np.ones(3)))
    r = cos_theta* id + sin_theta * u_skew + (1-cos_theta) * u_tensor
    if make_matrix_homogen:
        R = np.matrix(np.zeros(shape=(4, 4)))
        R[0, 0:3] = r[0, :]
        R[1, 0:3] = r[1, :]
        R[2, 0:3] = r[2, :]
        R[3, 3] = 1
        return R
    else:
        return r

def rx(radians, homo=False):
    R = np.matrix(np.diag([1.0,1.0,1.0,1.0]))
    R[1,1] = math.cos(radians)
    R[2,2] = math.cos(radians)
    R[1,2] = -math.sin(radians)
    R[2,1] = math.sin(radians)
    if homo:
        return R
    else:
        return R[0:3, 0:3]

def ry(radians, homo=False):
    R = np.matrix(np.diag([1.0, 1.0, 1.0, 1.0]))
    R[0, 0] = math.cos(radians)
    R[2, 2] = math.cos(radians)
    R[0, 2] = math.sin(radians)
    R[2, 0] = -math.sin(radians)
    if homo:
        return R
    else:
        return R[0:3, 0:3]

def rz(radians, homo=False):
    R = np.matrix(np.diag([1.0, 1.0, 1.0, 1.0]))
    R[0, 0] = math.cos(radians)
    R[1, 1] = math.cos(radians)
    R[0, 1] = -math.sin(radians)
    R[1, 0] = math.sin(radians)
    if homo:
        return R
    else:
        return R[0:3,0:3]




def detector_meet_points_tmp(ul_detector,ur_detector,ll_detector,lr_detector,line):
    ul = ul_detector.e()
    ur = ur_detector.e()
    ll = ll_detector.e()

    left_right_tmin = ul[1]
    left_right_tmax = ll[1]
    up_down_min = ul[0]
    up_down_max = ur[0]

    left = ul_detector.join(ll_detector)
    right = ur_detector.join(lr_detector)
    down = ll_detector.join(lr_detector)
    up = ul_detector.join(ur_detector)

    meet_points = []
    left_meet = line.meet(left)
    lme = left_meet.get_euclidean_point()
    if lme[1]>left_right_tmin and lme[1]<left_right_tmax:
        meet_points.append(lme)
    right_meet = line.meet(right)
    rme= right_meet.get_euclidean_point()
    if rme[1] > left_right_tmin and rme[1] < left_right_tmax:
        meet_points.append(rme)

    down_meet = line.meet(down)
    dme = down_meet.get_euclidean_point()
    if dme[0] > up_down_min and dme[0] < up_down_max:
        meet_points.append(dme)

    up_meet = line.meet(up)
    ume =up_meet.get_euclidean_point()
    if ume[0] > up_down_min and ume[0] < up_down_max:
        meet_points.append(ume)

    return meet_points

