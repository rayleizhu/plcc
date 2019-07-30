import numpy as np
import yaml
import os

def read_yaml_file():
    current_path = os.path.dirname(os.path.realpath(__file__))
    # 获取yaml文件路径
    yaml_path = os.path.join(current_path, "camera_camera_calib.yaml")
    # 读取yaml文件
    f = open(yaml_path, 'r', encoding='utf-8')
    d = yaml.safe_load(f)  # 用load方法转字典
    print(d)
    m_params = extract_params(d)
    return m_params


def extract_params(dict_d):
    m_params = {}
    # camera
    m_params['image_width'] = dict_d['image_width']
    m_params['image_height'] = dict_d['image_height']
    # mirror_parameters
    m_params['m_xi'] = dict_d['mirror_parameters']['xi']
    # distortion_parameters
    m_params['m_k1'] = dict_d['distortion_parameters']['k1']
    m_params['m_k2'] = dict_d['distortion_parameters']['k2']
    m_params['m_p1'] = dict_d['distortion_parameters']['p1']
    m_params['m_p2'] = dict_d['distortion_parameters']['p2']
    # projection_parameters
    m_params['m_gamma1'] = dict_d['projection_parameters']['gamma1']
    m_params['m_gamma2'] = dict_d['projection_parameters']['gamma2']
    m_params['m_u0'] = dict_d['projection_parameters']['u0']
    m_params['m_v0'] = dict_d['projection_parameters']['v0']
    return m_params


def mei_model_camera_constructor(m_params):
    m_inv_params = {}
    # Inverse camera projection matrix parameters
    m_inv_params['m_inv_K11'] = 1.0 / m_params['m_gamma1']
    m_inv_params['m_inv_K13'] = -m_params['m_u0'] / m_params['m_gamma1']
    m_inv_params['m_inv_K22'] = 1.0 / m_params['m_gamma2']
    m_inv_params['m_inv_K23'] = -m_params['m_v0'] / m_params['m_gamma2']
    return m_inv_params


def distortion(p_u, m_params):
    k1 = m_params['m_k1']
    k2 = m_params['m_k2']
    p1 = m_params['m_p1']
    p2 = m_params['m_p2']

    mx2_u = p_u[0] * p_u[0]
    my2_u = p_u[1] * p_u[1]
    mxy_u = p_u[0] * p_u[1]
    rho2_u = mx2_u + my2_u
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u
    d_u = [p_u[0] * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u[1] * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u)]
    return d_u


def space_to_plane_mei(p_3d, m_params):
    p_img = np.zeros((len(p_3d), 2))
    for i in range(len(p_3d)):
        z = p_3d[i, 2] + m_params['m_xi'] * np.linalg.norm(p_3d[i, :])
        # print(z)
        p_u = [p_3d[i, 0] / z, p_3d[i, 1] / z]
        # print(p_u)
        d_u = distortion(p_u, m_params)
        p_d = p_u + d_u
        # image point coordinates
        p_img[i, :] = np.array([m_params['m_gamma1'] * p_d[0] + m_params['m_u0'], m_params['m_gamma2'] * p_d[1] + m_params['m_v0']])
        # print(p_img)
    return p_img


__name__ = "mei_model"
