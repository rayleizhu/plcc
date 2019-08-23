from __future__ import print_function
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

#https://docs.opencv.org/master/d9/d6a/group__aruco.html
#https://longervision.github.io/2017/03/13/OpenCV/opencv-external-posture-estimation-ChArUco-board/

def get_apriltag_dict():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

def get_tag_cam_tf_from_apriltag(img, aruco_dict, marker_len, cam_mat, dist):
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img,
                                                              aruco_dict,
                                                              cameraMatrix=cam_mat,
                                                              distCoeff=dist)
    if ids is not None:
        img_with_markers = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                              marker_len,
                                                              cam_mat,
                                                              dist)
        r_quats = R.from_rotvec(rvecs.squeeze(1)).as_quat()
        t = tvecs.squeeze(1)
        return r_quats, t, img_with_markers
    else:
        raise ValueError('No markers detected!')

# square_len 90mm adapts to the size of A3 paper
def get_charuco_board_and_dict(id, board_size=(3, 4), marker_size=6, square_len=0.090, marker_len=0.072):
    size_dict = board_size[0]*board_size[1]
    aruco_dict = cv2.aruco.custom_dictionary(size_dict, marker_size, id)
    board = cv2.aruco.CharucoBoard_create(board_size[0],
                                          board_size[1],
                                          square_len,
                                          marker_len,
                                          aruco_dict)
    return board, aruco_dict


def get_tag_cam_tf_from_board(img, charuco_board, aruco_dict, cam_mat, dist):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img_gray,
                                                              aruco_dict,
                                                              cameraMatrix=cam_mat,
                                                              distCoeff=dist)
    corners, ids, rejectedImgPoints, _ = cv2.aruco.refineDetectedMarkers(img_gray,
                                                                         charuco_board,
                                                                         corners,
                                                                         ids,
                                                                         rejectedImgPoints,
                                                                         cameraMatrix=cam_mat,
                                                                         distCoeffs=dist)
    if ids is not None:
        charucoretval, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners,
                                                                                        ids,
                                                                                        img_gray,
                                                                                        charuco_board,
                                                                                        cameraMatrix=cam_mat,
                                                                                        distCoeffs=dist)
        img_with_corner = cv2.aruco.drawDetectedCornersCharuco(img, charucoCorners, charucoIds, (0,255,0))
        retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charucoCorners,
                                                                charucoIds,
                                                                charuco_board,
                                                                cameraMatrix=cam_mat,
                                                                distCoeffs=dist)
        if retval == True:
            axis_size = charuco_board.getSquareLength() * charuco_board.getChessboardSize()[0]
            img_with_axis = cv2.aruco.drawAxis(img_with_corner,
                                               cam_mat,
                                               dist,
                                               rvec,
                                               tvec,
                                               axis_size)
            r_quat = R.from_rotvec(rvec.reshape(-1)).as_quat()
            t = tvec.reshape(-1)
            tf = np.concatenate([t, r_quat], axis=0)
            return img_with_axis, tf
        else:
            raise ValueError('Failed to estimate pose of camera.')
    else:
        raise ValueError('No aruco pattern detected!')
        


if __name__ == '__main__':
    # import pandas as pd
    # board, aruco_dict = get_charuco_board_and_dict(2)
    # # img = board.draw((600, 800))
    # # cv2.imwrite('../data/output/chArucoBoard48.png', img)
    # img_left = cv2.imread('/home/rayleizhu/Data/LidarCamCalib/data0819/img_left.png')
    # cam_mat_left = np.array([671.8704223632812, 0.0, 652.4547729492188, 0.0, 671.8704223632812, 353.10394287109375, 0.0, 0.0, 1.0]).reshape((3,3))
    # dist_left = np.array([0.0, 0.0, 0.0, 0.0])
    # img_with_axis_left, r_left, t_left = get_tag_cam_tf_from_img(img_left, board, aruco_dict, cam_mat_left, dist_left)
    # cv2.imwrite('../data/output/img_with_axis_left.png', img_with_axis_left)
    # r_left = R.from_rotvec(r_left.reshape(-1)).as_dcm().T
    # t_left = -np.dot(r_left, t_left.reshape(-1))
    # r_left = R.from_dcm(r_left).as_quat()

    # output_left = np.append(t_left, r_left)
    # df_left = pd.DataFrame(data=np.expand_dims(output_left, axis=0), columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    # df_left.to_csv('../data/output/tf_left.csv', index=False)

    # img_right = cv2.imread('/home/rayleizhu/Data/LidarCamCalib/data0819/img_right.png')
    # cam_mat_right = np.array([671.8704223632812, 0.0, 652.4547729492188, 0.0, 671.8704223632812, 353.10394287109375, 0.0, 0.0, 1.0]).reshape((3,3))
    # dist_right = np.array([0.0, 0.0, 0.0, 0.0])
    # img_with_axis_right, r_right, t_right = get_tag_cam_tf_from_img(img_right, board, aruco_dict, cam_mat_right, dist_right)
    # cv2.imwrite('../data/output/img_with_axis_right.png', img_with_axis_right)
    # r_right = R.from_rotvec(r_right.reshape(-1)).as_dcm().T
    # t_right = -np.dot(r_right, t_right.reshape(-1))
    # r_right = R.from_dcm(r_right).as_quat()

    # output_right = np.append(t_right, r_right)
    # df_right = pd.DataFrame(data=np.expand_dims(output_right, axis=0), columns=['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    # df_right.to_csv('../data/output/tf_right.csv', index=False)

    img = cv2.imread('../data/output/orig.png')
    ap_dict = get_apriltag_dict()
    marker_len = 0.167
    cam_mat = np.array([671.8704223632812, 0.0, 652.4547729492188, 0.0, 671.8704223632812, 353.10394287109375, 0.0, 0.0, 1.0]).reshape((3,3))
    dist = np.array([0.0, 0.0, 0.0, 0.0])
    rvecs, tvecs, img_with_markers = get_tag_cam_tf_from_apriltag(img, ap_dict, marker_len, cam_mat, dist)
    cv2.imwrite('../data/output/img_with_markers.png', img_with_markers)
    print(rvecs, tvecs)