import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from cv2 import aruco
import json
from camera_calibration_mono import calibration_intrinsic
# Instantiate CvBridge
bridge = CvBridge()

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 5
CHARUCOBOARD_COLCOUNT = 5
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Board origin wrt to coc
T_board = np.array([[1.68],[-.30],[.70]])
R_board,_ = cv2.Rodrigues(np.float32([[0,0,-1],[1,0,0],[0,-1,0]]))

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = aruco.CharucoBoard_create(
    squaresX=CHARUCOBOARD_COLCOUNT,
    squaresY=CHARUCOBOARD_ROWCOUNT,
    squareLength=0.12,
    markerLength=0.06,
    dictionary=ARUCO_DICT)

# Returns inverse of rvec and tvec
def inversePerspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(-R, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(R)
    return invRvec, invTvec

# relative transformation
def relativePosition(rvec1, tvec1, rvec2, tvec2):
    rvec1 = np.array(rvec1).reshape((3, 1))
    rvec2 = np.array(rvec2).reshape((3, 1))
    tvec1 = np.array(tvec1).reshape((3, 1))
    tvec2= np.array(tvec2).reshape((3, 1))


    invRvec, invTvec = inversePerspective(rvec2, tvec2)

    orgRvec, orgTvec = inversePerspective(invRvec, invTvec)
    #print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]

    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec

# Returns pose wrt to coc
def coc_centered(cam,mat,dist,board_coc_rvec,board_coc_tvec):
    # pose of board wrt to camera frame
    err1, rvecs, tvecs, _ = cv2.solvePnPRansac(np.array(cam.objpts)[:1][0],
                                                                 cam.corners_all[:1][0], mat, dist)
    # pose of camera optical centre wrt to board origin
    rvecs_inv, tvecs_inv = inversePerspective(rvecs, tvecs)
    
    # pose of coc wrt board origin
    coc_board_rvec, coc_board_tvec = inversePerspective(board_coc_rvec, board_coc_tvec)
    
    # pose of camera optical center wrt to coc
    R, T = relativePosition(rvecs_inv, tvecs_inv, coc_board_rvec,coc_board_tvec)
    R,_ = cv2.Rodrigues(R)
    return R,T



if __name__ == "__main__":
    rospy.init_node('calibration_to_coc')
    # Define your image topic
    zed_left= calibration_intrinsic(CHARUCO_BOARD,"/freicar_6/zed/left_raw/image_raw_color")
    zed_right= calibration_intrinsic(CHARUCO_BOARD,"/freicar_6/zed/right_raw/image_raw_color")
    d435_color= calibration_intrinsic(CHARUCO_BOARD,"/freicar_6/d435/color/image_raw")

    # collect data
    zed_right.collect_data(1)
    zed_left.collect_data(1)
    d435_color.collect_data(1)

    # Load intrinsic values
    with open('camera_intrinsics.json') as file:
        data = json.load(file)
        print(data)
        mat1 = np.array(data['d435_color']['camera_matrix'])
        dist1 = np.array(data['d435_color']['dist_coeff'])
        mat2 = np.array(data['zed_left']['camera_matrix'])
        dist2 = np.array(data['zed_left']['dist_coeff'])
        mat3 = np.array(data['zed_right']['camera_matrix'])
        dist3 = np.array(data['zed_right']['dist_coeff'])


    # Calculate pose
    zed_left_rvecs,zed_left_tvecs = coc_centered(zed_right,mat3,dist3,R_board,T_board)
    zed_right_rvecs, zed_right_tvecs = coc_centered(zed_left, mat2, dist2, R_board, T_board)
    d435_color_rvecs, d435_color_tvecs = coc_centered(d435_color, mat1, dist1, R_board, T_board)

    data = {
        "zed_left": {"R": zed_left_rvecs.tolist(), "T": zed_left_tvecs.tolist()},\
    "zed_right": {"R": zed_right_rvecs.tolist(), "T": zed_right_tvecs.tolist()},\
    "d435_color": {"R": d435_color_rvecs.tolist(), "T": d435_color_tvecs.tolist()}}

    # Write data
    with open("coc_centered.json", "w") as file:
        json.dump(data, file)


    rospy.signal_shutdown("task_completed")






