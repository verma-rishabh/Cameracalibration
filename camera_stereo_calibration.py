import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from cv2 import aruco
from  camera_calibration_mono import calibration_intrinsic
import json

# Instantiate CvBridge
bridge = CvBridge()

# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 5
CHARUCOBOARD_COLCOUNT = 5
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = aruco.CharucoBoard_create(
    squaresX=CHARUCOBOARD_COLCOUNT,
    squaresY=CHARUCOBOARD_ROWCOUNT,
    squareLength=0.12,
    markerLength=0.06,
    dictionary=ARUCO_DICT)



class calibration_stereo:
    def __init__(self,charuco_board,image1_topic,image1_K,image1_D,image2_topic,image2_K,image2_D):

        self.image1 = calibration_intrinsic(charuco_board,image1_topic)                     
        self.image2 = calibration_intrinsic(charuco_board,image2_topic)
        self.image1.cameraMatrix = image1_K                                         # Camera matrix of camera1
        self.image1.distCoeffs = image1_D                                           # Distortion matrix of camera1
        self.image2.cameraMatrix = image2_K                                         # Camera matrix of camera2    
        self.image2.distCoeffs = image2_D                                           # Distortion matrix of camera2
        self.R = None                                                               # Rotation matrix 
        self.T = None                                                               # Translation matrix
        self.F = None                                                               # Fundamental matrix
        self.E = None                                                               # Essential matrix
        self.H = None                                                               # Homography matrix        

    # Collect images
    def collect_data(self,no_of_frames):
        self.image1.collect_data(no_of_frames)
        self.image2.collect_data(no_of_frames)

    # Stereo calibration
    def calibration(self):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        stereocalibration_flags = cv2.CALIB_FIX_INTRINSIC
        ret, CM1, dist1, CM2, dist2, self.R, self.T, self.E, self.F = cv2.stereoCalibrate(self.image1.objpts, self.image1.corners_all,
                                                                      self.image2.corners_all, self.image1.cameraMatrix, self.image1.distCoeffs ,
                                                                      self.image2.cameraMatrix, self.image2.distCoeffs, self.image1.image_size, criteria=criteria,
                                                                      flags=stereocalibration_flags)
        #print("Error:",ret)                                                        # reprojection error
        rows, cols= self.image1.image_size
        # Finding homography matrix
        _, H1, H2 = cv2.stereoRectifyUncalibrated(cv2.UMat(np.array(self.image1.corners_all[:1][0])),cv2.UMat(np.array(self.image2.corners_all[:1][0])), self.F, (cols, rows))
        self.H = H2.get() @ np.linalg.inv(H1.get())

if __name__ == "__main__":

    rospy.init_node('extrensic_calibration')
    # Load intrinsic data
    with open('camera_intrinsics_best.json') as file:
        data = json.load(file)
        print(data)
        mat1 = np.array(data['d435_color']['camera_matrix'])
        dist1 = np.array(data['d435_color']['dist_coeff'])
        mat2 = np.array(data['zed_left']['camera_matrix'])
        dist2 = np.array(data['zed_left']['dist_coeff'])
        mat3 = np.array(data['zed_right']['camera_matrix'])
        dist3 = np.array(data['zed_right']['dist_coeff'])

        # define image topics
        d435_zed_left = calibration_stereo(CHARUCO_BOARD,"/freicar_6/d435/color/image_raw",mat1,dist1,\
                                           "/freicar_6/zed/left_raw/image_raw_color",mat2,dist2)
        d435_zed_right = calibration_stereo(CHARUCO_BOARD, "/freicar_6/d435/color/image_raw", mat1, dist1, \
                                           "/freicar_6/zed/right_raw/image_raw_color", mat3, dist3)
        zed_left_zed_right = calibration_stereo(CHARUCO_BOARD, "/freicar_6/zed/left_raw/image_raw_color", mat2, dist2, \
                                           "/freicar_6/zed/right_raw/image_raw_color", mat3, dist3)
    # Start collecting images
    d435_zed_left.collect_data(1)
    # call stereo calibration
    d435_zed_left.calibration()

    # Start collecting images
    d435_zed_right.collect_data(1)
    # call stereo calibration
    d435_zed_right.calibration()

    # Start collecting images
    zed_left_zed_right.collect_data(1)
    # call stereo calibration
    zed_left_zed_right.calibration()

    data = {"d435_zed_left": {"R": d435_zed_left.R.tolist(), "T": d435_zed_left.T.tolist(),"F":d435_zed_left.F.tolist(),"E":d435_zed_left.E.tolist(),"H":d435_zed_left.H.tolist()},
            "d435_zed_right": {"R": d435_zed_right.R.tolist(), "T": d435_zed_right.T.tolist(),"F":d435_zed_right.F.tolist(),"E":d435_zed_right.E.tolist(),"H":d435_zed_right.H.tolist()}, \
            "zed_left_zed_right": {"R": zed_left_zed_right.R.tolist(), "T": zed_left_zed_right.T.tolist(), "F": zed_left_zed_right.F.tolist(),\
                                   "E": zed_left_zed_right.E.tolist(),"H": zed_left_zed_right.H.tolist()}}

    # Save data in file
    with open("camera_extrinsic.json", "w") as file:
        json.dump(data, file)
    rospy.signal_shutdown("task_completed")



