import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError


import numpy
import cv2
from cv2 import aruco
import json
import glob
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


class calibration_intrinsic:
    def __init__(self,charuco_board,image_topic):
        self.corners_all = []  # Corners discovered in all images processed
        self.ids_all = []  # Aruco ids corresponding to corners discovered
        self.image_size = None  # Determined at runtime
        self.images = 0
        self.calibration= 0
        self.cameraMatrix= None
        self.distCoeffs = None
        self.image_topic = image_topic
        self.charuco_board = charuco_board
        self.sub = None
        self.objpts=[]

    def image_callback(self,msg):
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

    # Find aruco markers in the query image
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=ARUCO_DICT)
    # Outline the aruco markers found in our query image
        img = aruco.drawDetectedMarkers(
            image=cv2_img,
            corners=corners)

    # Get charuco corners and ids from detected aruco markers
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=self.charuco_board)

    # If a Charuco board was found, let's collect image/corner points
   
        if response >=16:
            # Add these corners and ids to our calibration arrays
            charuco_chessboard_corners = self.charuco_board.chessboardCorners
            self.objpts.append(charuco_chessboard_corners)
            self.corners_all.append(charuco_corners)
            self.ids_all.append(charuco_ids)
            self.images+=1
            # Draw the Charuco board we've detected to show our calibrator the board was properly detected
            img = aruco.drawDetectedCornersCharuco(
                image=img,
                charucoCorners=charuco_corners,
                charucoIds=charuco_ids)

        # If our image size is unknown, set it now
            if not self.image_size:
                self.image_size = gray.shape[::-1]

        # Reproportion the image, maxing width or height at 1000
            proportion = max(img.shape) / 1000.0
            img = cv2.resize(img, (int(img.shape[1] / proportion), int(img.shape[0] / proportion)))
            # Pause to display each image, waiting for key press
            cv2.imshow(self.image_topic+'Charuco board', img)
            cv2.waitKey(1)

        else:
            print("Couldn't find charuco board")
            cv2.imshow(self.image_topic + 'Charuco board', cv2_img)
            cv2.waitKey(1)

    def collect_data(self,no_of_frames):
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        while (self.images < no_of_frames):
            pass
        cv2.destroyAllWindows()
        self.sub.unregister()
    def calibrate(self):
        self.calibration, self.cameraMatrix, self.distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=self.corners_all,
            charucoIds=self.ids_all,
            board=self.charuco_board,
            imageSize=self.image_size,
            cameraMatrix=None,
            distCoeffs=None)

if __name__ == "__main__":
    rospy.init_node('intrinsic_calibration')
    # Define your image topic
    zed_left= calibration_intrinsic(CHARUCO_BOARD,"/freicar_6/zed/left_raw/image_raw_color")
    zed_right= calibration_intrinsic(CHARUCO_BOARD,"/freicar_6/zed/right_raw/image_raw_color")
    d435_color= calibration_intrinsic(CHARUCO_BOARD,"/freicar_6/d435/color/image_raw")

    zed_right.collect_data(15)
    zed_left.collect_data(15)
    d435_color.collect_data(15)


    zed_left.calibrate()
    zed_right.calibrate()
    d435_color.calibrate()
    


    data = {"zed_left":{"camera_matrix": zed_left.cameraMatrix.tolist(),"dist_coeff":zed_left.distCoeffs.tolist()},
            "zed_right":{"camera_matrix": zed_right.cameraMatrix.tolist() ,"dist_coeff":zed_right.distCoeffs.tolist()},
            "d435_color":{"camera_matrix": d435_color.cameraMatrix.tolist(),"dist_coeff":d435_color.distCoeffs.tolist()}}

    with open("camera_intrinsics.json","w") as file:
        json.dump(data,file)
    rospy.signal_shutdown("task_completed")







