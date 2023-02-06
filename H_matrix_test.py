import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import numpy
import cv2
from cv2 import aruco
import pickle
import glob
# Instantiate CvBridge
bridge = CvBridge()




# Create the arrays and variables we'll use to store info like corners and IDs from images processed
corners_all_image1 = []  # Corners discovered in all images processed
ids_all_image1 = []  # Aruco ids corresponding to corners discovered
image_size_image1 = None  # Determined at runtime

# This requires a set of images or a video taken with the camera you want to calibrate
# I'm using a set of images taken with the camera with the naming convention:
# 'camera-pic-of-charucoboard-<NUMBER>.jpg'
# All images used should be the same size, which if taken with the same camera shouldn't be a problem



def image1_callback(msg):
    global image1_count, image_size_image1,image1
    print("Received an image!")
        try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except getopt.GetoptError as e:
        print(e)
        # Grayscale the image
        # Save your OpenCV2 image as a jpeg
    #cv2.imshow('camera_image', cv2_img)
    #cv2.waitKey(1)
    img = cv2_img
    image1 = img.copy()
    rows, cols, _ = image1.shape
    H=np.array( [[-4.52018445e+01, -5.25774457e-02,  9.51744259e+03],\
 [ 2.60533950e-01, -4.34107644e+01,  5.82263270e+03],\
 [-1.95208435e-03,  1.69986398e-03, -2.54814829e+01]])
    img2 = cv2.warpPerspective(image1, H, (cols,rows), cv2.INTER_LINEAR)
    stamp = rospy.Time.from_sec(time.time())
    transformed_img_msg = Image()
    transformed_img_msg.height = img2.shape[0]
    transformed_img_msg.width = img2.shape[1]
    transformed_img_msg.step = img2.strides[0]
    transformed_img_msg.encoding = 'bgr8'
    transformed_img_msg.header.frame_id = "freicar_6/zed_right_camera_optical_frame1"
    transformed_img_msg.header.stamp = stamp
    transformed_img_msg.data = img2.flatten().tolist()
    transformed_img_pub.publish(transformed_img_msg)








rospy.init_node('image_listener')
# Define your image topic
image_topic1 = "/freicar6/zed/right_raw/image_raw_color"

# Set up your subscriber and define its callback
sub1=rospy.Subscriber(image_topic1, Image, image1_callback)
transformed_img_pub = rospy.Publisher("/freicar_6/zed/right_raw/aligned/image_raw_color", Image,queue_size=10)

# Spin until ctrl + c
rospy.spin()

cv2.destroyAllWindows()
