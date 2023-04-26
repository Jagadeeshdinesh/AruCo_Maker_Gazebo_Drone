import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import String
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import numpy as np

detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

# Instantiate CvBridge
bridge = CvBridge()

# Camera calibration 
calib_path = ""
camera_matrix = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')


# Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
# def showImage(img):
#     cv2.imshow('image', img)
#     cv2.waitKey(1)

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        dim = (1280,720)
        cv2_img = cv2.resize(cv2_img,dim)

        # Detect the markers in the image
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(cv2_img, detector)
        height = cv2_img.shape[0]/2
        width = cv2_img.shape[1]/2
        center = int(width), int(height)
        cv2_img = cv2.drawMarker(cv2_img, center, (0,255,0), cv2.MARKER_CROSS, 15, 2)
        
        # print(markerCorners)

        if markerIds == 0:

            # Drawing a circle
            # center = [int(markerCorners[0][0][0][0]),int(markerCorners[0][0][0][1])]
            # print(center)
            # cv2_img = cv2.circle(cv2_img, center, 5, (255, 0, 0), 2)
            cv2_img = cv2.aruco.drawDetectedMarkers(cv2_img, markerCorners, markerIds)

            # Drawing a circle
            first_crn = [int(markerCorners[0][0][0][0]),int(markerCorners[0][0][0][1])]
            second_crn = [int(markerCorners[0][0][1][0]),int(markerCorners[0][0][1][1])]
            third_crn = [int(markerCorners[0][0][2][0]),int(markerCorners[0][0][2][1])]
            fourth_crn = [int(markerCorners[0][0][3][0]),int(markerCorners[0][0][3][1])]

            x_sum = markerCorners[0][0][0][0]+ markerCorners[0][0][1][0]+ markerCorners[0][0][2][0]+ markerCorners[0][0][3][0]
            y_sum = markerCorners[0][0][0][1]+ markerCorners[0][0][1][1]+ markerCorners[0][0][2][1]+ markerCorners[0][0][3][1]
                
            x_centerPixel = x_sum*.25
            y_centerPixel = y_sum*.25

            # Formula for finding center in aruco maker
            maker_center = int(x_centerPixel),int(y_centerPixel)
            
            x_difference = (width - x_centerPixel )
            y_difference =  ( height- y_centerPixel )

            x_difference = np.abs(x_difference)
            y_difference = np.abs(y_difference)

            if width < x_centerPixel and height > y_centerPixel :
                # y_difference = -y_difference
                print("Quard-I")
            elif width > x_centerPixel and height > y_centerPixel  :
                x_difference = -x_difference
                print("Quard-II")
            
            elif width > x_centerPixel and  y_centerPixel > height:
                x_difference = -x_difference
                y_difference = -y_difference
                print("Quard-III")
            elif width < x_centerPixel and  y_centerPixel > height:
                y_difference = -y_difference
                print("Quard-IV")
            
            print(x_difference, y_difference)

            pub = rospy.Publisher('chatter', String, queue_size=10)
            rate = rospy.Rate(10) # 10hz
            hello_str = "%d, %d" % (x_difference , y_difference)
            rospy.loginfo(hello_str)
            pub.publish(hello_str)

            cv2_img = cv2.circle(cv2_img, maker_center, 2, (255, 0, 0), 2)


            # Pose Estimation
            ret = cv2.aruco.estimatePoseSingleMarkers(markerCorners,10, camera_matrix, camera_distortion)
            
            # print(ret)
            # unpack the otuput, get only the first
            rvec, tvec = ret[0][0,0,:] , ret[1][0,0,:]

            # -- Draw the detected markers(frame, corners)
            cv2_img = cv2.aruco.drawDetectedMarkers(cv2_img, markerCorners)
            # cv2_img = cv2.drawFrameAxes(cv2_img, camera_matrix, camera_distortion,rvec, tvec, 10)
            # print(tvec)

            # print the tag position in camera frame
            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2_img = cv2.putText(cv2_img, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)



        cv2.imshow('image', cv2_img)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/rrbot/camera1/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()