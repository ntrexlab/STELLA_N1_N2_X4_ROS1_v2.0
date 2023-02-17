#! /usr/bin/env python

import time
import rospy
import cv2
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__=="__main__":
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
        
    rospy.init_node('stella_camera_node')
    bridge = CvBridge()
    pub = rospy.Publisher('camera', Image, queue_size=1)
    rate = rospy.Rate(10)
    
    try:
        while not rospy.is_shutdown():
            ref,frame = cap.read()
            if not ref:
                rospy.loginfo("Not Found Devices")
                break
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)
            image_msg = bridge.cv2_to_imgmsg(frame,"bgr8")
            pub.publish(image_msg)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Exiting Program")
    
    except Exception as exception_error:
        rospy.loginfo("Error occurred. Exiting Program")
        rospy.loginfo("Error: " + str(exception_error))
    
    finally:
        cap.release()
        pass

