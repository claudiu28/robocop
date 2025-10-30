#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

ROS_NODE_NAME = "image_processing_node"

img_pub = None

def img_process(img):
        #rospy.loginfo("image width: %s height: %s" % (img.width, img.height))
        frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
        cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255]))
        contours, _ =cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) != 0 :
                largest_contour = max(contours, key=cv2.contourArea)
                contour_frame = cv2_img.copy()
                (x,y), radius = cv2.minEnclosingCircle(largest_contour)
                centre = (int(x), int(y))
                circle = cv2.circle(contour_frame, centre, int(radius), [235, 90, 210], 4)
                cv2.imshow("Obj", circle)
        else:
                cv2.imshow("Obj", cv2_img)
        cv2.waitKey(1)

def cleanup():
        rospy.loginfo("Shutting down...")
        cv2.destroyWindow("Frame")

if __name__ == "__main__":
        rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
        rospy.on_shutdown(cleanup)
        rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
        
        img_pub = rospy.Publisher("/processed_image", Image, queue_size=10)
        

        try:
                rospy.spin()
        except KeyboardInterrupt:
                pass
