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
        center_x = img.width // 2
        center_y = img.height//2

        radius = 50
        cv2.circle(cv2_img, (center_x, center_y), radius, (0, 255, 0), 3)
        cv2.imshow("Frame", cv2_img)
        cv2.waitKey(1) #this forces the window opened by OpenCV to remain open
        modified_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        output_msg = Image()
        output_msg.header = img.header
        output_msg.height = img.height
        output_msg.width = img.width
        output_msg.encoding = "rgb8"
        output_msg.is_bigendian = img.is_bigendian
        output_msg.step = img.step
        output_msg.data = modified_img.tobytes()
        
        img_pub.publish(output_msg)

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
