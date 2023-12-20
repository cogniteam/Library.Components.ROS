#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageRotatorNode:
    def __init__(self):
        rospy.init_node('image_rotator_node', anonymous=True)
        self.cv_bridge = CvBridge()
        
        # Subscribe to the frontleft and frontright image topics
        self.frontleft_sub = rospy.Subscriber('/spot/camera/frontleft/image', Image, self.frontleft_image_callback)
        self.frontright_sub = rospy.Subscriber('/spot/camera/frontright/image', Image, self.frontright_image_callback)
        
        # Publish the rotated images as compressed images
        self.frontleft_rotated_pub = rospy.Publisher('rotated_fl/compressed', CompressedImage, queue_size=10)
        self.frontright_rotated_pub = rospy.Publisher('rotated_fr/compressed', CompressedImage, queue_size=10)

    def frontleft_image_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")
        
        # Rotate the image by 90 degrees
        rotated_image = np.rot90(cv_image, -1)
        
        # Convert the rotated image back to ROS format
        rotated_msg = self.cv_bridge.cv2_to_imgmsg(rotated_image, "mono8")
        
        # Publish the rotated image as a compressed image
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(cv2.imencode('.jpg', rotated_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])[1]).tobytes()
        self.frontleft_rotated_pub.publish(compressed_msg)

    def frontright_image_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Rotate the image by -90 degrees
        rotated_image = np.rot90(cv_image, -1)
        
        # Convert the rotated image back to ROS format
        rotated_msg = self.cv_bridge.cv2_to_imgmsg(rotated_image, "bgr8")
        
        # Publish the rotated image as a compressed image
        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(cv2.imencode('.jpg', rotated_image, [int(cv2.IMWRITE_JPEG_QUALITY), 40])[1]).tobytes()
        self.frontright_rotated_pub.publish(compressed_msg)

if __name__ == '__main__':
    try:
        image_rotator_node = ImageRotatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
