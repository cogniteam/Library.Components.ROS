#!/usr/bin/env python3
# import the necessary packages
import imutils
import cv2
import sys
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import numpy as np


class ArucoDetection(object):
	def __init__(self):
		self.imagePub = rospy.Publisher('aruco_detection_img/compressed',CompressedImage,queue_size=10)
		#self.imageRawPub = rospy.Publisher('aruco_detection_img/image_raw',Image,queue_size=10)
		self.idPub = rospy.Publisher('aruco_detected_id',Int32,queue_size=10)
		self.sub = rospy.Subscriber('/camera/color/image_raw',Image,self.imageCallback)
		paramName = rospy.search_param('type')
		self.type = rospy.get_param(paramName)

		# define names of each possible ArUco tag OpenCV supports
		self.ARUCO_DICT = {
			"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
			"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
			"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
			"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
			"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
			"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
			"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
			"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
			"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
			"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
			"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
			"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
			"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
			"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
			"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
			"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
			"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
			"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
			"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
			"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
			"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
		}

		# verify that the supplied ArUCo tag exists and is supported by
		# OpenCV
		if self.ARUCO_DICT.get(self.type, None) is None:
			print("[INFO] ArUCo tag of '{}' is not supported".format(self.type))
			sys.exit(0)

		# load the ArUCo dictionary and grab the ArUCo parameters
		print("[INFO] detecting '{}' tags...".format(self.type))
		self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.type])
		self.arucoParams = cv2.aruco.DetectorParameters_create()
	
	def imageCallback(self, rgbMsg):
		bgrImage = CvBridge().imgmsg_to_cv2(rgbMsg, desired_encoding="bgr8")
		self.runDetector(bgrImage)
	
	def runDetector(self, bgrImage):
		bgrImage = imutils.resize(bgrImage, width=1000)
		# detect ArUco markers in the input bgrImage
		(corners, ids, rejected) = cv2.aruco.detectMarkers(bgrImage,self.arucoDict, parameters=self.arucoParams)
		# verify *at least* one ArUco marker was detected
		if len(corners) > 0:
			# flatten the ArUco IDs list
			ids = ids.flatten()

			# loop over the detected ArUCo corners
			for (markerCorner, markerID) in zip(corners, ids):
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners

				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))
				# draw the bounding box of the ArUCo detection
				cv2.line(bgrImage, topLeft, topRight, (0, 255, 0), 2)
				cv2.line(bgrImage, topRight, bottomRight, (0, 255, 0), 2)
				cv2.line(bgrImage, bottomRight, bottomLeft, (0, 255, 0), 2)
				cv2.line(bgrImage, bottomLeft, topLeft, (0, 255, 0), 2)

				# compute and draw the center (x, y)-coordinates of the
				# ArUco marker
				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(bgrImage, (cX, cY), 4, (0, 0, 255), -1)

				# draw the ArUco marker ID on the bgrImage
				cv2.putText(bgrImage, str(markerID),
					(topLeft[0], topLeft[1] - 15),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.5, (0, 255, 0), 2)
			self.idPub.publish(markerID)

		#Publishing results
		#msg = CvBridge().cv2_to_imgmsg(bgrImage)
		#self.imageRawPub.publish(msg)

		msgToPublish = CompressedImage()
		msgToPublish.header.stamp = rospy.Time.now()
		msgToPublish.format = "jpg"
		msgToPublish.data = np.array(cv2.imencode('.jpg', bgrImage)[1]).tobytes()
		self.imagePub.publish(msgToPublish)		


if __name__ == '__main__':
	rospy.init_node("aruco_detection_node", anonymous = True)
	arucoDetector = ArucoDetection()
	rospy.spin()