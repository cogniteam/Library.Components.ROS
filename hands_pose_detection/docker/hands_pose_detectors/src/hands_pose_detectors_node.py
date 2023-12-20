#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2
 # pip install mediapipe
import mediapipe as mp
import json
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


class hands_pose_detector:

  def __init__(self):
    self.sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imageCallback, queue_size = 1)
    self.hands_skeleton_pub = rospy.Publisher("/hands_skeleton",String, queue_size = 1)
    self.image_pub = rospy.Publisher("/hands_img/compressed",
            CompressedImage, queue_size = 1)
    rospy.set_param('/hands_img/compressed/jpeg_quality', 20)

    self.hand = mp_hands.Hands(
      max_num_hands=4,
      model_complexity=0,
      min_detection_confidence=0.5,
      min_tracking_confidence=0.5)
    
    self.myJSON = dict()
  
  
  def detectHand(self,image):
    #To improve performance, optionally mark the image as not writeable to
    # pass by reference.

    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = self.hand.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    imageHeight, imageWidth, imageDepth = image.shape
    #print("Height: ",imageHeight)
    #print("Width: ", imageWidth)
    #print("Depth: ", imageDepth)
    if results.multi_hand_landmarks:
      handsArray = list()
      #print("Len of multi hand: ",len(results.multi_hand_landmarks))
      #print("multi hand landmarks: ", results.multi_hand_landmarks)
      #print('Handedness:', results.multi_handedness)
      i=0
      for hand_landmarks in results.multi_hand_landmarks:
        landmarksArray = list()
        objectInHandsArray = dict()
        #objectInLandmarksArray = dict()
        i+=1
        for point in mp_hands.HandLandmark:
          normalizedLandmark = hand_landmarks.landmark[point]
          pixelCoordinatesLandmark = mp_drawing._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, imageWidth, imageHeight)


          if(pixelCoordinatesLandmark != None):
            objectOf3dPoint = {"x":pixelCoordinatesLandmark[0],"y":pixelCoordinatesLandmark[1],"z":normalizedLandmark.z}
          else:
            objectOf3dPoint = {"x":-1, "y":-1,"z":-1}
          #print("objectOf3dPoint: ", objectOf3dPoint)

          #objectInLandmarksArray.update({"landmark_id": point, "3d_point":objectOf3dPoint})
          #print(objectInLandmarksArray)

          landmarksArray.append({"landmark_id": point, "3d_point":objectOf3dPoint})
          
        objectInHandsArray.update({"hand_id":i, "landmarks":landmarksArray})
        handsArray.append(objectInHandsArray)
        #print("Hands array: ",len(handsArray))
        self.myJSON.update({"hands":handsArray})
        jsonMsg = json.dumps(self.myJSON)
        #self.hands_skeleton_pub.publish(jsonMsg)

        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
    # cv2.imshow('MediaPipe Hands',image)
    # cv2.waitKey(1)
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    
    self.image_pub.publish(msg)
    if results.multi_hand_landmarks:
      self.hands_skeleton_pub.publish(jsonMsg)
  
  def imageCallback(self,rgb_msg):
    rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    self.detectHand(rgb_image)
  


if __name__ == '__main__':
    rospy.init_node('hands_pose_detectors_node', anonymous=True)

    hands_detector = hands_pose_detector()
  
    rospy.spin() 
