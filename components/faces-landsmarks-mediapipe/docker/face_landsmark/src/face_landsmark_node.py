#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

import json
import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_face_mesh = mp.solutions.face_mesh
import numpy as np

class FaceLandMarks():

    def __init__(self):
        self.sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imageCallback, queue_size = 1)
        self.image_pub = rospy.Publisher("/faces_landsmarks/compressed",
                CompressedImage, queue_size = 1)
        self.face_skeleton_pub = rospy.Publisher("/faces_skeleton",String, queue_size = 1)
        rospy.set_param('/faces_landsmarks/compressed/jpeg_quality', 20)

        self.drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

        self.face_mesh_ = mp_face_mesh.FaceMesh(
            max_num_faces=10,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) 
        
        self.myJSON = dict()

   

    def detectLandsmarks(self, image):

        height, width, _ = image.shape
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.face_mesh_.process(image)

        count = 0

        # Draw the face mesh annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_face_landmarks:
            facesArray = list()
            for face_landmarks in results.multi_face_landmarks:
                landmarksArray = list()
                objectInFacesArray = dict()
                #objectInLandmarksArray = dict()
                count+=1
                for i in range(0, 468):
                    normalizedLandmark = face_landmarks.landmark[i]
                    pixelCoordinatesLandmark = mp_drawing._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, width, height)
                    #print({"face_id":count, "landmarks":landmarksArray})
                    if(pixelCoordinatesLandmark != None):
                        objectOf3dPoint = {"x":pixelCoordinatesLandmark[0],"y":pixelCoordinatesLandmark[1],"z":normalizedLandmark.z}
                    else:
                        objectOf3dPoint = {"x":-1, "y":-1,"z":-1}
                    
                    #print(objectInLandmarksArray)
                    #objectInLandmarksArray.update({"landmark_id": i, "3d_point":objectOf3dPoint})
                    landmarksArray.append({"landmark_id": i+1, "3d_point":objectOf3dPoint})
                    #print("landmark number ", i) 
                    #print("object",{"landmark_id": i, "3d_point":objectOf3dPoint})

                objectInFacesArray.update({"face_id":count, "landmarks":landmarksArray})
                #print(objectInFacesArray)
                facesArray.append(objectInFacesArray)
                #print("Faces array: ",facesArray)
                self.myJSON.update({"faces":facesArray})
                jsonMsg = json.dumps(self.myJSON)
                #self.face_skeleton_pub.publish(jsonMsg)


                mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=mp_drawing_styles
                    .get_default_face_mesh_tesselation_style())
                mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=mp_drawing_styles
                    .get_default_face_mesh_contours_style())
                mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_IRISES,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=mp_drawing_styles
                    .get_default_face_mesh_iris_connections_style())

       
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        
        self.image_pub.publish(msg)
        if results.multi_face_landmarks:
            self.face_skeleton_pub.publish(jsonMsg)

    
    
    def imageCallback(self,rgb_msg):
        rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        self.detectLandsmarks(rgb_image)



if __name__ == '__main__':
    rospy.init_node('face_landsmark_detectors_node', anonymous=True)

    faceLandMarks = FaceLandMarks()
  
    rospy.spin() 
