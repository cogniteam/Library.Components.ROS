#!/usr/bin/env python3
import rospy
import sys
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from object_msgs.msg import ObjectsInBoxes
from object_msgs.msg import ObjectInBox
from sensor_msgs.msg import RegionOfInterest

from sensor_msgs.msg import CompressedImage

import numpy as np
import zipfile

"""Main script to run the object detection routine."""
import argparse
import sys
import time

import cv2
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
import utils


class object_detection:

  def __init__(self):
    self.counter = 0
    self.sub = rospy.Subscriber("/camera/color/image_raw/",Image,self.imageCallback, queue_size = 1)
    self.image_pub = rospy.Publisher("/object_detection_img/compressed",
            CompressedImage, queue_size = 1)
    self.object_in_boxes_pub = rospy.Publisher("/objects_msg",ObjectsInBoxes, queue_size = 1)
    full_param_name = rospy.search_param('num_threads')
    self.num_threads = rospy.get_param(full_param_name)
    full_param_name = rospy.search_param('score_threshold')
    self.score_threshold = rospy.get_param(full_param_name)
    full_param_name = rospy.search_param('max_results')
    self.max_results = rospy.get_param(full_param_name)

    rospy.set_param('/object_detection_img/compressed/jpeg_quality', 20)

    # path = os.getcwd()
    # path = os.path.abspath(os.path.join(path, os.pardir))
    # path_to_model = path + "/data/efficientdet_lite0.tflite"
    path_to_model = "/hamster_v8_ws/src/object_detection/data/efficientdet_lite0.tflite"

    try:
      with zipfile.ZipFile(path_to_model) as model_with_metadata:
        if not model_with_metadata.namelist():
          raise ValueError('Invalid TFLite model: no label file found.')

        file_name = model_with_metadata.namelist()[0]
        with model_with_metadata.open(file_name) as label_file:
          label_list = label_file.read().splitlines()
          self._label_list = [label.decode('ascii') for label in label_list]
    except zipfile.BadZipFile:
      print(
          'ERROR: Please use models trained with Model Maker or downloaded from TensorFlow Hub.'
      )
      raise ValueError('Invalid TFLite model: no metadata found.')

    # Initialize the object detection model
    options = ObjectDetectorOptions(
        num_threads=self.num_threads,
        score_threshold=self.score_threshold,
        max_results=self.max_results,
        enable_edgetpu=False)

    self.detector = ObjectDetector(self._label_list, path_to_model, options=options)

    
  
  def imageCallback(self,rgb_msg):
    self.counter+=1
    if self.counter%5==0:
      rgb_image = CvBridge().imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
      self.run(rgb_image)

  def run(self, image):
    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()
    end_time = -1

  
    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 255, 0)  # green
    font_size = 3
    font_thickness = 1
    fps_avg_frame_count = 10


    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    counter += 1

    # Run object detection estimation using the model.
    detections = self.detector.detect(image)

    # Draw keypoints and edges on input image
    image = utils.visualize(image, detections)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

      # Show the FPS
      fps_text = 'FPS = {:.1f}'.format(fps)
      text_location = (left_margin, row_size)
      cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN, font_size, text_color, font_thickness)

      
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()  
    self.image_pub.publish(msg)

    msgObject = ObjectsInBoxes()    

    msgObject.header.stamp = rospy.Time.now()
    if len(detections) != 0:
      for detection in detections:
        object_in_box = ObjectInBox()
        category = detection.categories[0]
        class_name = category.label
        probability = round(category.score, 2) 
        object_in_box.object.object_name = class_name
        object_in_box.object.probability = probability
        object_in_box.roi.x_offset = detection.bounding_box.left
        object_in_box.roi.y_offset = detection.bounding_box.top
        object_in_box.roi.height = abs(detection.bounding_box.top - detection.bounding_box.bottom)
        object_in_box.roi.width = abs(detection.bounding_box.left - detection.bounding_box.right)
        object_in_box.roi.do_rectify = False
        msgObject.objects_vector.append(object_in_box)
        if end_time != -1:
          msgObject.inference_time_ms = (end_time - start_time)
        else:
          msgObject.inference_time_ms = (time.time() - start_time)
      self.object_in_boxes_pub.publish(msgObject)



if __name__ == '__main__':
  rospy.init_node('object_detection_node', anonymous=True)
  
  print("Success")
  object_detection = object_detection()

  rospy.spin()




