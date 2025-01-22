#!/home/socrates/miniconda3/envs/yolo8/bin/python

import os
# 添加这些环境变量设置
os.environ['GDAL_DATA'] = ''
os.environ['PROJ_LIB'] = ''

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import logging

# Set logging level to suppress non-error messages
logging.getLogger('ultralytics').setLevel(logging.ERROR)

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.model = YOLO('/home/socrates/workspace/rmua_2025/drone_ws/src/navigation_vision/yolov8/runs/rmua16/weights/best.pt')
        
        # Initialize publisher and subscriber
        self.image_sub = rospy.Subscriber(
            '/airsim_node/drone_1/front_right/Scene',
            Image,
            self.image_callback
        )
        
        self.image_pub = rospy.Publisher(
            '/airsim_node/drone_1/front_right/yolo_detect',
            Image,
            queue_size=1
        )
        
        rospy.loginfo("YOLOv8 detector initialized")
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLOv8 inference with minimum line width
            results = self.model.predict(cv_image, conf=0.25, line_width=1)
            
            # Get the plotted image
            annotated_image = results[0].plot(line_width=1)
            
            # Convert back to ROS Image and publish
            ros_image = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            ros_image.header = msg.header  # Preserve the original header
            self.image_pub.publish(ros_image)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
