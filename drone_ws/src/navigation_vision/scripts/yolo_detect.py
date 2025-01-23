#!/home/socrates/miniconda3/envs/yolo8/bin/python

import os
# Add environment variable settings to avoid GDAL/PROJ errors
os.environ['GDAL_DATA'] = ''
os.environ['PROJ_LIB'] = ''

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import logging
import yaml

# Set logging level to suppress non-error messages from ultralytics
logging.getLogger('ultralytics').setLevel(logging.ERROR)

class YOLODetector:
    def __init__(self):
        """
        Initialize the YOLO detector node
        """
        rospy.init_node('yolo_detector', anonymous=True)
        
        # Load configuration from YAML
        config_path = rospy.get_param('~config_path', 'config/yaml/vision_params.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.bridge = CvBridge()
        
        # Initialize YOLO model with pretrained weights from config
        self.model = YOLO(self.config['yolo']['model_path'])
        
        # Initialize subscriber for raw camera images
        self.image_sub = rospy.Subscriber(
            self.config['topics']['camera_input'],
            Image,
            self.image_callback
        )
        
        # Initialize publisher for annotated detection images
        self.image_pub = rospy.Publisher(
            self.config['topics']['detection_output'],
            Image,
            queue_size=1
        )

        # Initialize publisher for target position data
        self.position_pub = rospy.Publisher(
            self.config['topics']['position_output'],
            Float32MultiArray,
            queue_size=1
        )

        # Initialize tracking variables
        self.highest_confidence = 0
        self.highest_class_id = 0
        self.highest_coordinates = [0, 0, 0, 0]
        self.min_area = self.config['yolo']['min_area']
        self.min_car_area = self.config['yolo']['min_car_area']
        self.cars = None
        self.position_msg = Float32MultiArray()

        rospy.loginfo("YOLOv8 detector initialized with configuration from %s", config_path)
        
    def image_callback(self, msg):
        """
        Callback function for processing incoming image messages
        Args:
            msg: ROS Image message
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLOv8 inference with parameters from config
            results = self.model.predict(
                cv_image, 
                conf=self.config['yolo']['confidence_threshold'],
                line_width=self.config['yolo']['line_width']
            )
            
            # Get the annotated image with detection boxes
            annotated_image = results[0].plot(line_width=self.config['yolo']['line_width'])

            confidences = results[0].boxes.conf.cpu().numpy()
            class_ids = results[0].boxes.cls.cpu().numpy()
            coordinates = results[0].boxes.xywh.cpu().numpy()
            
            try:
                if len(confidences) > 0:
                    # Find all the class_ids == 2 and areas > self.min_car_area
                    self.cars = [i for i in range(len(confidences)) if class_ids[i] == 2 and self._area(coordinates[i]) > self.min_car_area]
                    print(self.cars)
            except Exception as e:
                rospy.logerr(f"Error processing detection results: {e}")
                self.cars = None

            try:
                if len(confidences) == 0:
                    # Reset tracking if no detections found
                    self.highest_confidence = 0
                    self.highest_class_id = -1
                else:
                    # Find detection with highest confidence meeting criteria
                    max_conf_idx = max(
                        range(len(confidences)), 
                        key=lambda i: confidences[i] if (self._area(coordinates[i]) > self.min_area and confidences[i] > 0.25 and class_ids[i] != 2) else -1
                    )
                    
                    # Update tracking with highest confidence detection
                    if max_conf_idx >= 0:
                        self.highest_confidence = confidences[max_conf_idx]
                        self.highest_class_id = class_ids[max_conf_idx] 
                        self.highest_coordinates = coordinates[max_conf_idx]
                    else:
                        # Reset tracking if no valid detections
                        self.highest_confidence = 0
                        self.highest_class_id = -1
            except Exception as e:
                rospy.logerr(f"Error processing detection results: {e}")
                self.highest_confidence = 0
                self.highest_class_id = -1

            # Publish position and visualize detection
            self._pub_position()
            cv2.circle(annotated_image, 
                      (int(self.position_msg.data[0]), int(self.position_msg.data[1])), 
                      5, (0, 0, 255), -1)
            cv2.circle(annotated_image, 
                      (self.config['image']['center_x'], self.config['image']['center_y']), 
                      5, (0, 255, 0), -1)
            
            # Convert annotated image back to ROS format and publish
            ros_image = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            ros_image.header = msg.header  # Preserve the original header
            self.image_pub.publish(ros_image)
                   
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def _area(self, coordinates):
        """
        Calculate area of bounding box
        Args:
            coordinates: List of [x1, y1, x2, y2] coordinates
        Returns:
            Area of the bounding box
        """
        return coordinates[2]*coordinates[3]

    def _pub_position(self):
        """
        Publish position data based on highest confidence detection
        Adjusts position based on class ID and scaling factor
        """
        if self.highest_class_id == -1:
            self.position_msg.data = [0, 0 , -1]
        elif self.highest_class_id == 0:
            self.position_msg.data = [self.highest_coordinates[0] + (self.config['yolo']['radio_k'] + 0.5) * self.highest_coordinates[2], self.highest_coordinates[1] + 0.5 * self.highest_coordinates[3], 0]
        elif self.highest_class_id == 1:
            self.position_msg.data = [self.highest_coordinates[0] - (self.config['yolo']['radio_k'] + 0.5) * self.highest_coordinates[2], self.highest_coordinates[1] + 0.5 * self.highest_coordinates[3], 1]

        self._fliter_position()
        self.position_pub.publish(self.position_msg)

    def _fliter_position(self):
        """
        Filter position data to smooth out noise
        """
        self.position_msg.data[0] = self.position_msg.data[0] + 0.1 * (self.position_msg.data[0] - self.position_msg.data[0])
        self.position_msg.data[1] = self.position_msg.data[1] + 0.1 * (self.position_msg.data[1] - self.position_msg.data[1])


if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
