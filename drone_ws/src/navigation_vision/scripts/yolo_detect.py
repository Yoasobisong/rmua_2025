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

# Set logging level to suppress non-error messages from ultralytics
logging.getLogger('ultralytics').setLevel(logging.ERROR)

class YOLODetector:
    def __init__(self, min_area=0, radio_k=0.5):
        """
        Initialize the YOLO detector node
        Args:
            min_area: Minimum area threshold for detections
            radio_k: Scaling factor for position calculations
        """
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Initialize YOLO model with pretrained weights
        self.model = YOLO('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/yolov8/runs/rmua2/weights/best.pt')
        
        # Initialize subscriber for raw camera images
        self.image_sub = rospy.Subscriber(
            '/airsim_node/drone_1/front_right/Scene',
            Image,
            self.image_callback
        )
        
        # Initialize publisher for annotated detection images
        self.image_pub = rospy.Publisher(
            '/airsim_node/drone_1/front_right/yolo_detect',
            Image,
            queue_size=1
        )

        # Initialize publisher for target position data
        self.position_pub = rospy.Publisher(
            '/airsim_node/drone_1/front_right/position',
            Float32MultiArray,
            queue_size=1
        )

        # Initialize tracking variables for highest confidence detection
        self.highest_confidence = 0
        self.highest_class_id = 0
        self.highest_coordinates = [0, 0, 0, 0]
        self.min_area = min_area
        self.radio_k = radio_k

        self.position_msg = Float32MultiArray()

        rospy.loginfo("YOLOv8 detector initialized")
        
    def image_callback(self, msg):
        """
        Callback function for processing incoming image messages
        Args:
            msg: ROS Image message
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLOv8 inference with confidence threshold and minimal line width
            results = self.model.predict(cv_image, conf=0.6, line_width=1)
            
            # Get the annotated image with detection boxes
            annotated_image = results[0].plot(line_width=1)

            confidences = results[0].boxes.conf.cpu().numpy()
            class_ids = results[0].boxes.cls.cpu().numpy()
            coordinates = results[0].boxes.xywh.cpu().numpy()
            
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
            cv2.circle(annotated_image, (int(self.position_msg.data[0]), int(self.position_msg.data[1])), 5, (0, 0, 255), -1)

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
            self.position_msg.data = [self.highest_coordinates[0] + (self.radio_k + 0.5) * self.highest_coordinates[2], self.highest_coordinates[1] + 0.5 * self.highest_coordinates[3], 0]
        elif self.highest_class_id == 1:
            self.position_msg.data = [self.highest_coordinates[0] - (self.radio_k + 0.5) * self.highest_coordinates[2], self.highest_coordinates[1] + 0.5 * self.highest_coordinates[3], 1]
        self.position_pub.publish(self.position_msg)


if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
