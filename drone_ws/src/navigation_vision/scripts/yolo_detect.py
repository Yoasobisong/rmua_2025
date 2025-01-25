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
from geometry_msgs.msg import PoseStamped
import yaml
from tf.transformations import euler_from_quaternion

# Set logging level to suppress non-error messages from ultralytics
logging.getLogger('ultralytics').setLevel(logging.ERROR)

class YOLODetector:
    def __init__(self):
        """
        Initialize the YOLO detector node
        """
        rospy.init_node('yolo_detector', anonymous=True)
        
        # Load configuration from YAML
        config_path = rospy.get_param('~config_path', '/data/workspace/rmua_2025/drone_ws/src/navigation_vision/config/yaml/vision_params.yaml')
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

        self.pose_sub = rospy.Subscriber(
            self.config['topics']['pose_input'],
            PoseStamped,
            self.pose_callback
        )
        

        self.pose_z = 0.
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

        self.predict_z_pub = rospy.Publisher(
            self.config['topics']['predict_z_output'],
            Float32MultiArray,
            queue_size=1
        )

        self.avoid_car_pub = rospy.Publisher(
            self.config['topics']['avoid_car_pub'],
            Float32MultiArray,
            queue_size=1
        )

        # Initialize tracking variables
        self.highest_confidence = 0
        self.highest_class_id = 0
        self.highest_coordinates = [0, 0, 0, 0]
        self.min_area = self.config['yolo']['min_area']
        self.max_door_area = self.config['yolo']['max_door_area']
        self.min_car_area = self.config['yolo']['min_car_area']
        self.cars = None
        self.position_msg = Float32MultiArray()
        
        self.yaw = 0.
        self.pitch = 0.
        self.init_yaw = 0.

        self.car_error = Float32MultiArray()

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

            # Initialize detection variables
            self.cars = []
            self.highest_confidence = 0
            self.highest_class_id = -1
            self.highest_coordinates = [0, 0, 0, 0]

            # Process detection results if any exist
            if len(results[0].boxes) > 0:
                confidences = results[0].boxes.conf.cpu().numpy()
                class_ids = results[0].boxes.cls.cpu().numpy()
                coordinates = results[0].boxes.xywh.cpu().numpy()
                
                # Find car detections
                try:
                    self.cars = [
                        [coord[0], coord[1], coord[2]*4/3, coord[3]*4/3]  # Expand width and height by 1/6
                        for i, coord in enumerate(coordinates)
                        if class_ids[i] == 2 and 
                        self._area(coordinates[i]) > self.min_car_area
                    ]
                    
                    # Draw car detections
                    if self.cars:
                        for car in self.cars:
                            cv2.rectangle(annotated_image, 
                                        (int(car[0] - car[2]/2), 
                                         int(car[1] - car[3]/2)), 
                                        (int(car[0] + car[2]/2), 
                                         int(car[1] + car[3]/2)), 
                                        (255, 0, 255), 2)  # pink
                except Exception as e:
                    rospy.logwarn(f"Error processing car detections: {e}")
                    self.cars = []

                # Find highest confidence detection for doors
                try:
                    valid_detections = [
                        (i, confidences[i]) 
                        for i in range(len(confidences)) 
                        if (self.max_door_area > self._area(coordinates[i]) > self.min_area and
                            class_ids[i] != 2)
                    ]
                    
                    if valid_detections:
                        max_conf_idx, max_conf = max(valid_detections, key=lambda x: x[1])
                        self.highest_confidence = max_conf
                        self.highest_class_id = class_ids[max_conf_idx]
                        self.highest_coordinates = coordinates[max_conf_idx]
                except Exception as e:
                    rospy.logwarn(f"Error finding highest confidence detection: {e}")
                    self.highest_confidence = 0
                    self.highest_class_id = -1

            # Publish position and visualize detection
            self._pub_position()
            
            # Draw target and center points
            if self.position_msg.data:
                cv2.circle(annotated_image, 
                          (int(self.position_msg.data[0]), int(self.position_msg.data[1])), 
                          5, (255, 0, 0), -1)  # target point in blue
            if self._if_attack_car():
                self.avoid_car_pub.publish(self.car_error)
                cv2.circle(annotated_image, 
                        (self.config['image']['center_x'], self.config['image']['center_y']), 
                        5, (0, 0, 255), -1)  # center point in red
            else:
                cv2.circle(annotated_image, 
                        (self.config['image']['center_x'], self.config['image']['center_y']), 
                        5, (0, 255, 0), -1)  # center point in green
            
            # Convert annotated image back to ROS format and publish
            ros_image = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            ros_image.header = msg.header
            self.image_pub.publish(ros_image)
                   
        except Exception as e:
            rospy.logwarn(f"Error in image callback: {e}")
    def _area(self, coordinates):
        """
        Calculate area of bounding box
        Args:
            coordinates: List of [x1, y1, x2, y2] coordinates
        Returns:
            Area of the bounding box
        """
        return coordinates[2]*coordinates[3]


    def _if_attack_car(self):
        try:
            if self.cars:
                # Check if center point (480,360) falls within any car bounding box
                for car_coords in self.cars:
                        x1, y1 = car_coords[0] - car_coords[2]/2, car_coords[1] - car_coords[3]/2  # Top-left corner
                        x2, y2 = car_coords[0] + car_coords[2]/2, car_coords[1] + car_coords[3]/2
                        
                        # Check if center point is inside car bounding box
                        if (x1 < 480. < x2) and (y1 < 360. < y2):
                            if car_coords[0] - 480 > 0:
                                self.car_error.data = [-(car_coords[2]/2 - abs(car_coords[0] - 480.))/(car_coords[2]/2) * 100]
                            else:
                                self.car_error.data = [(car_coords[2]/2 - abs(car_coords[0] - 480.))/(car_coords[2]/2) * 100]
                            return True
                return False
            return False
        except Exception as e:
            rospy.logwarn(f"Error in _if_attack_car: {e}")
            return False

    def _pub_position(self):
        """
        Publish position data based on highest confidence detection
        """
        try:
            if self.highest_class_id == 0:  # left door
                self.position_msg.data = [
                    self.highest_coordinates[0] + (self.config['yolo']['radio_k'] + 0.5) * self.highest_coordinates[2],
                    self.highest_coordinates[1] - self.highest_coordinates[3] * 0.5,
                    0
                ]
            elif self.highest_class_id == 1:  # right door
                self.position_msg.data = [
                    self.highest_coordinates[0] - (self.config['yolo']['radio_k'] + 0.5) * self.highest_coordinates[2],
                    self.highest_coordinates[1] - self.highest_coordinates[3] * 0.5,
                    1
                ]
            else:
                self.position_msg.data = [
                    480,
                    240,
                    -1
                ]
            
            self.position_pub.publish(self.position_msg)
            
            # write position data to csv
            # with open('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/position_fliter/only_windows_fliter.csv', 'a') as f:
            #     f.write(f"{self.position_msg.data[0]}, {self.position_msg.data[1]}, {self.position_msg.data[2]}\n")
            
        except Exception as e:
            rospy.logwarn(f"Error in position publishing: {e}")

    def pose_callback(self, msg):
        # Calculate yaw angle from quaternion
        _, pitch_rad, yaw_rad = euler_from_quaternion([msg.pose.orientation.x, 
                                              msg.pose.orientation.y,
                                              msg.pose.orientation.z,
                                              msg.pose.orientation.w])
        self.yaw = yaw_rad * 180 / np.pi  # Convert radians to degrees
        self.pitch = pitch_rad * 180 / np.pi  # Convert radians to degrees
        self.pose_z = msg.pose.position.z
        # rospy.loginfo(f"Yaw: {self.yaw}, Pitch: {self.pitch}, Z: {self.pose_z}")
        

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
