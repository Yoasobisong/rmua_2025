#!/home/socrates/miniconda3/envs/yolo8/bin/python

import os
# 添加这些环境变量设置
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

# Set logging level to suppress non-error messages
logging.getLogger('ultralytics').setLevel(logging.ERROR)

class YOLODetector:
    def __init__(self, min_area=0):
        rospy.init_node('yolo_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.model = YOLO('/data/workspace/rmua_2025/drone_ws/src/navigation_vision/yolov8/runs/rmua2/weights/best.pt')
        
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

        self.position_pub = rospy.Publisher(
            '/airsim_node/drone_1/front_right/position',
            Float32MultiArray,
            queue_size=1
        )
        self.highest_confidence = 0
        self.highest_class_id = 0
        self.highest_coordinates = [0, 0, 0, 0]
        self.min_area = min_area
        
        rospy.loginfo("YOLOv8 detector initialized")
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLOv8 inference with minimum line width
            results = self.model.predict(cv_image, conf=0.6, line_width=1)
            
            # Get the plotted image
            annotated_image = results[0].plot(line_width=1)
            
            # Convert back to ROS Image and publish
            ros_image = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            ros_image.header = msg.header  # Preserve the original header
            self.image_pub.publish(ros_image)


            # Print detection results
            for result in results:
                boxes = result.boxes
                confidences = []  # 存储置信度
                class_ids = []    # 存储类别ID
                coordinates = []  # 存储坐标
                
                for box in boxes:
                    # Get confidence
                    confidences.append(round(float(box.conf), 3))
                    # Get class index
                    class_ids.append(int(box.cls))
                    # Get coordinates (convert tensor to list)
                    coords = [round(x, 2) for x in box.xyxy[0].tolist()]  # [x1, y1, x2, y2]
                    coordinates.append(coords)
            
            try:
                if len(confidences) == 0:
                    # 如果没有检测到任何目标
                    self.highest_confidence = 0
                    self.highest_class_id = -1
                    # 保持位置不变
                else:
                    # Get index of highest confidence detection above min area
                    max_conf_idx = max(
                        range(len(confidences)), 
                        key=lambda i: confidences[i] if (self._area(coordinates[i]) > self.min_area and confidences[i] > 0.25 and class_ids[i] != 2) else -1
                    )
                    
                    # Update highest confidence detection
                    if max_conf_idx >= 0:
                        self.highest_confidence = confidences[max_conf_idx]
                        self.highest_class_id = class_ids[max_conf_idx] 
                        self.highest_coordinates = coordinates[max_conf_idx]
                    else:
                        # 如果没有满足条件的目标
                        self.highest_confidence = 0
                        self.highest_class_id = -1
                        # 保持位置不变
            except Exception as e:
                rospy.logerr(f"Error processing detection results: {e}")
                self.highest_confidence = 0
                self.highest_class_id = -1
                # 保持位置不变


                   
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")


    def _area(self, coordinates):
        return coordinates[2]*coordinates[3]

    def _pub_position(self):
        position_msg = Float32MultiArray()
        position_msg.data = [self.highest_coordinates[0], self.highest_coordinates[1], self.highest_coordinates[2], self.highest_coordinates[3]]
        self.position_pub.publish(position_msg)


if __name__ == '__main__':
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

