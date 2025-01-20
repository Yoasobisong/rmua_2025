#!/home/socrates/miniconda3/envs/yolo8/bin/python

# results[0].boxes.xywh.cpu().numpy()
# classes = results[0].boxes.cls.cpu().numpy()
import rospy
import cv2
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
from navigation_vision.msg import VisionYolo, Box

class YOLODetector:
    def __init__(self, conf=0.35, radio_k=0.5):
        # Initialize ROS node
        rospy.init_node('yolo_detector', anonymous=True)
        self.conf = conf
        self.classes = ["left_door", "right_door", "car"]
        # Load YOLO model
        self.highest_door_ = None
        self.radio_k_ = radio_k
        self.highest_door_position_ = Box()
        self.target_position_ = Box()
        self.model = YOLO('/home/socrates/workspace/rmru_2025/yolov8/runs/rmru_train_22/weights/rmru_l.pt')
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('/airsim_node/drone_1/front_right/Scene', Image, self.image_callback)
        
        # Create publisher for processed image
        self.image_pub = rospy.Publisher('/airsim_node/drone_1/front_right/vision_detection', Image, queue_size=1)
        self.yolo_pub = rospy.Publisher('/airsim_node/drone_1/front_right/vision_result', VisionYolo, queue_size=1)
        self.target_pub = rospy.Publisher('/airsim_node/drone_1/front_right/target_position', Box, queue_size=1)
        rospy.loginfo("YOLODetector initialized")

    def image_callback(self, msg):
        try:
            # Convert image data to numpy array
            height = msg.height
            width = msg.width
            channels = 3
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)
            
            # Run YOLO detection
            results = self.model(cv_image, verbose=False, conf=self.conf)
            
            # Get the first result
            result = results[0]
            xy = results[0].boxes.xywh.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy()
            confidences = results[0].boxes.conf.cpu().numpy()
            # print(f"xy: {xy}, classes: {classes}, con: {confidences}")
            # print(f" {type(xy)}, {type(classes)}, {type(confidences)}")

            result_msg = VisionYolo()
            result_msg.header.stamp = rospy.Time.now()
            result_msg.header.frame_id = "vision_result"
            result_msg.num_detections = len(xy)
            
            # 创建检测框数组
            boxes = []
            for box_coords in xy:
                box_msg = Box()
                box_msg.box = box_coords.tolist()  # 直接赋值 [x,y,w,h]
                boxes.append(box_msg)
            result_msg.boxes = boxes
            
            result_msg.classes = classes.astype(int).tolist()
            result_msg.confidences = confidences.tolist()
            result_msg.class_names = [self.classes[int(name)] for name in classes]
            # Publish the result message
            self.yolo_pub.publish(result_msg)
            
            # Get highest confidence detection excluding 'car' class
            confidences = np.array(result_msg.confidences)
            valid_indices = [i for i, name in enumerate(result_msg.class_names) if name != 'car']
            # rospy.loginfo(f"valid_indices: {valid_indices}")
            if len(valid_indices) > 0:
                max_idx = valid_indices[np.argmax(confidences[valid_indices])]
                self.highest_door_ = result_msg.class_names[max_idx]
                self.highest_door_position_ = result_msg.boxes[max_idx]
                if self.highest_door_ == "left_door":
                    self.target_position_.box[0] = self.highest_door_position_.box[0] + (0.5 + self.radio_k_) * self.highest_door_position_.box[2]
                    self.target_position_.box[1] = self.highest_door_position_.box[1] + 0.5* self.highest_door_position_.box[3]
                    self.target_position_.box[2] = 0
                elif self.highest_door_ == "right_door":
                    self.target_position_.box[0] = self.highest_door_position_.box[0] - (0.5 + self.radio_k_) * self.highest_door_position_.box[2]
                    self.target_position_.box[1] = self.highest_door_position_.box[1] + 0.5* self.highest_door_position_.box[3]
                    self.target_position_.box[2] = 0
                else:
                    self.target_position_.box[0] = 0
                    self.target_position_.box[1] = 0
                    self.target_position_.box[2] = -1
            else:
                rospy.loginfo("No valid detections found (excluding car)")
                self.target_position_.box[0] = 0
                self.target_position_.box[1] = 0
                self.target_position_.box[2] = -1
            #rospy.loginfo(f"target_position_: {self.target_position_}") 
            self.target_pub.publish(self.target_position_)

            # draw the result
            annotated_frame = result.plot(
                line_width=1,      # Set box line width to 1
                font_size=8,       # Set font size to 8
                boxes=True,        # Show boxes
                labels=True,       # Show labels
                conf=True          # Show confidence scores
            )
            # print(f"annotated_frame_shape: {annotated_frame.shape}")
            # draw the target position
            cv2.circle(annotated_frame, (int(self.target_position_.box[0]), int(self.target_position_.box[1])), 5, (0, 0, 255), -1)
            cv2.circle(annotated_frame, (480, 360), 5, (0, 255, 0), -1)
            # Convert back to ROS Image message
            img_msg = Image()
            img_msg.header = msg.header
            img_msg.height = annotated_frame.shape[0]
            img_msg.width = annotated_frame.shape[1]
            img_msg.encoding = 'bgr8'
            img_msg.is_bigendian = False
            img_msg.step = annotated_frame.shape[1] * 3
            img_msg.data = annotated_frame.tobytes()
            # Publish the processed image
            self.image_pub.publish(img_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")
        
        
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass