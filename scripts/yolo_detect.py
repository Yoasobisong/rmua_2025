#!/home/socrates/miniconda3/envs/yolo8/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np

class YOLODetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('yolo_detector', anonymous=True)
        
        # Load YOLO model
        self.model = YOLO('/home/socrates/workspace/rmru_2025/yolov8/runs/rmru_train_22/weights/rmru_l.pt')
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('/airsim_node/drone_1/front_right/Scene', Image, self.image_callback)
        
        # Create publisher for processed image
        self.image_pub = rospy.Publisher('/airsim_node/drone_1/yolo8/detection', Image, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert image data to numpy array
            height = msg.height
            width = msg.width
            channels = 3
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, channels)
            
            # Run YOLO detection
            results = self.model(cv_image, verbose=False)
            
            # Get the first result
            result = results[0]
            
            # Visualize the results on the image with custom parameters
            annotated_frame = result.plot(
                line_width=1,      # Set box line width to 1
                font_size=8,       # Set font size to 8
                boxes=True,        # Show boxes
                labels=True,       # Show labels
                conf=True          # Show confidence scores
            )
            
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