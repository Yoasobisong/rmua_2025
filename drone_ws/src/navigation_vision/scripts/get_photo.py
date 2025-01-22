#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        self.bridge = CvBridge()
        self.latest_image = None
        self.save_dir = '/home/socrates/workspace/rmua_2025/drone_ws/src/navigation_vision/yolov8/images/train'
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # Get the starting index based on existing files
        self.current_index = self.get_starting_index()
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(
            '/airsim_node/drone_1/front_right/Scene',
            Image,
            self.image_callback
        )
        
        print("Image saver initialized. Press SPACE to save images, ESC to quit.")
        
    def get_starting_index(self):
        # Get list of existing image files
        existing_files = [f for f in os.listdir(self.save_dir) if f.endswith('.jpg')]
        if not existing_files:
            return 0
        
        # Extract indices from filenames and find the maximum
        indices = []
        for filename in existing_files:
            try:
                index = int(filename.split('.')[0])
                indices.append(index)
            except ValueError:
                continue
        
        return max(indices) + 1 if indices else 0
    
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            
    def run(self):
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                # Display the image
                cv2.imshow('Camera Feed', self.latest_image)
                
                # Wait for key press
                key = cv2.waitKey(1) & 0xFF
                
                # If space bar is pressed, save the image
                if key == 32:  # Space bar
                    filename = os.path.join(self.save_dir, f"{self.current_index}.jpg")
                    cv2.imwrite(filename, self.latest_image)
                    print(f"Saved image {filename}")
                    self.current_index += 1
                
                # If ESC is pressed, exit
                elif key == 27:  # ESC
                    break
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        saver = ImageSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
