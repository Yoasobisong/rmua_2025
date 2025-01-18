#!/usr/bin/env python3

# Import required packages
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from datetime import datetime

class ImageSaver:
    def __init__(self):
        # Initialize the node
        rospy.init_node('image_saver', anonymous=True)
        
        # Create a CV bridge
        self.bridge = CvBridge()
        
        # Store the latest image
        self.current_image = None
        
        # Initialize image counter
        self.count = 26
        
        # Create save directory if it doesn't exist
        self.save_dir = os.path.expanduser('/home/socrates/workspace/rmru_2025/yolov8/images/train')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # Get the current maximum number in the directory
        existing_files = os.listdir(self.save_dir)
        for f in existing_files:
            if f.endswith('.jpg'):
                try:
                    num = int(f.split('.')[0])
                    self.count = max(self.count, num + 1)
                except:
                    continue
        
        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/airsim_node/drone_1/front_right/Scene', Image, self.image_callback)
        
        rospy.loginfo("Image saver node initialized. Press SPACE to save images.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {str(e)}")

    def save_image(self):
        if self.current_image is not None:
            # Generate filename with count
            filename = os.path.join(self.save_dir, f'{self.count}.jpg')
            
            # Save the image
            try:
                cv2.imwrite(filename, self.current_image)
                rospy.loginfo(f"Image saved to {filename}")
                self.count += 1
            except Exception as e:
                rospy.logerr(f"Failed to save image: {str(e)}")
        else:
            rospy.logwarn("No image received yet")

    def run(self):
        # Create a window to capture keyboard events
        window_name = 'Press SPACE to save image (ESC to quit)'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        while not rospy.is_shutdown():
            if self.current_image is not None:
                # Show the current image
                cv2.imshow(window_name, self.current_image)
                
                # Wait for key press
                key = cv2.waitKey(1) & 0xFF
                
                # If SPACE is pressed, save the image
                if key == 32:  # ASCII code for SPACE
                    self.save_image()
                # If ESC is pressed, exit
                elif key == 27:  # ASCII code for ESC
                    break
            
            # Sleep to prevent high CPU usage
            rospy.sleep(0.03)
        
        # Cleanup
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        saver = ImageSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
