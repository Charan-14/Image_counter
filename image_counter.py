#!/usr/bin/env python

import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty

class ImageCounterNode:
    def __init__(self):
        # Initializing counter and bridge object
        self.counter = 0
        self.bridge = CvBridge()

        # Subscribing to image and reset topics
        self.image_sub = rospy.Subscriber("/cam_img", Image, self.image_callback)
        self.reset_sub = rospy.Subscriber("/reset", Empty, self.reset_callback)

        # Publisher for image paths
        self.path_pub = rospy.Publisher("/img_path", String, queue_size=10)

        # Defining output directory and creating if it does not exist
        self.output_dir = os.path.join(os.path.expanduser('~'), 'ros_images')
        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir)
            except OSError as e:
                rospy.logerr("Failed to create output directory: %s", str(e))
                rospy.signal_shutdown("Shutting down")

    
    # Function to check if a number is prime
    def is_prime(self, n):
        if n < 2:
            return False
        for i in range(2, int(n**0.5) + 1):
            if n % i == 0:
                return False
        return True
    
    # Callback for receiving images
    def image_callback(self, img_msg):

        # Increment counter
        self.counter += 1

        # Return if counter is not prime
        if not self.is_prime(self.counter):
            return

        # Converting ROS Image message to OpenCV format
        try:
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: %s", str(e))
            return

        # Defining the output path for the image
        path = os.path.join(self.output_dir, f'image_{self.counter}.png')

        # Saving the image and check if successful
        if not cv2.imwrite(path, image):
            rospy.logwarn("Failed to save image at %s", path)
            return

        rospy.loginfo("Saved image at %s", path)

        # Publishing the path of the saved image
        self.path_pub.publish(path)

    # Callback for resetting counter
    def reset_callback(self, msg):
        # Reset counter to 0
        self.counter = 0
        rospy.loginfo("Counter reset")


if __name__ == '__main__':
    # Initializing ROS node and run
    rospy.init_node('image_counter_node', anonymous=True)
    node = ImageCounterNode()
    rospy.spin()