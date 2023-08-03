#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
import os

class ImagePathSaverNode:
    def __init__(self):

        # Subscribing to image path and reset topics
        self.path_sub = rospy.Subscriber("/img_path", String, self.path_callback)
        self.reset_sub = rospy.Subscriber("/reset", Empty, self.reset_callback)

        # Define path to the text file
        self.file_path = os.path.join(os.path.expanduser('~'), 'image_paths.txt')

    # Callback for receiving image paths
    def path_callback(self, msg):
        path = msg.data
        rospy.loginfo("Received path: %s", path)

        # Append the path to the text file
        try:
            with open(self.file_path, 'a') as file:
                file.write(path + '\n')
        except IOError as e:
            rospy.logerr("Failed to write to file: %s", str(e))

    # Callback for resetting the text file
    def reset_callback(self, msg):
        rospy.loginfo("Resetting file")

        # Empty the text file
        try:
            with open(self.file_path, 'w') as file:
                file.truncate()
        except IOError as e:
            rospy.logerr("Failed to reset file: %s", str(e))

if __name__ == '__main__':
    # Initialize ROS node and run
    rospy.init_node('image_path_saver_node', anonymous=True)
    node = ImagePathSaverNode()
    rospy.spin()