#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebcamPublisher(object):
    def __init__(self, topic, device=0, rate=10):
        # Initializing the ROS node
        rospy.init_node('webcam_publisher', anonymous=True)

        # Creating a publisher to send images
        self.pub = rospy.Publisher(topic, Image, queue_size=10)

        # Using cv_bridge to convert OpenCV images to ROS messages
        self.bridge = CvBridge()

        # Using OpenCV to capture images from the webcam
        self.cap = cv2.VideoCapture(device)

        # Checking if the webcam is opened correctly
        if not self.cap.isOpened():
            rospy.logerr("Could not open webcam")
            return

        # Setting the rate for publishing images
        self.rate = rospy.Rate(rate)
    
    def run(self):
        while not rospy.is_shutdown():
            # Capturing frames from the webcam
            ret, frame = self.cap.read()

            # If the frame was captured correctly, publish it
            if ret:
                # Converting the image to a ROS message
                ros_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.pub.publish(ros_img)
            else:
                rospy.logwarn("Failed to capture frame")

            self.rate.sleep()

        # Releasing the webcam when done
        self.cap.release()

if __name__ == '__main__':
    try:
        publisher = WebcamPublisher('/cam_img')
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception: %s", str(e))