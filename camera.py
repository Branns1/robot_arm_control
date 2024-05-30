#!/usr/bin/env python

#source /opt/ros/noetic/setup.bash
#roslaunch realsense2_camera rs_camera.launch
#chmod +x depth_image_subscriber.py
#./depth_image_subscriber.py

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class RGBImageSubscriber:
    def __init__(self):
        rospy.init_node('rgb_image_subscriber')

        # Subscriber for RGB image
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)

        # Bridge for converting ROS Image to OpenCV image
        self.bridge = CvBridge()
        
        # Create a directory to save videos if it doesn't exist
        self.save_dir = "rgb_videos"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Initialize video writer
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = None
        self.start_time = None

    def rgb_callback(self, data):
        try:
            # Convert ROS Image to OpenCV image
            rgb_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            # Initialize video writer if not initialized
            if self.out is None:
                height, width, _ = rgb_image.shape
                self.start_time = rospy.Time.now()
                file_name = os.path.join(self.save_dir, f"rgb_video_{self.start_time.to_sec()}.mp4")
                self.out = cv2.VideoWriter(file_name, self.fourcc, 30.0, (width, height))

            # Write frame to video
            self.out.write(rgb_image)

            rospy.loginfo("Saving RGB video...")

        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        rgb_image_subscriber = RGBImageSubscriber()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        if rgb_image_subscriber.out is not None:
            rgb_image_subscriber.out.release()
 