#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime
from myRobot import MyRobot
from myConfig import MyConfig

class RGBImageSubscriberAndArmController:
    def __init__(self):
        rospy.init_node('camera_startup_and_arm_control_node', anonymous=True)
        # Initialize robot object
        self.my_robot = MyRobot() 

        # Initialize joint configurations
        self.joint_configs = MyConfig()  

        # Initialize video writer
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = None
        self.start_time = None

        # Subscriber for RGB image
        self.rgb_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)

        # Bridge for converting ROS Image to OpenCV image
        self.bridge = CvBridge()

        # Create a directory to save videos if it doesn't exist
        self.save_dir = "rgb_videos"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

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
    
    def control_arm(self):
        # Add arm control logic here
        keys = self.joint_configs.config_dict.keys()
        print(self.joint_configs.config_dict.keys())
        dur = 1
        for key in keys:
            pose = self.joint_configs.get(key)
            self.my_robot.move_joints(pose, duration=dur)
            print(key, pose)
            rospy.sleep(dur+1)
    
    def run(self):
        while not rospy.is_shutdown():
            self.control_arm()  # Control the robot arm
            self.rate.sleep()    # Control the loop rate

if __name__ == '__main__':
    try:
        rgb_image_and_arm_controller = RGBImageSubscriberAndArmController()
        rgb_image_and_arm_controller.run()  # Keep the node running
    except rospy.ROSInterruptException:
        if rgb_image_and_arm_controller.out is not None:
            rgb_image_and_arm_controller.out.release()