#!/usr/bin/env python3
import rospy, sys
from myRobot import MyRobot
from myConfig import MyConfig # Import the JointConfig class

if __name__ == '__main__':
    try:
        rospy.init_node('reset', anonymous=True)
        my_robot = MyRobot()  # Initialize the robot object

        # Create a JointConfig instance
        joint_configs = MyConfig()

        
        keys = joint_configs.config_dict.keys()
        print(joint_configs.config_dict.keys())
        dur = 1
        for key in keys:
            pose = joint_configs.get(key)
            my_robot.move_joints(pose, duration=dur)
            print(key, pose)
            rospy.sleep(dur+0.5)
        
    except rospy.ROSInterruptException:
        pass