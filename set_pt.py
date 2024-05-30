#!/usr/bin/env python3
import sys
import rospy
from myRobot import MyRobot
from myConfig import MyConfig

if __name__ == '__main__':
    try:
        rospy.init_node('move_to_point', anonymous=True)
        my_robot = MyRobot()  # 初始化机器人对象

        # 创建一个 JointConfig 实例
        joint_configs = MyConfig()

        # 检查是否提供了点的名称作为命令行参数
        if len(sys.argv) < 2:
            print("Usage: python script_name.py <point_name>")
            sys.exit(1)

        target_config_name = sys.argv[1]  # 从命令行参数中获取目标点的名称

        # 获取目标关节配置的姿态
        target_pose = joint_configs.get(target_config_name)

        # 移动机器人到目标姿态
        duration = 1  # 移动持续时间
        my_robot.move_joints(target_pose, duration=duration)
        
    except rospy.ROSInterruptException:
        pass
