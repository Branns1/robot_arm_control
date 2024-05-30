#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import sin, cos, pi

def move_ur5_circle():
    # 初始化ROS节点
    rospy.init_node('move_ur5_circle', anonymous=True)

    # 初始化moveit_commander和move_group
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # 设置目标位置参考坐标系
    move_group.set_pose_reference_frame("base_link")

    # 创建一个用于控制发布轨迹消息的发布者
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # 将机械臂移动到初始位置
    move_group.set_named_target("home")
    move_group.go(wait=True)

    # 设置圆的参数
    center_x = 0.5  # 圆心x坐标
    center_y = 0.0  # 圆心y坐标
    radius = 0.2    # 圆的半径
    num_points = 100  # 圆上的点数

    # 生成圆上的路径点
    waypoints = []
    for i in range(num_points):
        angle = 2 * pi * i / num_points
        x = center_x + radius * cos(angle)
        y0 = center_y + radius * sin(angle)
        y = y0 * cos(pi/4) - 0.3 * sin(pi/4) 
        z = y0 * sin(pi/4) + 0.3 * cos(pi/4)  # 保持Z坐标不变
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = 1.0  # 保持朝向不变
        waypoints.append(pose_goal)

    # 规划路径
    (plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # 路径点
                                   0.01,        # 距离步进
                                   0.0)         # 方向阈值

    # 显示规划路径
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

    # 执行规划路径
    move_group.execute(plan, wait=True)

if __name__ == '__main__':
    try:
        move_ur5_circle()
    except rospy.ROSInterruptException:
        pass

