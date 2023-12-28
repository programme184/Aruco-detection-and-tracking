#!/usr/bin/env python
import rospy, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from copy import deepcopy
import math
import time

# global difinition
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('listener',
                anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
end_effector_link = move_group.get_end_effector_link()


class ManiControl:
    # initialization
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('moveit_cartesian_demo',
    #             anonymous=True)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()

    # move_group = moveit_commander.MoveGroupCommander("manipulator")

    # display_trajectory_publisher = rospy.Publisher(
    #                                 '/move_group/display_planned_path',
    #                                 moveit_msgs.msg.DisplayTrajectory)
    # end_effector_link = move_group.get_end_effector_link()
    
    reference_frame = "dummy"
    move_group.set_pose_reference_frame(reference_frame)
        
        # allow replanning after failed
    move_group.allow_replanning(True)

        # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
    move_group.set_max_acceleration_scaling_factor(0.5)
    move_group.set_max_velocity_scaling_factor(0.2)
    
        
    
    def info_get(self):
        # # basic info
        # print("============ End effector: %s" % move_group.get_end_effector_link())

        # print ("============ Robot Groups name:")
        # print (robot.get_group_names())

        # print("============ Printing robot state")
        state_all = robot.get_current_state()
        # print(state_all)
        # print("============")
        # robot position
        end_effector_link = move_group.get_end_effector_link()
        current_pose = move_group.get_current_pose(end_effector_link).pose
        px, py, pz = current_pose.position.x, current_pose.position.y, current_pose.position.z
        ox, oy, oz, ow = current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w
        position = [px, py, pz]
        orientation = [ox, oy, oz, ow]
        # print("position:", position)
        # print("Orientation:", orientation)
        
        return position, orientation
        
    def back_home(self):
        move_group.set_named_target("home")
        success = move_group.plan()
        if success:
            move_group.go(wait=True)
            rospy.loginfo("Move success")
            time.sleep(3)
        else:
            rospy.logerr("Move fail")
    
    # def rotation():
    #     pass
    
    def pos_assume(self, p, o=None):
        # planning to a pose goal
        pose_target = geometry_msgs.msg.Pose()
        if o is not None:
            
            pose_target.orientation.x = o[0]
            pose_target.orientation.y = o[1]
            pose_target.orientation.z = o[2]
            pose_target.orientation.w = o[3]
        pose_target.position.x = p[0]
        pose_target.position.y = p[1]
        pose_target.position.z = p[2]
        move_group.set_pose_target(pose_target)
        plan1 = move_group.plan()

        if plan1:
            move_group.go(wait=True)
            rospy.loginfo("Move success")
            time.sleep(3)
        else:
            rospy.logerr("Move fail")
    
    def joint_assume():
        pass
    
# print("============ Starting tutorial setup")
robot_control = ManiControl()
# robot_control.back_home()

# p = [-0.15503826, -0.60239023,  0.30754451]
# o = [-0.99994678,  0.00318444,  0.00153668,  0.00969242]

# print("============ From moveit to get realtime postion of end effector")
# # # robot_control = ManiControl()
# robot_control.pos_assume(p, o)
# position, orientation = robot_control.info_get()
# print('position, orientation:\n', position, orientation)