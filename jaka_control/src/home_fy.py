
#!/usr/bin/env python
import rospy, sys
import moveit_commander
import moveit_msgs.msg
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from sensor_msgs.msg import JointState
from copy import deepcopy
import math

# global variables


print("============ Starting tutorial setup")

# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('home_fy',
#                 anonymous=True)
# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()


# move_group = moveit_commander.MoveGroupCommander("manipulator")


# display_trajectory_publisher = rospy.Publisher(
#                                     '/move_group/display_planned_path',
#                                     moveit_msgs.msg.DisplayTrajectory)

# print ("============ Waiting for RVIZ...")
# rospy.sleep(10)
# print( "============ Starting tutorial ")

# print ("============ Robot Groups:")
# print (robot.get_group_names())


# move_group.set_named_target("home")

# plan1 = move_group.plan()

# rospy.sleep(5)

# moveit_commander.roscpp_shutdown()


# # sget real-time positions
# end_effector_link = move_group.get_end_effector_link()

# current_pose = move_group.get_current_pose(end_effector_link).pose
# print("Start Pose:")
# print("position x:", current_pose.position.x)