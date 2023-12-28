#!/usr/bin/env python
import cv2 as cv
import numpy as np
# from numpy import *
import rospy
import math
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from jaka_move import ManiControl
import transforms3d as tfs
import time

class Transformation:
  # end_pos = [-0.00184716, 3.5662, 0.543051 ]
  # end_pos =[-0.08994488544878099, -0.6225722642461946, 0.458246683265947]   # To
  # end_ori = [0.985299175247485, 0.06530378754527286, -0.14526639836596023, 0.061795016746094444]#x, y, z, w
  
  cam_pos =[-0.038007,   0.0431236,  0.0367012]#[-0.0348418,  0.0428151,  0.0338341]#[-0.0368686, -0.0338163, -0.0333575]
  # xyzw
  cam_ori = [0.014383107367188808, -6.700755339672515e-05, 0.3911408087157479, 0.9202184466145209]#[0.01414568946395788, -0.00257750271097712, 0.39945193875924767, 0.9166413718411591] # xyzw[0, 0, 0,1]#[-0.9200639780864388, 0.3916255267279043, -0.006474514547970843, 0.008354861112946355]
  H_h2c = None
  
  def __init__(self, obj_pos, obj_ori, end_pos, end_ori):
    self.end_pos = np.array(end_pos)
    self.end_ori = np.array(end_ori)
    print("\n===end effector==")
    print("position:", self.end_pos)
    print("orientation:", self.end_ori)
    self.obj_pos = np.array(obj_pos)
    self.obj_ori = np.array(obj_ori)
    
    # print("\n===marker pose==")
    # print("position:", self.obj_pos)
    # print("orientation:", self.obj_ori)

    
    self.R1 = self.quat2rotation(self.cam_ori)
    self.T1 = np.array(self.cam_pos)
    # print("\n===camara pose==")
    # print("position:", self.cam_pos)
    # print("orientation':", self.cam_ori)

  
  def quat2rotation(self, quat):
    rot = tfs.quaternions.quat2mat((quat[3], quat[0], quat[1], quat[2])) # quat:xyzw but the quat2mat input should be wxyz
    return rot
  
  
  def express_transform(self, item = None):
    # use 4x4 translation matrix T to realize
    
    Ro = self.quat2rotation(self.end_ori)
    To = np.array(self.end_pos)

    R2 = self.quat2rotation(self.obj_ori)
    T2 = np.array(self.obj_pos)
    hand_camera_tr = self.T1
    hand_camera_rot = self.R1
    hand_world_tr = To
    hand_world_rot = Ro
    marker_camera_tr = T2
    marker_camera_rot = R2
    
    
    H_h2c = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])# hand to camera
    
    # base_link->end_link
    H_b2h = tfs.affines.compose(np.squeeze(hand_world_tr), hand_world_rot, [1, 1, 1]) # get a translation matrix, like [r, t; 000 1], 4x4, base to hand
            # marker->camera  or camera->marker
    H_c2marker = tfs.affines.compose(np.squeeze(marker_camera_tr), marker_camera_rot, [1, 1, 1])       # camera to marker
            # base_link->end_link->marker   or base_link->end_link->camera 
    H_b2c = np.dot(H_b2h,H_h2c)         # base to camera 右乘<-运动坐标变换
    
    
    self.H_h2c = H_h2c
    
    
    if item is None or item ==3:                                            # calculate the world pose of a detected object
            # base_link->end_link->marker->camera  or base_link->end_link->camera->marker
      H_b2marker = np.dot(H_b2c,H_c2marker)           # base to marker
      tr = H_b2marker[0:3,3:4].T[0]
    # rot =tfs.euler.mat2euler(temp[0:3,0:3]) 
      quat = tfs.quaternions.mat2quat(H_b2marker[0:3,0:3])    # wxyz form
      # print("rotation matrix:", H_b2marker)
      
    elif item == 2:                                                         # calculate the world pose of the camera
      # H_b2c = H_b2h *H_h2c 
      tr = H_b2c[0:3,3:4].T[0]
    # rot =tfs.euler.mat2euler(temp[0:3,0:3]) 
      quat = tfs.quaternions.mat2quat(H_b2c[0:3,0:3])    # wxyz form
      
    quat_t = np.roll(quat, -1)                      #xyzw
    # print('element shift:\n', 'before:', quat, '\nafter:', quat_t)
    
    return tr, quat_t                           
    # pass


  def camworld2endworld(self, Q_cworld, T_cworld):
    
    R_cworld = self.quat2rotation(Q_cworld)
    H_b2c = tfs.affines.compose(np.squeeze(T_cworld), R_cworld, [1, 1, 1]) # homogeneous matrix of base to camera
    # H_b2c = H_b2h * H_h2c
    H_h2c = self.H_h2c                                                    # homogeneous matrix of hand to camera
    
    H_b2h = np.dot(H_b2c, np.linalg.inv(H_h2c)) 
    tr = H_b2h[0:3,3:4].T[0]
    quat = tfs.quaternions.mat2quat(H_b2h[0:3,0:3]) #wxyz
    quat_t = np.roll(quat, -1)                      #xyzw
    # print('element shift:\n', 'before:', quat, '\nafter:', quat_t)
    
    return tr, quat_t

class ArucoPose:
  
  def __init__(self):
      self.position = None
      self.ori = None
      self.pose = None
    
  
  def generation(self):
    # 加载用于生成标记的字典
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)
 
    # Generate the marker
    markerImage = np.zeros((200, 200), dtype=np.uint8)
    markerImage = cv.aruco.drawMarker(dictionary, 33, 200, markerImage, 1)
    
    cv.imwrite("marker33.png", markerImage)

  def detection_test(self):    
    # Pos = PoseStamped()
    # Pos.pos
    topic = '/aruco_single/pose' # 更改为你想要获取输出的话题名称
    
    rospy.init_node('listener', anonymous=True)
    msg = rospy.wait_for_message(topic, PoseStamped)
    
    self.position = msg.pose.position
    self.ori = msg.pose.orientation
    # return selfposition, orien
    
  def detection(self):
    
    topic = '/aruco_single/pose' # 更改为你想要获取输出的话题名称
    
    rospy.init_node('listener', anonymous=True)
    subscriber = rospy.Subscriber(topic, PoseStamped, self.callback)
    # rospy.spin()    #无限循环
    # self.position, self.ori = self.callback
    # return position, ori
    
    
  def callback(self, data):
    # print("Received message:", data)
    self.pose = data.pose
    
    self.position = self.pose.position
    self.ori = self.pose.orientation
    print("Position X: ", self.position.x)
    print("Position: ", self.position)
    print("Orientation: ", self.ori)
    # return self.position, self.ori
  
  def pos_trans(self):
    pass
 

if __name__ == '__main__':
    p = [-0.15503826, -0.60239023,  0.30754451]
    o = [-0.99994678,  0.00318444,  0.00153668,  0.00969242]

    print("============ From moveit to get realtime postion of end effector")
    robot_control = ManiControl()
    # robot_control.pos_assume(p, o)
    # end_position, end_orientation = robot_control.info_get()
    
    # end_position = p
    # end_orientation = o

    
    pose = ArucoPose()
    # pose.detection_test()

    # # pose tranformation test:
    # position = pose.position
    # orien = pose.ori
    # obj_position = [position.x, position.y, position.z]
    # obj_orientation = [orien.x, orien.y, orien.z, orien.w]

    # tran = Transformation(obj_position, obj_orientation, end_position, end_orientation)

    
    # T_final, R_final= tran.express_transform()
    # print('\nfinal rotation \n', R_final, R_final.shape, '\n translation \n', T_final, T_final.shape)
    
    # ccam_p, ccam_o = tran.express_transform(2)
    
    # # orientation is in the form of xyzw
    # print("\nccam_p", ccam_p, "\n ccam_o:", ccam_o)
    
    # # control camera approaches the detected aruco
    # mcam_p = [T_final[0], T_final[1], T_final[2]+0.25]
    # mcam_o = ccam_o
    # print("\nmcam_p", mcam_p, "\nmcam_o:", mcam_o)

    
    # # transfer the controlled camera pose to end link pose
    # mend_p, mend_o = tran.camworld2endworld(mcam_o, mcam_p)
    
    # print("\nmend_p1", mend_p, "\n mend_o1:", mend_o)
    # robot_control.pos_assume(mend_p, mend_o)
    # end_position, end_orientation = robot_control.info_get()
    # print("\nmoved_position:", end_position)
    # print("current orientation\n", end_orientation)
    
    
    # track the aruch code real time
    for i in range(0, 10):
      print('\n new epo:', i)
      pose.detection_test()

      if i == 0:
        robot_control.pos_assume(p, o)
      
      # calculate aruco pose in world coordinate
      position = pose.position
      orien = pose.ori
      obj_position = [position.x, position.y, position.z]
      obj_orientation = [orien.x, orien.y, orien.z, orien.w]

      end_position, end_orientation = robot_control.info_get()
    
      tran = Transformation(obj_position, obj_orientation, end_position, end_orientation)
      T_final, R_final= tran.express_transform()
      print('\naruco rotation \n', R_final, '\n translation \n', T_final)
      mcam_p = [T_final[0], T_final[1], T_final[2]+0.25]
      mcam_o = [-0.91878439,  0.39407182,  0.00522638,  0.02270064]
      print("\nmcam_p", mcam_p, "\nmcam_o:", mcam_o)

    
      # transfer the controlled camera pose to end link pose
      mend_p, mend_o = tran.camworld2endworld(mcam_o, mcam_p)
    
      # print("\nmend_p1", mend_p, "\n mend_o1:", mend_o)
      robot_control.pos_assume(mend_p, mend_o)
      time.sleep(6)
      

    
   
    rospy.spin()    #无限循环