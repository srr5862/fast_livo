'''
Author: songrr 1364117098@qq.com
Date: 2023-02-06 17:01:16
LastEditors: songrr 1364117098@qq.com
LastEditTime: 2023-02-09 15:41:54
FilePath: /FAST-LIVO/scripts/global_localization.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#!
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import numpy as np
import ros_numpy
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose,Point,Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import open3d as o3d


global_map = None
FOV = 120
FOV_DIS = 150

def msg2array(msg):
    pc_array = ros_numpy.numpify(msg)
    pc = np.zeros([len(pc_array),3])
    pc[:, 0] = pc_array['x'] 
    pc[:, 1] = pc_array['y'] 
    pc[:, 2] = pc_array['z'] 
    return pc
def inverse_se3(trans):
    trans_inverse = np.eye(4)
    trans_inverse[:3,:3] = trans[:3,:3].T
    trans_inverse[:3,3] = np.matmul(trans[:3,:3].T,trans[:3,3])
    return  trans_inverse
    

def pose_to_mat(pose_msg):
    return np.matmul(tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
                     tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation))
  
  
def publish_point_cloud(publisher,header,pc):
    data = np.zeros(len(pc),dtype=[
        ('x',np.float32),
        ('y',np.float32),
        ('z',np.float32),
        ('intensity',np.float32)
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2,data)
    msg.header = header
    publisher.publish(msg)
  
def crop_global_map_in_FOV(global_map,pose_estimation,cur_odom):
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation,T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)
    
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.vstack([global_map_in_map,np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map,global_map_in_map.T).T
    
    if FOV > 3.14:
        indices = np.where(
            (global_map_in_base_link[:,0] < FOV_DIS) &
            (np.abs(np.arctan2(global_map_in_base_link[:,1],global_map_in_base_link[:,0])) < FOV / 2.0)        
        )
    else:
        indices = np.where(
            (global_map_in_base_link[:,0] > 0) &
            (global_map_in_base_link[:,0] < FOV) &
            (np.abs(np.arctan2(global_map_in_base_link[:,1],global_map_in_base_link[:,0]) < FOV / 2.0))
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices,:3]))
    
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap,header,np.array(global_map_in_FOV.points)[::10])
    return global_map_in_FOV
    
    
def cb_scan_map():
    pass
   
def cb_scan_odometry():    
    pass

def registration_at_scale(pc_scan,pc_map,initial,scale):
    result_icp = o3d.registration.registration_icp(
        voxel_down_sample(pc_scan,0.5 * scale),
        voxel_down_sample(pc_map,0.4 * scale),
        1.0 * scale,initial,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=20)
    )
    return result_icp.transformation,result_icp.fitness

def global_localization(pose_estimation):
    global global_map,cur_scan, cur_odom, T_map_to_odom
    rospy.loginfo("start global localization")
    
    scan_tobe_mapped = copy.copy(cur_scan)
    
    tic = time.time()
    global_map_in_FOV = crop_global_map_in_FOV(global_map,pose_estimation,cur_odom)
    transformation = registration_at_scale(scan_tobe_mapped,global_map_in_FOV,initial=pose_estimation,scale=5)
    
    #精配准
    
    transformation,fitness = registration_at_scale(scan_tobe_mapped,global_map_in_FOV,initial=transformation,scale=1)
    
    toc = time.time()
    
    rospy.loginfo(f"cost time: {toc - tic} ")
    
    if fitness > 0.95:
        T_map_to_odom = transformation
        
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz),Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:   
        rospy.logwarn("no match")
        rospy.logwarn(transformation)
        rospy.logwarn(f"fitness score: {fitness}")
        return False
    
    
    
    
    pass
def thread_localization():
    global T_map_to_odom
    while True:
        rospy.sleep(0.001)
        global_localization(T_map_to_odom)

def voxel_down_sample(pcd,MAP_VOXEL_SIZE):
    return o3d.geometry.voxel_down_sample(pcd,MAP_VOXEL_SIZE)

def initialize_global_map(pc_data):
    global_map = o3d.geometry.PointCloud()
    global_map.points = o3d.utility.Vector3dVector(msg2array(pc_data)[:, :3])
    #降采样
    global_map = voxel_down_sample(global_map,MAP_VOXEL_SIZE=0.5)
    rospy.loginfo("global_map_init")


if __name__ == 'main':
    rospy.init_node('localization')
    rospy.loginfo("init localization node")
    pub_pc_in_map = rospy.Publisher('/cur_scan_map',PointCloud2,queue_size=1)
    pub_submap = rospy.Publisher('/submap',PointCloud2,queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom',PointCloud2,queue_size=1)
    
    rospy.Subscriber('/cloud_registered',PointCloud2,cb_scan_map,queue_size=1)
    rospy.Subscriber('/Odometry',PointCloud2,cb_scan_odometry,queue_size=1)
    
    #init globalMap
    initialize_global_map(rospy.wait_for_message('/map',PointCloud2))
    
    #start globalLocalization
    _thread.start_new_thread(global_localization,())