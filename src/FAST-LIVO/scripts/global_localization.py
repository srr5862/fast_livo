#!/usr/bin/env 
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

def msg2array(msg):
    pc_array = ros_numpy.numpify(msg)
    pc = np.zeros([len(pc_array),3])
    pc[:, 0] = pc_array['x'] 
    pc[:, 1] = pc_array['y'] 
    pc[:, 2] = pc_array['z'] 
    return pc
    
    
def cb_scan_map():
    pass
   
def cb_scan_odometry():    
    pass

def global_localization():
    
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
    global_map = voxel_down_sample(global_map,MAP_VOXEL_SIZE=0.1)
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