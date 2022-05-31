#!/usr/bin/env python3
from matplotlib.pyplot import grid
import rospy
import struct
import numpy as np
import pdb
from time import time

# ROS dependencies
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg    import PointCloud2, PointField
from geometry_msgs.msg  import PoseStamped, Pose2D
from nav_msgs.msg import OccupancyGrid
from skimage.draw import line

import tf2_ros
import tf2_geometry_msgs
import std_msgs

def normalize_angle(angle):
    while angle > np.pi:
        angle = angle - 2. * np.pi

    while angle < -np.pi:
        angle = angle + 2. * np.pi

    return angle

class Map:
    def __init__(self, w = 200, h = 200):
        self.w = w # width
        self.h = h # height

        p_occ = 0.75 # probability of occupancy in the sensor model
        p_free = 0.45 # probability of free in the sensor model
        p_prior = 0.5 # prior probability of occupancy in the sensor model
        self.l_occ = prob_to_log(p_occ) # log odds of occupancy in the sensor model
        self.l_free = prob_to_log(p_free) # log odds of free in the sensor model
        self.l_prioir = prob_to_log(p_prior) # log odds of prior occupancy in the sensor model

        self.map =  np.ones((w, h)) * self.l_prioir

class Particle:
    def __init__(self, pose):
        self.pose = pose

def scan_matching_callback(data):
    global particle
    particle.pose['x'] = data.x
    particle.pose['y'] = data.y
    particle.pose['theta'] = data.theta

def translate_points_to_center(p):
    return p[0] + map.h/2, p[1] + map.w/2

def translate_points_from_center(p):
    return p[0] - map.h/2 + 1, p[1] - map.w/2 + 1

def transform_point_to_basis(p):
    pt = tf2_geometry_msgs.PointStamped()
    pt.header.frame_id = "base_footprint"
    pt.point.x = p[0]
    pt.point.y = p[1]
    pt.point.z = p[2]

    try:
        pt = world_base_footprint_tf_buffer.transform(pt, "world", rospy.Duration(1))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        print(err)

    return np.array([pt.point.x, pt.point.y, pt.point.z])


def normalize_and_publish_map():
    grid_map_p = log_to_prob(map.map)
    
    points = []

    for i in range(grid_map_p.shape[0]):
        for j in range(grid_map_p.shape[1]):
            a = int(grid_map_p[i, j] * 255)
            rgb = struct.unpack('I', struct.pack('BBBB', a, a, a, a))[0]
            q = translate_points_from_center([i, j])
            points.append([q[1], q[0], 1, rgb])
        
  
    fields = [PointField('x',    0,  PointField.FLOAT32, 1),
              PointField('y',    4,  PointField.FLOAT32, 1),
              PointField('z',    8,  PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32,  1)]   

    header = std_msgs.msg.Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()
    generated_pc2 = pc2.create_cloud(header, fields, points)
    map_pc2_pub.publish(generated_pc2)

def log_to_prob(l):
    return 1 - (1 / (1 + np.exp(l)))

def prob_to_log(p):
    return np.log(p / (1 - p))

def lidar_reading_callback(data):
    global map

    data_gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    lidar_gen = np.array(list(data_gen))

    robot_pose = [particle.pose['x'], particle.pose['y']]
    robot_pose = translate_points_to_center(robot_pose)

    for p in lidar_gen:
        if not (p[2] > 1.0 and p[2] < 2.0):
            continue
        q = p
        q = transform_point_to_basis([q[0], q[1], q[2]])
        q = translate_points_to_center([q[0], q[1]])
        
        if q[0] < 0 or q[0] > map.w or q[1] < 0 or q[1] > map.h:
            continue

        # cast ray from robot to lidar point
        rr, cc = line(int(robot_pose[0]), int(robot_pose[1]), int(q[0]), int(q[1]))

        # miss
        map.map[cc, rr] += map.l_free - map.l_prioir

        # hit, but need to subtract the added value above
        map.map[cc[-1], rr[-1]] -= map.l_free - map.l_prioir
        map.map[cc[-1], rr[-1]] += map.l_occ - map.l_prioir

        # constraint ranges ranges
        map.map[map.map > 100] = 100
        map.map[map.map < 0] = 0

prev_time = time() # in sec
initial_pose = { 'x': 0, 'y': 0, 'theta': 0}
particle = Particle (initial_pose)

map = Map()
world_base_footprint_tf_buffer = None
world_base_footprint_tf_listener = None
world_base_footprint_tf_transform = None

if __name__ == '__main__':
    try:
        # Initialize SLAM code
        rospy.init_node('slam', anonymous=True)

        # Subscribers
        rospy.Subscriber("/pose2D", Pose2D,      scan_matching_callback)
        rospy.Subscriber("/cloud",  PointCloud2, lidar_reading_callback)
 
        # Publishers
        cloud_pub = rospy.Publisher('/cloud', PointCloud2,   queue_size=10)
        pose_pub  = rospy.Publisher("/pose",  PoseStamped,   queue_size=10)
        map_pc2_pub = rospy.Publisher("/map_pc2", PointCloud2, queue_size=10)

        world_base_footprint_tf_buffer = tf2_ros.Buffer()
        world_base_footprint_tf_listener = tf2_ros.TransformListener(world_base_footprint_tf_buffer, queue_size=30)

        # Execute SLAM code
        rate = rospy.Rate(1)  # 1hz
        while not rospy.is_shutdown():
            world_base_footprint_tf_transform = world_base_footprint_tf_buffer.lookup_transform('world', 'base_footprint', rospy.Time(0), rospy.Duration(1))
            normalize_and_publish_map()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass