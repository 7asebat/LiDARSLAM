#!/usr/bin/env python3
import rospy
import numpy as np
from time import time

# ROS dependencies
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg    import PointCloud2, PointField
from geometry_msgs.msg  import PoseStamped, Pose2D
from skimage.draw import line

import std_msgs

from bresenham3d import Bresenham3D
class Map:
    def __init__(self, w = 125, h = 125, e = 10):
        self.w = w # width
        self.h = h # height
        self.e = e # elevation

        p_occ = 0.75 # probability of occupancy in the sensor model
        p_free = 0.45 # probability of free in the sensor model
        p_prior = 0.5 # prior probability of occupancy in the sensor model
        self.l_occ = prob_to_log(p_occ) # log odds of occupancy in the sensor model
        self.l_free = prob_to_log(p_free) # log odds of free in the sensor model
        self.prior = prob_to_log(p_prior) # log odds of prior occupancy in the sensor model

        self.map =  np.ones((w, h, e)) * self.prior

class Particle:
    def __init__(self, pose):
        self.pose = pose

def scan_matching_callback(data):
    global particle
    particle.pose['x'] = data.x
    particle.pose['y'] = data.y
    particle.pose['theta'] = data.theta

def translate_points_to_center(p):
    return p[0] + map.h/2, p[1] + map.w/2, p[2]

def translate_points_from_center(p):
    return p[0] - map.h/2 + 1, p[1] - map.w/2 + 1, p[2]

def enumerate_3(M):
    def indices_3(*shape):
        idx = np.transpose(np.indices(shape), (2, 1, 3, 0))
        return idx.reshape((shape[0] * shape[1] * shape[2], 3))
    
    data = np.asmatrix(M.flatten()).T
    augmented = indices_3(*M.shape)
    augmented = np.hstack((augmented, data))
    
    return augmented

def normalize_and_publish_map():
    grid_map_p = log_to_prob(map.map)

    points = enumerate_3(grid_map_p)
    points = np.delete(points, np.where(points[:, 3] < 0.7), axis=0)
    points = points[:, :3]
    points += [-map.h / 2 + 1, -map.w / 2, 0]
    points = points.tolist()

    fields = [PointField('x',    0,  PointField.FLOAT32, 1),
              PointField('y',    4,  PointField.FLOAT32, 1),
              PointField('z',    8,  PointField.FLOAT32, 1)]   

    header = std_msgs.msg.Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()
    generated_pc2 = pc2.create_cloud(header, fields, points)
    map_pc2_pub.publish(generated_pc2)

def log_to_prob(l):
    return 1 - (1 / (1 + np.exp(l)))

def prob_to_log(p):
    return np.log(p / (1 - p))

def get_points_between(p1, p2, method='naive'):
    if method == 'naive':
        diff = p2 - p1
        num_points = np.linalg.norm(diff)
        unit_diff = diff / num_points
        points = np.array([p1 + i * unit_diff for i in range(int(num_points + 1))]).astype(int)
        return points
    elif method == 'bresenham':
        return Bresenham3D(p1, p2)
    else:
        raise Exception("Wrong method parameter given, expected one of 'naive' or 'bresenham'")

def cast_ray_and_update_map(current_robot_pose, transformed_lidar_p, method='naive'):
    if method == 'skimage_line_2d':
        # cast ray from robot to lidar point
        rr, cc = line(int(current_robot_pose[0]), int(current_robot_pose[1]), int(transformed_lidar_p[0]), int(transformed_lidar_p[1]))

        # miss
        map.map[cc, rr, 0] += map.l_free - map.prior

        # hit, but need to subtract the added value above
        map.map[cc[-1], rr[-1], 0] -= map.l_free - map.prior
        map.map[cc[-1], rr[-1], 0] += map.l_occ - map.prior
        return
    else:
        matched_points = get_points_between(np.array(current_robot_pose).astype(int), np.array(transformed_lidar_p).astype(int), method)
        map.map[matched_points[:, 1], matched_points[:, 0], matched_points[:, 2]] += map.l_free - map.prior

        map.map[matched_points[-1, 1], matched_points[-1, 0], matched_points[-1, 2]] -= map.l_free - map.prior
        map.map[matched_points[-1, 1], matched_points[-1, 0], matched_points[-1, 2]] += map.l_occ - map.prior
        return

def transformed_lidar_reading_callback(data):
    global map
    data = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    data = np.array(list(data))

    current_robot_pose = [particle.pose['x'], particle.pose['y'], 0]
    current_robot_pose = translate_points_to_center(current_robot_pose)

    for lidar_p in data:
        if not (lidar_p[2] > 0.3):
            continue
        transformed_lidar_p = lidar_p
        transformed_lidar_p = translate_points_to_center([transformed_lidar_p[0], transformed_lidar_p[1], transformed_lidar_p[2]])
        
        # check bounds
        if transformed_lidar_p[0] < 0 \
            or transformed_lidar_p[0] > map.w \
            or transformed_lidar_p[1] < 0 \
            or transformed_lidar_p[1] > map.h \
            or transformed_lidar_p[2] < 0 \
            or transformed_lidar_p[2] > map.e:
                continue
        
        # filter out car particles
        if transformed_lidar_p[0] - current_robot_pose[0] < 4.0 \
            and np.abs(transformed_lidar_p[1] - current_robot_pose[1]) < 1.0:
            continue

        cast_ray_and_update_map(current_robot_pose, transformed_lidar_p, 'bresenham')
        
        # constraint ranges ranges
        map.map[map.map > 100] = 100
        map.map[map.map < 0] = 0

prev_time = time() # in sec
initial_pose = { 'x': 0, 'y': 0, 'theta': 0}
particle = Particle(initial_pose)

map = Map(125, 125, 10)
world_base_footprint_tf_buffer = None
world_base_footprint_tf_listener = None
world_base_footprint_tf_transform = None

if __name__ == '__main__':
    try:
        # Initialize SLAM code
        rospy.init_node('slam', anonymous=True)

        # Subscribers
        rospy.Subscriber("/pose2D",            Pose2D,      scan_matching_callback)
        rospy.Subscriber("/transformed_cloud", PointCloud2, transformed_lidar_reading_callback)

        # Publishers
        cloud_pub = rospy.Publisher('/cloud', PointCloud2,   queue_size=10)
        pose_pub  = rospy.Publisher("/pose",  PoseStamped,   queue_size=10)
        map_pc2_pub = rospy.Publisher("/map_pc2", PointCloud2, queue_size=10)

        # Execute SLAM code
        rate = rospy.Rate(1)  # 1hz
        while not rospy.is_shutdown():
            normalize_and_publish_map()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass