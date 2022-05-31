#!/usr/bin/env python3
from matplotlib.pyplot import grid
import rospy
import numpy as np
from time import time
import timeit

# ROS dependencies
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg    import PointCloud2, PointField
from geometry_msgs.msg  import PoseStamped, Pose2D
from skimage.draw import line

import tf2_ros
import tf2_geometry_msgs
import std_msgs
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
    points += [-map.h / 2 + 1, -map.w / 2 + 1, 0]
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

def lidar_reading_callback(data):
    global map

    data = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    data = np.array(list(data))

    current_robot_pose = [particle.pose['x'], particle.pose['y'], 0]
    current_robot_pose = translate_points_to_center(current_robot_pose)

    for lidar_p in data:
        if not (lidar_p[2] > 1.0 and lidar_p[2] < 2.0):
            continue
        transformed_lidar_p = lidar_p
        transformed_lidar_p = transform_point_to_basis([transformed_lidar_p[0], transformed_lidar_p[1], transformed_lidar_p[2]])
        transformed_lidar_p = translate_points_to_center([transformed_lidar_p[0], transformed_lidar_p[1], transformed_lidar_p[2]])
        
        # check bounds
        if transformed_lidar_p[0] < 0 \
            or transformed_lidar_p[0] > map.w \
            or transformed_lidar_p[1] < 0 \
            or transformed_lidar_p[1] > map.h:
                continue

        # cast ray from robot to lidar point
        rr, cc = line(int(current_robot_pose[0]), int(current_robot_pose[1]), int(transformed_lidar_p[0]), int(transformed_lidar_p[1]))

        # miss
        map.map[cc, rr, 0] += map.l_free - map.prior

        # hit, but need to subtract the added value above
        map.map[cc[-1], rr[-1], 0] -= map.l_free - map.prior
        map.map[cc[-1], rr[-1], 0] += map.l_occ - map.prior

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