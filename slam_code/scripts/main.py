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
from geometry_msgs.msg  import PointStamped, PoseStamped, Pose2D
from nav_msgs.msg import OccupancyGrid
from skimage.draw import line

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import tf2_ros
import tf2_geometry_msgs
import sensor_msgs
import std_msgs
import ros_numpy

def normalize_angle(angle):
    while angle > np.pi:
        angle = angle - 2. * np.pi

    while angle < -np.pi:
        angle = angle + 2. * np.pi

    return angle

class Map:
    def __init__(self, w = 200, h = 200, e = 10):
        self.w = w # width
        self.h = h # height
        self.e = e # elevation

        # self.map = np.ones(w, h) * 0.5
        self.map = np.zeros((w, h, e))
class Particle:
    def __init__(self, motion_noise, pose):
        self.motion_noise = motion_noise
        self.pose = pose

def pose_2d_callback(data):
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
        pt = world_base_footprint_tf_buffer.transform(pt, "world")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        print(err)

    return np.array([pt.point.x, pt.point.y, pt.point.z])

def get_points_between(p1, p2):
    diff = p2 - p1
    num_points = np.linalg.norm(diff)
    unit_diff = diff / num_points
    points = np.array([p1 + i * unit_diff for i in range(int(num_points + 1))]).astype(int)
    # unique_points = np.unique(points, axis=0)
    return points


def normalize_and_publish_map():
    grid_map = map.map

    grid_map_occupied = (grid_map > 3).astype(int) * 127
    grid_map_occupied_2d = grid_map_occupied[:, :, 2]

    occup_grid = OccupancyGrid()
    occup_grid.info.width = map.w
    occup_grid.info.height = map.h
    occup_grid.data = grid_map_occupied_2d.ravel()
    occup_grid.header.stamp = rospy.Time.now()
    occup_grid.header.frame_id = "world"
    occup_grid.info.resolution = 1.0
    occup_grid.info.origin.position.x = -map.w/2
    occup_grid.info.origin.position.y = -map.h/2
    occup_grid.info.origin.position.z = 0.0
    occup_grid.info.origin.orientation.x = 0.0
    occup_grid.info.origin.orientation.y = 0.0
    occup_grid.info.origin.orientation.z = 0.0
    occup_grid.info.origin.orientation.w = 1.0

    map_pub.publish(occup_grid)
    
    # points = []

    # for i in range(grid_map.shape[0]):
    #     for j in range(grid_map.shape[1]):
    #         for k in range(grid_map.shape[2]):
    #             a = (grid_map[i, j, k] > 3) * 255
    #             rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 0, a))[0]
    #             q = [i, j, k]
    #             points.append([q[1], q[0], q[2], rgb])
        
  
    # fields = [PointField('x',    0,  PointField.FLOAT32, 1),
    #           PointField('y',    4,  PointField.FLOAT32, 1),
    #           PointField('z',    8,  PointField.FLOAT32, 1),
    #           PointField('rgba', 12, PointField.UINT32,  1)]   

    # header = std_msgs.msg.Header()
    # header.frame_id = "world"
    # header.stamp = rospy.Time.now()
    # generated_pc2 = pc2.create_cloud(header, fields, points)
    # map_pc2_pub.publish(generated_pc2)



def lidar_callback(data):
    global map

    data_gen = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
    lidar_gen = np.array(list(data_gen))

    robot_pose = [particle.pose['x'], particle.pose['y'], 1]
    robot_pose = translate_points_to_center(robot_pose)

    for p in lidar_gen:
        # filter ground
        if not (p[2] > 0.5 and p[2] < 10):
            continue

        q = p
        q = transform_point_to_basis([q[0], q[1], q[2]])
        q = translate_points_to_center([q[0], q[1], q[2]])
        
        # filter out of bound
        if q[0] < 0 or q[0] > map.w or q[1] < 0 or q[1] > map.h or q[2] < 0 or q[2] > map.e:
            continue

        matched_points = get_points_between(
            np.array([robot_pose[0], robot_pose[1], robot_pose[2]]).astype(int), 
            np.array([q[0], q[1], q[2]]).astype(int))

        # map.map[matched_points[:, 1], matched_points[:, 0], matched_points[:, 2]] -= 1
        map.map[matched_points[-1, 1], matched_points[-1, 0], matched_points[-1, 2]] += 50
        


prev_time = time() # in sec
motion_noise = { 'v': 0, 'w': 0 }
initial_pose = { 'x': 0, 'y': 0, 'theta': 0}
particle = Particle(motion_noise, initial_pose)
map = Map()
world_base_footprint_tf_buffer = None
world_base_footprint_tf_listener = None
world_base_footprint_tf_transform = None

if __name__ == '__main__':
    try:
        # Initialize SLAM code
        rospy.init_node('slam', anonymous=True)

        # Subscribers
        rospy.Subscriber("/pose2D", Pose2D,      pose_2d_callback)
        rospy.Subscriber("/cloud",  PointCloud2, lidar_callback)
 
        # Publishers
        cloud_pub = rospy.Publisher('/cloud', PointCloud2,   queue_size=10)
        pose_pub  = rospy.Publisher("/pose",  PoseStamped,   queue_size=10)
        map_pub   = rospy.Publisher("/map",   OccupancyGrid, queue_size=10)
        map_pc2_pub = rospy.Publisher("/map_pc2", PointCloud2, queue_size=10)

        world_base_footprint_tf_buffer = tf2_ros.Buffer()
        world_base_footprint_tf_listener = tf2_ros.TransformListener(world_base_footprint_tf_buffer)

        # Execute SLAM code
        rate = rospy.Rate(1)  # 10hz
        while not rospy.is_shutdown():
            world_base_footprint_tf_transform = world_base_footprint_tf_buffer.lookup_transform('world', 'base_footprint', rospy.Time(0), rospy.Duration(3))
            normalize_and_publish_map()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass