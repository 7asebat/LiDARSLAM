#!/usr/bin/env python
import rospy
import numpy as np
from time import time

# ROS dependencies
from std_msgs.msg       import String
from sensor_msgs.msg    import PointCloud2
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PoseArray
from geometry_msgs.msg  import TransformStamped

from tf.transformations import quaternion_from_euler
from tf2_ros            import TransformBroadcaster

import sensor_msgs.pointcloud2 as pc2

def lidar_callback(data):
    #points = np.array(pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True))
    pass

def normalize_angle(angle):
    pass

def pose_update():
    global particles

    pose_stamped = PoseStamped()
    br = TransformBroadcaster()
    t = TransformStamped()

    # header   
    pose_stamped.header.stamp    = t.header.stamp    = rospy.Time.now()
    pose_stamped.header.frame_id = t.header.frame_id = "world"
    t.child_frame_id = "base_footprint"

    # position
    particle = particles[0]
    pose_stamped.pose.position.x = t.transform.translation.x = particle.pose['x']
    pose_stamped.pose.position.y = t.transform.translation.y = particle.pose['y']
    pose_stamped.pose.position.z = t.transform.translation.z = 0

    # orientation
    yaw = particle.pose['theta']
    q = quaternion_from_euler(0, 0, yaw)
    pose_stamped.pose.orientation.x = t.transform.rotation.x = q[0] # 0
    pose_stamped.pose.orientation.y = t.transform.rotation.y = q[1] # 0
    pose_stamped.pose.orientation.z = t.transform.rotation.z = q[2] # sin(yaw/2)
    pose_stamped.pose.orientation.w = t.transform.rotation.w = q[3] # cos(yaw/2)

    br.sendTransform(t)
    return pose_stamped

class Map:
    def __init():
        pass

class Particle:
    def __init__(self, num_particles, motion_noise, pose):
        self.num_particles = num_particles
        self.motion_noise = motion_noise
        self.pose = pose
        # self.map = Map(100, 100)

    def predict(self, v, w, dt):
        # sample control
        v = np.random.normal(v, self.motion_noise['v'])
        w = np.random.normal(w, self.motion_noise['w'])

        # Apply motion model
        self.pose['x'] += v * dt * np.cos(self.pose['theta'])
        self.pose['y'] += v * dt * np.sin(self.pose['theta'])
        self.pose['theta'] = normalize_angle(self.pose['theta'] + w * dt)

    def correct(self):
        pass

def control_callback(data):
    global particles, prev_time
    current_time = time()
    dt = current_time - prev_time # in sec
    prev_time = current_time

    if (dt > 1): # more than 1 sec
        return

    v = data.linear.x # in m/sec
    w = data.angular.z # in rad/s

    for particle in particles:
        particle.predict(v, w, dt)    


prev_time = time() # in sec
motion_noise = { 'v': 0, 'w': 0 }
initial_pose = { 'x': 0, 'y': 0, 'z': 0}
num_particles = 1
map_size = { 'x': 200, 'y': 200, 'RES': 1/4 } # send to the particle to create map
particles = [Particle(num_particles, motion_noise, initial_pose) for _ in range(num_particles)]

if __name__ == '__main__':
    try:
        # Initialize SLAM code
        rospy.init_node('slam', anonymous=True)

        # Subscribers
        rospy.Subscriber("/points_raw", PointCloud2, lidar_callback)
        rospy.Subscriber("/cmd_vel",    Twist,       control_callback)
 
        # Publishers
        pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        map_pub  = rospy.Publisher("/map",  PointCloud2, queue_size=10)

        # Execute SLAM code
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            pose_pub.push(pose_update())

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
