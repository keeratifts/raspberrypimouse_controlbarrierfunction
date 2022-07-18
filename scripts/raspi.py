#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import *

def getposition(N): #get position in Gazebo simulator.
    pose = np.zeros((3, N))
    for i in range(0, N):
        odom = rospy.wait_for_message('/raspi_'+str(i)+'/odom', Odometry)
        pose[0, i] = odom.pose.pose.position.x
        pose[1, i] = odom.pose.pose.position.y
        orientation_q = odom.pose.pose.orientation
        orientation_list =  [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        pose[2, i] = yaw

    return pose

def get_robot_position(N): #get position of ARtag markers attached on the robots using Intel Realsense Camera
    pose = np.zeros((4, N))
    i = 0
    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
    for m in AlvarMsg.markers:
        marker_id = m.id
        if marker_id >= 0:
            pose[0, i] = m.pose.pose.position.y
            pose[1, i] = -m.pose.pose.position.z
            orientation_q = m.pose.pose.orientation
            orientation_list =  [orientation_q.y, orientation_q.z, orientation_q.x, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            pose[2, i] = -yaw
            pose[3, i] = m.id
            i += 1
    ind = np.argsort(pose[3,:])
    pose = pose[:,ind]
    return pose[[0,1,2], :]
    # AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
    # pose = np.empty((4, N), float)

    # while len(AlvarMsg.markers) == N:
    #     for m in AlvarMsg.markers:
    #         pose[0, m.id] = m.pose.pose.position.y #Since the camera is attached on a ceil. TF frame need to be transformed.
    #         pose[1, m.id] = -m.pose.pose.position.z
    #         orientation_q = m.pose.pose.orientation
    #         orientation_list =  [orientation_q.y, orientation_q.z, orientation_q.x, orientation_q.w]
    #         (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #         pose[2, m.id] = -yaw
    #         pose[3, m.id] = m.id

    #     return pose[[0,1,2], :]

        
def put_velocities(N, dxu):
    for i in range(0, N):
        velPub = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=3)
        velMsg = create_vel_msg(np.round(dxu[0, i], 2), np.round(dxu[1, i], 2))
        velPub.publish(velMsg)

def create_vel_msg(v, w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

def set_velocities(ids, velocities, max_linear_velocity = 0.1, max_angular_velocity = 4):

        # Threshold linear velocities
    idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
    velocities[0, idxs] = max_linear_velocity*np.sign(velocities[0, idxs])

    idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
    velocities[1, idxs] = max_angular_velocity*np.sign(velocities[1, idxs])
    return velocities
