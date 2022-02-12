#! /usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import *

def getposition(N):
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

def get_robot_location(robot):
    AlvarMsg = rospy.wait_for_message('/ar_pose_marker', AlvarMarkers)
    pose = np.zeros((3, N))
    for m in AlvarMsg.markers:
        pose[0, ] = m.pose.pose.position
        if marker_id <= 5:
            marker_pose = m.pose.pose
            pos = marker_pose.position
            ori = marker_pose.orientation
            ori_list = [ori.y, ori.z, ori.x, ori.w]

            #transform orientation to euler
            (roll, pitch, yaw) = euler_from_quaternion(ori_list)
            robot[marker_id] = {'x': pos.y, 'y': pos.z, 'yaw': yaw}
            robot = collections.OrderedDict(sorted(robot.items())) #sort dict by markers no.

    return robot

def put_velocities(N, dxu):
    for i in range(0, N):
        velPub = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=3)
        velMsg = create_vel_msg(dxu[0, i], dxu[1, i])
        velPub.publish(velMsg)

def create_vel_msg(v, w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w * 0.65
    return velMsg

def set_velocities(ids, velocities, max_linear_velocity = 0.3, max_angular_velocity = 8):

        # Threshold linear velocities
    idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
    velocities[0, idxs] = max_linear_velocity*np.sign(velocities[0, idxs])

    idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
    velocities[1, idxs] = max_angular_velocity*np.sign(velocities[1, idxs])
    return velocities
