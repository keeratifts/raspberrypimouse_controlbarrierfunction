#! /usr/bin/env python

import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from time import sleep

N = 1

def get_robot_position(N):
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

def robotPDFeedbackControl(xi, positions, n, a, GOAL_DIST_THRESHOLD=0.08, K_RO=2, K_ALPHA=4, Kd_RO = 1.5, Kd_ALPHA = 5, V_CONST=0.25):
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

    norms = np.linalg.norm(xi[0:2, 0:N]-positions, axis = 0)
    lamda = np.arctan2(positions[1][:]-xi[1][:], positions[0][:]-xi[0][:])
    alpha = np.array([(lamda[:] - xi[2][:] + np.pi) % (2 * np.pi) - np.pi]) #-360degrees
    d_norms = norms - n
    d_alpha = alpha - a
    v = (K_RO * norms) + (Kd_RO * d_norms)
    w = (K_ALPHA * alpha) + (Kd_ALPHA * d_alpha)
    n = norms
    a = alpha
    dxi[0][:] = v[:] / abs(v[:]) * V_CONST
    dxi[1][:] = w[:] / abs(v[:]) * V_CONST
    idxs = np.where(norms < GOAL_DIST_THRESHOLD)
    dxi[:, idxs] = 0
    return dxi, n, a

def create_vel_msg(v, w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

def put_velocities(N, dxu):
    for i in range(0, N):
        velPub = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=10)
        velMsg = create_vel_msg(dxu[0, i], dxu[1, i])
        velPub.publish(velMsg)

def set_velocities(ids, velocities, max_linear_velocity = 0.3, max_angular_velocity = 1.2):

        # Threshold linear velocities
    idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
    velocities[0, idxs] = max_linear_velocity*np.sign(velocities[0, idxs])

    idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
    velocities[1, idxs] = max_angular_velocity*np.sign(velocities[1, idxs])
    return velocities

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        N = 1
        x_goal = np.array([[-0.5], [0.5]])
        norms = np.zeros((1,N))
        alpha = np.zeros((1,N))
        while not rospy.is_shutdown():
            pose = get_robot_position(N)
            (dxu, norms, alpha) = robotPDFeedbackControl(pose, x_goal, norms, alpha)
            k = set_velocities(N, dxu)
            put_velocities(N, k)
            print (k)
            if(np.linalg.norm(pose[0:2, 0:N]-x_goal)< 0.05):
                sleep(1)
                x_goal[[0, 1]] = x_goal[[1, 0]]

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass