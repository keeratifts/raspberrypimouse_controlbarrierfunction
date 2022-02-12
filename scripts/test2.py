#! /usr/bin/env python

from turtle import pos
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import *

x_velocity_gain=0.45
y_velocity_gain=0.45
Kd_RO = 5
Kd_ALPHA = 25
DATA_PATH = '/home/robolab/raspi_ws/src/coverage_control/Data'
LOG_DIR = DATA_PATH + '/Log_feedback_3'

X_traj = np.array([])
Y_traj = np.array([])
THETA_traj = np.array([])
X_goal = np.array([])
Y_goal = np.array([])
THETA_goal = np.array([])

def getposition(N):
    pose = np.zeros((3, N))
    for i in range(0, N):
        odom = rospy.wait_for_message('/odom', Odometry)
        pose[0, i] = odom.pose.pose.position.x
        pose[1, i] = odom.pose.pose.position.y
        orientation_q = odom.pose.pose.orientation
        orientation_list =  [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        pose[2, i] = yaw

    return pose

def put_velocities(N, dxu):
    for i in range(0, N):
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        velMsg = create_vel_msg(dxu[0, i], dxu[1, i])
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

def set_velocities(ids, velocities, max_linear_velocity = 0.3, max_angular_velocity = 5):

        # Threshold linear velocities
    idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
    velocities[0, idxs] = max_linear_velocity*np.sign(velocities[0, idxs])

    idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
    velocities[1, idxs] = max_angular_velocity*np.sign(velocities[1, idxs])
    return velocities

def si_position_controller(xi, positions, velocity_magnitude_limit=0.15):
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

        # Calculate control input
    dxi[0][:] = x_velocity_gain*(positions[0][:]-xi[0][:])
    dxi[1][:] = y_velocity_gain*(positions[1][:]-xi[1][:])

        # Threshold magnitude
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]

    return dxi

def uni_to_si_states(poses, projection_distance=0.05):
    """Takes unicycle states and returns single-integrator states
    poses: 3xN numpy array of unicycle states
    -> 2xN numpy array of single-integrator states
    """

    _,N = np.shape(poses)

    si_states = np.zeros((2, N))
    si_states[0, :] = poses[0, :] + projection_distance*np.cos(poses[2, :])
    si_states[1, :] = poses[1, :] + projection_distance*np.sin(poses[2, :])

    return si_states

def si_to_uni_dyn(dxi, poses, projection_distance=0.05, angular_velocity_limit = np.pi):
    M,N = np.shape(dxi)

    cs = np.cos(poses[2, :])
    ss = np.sin(poses[2, :])

    dxu = np.zeros((2, N))
    dxu[0, :] = (cs*dxi[0, :] + ss*dxi[1, :])
    dxu[1, :] = (1/projection_distance)*(-ss*dxi[0, :] + cs*dxi[1, :])
        #Impose angular velocity cap.
    dxu[1,dxu[1,:]>angular_velocity_limit] = angular_velocity_limit
    dxu[1,dxu[1,:]<-angular_velocity_limit] = -angular_velocity_limit 

    return dxu

def robotPDFeedbackControl(xi, positions, n, a, GOAL_DIST_THRESHOLD=0.05, K_RO=3, K_ALPHA=9, V_CONST=0.2):
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))
    
    norms = np.linalg.norm(xi[0:2, 0:N]-positions, axis = 0)
    lamda = np.arctan2(positions[1][:]-xi[1][:], positions[0][:]-xi[0][:])
    alpha = np.array([(lamda[:] - xi[2][:] + pi) % (2 * pi) - pi]) #-360degrees
    d_norms = norms - n
    d_alpha = alpha - a
    v = (K_RO * norms) + (Kd_RO * d_norms)
    w = (K_ALPHA * alpha) + (Kd_ALPHA * d_alpha)
    print ((Kd_ALPHA * d_alpha))
    n = norms
    a = alpha
    dxi[0][:] = v[:] / abs(v[:]) * V_CONST
    dxi[1][:] = w[:] / abs(v[:]) * V_CONST
    idxs = np.where(norms < GOAL_DIST_THRESHOLD)
    dxi[:, idxs] = 0
    return dxi, n, a


if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_goal = np.array([[-2], [-2]])
        N = 1
                #Plot feedback control graph
        log_sim_params = open(LOG_DIR+'/LogSimParams.txt','w+')

        text = 'Simulation parameters: \r\n'
        text = text + 'k_rho = %.3f \r\n' % Kd_RO
        text = text + 'k_alpha = %.3f \r\n' % Kd_ALPHA
        log_sim_params.write(text)
        log_sim_params.close()
        norms = np.zeros((1,N))
        alpha = np.zeros((1,N))


        while not rospy.is_shutdown():
            pose = getposition(N)
            # pose_si = uni_to_si_states(pose)
            if(np.linalg.norm(pose[0:2, 0:N]-x_goal)< 0.05):
                rospy.signal_shutdown('End of testing')
                np.savetxt(LOG_DIR+'/X_traj.csv', X_traj, delimiter=' , ')
                np.savetxt(LOG_DIR+'/Y_traj.csv', Y_traj, delimiter=' , ')
                np.savetxt(LOG_DIR+'/THETA_traj.csv', THETA_traj, delimiter=' , ')
                np.savetxt(LOG_DIR+'/X_goal.csv', X_goal, delimiter=' , ')
                np.savetxt(LOG_DIR+'/Y_goal.csv', Y_goal, delimiter=' , ')

            # dxi = si_position_controller(pose_si, x_goal)
            # dxu = si_to_uni_dyn(dxi, pose)

            (dxu, norms, alpha) = robotPDFeedbackControl(pose, x_goal, norms, alpha)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

            X_traj = np.append(X_traj, pose[0, 0])
            Y_traj = np.append(Y_traj, pose[1, 0])
            THETA_traj = np.append(THETA_traj, degrees(pose[2, 0]))
            X_goal = np.append(X_goal, x_goal[0, 0])
            Y_goal = np.append(Y_goal, x_goal[1, 0])

    except rospy.ROSInterruptException:
        
        pass