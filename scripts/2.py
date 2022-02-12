#! /usr/bin/env python
from shapely.ops import orient
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from math import *
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
from time import sleep
R = 20

def getposition(N):
    odom = {}
    y = []
    x = []
    orient = []
    for i in range(0, N):
        odom[i] = rospy.wait_for_message('/raspi_'+str(i)+'/odom', Odometry)
        x.append(odom[i].pose.pose.position.x)
        y.append(odom[i].pose.pose.position.y)
        orientation_q = odom[i].pose.pose.orientation
        orientation_list =  [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        orient.append(yaw)
    pose = np.array([[x for x in x], [y for y in y], [z for z in orient]])

    return pose

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

def si_position_controller(xi, positions, x_velocity_gain=0.45, y_velocity_gain=0.45, velocity_magnitude_limit=0.15):
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

def si_barrier_cert(dxi, x, barrier_gain=80, safety_radius=0.3, magnitude_limit=0.1):
    N = dxi.shape[1]
    num_constraints = int(comb(N, 2))
    A = np.zeros((num_constraints, 2*N))
    b = np.zeros(num_constraints)
    H = sparse(matrix(2*np.identity(2*N)))

    count = 0
    for i in range(N-1):
        for j in range(i+1, N):
            error = x[:, i] - x[:, j]
            h = (error[0]*error[0] + error[1]*error[1]) - np.power(safety_radius, 2)

            A[count, (2*i, (2*i+1))] = -2*error
            A[count, (2*j, (2*j+1))] = 2*error
            b[count] = barrier_gain*np.power(h, 3)

            count += 1

        # Threshold control inputs before QP
    norms = np.linalg.norm(dxi, 2, 0)
    idxs_to_normalize = (norms > magnitude_limit)
    dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

    f = -2*np.reshape(dxi, 2*N, order='F')
    result = qp(H, matrix(f), matrix(A), matrix(b))['x']

    return np.reshape(result, (2, -1), order='F')

def create_vel_msg(v, w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w*0.65
    return velMsg

def put_velocities(N, dxu):
    for i in range(0, N):
        velPub = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=10)
        velMsg = create_vel_msg(dxu[0][i], dxu[1][i])
        velPub.publish(velMsg)

def set_velocities(ids, velocities, max_linear_velocity = 0.3, max_angular_velocity = 3):

        # Threshold linear velocities
        idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
        velocities[0, idxs] = max_linear_velocity*np.sign(velocities[0, idxs])

        idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
        velocities[1, idxs] = max_angular_velocity*np.sign(velocities[1, idxs])
        return velocities

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        radius = 1.5
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*R, 2)/(2*R))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        flag = 0
        x_goal = p_circ[:, :R]

        while not rospy.is_shutdown():
            pose = getposition(R)
            pose_si = uni_to_si_states(pose)
            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1-flag

            if(flag == 0):
                x_goal = p_circ[:, :R]
            else:
                x_goal = p_circ[:, R:]


            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(R, dxu)
            print (k)
            put_velocities(R, k)

    except rospy.ROSInterruptException:
        pass
