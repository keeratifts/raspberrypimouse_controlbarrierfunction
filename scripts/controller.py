#! /usr/bin/env python
import numpy as np
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
from scipy.integrate import odeint
from math import *

#Feedback control parameter for REAL ROBOT
GOAL_DIST_THRESHOLD=0.08
K_RO=2
K_ALPHA=8
V_CONST=0.25

#Feedback control parameter for SIMULATED ROBOT
GOAL_DIST_THRESHOLD=0.05
K_RO=3
K_ALPHA=13
V_CONST=0.2

def si_position_controller(xi, positions, x_velocity_gain=1, y_velocity_gain=1, velocity_magnitude_limit=0.5):
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

def si_barrier_cert(dxi, x, barrier_gain=80, safety_radius=0.12, magnitude_limit=0.25):
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

def robotFeedbackControl(xi, positions, GOAL_DIST_THRESHOLD, K_RO, K_ALPHA, V_CONST): #P controller
    
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

    norms = np.linalg.norm(xi[0:2, 0:N]-positions, axis = 0)
    lamda = np.arctan2(positions[1][:]-xi[1][:], positions[0][:]-xi[0][:])
    alpha = np.array([(lamda[:] - xi[2][:] + pi) % (2 * pi) - pi]) #-360degrees
    v = K_RO * norms
    w = K_ALPHA * alpha

    dxi[0][:] = v[:] / abs(v[:]) * V_CONST
    dxi[1][:] = w[:] / abs(v[:]) * V_CONST

    idxs = np.where(norms < GOAL_DIST_THRESHOLD)
    dxi[:, idxs] = 0
    return dxi

def robotPDFeedbackControl(xi, positions, n, a, GOAL_DIST_THRESHOLD=0.05, K_RO=3, K_ALPHA=13, Kd_RO = 5, Kd_ALPHA = 5, V_CONST=0.3): #PD controller
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

    norms = np.linalg.norm(xi[0:2, 0:N]-positions, axis = 0)
    lamda = np.arctan2(positions[1][:]-xi[1][:], positions[0][:]-xi[0][:])
    alpha = np.array([(lamda[:] - xi[2][:] + pi) % (2 * pi) - pi]) #-360degrees
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

def diff_equation_fatii(y_list,t,e,omega):
    ki = 22
    sum_fu = y_list[1] + e
    coe = 0
    for i in range(2,len(y_list)):
        if i%2 == 0:
            coe = int(i/2)
            sum_fu += (y_list[i] + e*cos(coe*omega*t)) * cos(coe*omega*t)
        else:
            coe = int((i-1)/2)
            sum_fu += (y_list[i] + e*sin(coe*omega*t)) * sin(coe*omega*t)

        result = []
        result.append(-ki*e-sum_fu + 20*cos(pi*t))
        result.append(-e)
    for i in range(2, len(y_list)):
        if i%2 == 0:
            coe = int(i/2)
            result.append(coe*e*omega*sin(coe*omega*t) + ki*e*cos(coe*omega*t))
        else:
            coe = int((i-1)/2)
            result.append((-e)*coe*omega*cos(coe*omega*t)+ki*e*sin(coe*omega*t))
    return np.array(result)
                     

def cal_tra_fatii(new_coords,new_centroids):
    T=5
    t=np.linspace(0,T,num=1)
    omega = pi*2/T
    point_lists = []
    for i in range(len(new_coords)):
        y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
        y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
        result_x = odeint(diff_equation_fatii, y_list_x, t, args=(new_coords[i][0]-new_centroids[i][0],omega))
        result_y = odeint(diff_equation_fatii, y_list_y, t, args=(new_coords[i][1]-new_centroids[i][1],omega))
        result_xt = result_x[:,0]
        result_yt = result_y[:,0]
        new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
        point_lists.append(list(new_result))
    return point_lists