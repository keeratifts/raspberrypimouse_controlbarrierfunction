#! /usr/bin/env python
import numpy as np
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
from math import *

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

def si_barrier_cert(dxi, x, barrier_gain=80, safety_radius=0.25, magnitude_limit=0.1):
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

def robotFeedbackControl(xi, positions, GOAL_DIST_THRESHOLD=0.05, K_RO=3, K_ALPHA=13, V_CONST=0.2):
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

def robotPDFeedbackControl(xi, positions, n, a, GOAL_DIST_THRESHOLD=0.05, K_RO=3, K_ALPHA=9, Kd_RO = 5, Kd_ALPHA = 5, V_CONST=0.2):
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