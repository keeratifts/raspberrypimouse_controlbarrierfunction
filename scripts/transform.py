#! /usr/bin/env python
import numpy as np

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