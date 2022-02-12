#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *

N = 20

def uni_to_si_dyn(dxu, poses, projection_distance=0.05):
    M,N = np.shape(dxu)

    cs = np.cos(poses[2, :])
    ss = np.sin(poses[2, :])

    dxi = np.zeros((2, N))
    dxi[0, :] = (cs*dxu[0, :] - projection_distance*ss*dxu[1, :])
    dxi[1, :] = (ss*dxu[0, :] + projection_distance*cs*dxu[1, :])

    return dxi


if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        radius = 1.5
        xybound = radius*np.array([-1, 1, -1, 1])
        p_theta = 2*np.pi*(np.arange(0, 2*N, 2)/(2*N))
        p_circ = np.vstack([
            np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
            np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
            ])
        flag = 0
        x_goal = p_circ[:, :N]
        norms = np.zeros((1,N))
        alpha = np.zeros((1,N))
        while not rospy.is_shutdown():
            pose = getposition(N)
            (dxu, norms, alpha) =robotPDFeedbackControl(pose, x_goal, norms, alpha)
            pose_si = uni_to_si_states(pose)
            dxi = uni_to_si_dyn(dxu, pose)
            print (np.linalg.norm(x_goal - pose_si))
            if(np.linalg.norm(x_goal - pose_si) < 0.3):

                flag = 1-flag

            if(flag == 0):
                x_goal = p_circ[:, :N]
            else:
                x_goal = p_circ[:, N:]

            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        
        pass
