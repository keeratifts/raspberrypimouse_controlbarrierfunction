#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *

N = 20

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
        while not rospy.is_shutdown():
            pose = getposition(N)
            print (np.column_stack((pose[0:2])))
            pose_si = uni_to_si_states(pose)
            if(np.linalg.norm(x_goal - pose_si) < 0.05):
                flag = 1-flag

            if(flag == 0):
                x_goal = p_circ[:, :N]
            else:
                x_goal = p_circ[:, N:]


            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
