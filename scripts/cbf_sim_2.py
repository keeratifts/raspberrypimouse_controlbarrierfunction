#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *

N = 8
LOG_DIR = '/home/robolab/raspi_ws/src/coverage_control/Data'
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_goal = np.array([[-2, -2, 2, 2, 0.25, -0.25, 0.25, -0.25], [0.25, -0.25, 0.25, -0.25, -2, -2, 2, 2]])
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)
        count = 0
        norms = np.zeros((1,N))
        alpha = np.zeros((1,N))
        while not rospy.is_shutdown():
            pose = getposition(N)
            pose_si = uni_to_si_states(pose)
            x_traj = np.append(x_traj, pose[0:1], axis=0)
            y_traj = np.append(y_traj, pose[1:2], axis=0)
            if(np.linalg.norm(x_goal - pose_si) < 0.01):
                for i in range(len(x_goal)):
                    for j in range(len(x_goal[i])):
                        if abs(x_goal[i][j]) == 2:
                            x_goal[i][j] *= -1
                # for i in range(len(x_goal)): #row
                #     for j in range(len(x_goal[i]) - 1): #column
                #         t = x_goal[i][j]
                #         x_goal[i][j] = x_goal[i][j - 2]
                #         x_goal[i][j-2] = 

                count += 1
                if count == iterations:
                    np.savetxt(LOG_DIR+'/X_traj.csv', x_traj, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/Y_traj.csv', y_traj, delimiter=' , ')
                    rospy.signal_shutdown('End of testing')
                    pass


            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
