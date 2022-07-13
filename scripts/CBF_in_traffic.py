#! /usr/bin/env python
import rospy
import numpy as np
from raspi import *
from transform import *
from controller import *

N = 8
iterations = 1

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_goal = np.array([[-2, -2, 2, 2, 0.25, -0.25, 0.25, -0.25], [0.25, -0.25, 0.25, -0.25, -2, -2, 2, 2]])

        # For plotting Trajectories
        # LOG_DIR = '/home/robolab/raspi_ws/src/coverage_control/Data'
        # x_traj = np.empty((0, N), float)
        # y_traj = np.empty((0, N), float)
        # x_traj_md = np.empty((0, N), float)
        # y_traj_md = np.empty((0, N), float)
        # pose_md = getposition(N)
        # pose_md_si = uni_to_si_states(pose_md)

        count = 0
        norms = np.zeros((1,N))
        alpha = np.zeros((1,N))
        while not rospy.is_shutdown():
            pose = getposition(N)
            pose_si = uni_to_si_states(pose)

            # For plotting Trajectories
            # x_traj = np.append(x_traj, pose[0:1], axis=0)
            # y_traj = np.append(y_traj, pose[1:2], axis=0)

            # if(np.linalg.norm(x_goal - pose_md_si) > 0.02):
            #     x_traj_md = np.append(x_traj_md, pose_md_si[0:1], axis=0)
            #     y_traj_md = np.append(y_traj_md, pose_md_si[1:2], axis=0)
            # print ('X', x_traj_md)
            # print ('Y', y_traj_md)
            
            print ((np.linalg.norm(x_goal - pose_si)))
            if(np.linalg.norm(x_goal - pose_si) < 0.02 and np.linalg.norm(x_goal - pose_md_si) < 0.02):
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

                    # For plotting Trajectories
                    # np.savetxt(LOG_DIR+'/X_traj.csv', x_traj, delimiter=' , ')
                    # np.savetxt(LOG_DIR+'/Y_traj.csv', y_traj, delimiter=' , ')
                    # np.savetxt(LOG_DIR+'/X_traj_md.csv', x_traj_md, delimiter=' , ')
                    # np.savetxt(LOG_DIR+'/Y_traj_md.csv', y_traj_md, delimiter=' , ')
                    rospy.signal_shutdown('End of testing')
                    pass


            dxi = si_position_controller(pose_si, x_goal)

            #for plotting trajectories
            # dxi_md = si_position_controller_2(pose_md_si, x_goal)
            # pose_md_si = np.add(pose_md_si, dxi_md)

            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
