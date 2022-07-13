#!/usr/bin/env python
import rospy
import numpy as np
from controller import *
from transform import *
from raspi import *
from random import uniform

#This script used for experimenting with real robots.

N = 4 #no. of robots
LOG_DIR = '/home/robolab/raspi_ws/src/coverage_control/Data/real_ex'

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(100)
        x_goal = np.array([[-0.7, 0.7, 0, 0], [0, 0, -0.7, 0.7]])
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)
        iter = 0
        # x_goal = np.array([[uniform(-0.7,0.7), uniform(-0.7,0.7)], [uniform(-0.7,0.7), uniform(-0.7,0.7)]]) #(x, y)
        while not rospy.is_shutdown():
            pose = get_robot_position(N)
            dxu = robotFeedbackControl(pose, x_goal)
            pose_si = uni_to_si_states(pose)

            x_traj = np.append(x_traj, pose[0:1], axis=0)
            y_traj = np.append(y_traj, pose[1:2], axis=0)
            
            print (np.linalg.norm(pose[0:2, 0:N]-x_goal))
            if(np.linalg.norm(pose[0:2, 0:N]-x_goal)< 0.2):
                iter += 1
                rate.sleep()
                # for i in range(len(x_goal)):
                #     for j in range(len(x_goal[i])):
                #         x_goal = np.array([[uniform(-0.7,0.7), uniform(-0.7,0.7)], [uniform(-0.7,0.7), uniform(-0.7,0.7)]])
                for i in range(len(x_goal)):
                    for j in range(len(x_goal[i])):
                        if abs(x_goal[i][j]) == 0.7:
                            x_goal[i][j] *= -1
                if iter % 3 == 0:
                    np.savetxt(LOG_DIR+'/X_traj.csv', x_traj, delimiter=' , ')
                    np.savetxt(LOG_DIR+'/Y_traj.csv', y_traj, delimiter=' , ')
                    rospy.signal_shutdown('End of testing')
                    pass

            # dxi = si_position_controller(pose_si, x_goal)
            dxi = uni_to_si_dyn(dxu, pose)
            try:
                dxi = si_barrier_cert(dxi, pose_si)
            except:
                print ('Error')
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass


