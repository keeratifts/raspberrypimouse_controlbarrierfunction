#!/usr/bin/env python
import rospy
import numpy as np
from controller import *
from transform import *
from raspi import *
from random import uniform

#This script used for experimenting with real robots.

N = 2 #no. of robots

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(50)
        x_goal = np.array([[uniform(-0.8,0.8), uniform(-0.8,0.8)], [uniform(-0.8,0.8), uniform(-0.8,0.8)]]) #(x, y)
        while not rospy.is_shutdown():
            pose = get_robot_position(N)
            dxu = robotFeedbackControl(pose, x_goal)
            pose_si = uni_to_si_states(pose)

            if(np.linalg.norm(pose[0:2, 0:N]-x_goal)< 0.09):
                rate.sleep()
                for i in range(len(x_goal)):
                    for j in range(len(x_goal[i])):
                        x_goal = np.array([[uniform(-0.8,0.8), uniform(-0.8,0.8)], [uniform(-0.8,0.8), uniform(-0.8,0.8)]])
                        
            dxi = uni_to_si_dyn(dxu, pose)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass


