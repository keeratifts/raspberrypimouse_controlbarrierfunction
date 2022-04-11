#! /usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
from voronoi_gen import *
from raspi import *
from transform import *
from controller import *

N = 20
LOG_DIR = '/home/robolab/raspi_ws/src/coverage_control/Data'

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        x_traj = np.empty((0, N), float)
        y_traj = np.empty((0, N), float)

        fig = plt.figure(figsize=(5.7,5))
        fig.subplots_adjust(wspace=0, hspace=0, top=0.95, bottom=0.15)
        ax = plt.subplot()
        plt.axis('scaled')

        while not rospy.is_shutdown():
            plotting(fig, ax, 50)
            pose = getposition(N)
            pose_si = uni_to_si_states(pose)
            x_traj = np.append(x_traj, pose[0:1], axis=0)
            y_traj = np.append(y_traj, pose[1:2], axis=0)

            (area, shape, poly2pt, centroids) = gen_voronoi(pose)
            plot_voronoi_polys_with_points_in_area(ax, area, shape, np.column_stack((pose[0:2])), poly2pt,voronoi_edgecolor='black', points_color='black', 
                                        points_markersize=30, voronoi_and_points_cmap=None)
            for x in np.column_stack((centroids)):
                c1 = x
                ax.plot(c1[0],c1[1], 'rs', markersize = 12, alpha = 0.4)

            if(np.linalg.norm(centroids - pose_si) < 0.01):
                np.savetxt(LOG_DIR+'/X_traj.csv', x_traj, delimiter=' , ')
                np.savetxt(LOG_DIR+'/Y_traj.csv', y_traj, delimiter=' , ')
                rospy.signal_shutdown('End of testing')
                pass

            dxi = si_position_controller(pose_si, centroids)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose)
            k = set_velocities(N, dxu)
            put_velocities(N, k)

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
