#! /usr/bin/env python
from shapely.ops import orient
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from math import *

N = 20

def getposition(N):
    odom = {}
    position = []
    orient = []
    for i in range(0, N):
        odom[i] = rospy.wait_for_message('/raspi_'+str(i)+'/odom', Odometry)
        x = odom[i].pose.pose.position.x
        y = odom[i].pose.pose.position.y
        orientation_q = odom[i].pose.pose.orientation
        orientation_list =  [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        position.append([x, y])
        orient.append([yaw])
    pose = np.array([x for x in position])
    angle = np.array([y for y in orient])
    return pose, angle

def reshape_coords(pose):
    coords = []
    for n in pose:
        m = Point(n)
        coords.append(n)
    return coords

def match_pair(poly_shapes, new_coords, centroids):
    new_centroids = []
    for n in centroids:
        m = Point(n)
        new_centroids.append(n)
    sorted_centroids = []
    points = coords_to_points(new_coords)
    for i, p in enumerate(points):
        for j, poly in enumerate(poly_shapes):
            if p.within(poly):
                pair = new_centroids[j]
                sorted_centroids.append(pair)
    sorted_cen = np.column_stack((sorted_centroids))
    return sorted_centroids

def _plotting(fig, ax, iter, save_fig=False):
    major_locator=MultipleLocator(100)
    ax.set_xlim([-13,13])
    ax.set_ylim([-13,13])
    font = {'size':13}
    ax.set_xticklabels(["-2", "-1", "0", "1", "2"])
    ax.set_yticklabels(["-2", "", "-1", "", "0", "", "1", "", "2"])
    # ax.xaxis.set_major_locator(major_locator)
    # ax.yaxis.set_major_locator(major_locator)
    ax.set_xlabel('x(m)', font, labelpad=15)
    ax.set_ylabel('y(m)', font, labelpad=15)    
    if save_fig:
        plt.savefig('/home/robolab/Coverage control/Trajectory_Log/Figure/FIG_'+str(iter)+'.png')
    #plt.title(str(j)  +"th itr")
    plt.tick_params(labelsize=13) #设置刻度字体大小
    plt.pause(0.001)
    ax.clear()

def gen_voronoi(pose):
    outer = [(0, 10), (10, -10), (-10, -10)]
    area_shape = Polygon(outer)
    poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(pose, area_shape, accept_n_coord_duplicates=0)
    new_coords = reshape_coords(pose)
    poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes])
    new_centroids = match_pair(poly_shapes, new_coords, poly_centroids)
    return area_shape, poly_shapes, poly_to_pt_assignments, new_centroids

def create_vel_msg(v, w):
    velMsg = Twist()
    velMsg.linear.x = v
    velMsg.linear.y = 0
    velMsg.linear.z = 0
    velMsg.angular.x = 0
    velMsg.angular.y = 0
    velMsg.angular.z = w
    return velMsg

def feedback_control(pose, orient, goal, distance=0.2, k_ro=3, k_alp=5.0, k_beta=-8.0, v_const=0.2, w_const=0.2):
    if len(pose) == len(orient) and len(orient) == len(goal):
        for i in range(0, len(pose)):
            velPub = rospy.Publisher('raspi_'+str(i)+'/cmd_vel', Twist, queue_size=10)
            ro = np.linalg.norm(goal[i]-pose[i], axis=0)
            lamda = atan2(goal[i][1] - pose[i][1] , goal[i][0] - pose[i][0])
            alpha = (lamda - orient[i] + pi) % (2 * pi) - pi #-360degrees
            if ro <= distance:
                status = True
                v = 0
                w = 0
                v_scal = 0
                w_scal = 0
            else:
                status = False
                v = k_ro * ro
                w = k_alp * alpha
                if v >= v_const:
                    v_scal = v / abs(v) * v_const
                else:
                    v_scal = k_ro * ro
                w_scal = w / abs(v) * w_const

            velMsg = create_vel_msg(v_scal, w_scal)
            velPub.publish(velMsg)
    return status

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        fig = plt.figure(figsize=(5.7,5))
        fig.subplots_adjust(wspace=0, hspace=0, top=0.95, bottom=0.15)
        ax = plt.subplot()
        plt.axis('scaled')
        a = 0

        while not rospy.is_shutdown():
            _plotting(fig, ax, 50)
            (pose, yaw) = getposition(N)
            (area, shape, poly2pt, centroids) = gen_voronoi(pose)
            plot_voronoi_polys_with_points_in_area(ax, area, shape, pose, poly2pt,voronoi_edgecolor='black', points_color='black', 
                                        points_markersize=30, voronoi_and_points_cmap=None)
            for x in centroids:
                c1 = x
                ax.plot(c1[0],c1[1], 'rs', markersize = 12, alpha = 0.4)
            feedback_control(pose, yaw, centroids)
    except rospy.ROSInterruptException:
        pass
