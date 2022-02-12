#! /usr/bin/env python

import numpy as np
from math import *
import matplotlib.pyplot as plt

import sys

DATA_PATH = '/home/robolab/raspi_ws/src/coverage_control/Data'
sys.path.insert(0, DATA_PATH)

def extract(log_file_dir):
    x_traj = np.genfromtxt(log_file_dir + '/X_traj.csv', delimiter=' , ')
    y_traj = np.genfromtxt(log_file_dir + '/Y_traj.csv', delimiter=' , ')
    theta_traj = np.genfromtxt(log_file_dir + '/THETA_traj.csv', delimiter=' , ')
    x_goal = np.genfromtxt(log_file_dir + '/X_goal.csv', delimiter=' , ')
    y_goal = np.genfromtxt(log_file_dir + '/Y_goal.csv', delimiter=' , ')

    file_params = open(log_file_dir+'/LogSimParams.txt', 'r')
    lines = file_params.readlines()
    file_params.close()

    k_rho = float(lines[1].split()[-1])
    k_alpha = float(lines[2].split()[-1])

    return (x_traj, y_traj, theta_traj, x_goal, y_goal, k_rho, k_alpha)

def plot_feedback_control(log_file_1, log_file_2, log_file_3):
    
    x_traj_1, y_traj_1, theta_traj_1, x_goal_1, y_goal_1, k_rho_1, k_alpha_1 = extract(log_file_1)
    x_traj_2, y_traj_2, theta_traj_2, x_goal_2, y_goal_2, k_rho_2, k_alpha_2 = extract(log_file_2)
    x_traj_3, y_traj_3, theta_traj_3, x_goal_3, y_goal_3, k_rho_3, k_alpha_3 = extract(log_file_3)

    plt.style.use('seaborn-ticks')

    plt.figure(1)
    plot_label = r'$k_\rho$ = %.2f $k_\alpha$ = %.2f' % (k_rho_1, k_alpha_1)
    plt.plot(x_traj_1, y_traj_1, label = plot_label)
    plot_label = r'$k_\rho$ = %.2f $k_\alpha$ = %.2f' % (k_rho_2, k_alpha_2)
    plt.plot(x_traj_2, y_traj_2, label = plot_label)
    plot_label = r'$k_\rho$ = %.2f $k_\alpha$ = %.2f ' % (k_rho_3, k_alpha_3)
    plt.plot(x_traj_3, y_traj_3, label = plot_label)
    plt.title('X-Y Plane trajectory')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.xlim(np.min(x_traj_1) - 0.05, np.max(x_traj_1) + 0.05)
    plt.ylim(np.min(y_traj_1) - 0.05, np.max(y_traj_1) + 0.05)
    plt.legend(loc = 4)
    plt.grid()

    plt.figure(2)
    plt.subplot(3,1,1)
    plt.plot(x_goal_1, color = 'red', label = 'reference')
    plt.plot(x_traj_1, color = 'blue', label ='actual')
    plt.legend(loc = 4)
    plt.title('X coordinate response')
    plt.xlabel('sample time')
    plt.ylabel('x [m]')
    plt.xlim(0, len(x_goal_1) + 3)
    plt.ylim(np.min(x_traj_1) - 0.1, np.max(x_traj_1) + 0.1)
    plt.grid()

    plt.figure(2)
    plt.subplot(3,1,2)
    plt.plot(y_goal_1, color = 'red', label = 'reference')
    plt.plot(y_traj_1, color = 'blue', label = 'actual')
    plt.legend(loc = 4)
    plt.title('Y coordinate response')
    plt.xlabel('sample time')
    plt.ylabel('y [m]')
    plt.xlim(0, len(y_goal_1) + 3)
    plt.ylim(np.min(y_traj_1) - 0.075, np.max(y_traj_1) + 0.075)
    plt.grid()


    plt.tight_layout()

    plt.show()

plot_feedback_control(DATA_PATH + '/Log_feedback_1', DATA_PATH + '/Log_feedback_2', DATA_PATH + '/Log_feedback_3')