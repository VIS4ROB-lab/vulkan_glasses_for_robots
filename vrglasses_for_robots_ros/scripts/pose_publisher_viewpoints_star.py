#!/usr/bin/env python

from Tkinter import Label
from tokenize import PlainToken
from matplotlib.pyplot import subplot
from numpy.core.defchararray import split
from numpy.lib.function_base import angle
import rospy
import math
import numpy as np
import os
import sys
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from transforms3d import quaternions
from tf.transformations import *
from tqdm import tqdm
import argparse
import yaml

def single_sample(num, x=0, y=0, height=100, yaw=90):
    points = []
    yaws = []

    for i in range(num):
        points.append((x,y,height))
        yaws.append(yaw)
    return points, yaws


def star_positions_old(center_x = 0.0, center_y = 0.0, min_radius = 10, max_radius = 120, height = 50.0):
    points = []
    yaw = []

    radius_step = 10;

    min_angle = 0;
    max_angle = 361;
    angle_step = 15;

    for r in range(min_radius, max_radius, radius_step):
        for a in range(min_angle, max_angle, angle_step):
            angle_rad = np.deg2rad(a);
            x = center_x + (r * np.cos(angle_rad))  
            y = center_y + (r * np.sin(angle_rad))
            points.append((x,y,height))
            yaw.append(a+180)#a+ 180

    return points, yaw

def star_positions(center_x = 0.0, center_y = 0.0, min_radius = 10, max_radius = 221, height = 50.0):
    points = []
    yaw = []

    radius_step = 10;

    min_angle = 0;
    max_angle = 360;
    angle_step = 15;
    print('Height: {}'.format(height))
    for a in range(min_angle, max_angle, angle_step):
        angle_rad = np.deg2rad(a);
        yaw_cam = - a  
        for r in range(min_radius, max_radius, radius_step):            
            x = center_x + (r * np.cos(angle_rad))  
            y = center_y + (r * np.sin(angle_rad))
            #print("({},{},{})".format(x,y,yaw_cam))
            points.append((x,y,height))
            yaw.append(yaw_cam)

    return points, yaw

    
def compute_star_orientations(points, yaw_angles, pitch):
    orientations = []
    print('Pitch Angle: {}'.format(pitch))
    roll = 0
    nPoints = len(points)
    for i in range(nPoints):
        r_rad = np.deg2rad(roll);
        p_rad = np.deg2rad(pitch); 			# descent angle
        y_rad = np.deg2rad(yaw_angles[i]);  # direction of arrival (star line)
        # print(yaw_angles[i])
        quat = quaternion_from_euler(y_rad, p_rad, r_rad)
        orientations.append((quat[0], quat[1], quat[2], quat[3])) # w, x, y, z

    return orientations

def publisher(points, orientations,rate=1):
    pub = rospy.Publisher('firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)  #pose_publisher/odometry
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(rate) # Hz
    
    #file_dir = '/home/shiming/2.MA_ws/99.data_pose/random_poses.txt'
    file_dir = '/home/fabiola/datasets/shiming/data_pose/random_poses.txt'
    with open(file_dir, 'w') as f:
        print('Number of Points: {}'.format(len(points)))
        for i in tqdm(range(1,len(points))):
            p = Odometry()
            p.header.stamp = rospy.Time.now()
            p.pose.pose.position.x = points[i][0]
            p.pose.pose.position.y = points[i][1]
            p.pose.pose.position.z = 10+points[i][2]
		    # Make sure the quaternion is valid and normalized
            p.pose.pose.orientation.x = orientations[i][1]
            p.pose.pose.orientation.y = orientations[i][2]
            p.pose.pose.orientation.z = orientations[i][3]
            p.pose.pose.orientation.w = orientations[i][0]
            np.savetxt(f, [points[i][0], points[i][1], points[i][2], orientations[i][0],orientations[i][1],orientations[i][2],orientations[i][3]], fmt='%f', newline=' ', delimiter=',') # w, x, y, z
            f.write("\n")
            pub.publish(p)
            if rospy.is_shutdown():
                break
            rate.sleep()

    f.close()

def plot_position(points_array,yaws_array,x_min=-95,x_max=175,y_min=-120,y_max=120, split_factor=0.8):
    fig = plt.figure(figsize=(20,12))
    ax = plt.subplot()
    X_train = []
    Y_train = []
    angle_train = []
    X_val = []
    Y_val = []
    angle_val = []

    for index in range(int(split_factor*len(points))):
        X_train.append(points[index][0])
        Y_train.append(points[index][1])
        angle_rad_train = np.deg2rad(180-yaws_array[index])
        u = math.sin(angle_rad_train)
        v = math.cos(angle_rad_train)
        ax.quiver(points[index][0],points[index][1], v, u, color='b',width=0.001, scale = 80)
    for index in range(int(split_factor*len(points)),len(points)):
        X_val.append(points[index][0])
        Y_val.append(points[index][1])
        angle_rad_val = np.deg2rad(180-yaws_array[index])
        u = math.sin(angle_rad_val)
        v = math.cos(angle_rad_val)
        ax.quiver(points[index][0],points[index][1], v, u, color='r',width=0.001, scale = 80)


    ax.scatter(X_train,Y_train,alpha=1, color='blue',label='Training Set')
    ax.scatter(X_val,Y_val,alpha=1, color='red',label='Validation Set')
    ax.set_title('Sampling Position Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc=1)
    fig.savefig('/home/fabiola/datasets/shiming/pose_publisher_plots/sampling_position_1.png')
    #fig.savefig('/media/shiming/Ming_Passport/MA_ws/pose_publisher_plots/sampling_position_1.png')
    print("Plot is saved")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str)
    parser.add_argument('--height', type=int)
    parser.add_argument('--pitch', type=int)
    parser.add_argument('--rate', type=int, default=2)
    args = parser.parse_args()

    sampling_rate = 2

    config_path = args.config
    with open(config_path, 'r') as f:
        config = yaml.load(f)



    sampling_rate = args.rate

    height = args.height
    pitch = args.pitch
    min_radius = config['star_sampling']['min_radius']
    max_radius = config['star_sampling']['max_radius']
    points, yaw_angles = star_positions(min_radius= min_radius , max_radius = max_radius, height = height)
    orientations = compute_star_orientations(points, yaw_angles, pitch)
    try:
        publisher(points, orientations, sampling_rate)
    except rospy.ROSInterruptException:
        pass

    print("Sampling done!")
