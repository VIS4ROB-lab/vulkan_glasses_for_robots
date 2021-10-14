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

def fibonacci_sphere(num, radius_sphere):
    # num: number of point you want to collect

    points = []
    phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians
    for i in range(num):
        z = 1 - (i / float(num - 1))  # z goes from 1 to 0
        z = radius_sphere * z
        radius_circle = math.sqrt(radius_sphere*radius_sphere - z * z)  # radius at z
        #print radius_circle

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius_circle
        y = math.sin(theta) * radius_circle

        points.append((x, y, z))
    return points

def compute_fibonacci_orientations(points):
    orientations = []

    pt = np.array([0,0,0])
    for pf in points:
        vft = pt - pf
        vft = vft / np.linalg.norm(vft)
        tmp = np.array([0,0,1])
        vright = np.cross(tmp, vft)
        if np.linalg.norm(vright) == 0:
            vright = np.array([0,1,0])
        else:
            vright = vright / np.linalg.norm(vright)
        vup = np.cross(vft, vright)
        vup = vup / np.linalg.norm(vup)

        transf_matrix = np.zeros((3, 3))
        transf_matrix[:, 0] = vft
        transf_matrix[:, 1] = vright
        transf_matrix[:, 2] = vup

        quat = quaternions.mat2quat(transf_matrix)
        orientations.append((quat[0], quat[1], quat[2], quat[3])) # w, x, y, z
       
    return orientations

    
def randomly_sample(number, height_min = 80, height_max = 150, x_min=-200, x_max = 220, y_min = -230, y_max =200, seed = 520102):
    points = []
    yaws = []
    pitches = []
    np.random.seed()
    for i in range(number):
        x = np.random.randint(x_min, x_max)
        y = np.random.randint(y_min, y_max)
        height = np.random.randint(1, height_max+1)     

        points.append((x,y,height))

        if x >= 0 and y >= 0:
            angle_min = -90
            angle_max = 0
        elif x <= 0 and y >= 0:
            angle_min = -180
            angle_max = -90
        elif x<=0 and y <=0:
            angle_min = 90
            angle_max = 180
        elif x>=0 and y <=0:
            angle_min = 0
            angle_max = 90
        else:
            angle_min = 0
            angle_max = 360                      
        yaw = np.random.randint(angle_min, angle_max)
        yaws.append(yaw)
        pitch = np.random.randint(-90, -14)
        pitches.append(pitch)
    return points, yaws, pitches

def compute_random_orientations(points, yaw_angles, pitch):
    orientations = []
    roll = 0
    nPoints = len(points)
    for i in range(nPoints):
        r_rad = np.deg2rad(roll);
        if isinstance(pitch, list):
            p_rad = np.deg2rad(pitch[i]); 			# descent angle
        else:
            p_rad = np.deg2rad(pitch);       
        y_rad = np.deg2rad(yaw_angles[i]);  # direction of arrival (star line)
        # print(yaw_angles[i])
        quat = quaternion_from_euler(y_rad, p_rad, r_rad)
        orientations.append((quat[0], quat[1], quat[2], quat[3])) # w, x, y, z

    return orientations

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
    print("Plot is saved")


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

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str)
    parser.add_argument('--rate', type=int, default=2)
    args = parser.parse_args()

    config_file = args.config
    with open(config_file, 'r') as f:
        config = yaml.load(f)


    sampling_rate = int(args.rate)
    points= []
    orientations=[]

    print('Model: {}'.format(config['name']))

    h_min = config['geometry_dimensions']['h_min']
    h_max = config['geometry_dimensions']['h_max']
    x_min = config['geometry_dimensions']['x_min']
    x_max = config['geometry_dimensions']['x_max']
    y_min = config['geometry_dimensions']['y_min']
    y_max = config['geometry_dimensions']['y_max']
    radius_list =  config['fibonacci_radius_list']

    num_fixed = 2
    points_fixed, yaw_angles_fixed = single_sample(num=num_fixed)
    orientations_fixed = compute_random_orientations(points=points_fixed, yaw_angles=yaw_angles_fixed, pitch=-90)

    points.extend(points_fixed)
    orientations.extend(orientations_fixed)

    print('Samping on fixed position: {}'.format(num_fixed))
    num_fibo = 0
    # Fibonacci Sphere sampling: total: 4*200= 800
    dense_factor = config['dense_factor']
    print('Dense factor: {}'.format(dense_factor))
    for radius in radius_list:
        num_fibo_radius= (radius**2)/dense_factor
        print('++ Sampling on radius {}: {}'.format(radius, num_fibo_radius))
        #points_fibo = fibonacci_sphere(num_fibo_radius,radius)
        points_fibo = fibonacci_sphere(1000,radius)
        orientations_fibo = compute_fibonacci_orientations(points_fibo)
    
        points.extend(points_fibo)
        orientations.extend(orientations_fibo)
        num_fibo += num_fibo_radius
    print('Sampling total on Fibonacci Sphere: {}'.format(num_fibo))

    #num_rand = num_fibo
    #points_random, yaw_angles, pitches = randomly_sample(num_rand, height_min= h_min, height_max= h_max, x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max, seed=520102)
    #orientations_random = compute_random_orientations(points_random, yaw_angles, pitches)

    #points.extend(points_random)
    #orientations.extend(orientations_random)

    #print('Sampling randomly: {}'.format(num_rand))
    try:
        publisher(points, orientations, sampling_rate)
    except rospy.ROSInterruptException:
        pass

    print("Sampling done!")
