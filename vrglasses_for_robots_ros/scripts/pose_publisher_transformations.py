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
#from transforms3d import quaternions
#from tf.transformations import *
from tqdm import tqdm
from csv import reader
import time

def read_transformations_file( transformations_filename ):
    points = []
    orientations = []

    with open(transformations_filename, "r") as filestream:
        for line in filestream:
            currentline = line.split(",")
            points.append((currentline[2], currentline[3], currentline[4]));
            orientations.append((currentline[6],currentline[7], currentline[8], currentline[9]));
    return points, orientations

def read_transformations_csv_file( transformations_filename ):
    points = []
    orientations = []

    with open(transformations_filename, 'r') as read_obj:
        # pass the file object to reader() to get the reader object
        csv_reader = reader(read_obj)
        # Iterate over each row in the csv using reader object
        for row in csv_reader:
            # row variable is a list that represents a row in csv
            #print(row)
            points.append((float(row[2]), float(row[3]), float(row[4])))#((0,0,-5))#
            orientations.append((float(row[5]),float(row[6]), float(row[7]), float(row[8])))#((0,1,0,0))#
    return points, orientations

# def single_sample(num, x=0, y=0, height=100, yaw=90):
#     points = []
#     yaws = []
#
#     for i in range(num):
#         points.append((x,y,height))
#         yaws.append(yaw)
#     return points, yaws
#
#
# def star_positions_old(center_x = 0.0, center_y = 0.0, min_radius = 10, max_radius = 120, height = 50.0):
#     points = []
#     yaw = []
#
#     radius_step = 10;
#
#     min_angle = 0;
#     max_angle = 360;
#     angle_step = 5;
#
#     for r in range(min_radius, max_radius, radius_step):
#         for a in range(min_angle, max_angle, angle_step):
#             angle_rad = np.deg2rad(a);
#             x = center_x + (r * np.cos(angle_rad))
#             y = center_y + (r * np.sin(angle_rad))
#             points.append((x,y,height))
#             yaw.append(a+180)#a+ 180
#
#     return points, yaw
#
# def star_positions(center_x = 0.0, center_y = 0.0, min_radius = 10, max_radius = 221, height = 50.0, radius_step=40):
#     points = []
#     yaw = []
#
#     radius_step = radius_step
#
#     min_angle = 0;
#     max_angle = 360;
#     angle_step = 5;
#     print('Height: {}'.format(height))
#     for a in range(min_angle, max_angle, angle_step):
#         angle_rad = np.deg2rad(a);
#         yaw_cam = - a
#         for r in range(min_radius, max_radius, radius_step):
#             x = center_x + (r * np.cos(angle_rad))
#             y = center_y + (r * np.sin(angle_rad))
#             #print("({},{},{})".format(x,y,yaw_cam))
#             points.append((x,y,height))
#             yaw.append(yaw_cam)
#
#     return points, yaw
#
# def compute_star_orientations(points, yaw_angles, pitch):
#     orientations = []
#     print('Pitch Angle: {}'.format(pitch))
#     roll = 0
#     nPoints = len(points)
#     for i in range(nPoints):
#         r_rad = np.deg2rad(roll);
#         p_rad = np.deg2rad(pitch); 			# descent angle
#         y_rad = np.deg2rad(yaw_angles[i]);  # direction of arrival (star line)
#         # print(yaw_angles[i])
#         quat = quaternion_from_euler(y_rad, p_rad, r_rad)
#         orientations.append((quat[0], quat[1], quat[2], quat[3])) # w, x, y, z
#
#     return orientations
#
# def pol2cart(rho, phi):
#     x = rho * np.cos(phi)
#     y = rho * np.sin(phi)
#     return np.vstack([x, y])
#
# def circle_gen(center, radius, height, samples):
# 	phis = np.linspace(0, 2 * np.pi, num=samples)
# 	rho = np.repeat(radius, samples)
# 	centers = np.repeat([[center[0]], [center[1]]], samples, axis=1)
# 	heights = np.repeat(height, samples)
# 	xys = centers + pol2cart(rho, phis)
# 	yaws = np.linspace(0, -360, num=samples, endpoint=False)
# 	return np.vstack([xys, heights]), yaws
#
# def circle_positions(center_x = 0.0, center_y = 0.0, radius = 10, height = 10, nSamples = 20):
# 	points = []
# 	yaw = []
#
# 	min_angle = 0;
# 	max_angle = 360;
# 	angle_step = (max_angle - min_angle) / nSamples;
#
# 	for a in range(min_angle, max_angle, angle_step):
# 		angle_rad = np.deg2rad(a);
# 		x = center_x + (radius * np.cos(angle_rad));
# 		y = center_y + (radius * np.sin(angle_rad));
# 		points.append((x,y,height));
# 		yaw.append(-a)
#
# 	return points, yaw
#
def publisher(points, orientations, rate=1):
    pub = rospy.Publisher('firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)  #pose_publisher/odometry
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(rate) # Hz

#     file_dir = '/home/fabiola/datasets/shiming/data_pose/random_poses.txt'
#     with open(file_dir, 'w') as f:
    print('Number of Points: {}'.format(len(points)))
    for i in tqdm(range(0,len(points))):
        p = Odometry()
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x = points[i][0]
        p.pose.pose.position.y = points[i][1]
        p.pose.pose.position.z = points[i][2]

        # Make sure the quaternion is valid and normalized
        p.pose.pose.orientation.x = orientations[i][1]
        p.pose.pose.orientation.y = orientations[i][2]
        p.pose.pose.orientation.z = orientations[i][3]
        p.pose.pose.orientation.w = orientations[i][0]
#             np.savetxt(f, [points[i][0], points[i][1], points[i][2], orientations[i][0],orientations[i][1],orientations[i][2],orientations[i][3]], fmt='%f', newline=' ', delimiter=',') # w, x, y, z
#             f.write("\n")
        pub.publish(p)
        if rospy.is_shutdown():
            break
        rate.sleep()
#
#     f.close()
#
# def plot_position(points_array,yaws_array,x_min=-95,x_max=175,y_min=-120,y_max=120, split_factor=0.8):
#     fig = plt.figure(figsize=(20,12))
#     ax = plt.subplot()
#     X_train = []
#     Y_train = []
#     angle_train = []
#     X_val = []
#     Y_val = []
#     angle_val = []
#
#     for index in range(int(split_factor*len(points))):
#         X_train.append(points[index][0])
#         Y_train.append(points[index][1])
#         angle_rad_train = np.deg2rad(180-yaws_array[index])
#         u = math.sin(angle_rad_train)
#         v = math.cos(angle_rad_train)
#         ax.quiver(points[index][0],points[index][1], v, u, color='b',width=0.001, scale = 80)
#     for index in range(int(split_factor*len(points)),len(points)):
#         X_val.append(points[index][0])
#         Y_val.append(points[index][1])
#         angle_rad_val = np.deg2rad(180-yaws_array[index])
#         u = math.sin(angle_rad_val)
#         v = math.cos(angle_rad_val)
#         ax.quiver(points[index][0],points[index][1], v, u, color='r',width=0.001, scale = 80)
#
#
#     ax.scatter(X_train,Y_train,alpha=1, color='blue',label='Training Set')
#     ax.scatter(X_val,Y_val,alpha=1, color='red',label='Validation Set')
#     ax.set_title('Sampling Position Visualization')
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.legend(loc=1)
#     fig.savefig('/home/fabiola/datasets/shiming/pose_publisher_plots/sampling_position_1.png')
#     #fig.savefig('/media/shiming/Ming_Passport/MA_ws/pose_publisher_plots/sampling_position_1.png')
#     print("Plot is saved")

if __name__ == '__main__':
	# sampling_rate = 4
    # # # if sys.argv[0] >= 3:
    # # #	sampling_rate = int(sys.argv[3])
    # #
    # # # height = int(sys.argv[1])
    # # # pitch = int(sys.argv[2])
    # #
	# points = []
	# orientations = []
    #
	# center_x = 0
	# center_y = 0
	# pitch = 60
    # #
    # # radius = 130
    # # height = 90
	# nSamplesPerCircle = 60
    # #
	# #min_height  = 50
	# #max_height  = 90
	# height_step = 10
	# #current_height =
    #
	# max_radius = 220
	#
    #
	# max_height = max_radius * (1 / np.tan(np.deg2rad(pitch)))
    #
	# num_fixed = 2
	# points_fixed, yaw_angles_fixed = single_sample(num=num_fixed)
	# orientations_fixed = compute_star_orientations(points=points_fixed, yaw_angles=yaw_angles_fixed, pitch=-90)
	# points.extend(points_fixed)
	# orientations.extend(orientations_fixed)
    #
	# medium_height = 80#(int)(math.floor(max_height / 2))
	# min_height_range = medium_height - 10
	# max_height_range = medium_height + 10
	# print('min_height_range --> ', min_height_range, 'max_height_range', max_height_range )
	# for current_height in range(min_height_range, max_height_range + 1, height_step):
	# 	current_radius = np.tan(np.deg2rad(pitch)) * (max_height - current_height)
	# 	print('current_height --> ', current_height, 'current_radius', current_radius)
	# 	points_circle, yaws = circle_positions(center_x, center_y, current_radius, current_height, nSamplesPerCircle)
	# 	orientations_circle = compute_star_orientations(points_circle, yaws, -pitch)
	# 	points.extend(points_circle)
	# 	orientations.extend(orientations_circle)
    #
	# try:
	# 	publisher(points, orientations, sampling_rate)
	# except rospy.ROSInterruptException:
	# 	pass
    sampling_rate = 1

    points, orientations = read_transformations_csv_file("/home/fabiola/code/landing_localization/result.txt")

    try:
        publisher(points, orientations, sampling_rate)
    except rospy.ROSInterruptException:
        pass

    print("Done!")
