#!/usr/bin/env python

# import rospy
#
# from nav_msgs.msg import Path
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
#
# class Parameter:
#     def __init__(self):
#         self.f = 0
#         self.cx = 0
#         self.cy = 0
#         self.w = 0
#         self.h = 0
#         self.base = ''
#         self.start = 0
#         self.end = 0
#         self.reductor = 4
#         self.output_folder = ''
#         self.proj_name = ''
#
# path = Path()
#
# def load_traj():
#     global path
#     path.header.stamp = rospy.Time.now()
#     path.header.frame_id = 'world'
#
#
#
# def build_path():
#     global path
#     path.header.stamp = rospy.Time.now()
#     path.header.frame_id = 'world'
#     pose = PoseStamped()
#     pose.header.stamp = rospy.Time.now()
#     pose.header.frame_id = 'world'
#
#     pose.pose.position.x = -57.825359000009485
#     pose.pose.position.y = -23.033219000324607
#     pose.pose.position.z = -25.206523000000004
#     pose.pose.orientation.w = 0.738613
#     pose.pose.orientation.x = 0.00291168
#     pose.pose.orientation.y = -0.658953
#     pose.pose.orientation.z = 0.142212
#     path.poses.append(pose)
#
#
# def publisher():
#     global path
#     pub = rospy.Publisher('/firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)
#     path_pub = rospy.Publisher('/path', Path, queue_size=10)
#
#     rospy.init_node('pose_publisher', anonymous=True)
#     build_path()
#     rate = rospy.Rate(1) # Hz
#     while not rospy.is_shutdown():
#         p = Odometry()
#         p.header.stamp = rospy.Time.now()
#         p.pose.pose.position.x = -57.825359000009485
#         p.pose.pose.position.y = -23.033219000324607
#         p.pose.pose.position.z = -25.206523000000004
#         # Make sure the quaternion is valid and normalized
#
#         p.pose.pose.orientation.w = 0.738613
#         p.pose.pose.orientation.x = 0.00291168
#         p.pose.pose.orientation.y = -0.658953
#         p.pose.pose.orientation.z = 0.142212
#
#
# #        pub.publish(p)
#         path_pub.publish(path)
#         rate.sleep()
#
# #
# # if __name__ == '__main__':
# #     try:
# #
# #         publisher()
# #     except rospy.ROSInterruptException:
# #         pass
# #
#
# if __name__ == '__main__':
#
#
#     params = Parameter()
#     params.w = 4000
#     params.h = 3000
#     params.f = 3040.61969930976147225010
#     params.cx = 1939.82730497851412110322
#     params.cy = 1512.67209180559916603670
#     params.reductor = 1.0
#     params.output_folder = '/home/lucas/temp/pix4d_temp_test'
#     params.input_folder = '/media/secssd/dataset/amazon_models/arche2021'
#     params.proj_name = 'arche21_warehouse_only_circles'
#
#     print('\n\n asd \n\n')
#
#     proj = Pix4DProj(params)
#     proj.process()
#
#     try:
#
#         publisher()
#     except rospy.ROSInterruptException:
#         pass
#




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


def single_sample(num, x=0, y=0, height=100, yaw=90):
    points = []
    yaws = []

    for i in range(num):
        points.append((x, y, height))
        yaws.append(yaw)
    return points, yaws


def star_positions_old(center_x=0.0, center_y=0.0, min_radius=10, max_radius=120, height=50.0):
    points = []
    yaw = []

    radius_step = 10;

    min_angle = 0;
    max_angle = 360;
    angle_step = 5;

    for r in range(min_radius, max_radius, radius_step):
        for a in range(min_angle, max_angle, angle_step):
            angle_rad = np.deg2rad(a);
            x = center_x + (r * np.cos(angle_rad))
            y = center_y + (r * np.sin(angle_rad))
            points.append((x, y, height))
            yaw.append(a + 180)  # a+ 180

    return points, yaw


def star_positions(center_x=0.0, center_y=0.0, min_radius=10, max_radius=221, height=50.0, radius_step=40):
    points = []
    yaw = []

    radius_step = radius_step

    min_angle = 0;
    max_angle = 360;
    angle_step = 5;
    print('Height: {}'.format(height))
    for a in range(min_angle, max_angle, angle_step):
        angle_rad = np.deg2rad(a);
        yaw_cam = - a
        for r in range(min_radius, max_radius, radius_step):
            x = center_x + (r * np.cos(angle_rad))
            y = center_y + (r * np.sin(angle_rad))
            # print("({},{},{})".format(x,y,yaw_cam))
            points.append((x, y, height))
            yaw.append(yaw_cam)

    return points, yaw


def compute_star_orientations(points, yaw_angles, pitch):
    orientations = []
    print('Pitch Angle: {}'.format(pitch))
    roll = 0
    nPoints = len(points)
    for i in range(nPoints):
        r_rad = np.deg2rad(roll);
        p_rad = np.deg2rad(pitch);  # descent angle
        y_rad = np.deg2rad(yaw_angles[i]);  # direction of arrival (star line)
        # print(yaw_angles[i])
        quat = quaternion_from_euler(y_rad, p_rad, r_rad)
        orientations.append((quat[0], quat[1], quat[2], quat[3]))  # w, x, y, z

    return orientations


def publisher(points, orientations, rate=1):
    pub = rospy.Publisher('firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)  # pose_publisher/odometry
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(rate)  # Hz

    file_dir = '/home/lucas/Downloads/random_poses.txt'
    with open(file_dir, 'w') as f:
        print('Number of Points: {}'.format(len(points)))
        for i in tqdm(range(1, len(points))):
            p = Odometry()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = 'world'
            p.pose.pose.position.x = points[i][0]
            p.pose.pose.position.y = points[i][1]
            p.pose.pose.position.z = 10 + points[i][2]
            # Make sure the quaternion is valid and normalized
            p.pose.pose.orientation.x = orientations[i][1]
            p.pose.pose.orientation.y = orientations[i][2]
            p.pose.pose.orientation.z = orientations[i][3]
            p.pose.pose.orientation.w = orientations[i][0]
            np.savetxt(f, [points[i][0], points[i][1], points[i][2], orientations[i][0], orientations[i][1],
                           orientations[i][2], orientations[i][3]], fmt='%f', newline=' ', delimiter=',')  # w, x, y, z
            f.write("\n")
            pub.publish(p)
            if rospy.is_shutdown():
                break
            rate.sleep()

    f.close()


def plot_position(points_array, yaws_array, x_min=-95, x_max=175, y_min=-120, y_max=120, split_factor=0.8):
    fig = plt.figure(figsize=(20, 12))
    ax = plt.subplot()
    X_train = []
    Y_train = []
    angle_train = []
    X_val = []
    Y_val = []
    angle_val = []

    for index in range(int(split_factor * len(points))):
        X_train.append(points[index][0])
        Y_train.append(points[index][1])
        angle_rad_train = np.deg2rad(180 - yaws_array[index])
        u = math.sin(angle_rad_train)
        v = math.cos(angle_rad_train)
        ax.quiver(points[index][0], points[index][1], v, u, color='b', width=0.001, scale=80)
    for index in range(int(split_factor * len(points)), len(points)):
        X_val.append(points[index][0])
        Y_val.append(points[index][1])
        angle_rad_val = np.deg2rad(180 - yaws_array[index])
        u = math.sin(angle_rad_val)
        v = math.cos(angle_rad_val)
        ax.quiver(points[index][0], points[index][1], v, u, color='r', width=0.001, scale=80)

    ax.scatter(X_train, Y_train, alpha=1, color='blue', label='Training Set')
    ax.scatter(X_val, Y_val, alpha=1, color='red', label='Validation Set')
    ax.set_title('Sampling Position Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend(loc=1)
    fig.savefig('/media/shiming/Ming_Passport/MA_ws/pose_publisher_plots/sampling_position_1.png')
    print("Plot is saved")


if __name__ == '__main__':
    print("circ!")
    sampling_rate = 2
    if sys.argv[0] >= 3:
        sampling_rate = 2 #int(sys.argv[3])

    height = 20 # int(sys.argv[1])
    pitch = 0 #int(sys.argv[2])

    points = []
    orientations = []

    num_fixed = 10
    points_fixed, yaw_angles_fixed = single_sample(num=num_fixed)
    orientations_fixed = compute_star_orientations(points=points_fixed, yaw_angles=yaw_angles_fixed, pitch=-90)

    points.extend(points_fixed)
    orientations.extend(orientations_fixed)

    print('Samping on fixed pisition: {}'.format(num_fixed))

    points_star, yaw_angles_star = star_positions(min_radius=10, max_radius=221, height=height, radius_step=70)
    orientations_star = compute_star_orientations(points_star, yaw_angles_star, pitch)
    num_star = len(points_star)

    print('Sampling total on Star: {}'.format(num_star))

    points.extend(points_star)
    orientations.extend(orientations_star)
    try:
        publisher(points, orientations, sampling_rate)
    except rospy.ROSInterruptException:
        pass

    print("Sampling done!")