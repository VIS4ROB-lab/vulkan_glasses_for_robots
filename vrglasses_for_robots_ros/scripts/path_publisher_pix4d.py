#!/usr/bin/env python

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
from os import path
import csv
from math import cos, sin
import math
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


file_extern_cam = '/media/secssd/dataset/amazon_models/irchel140821/pix4d_params/irchel140821-merged_calibrated_external_camera_parameters.txt'
folder_undistorted = '/media/secssd/dataset/amazon_models/irchel140821/undistorted_images'
file_intern_cam = ''



def angle_to_rotmat( omega, phi, kappa):

    C_B_b = np.array([[1, 0, 0],
                   [0, -1, 0],
                   [0, 0, -1]])
    sina = math.sin(omega)
    cosa = math.cos(omega)
    Rx = np.array([[1, 0, 0],
                   [0, cosa, -sina],
                   [0, sina, cosa]])

    sina = math.sin(phi)
    cosa = math.cos(phi)
    Ry = np.array([[cosa, 0, sina],
                   [0, 1, 0],
                   [-sina, 0, cosa]])

    sina = math.sin(kappa)
    cosa = math.cos(kappa)
    Rz = np.array([[cosa, -sina, 0],
                   [sina, cosa, 0],
                   [0, 0, 1]])

    C_B_E = np.dot(C_B_b , np.dot(Rx , np.dot(Ry,Rz)))

    return C_B_E

def angle_to_rotmat1( omega, phi, kappa):
    C_B_b = np.array([[1, 0, 0],
                      [0, -1, 0],
                      [0, 0, -1]])

    cw = cos(omega)
    cp = cos(phi)
    ck = cos(kappa)
    sw = sin(omega)
    sp = sin(phi)
    sk = sin(kappa)

    R =  np.dot(C_B_b ,np.array ([[cp * ck, cw * sk + sw * sp * ck, sw * sk - cw * sp * ck],
                   [-cp * sk, cw * ck - sw * sp * sk, sw * ck + cw * sp * sk],
                   [ sp, -sw * cp, cw * cp]]))

    return R

def load_pix4d():
    #offsetx, offsety, offsetz = 400302.000, 5232070.000, 466.000
    offsetx, offsety, offsetz = 2683905.000, 1249997.000, 457.000
    points = []
    orientations = []
    filenames = []
    with open(file_extern_cam, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=' ')
        for row in csv_reader:
            # cam = CameraPose(row)
            # self.cameras[cam.img_name] = cam
            filenames.append(row['imageName'])
            points.append((float(row['X'])-offsetx, float(row['Y'])-offsety, float(row['Z'])-offsetz))
            rotmat = angle_to_rotmat1(math.radians(float(row['Omega'])),
                                      math.radians(float(row['Phi'])),
                                      math.radians(float(row['Kappa'])))
            quat = quaternions.mat2quat(np.transpose(rotmat))
            orientations.append((quat[0], quat[1], quat[2], quat[3]))  # w, x, y, z
            # orientations.append((cam.x, cam.y, cam.z))

    return points, orientations, filenames

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


def publisher( points, orientations,filenames, rate=100):
    pub = rospy.Publisher('firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)  # pose_publisher/odometry
    image_pub = rospy.Publisher("/firefly/real_image", Image, queue_size=10)
    bridge = CvBridge()
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(rate)  # Hz

    original_w = 4000
    original_h = 3000
    end_w = 752
    end_h = 480

    scaling_factor = float(end_w) / original_w
    temp_h = original_h * scaling_factor
    half_offset = math.floor((temp_h - end_h) / 2)
    print(scaling_factor, half_offset)



    file_dir = '/home/lucas/Downloads/random_poses.txt'
    with open(file_dir, 'w') as f:
        print('Number of Points: {}'.format(len(points)))
        for i in tqdm(range(0, 270)):#len(points))):
            p = Odometry()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = 'world'
            p.pose.pose.position.x = points[i][0]
            p.pose.pose.position.y = points[i][1]
            p.pose.pose.position.z = points[i][2]
            # Make sure the quaternion is valid and normalized
            p.pose.pose.orientation.x = orientations[i][1]
            p.pose.pose.orientation.y = orientations[i][2]
            p.pose.pose.orientation.z = orientations[i][3]
            p.pose.pose.orientation.w = orientations[i][0]
            f.write(filenames[i]+' '+ str(p.header.stamp.to_nsec()) +' ')
            np.savetxt(f, [points[i][0], points[i][1], points[i][2], orientations[i][0], orientations[i][1],
                           orientations[i][2], orientations[i][3]], fmt='%f', newline=' ', delimiter=',')  # w, x, y, z
            f.write("\n")

            cv_image = cv2.imread(folder_undistorted + '/' + filenames[i])
            newimage = cv2.resize(cv_image, None, fx=scaling_factor, fy=scaling_factor)
            crop_img = newimage[int(half_offset):int(half_offset + end_h), int(0):int(0 + end_w)]


            try:
                pub.publish(p)
                img_msg = bridge.cv2_to_imgmsg(crop_img, "bgr8")
                img_msg.header.stamp = p.header.stamp
                image_pub.publish(img_msg)
            except CvBridgeError as e:
                print(e)

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
    sampling_rate = 10
    if len(sys.argv) >= 3:
        sampling_rate = 10 #int(sys.argv[3])

    height = 20 # int(sys.argv[1])
    pitch = 15 #int(sys.argv[2])

    points = []
    orientations = []
    filenames = []

    num_fixed = 100

    points_pix4d, orientation_pix4d, filename_pix4d = load_pix4d()
    points.extend(points_pix4d)
    orientations.extend(orientation_pix4d)
    filenames.extend(filename_pix4d)

    try:
        publisher(points, orientations, filenames, sampling_rate)
    except rospy.ROSInterruptException:
        pass

    print("Sampling done!")