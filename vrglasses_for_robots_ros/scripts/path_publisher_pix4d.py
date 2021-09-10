#!/usr/bin/env python
import time
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
file_img_list = '/media/lucas/ntfs1t/3drecons/images/ircherl140821/top_image_db_20.txt'

#file_extern_cam = '/media/lucas/ntfs1t/3drecons/pix4d/irchel070921-base/1_initial/params/irchel070921-base_calibrated_external_camera_parameters.txt'
#folder_undistorted = '/media/lucas/ntfs1t/3drecons/pix4d/irchel070921-base/1_initial/images/undistorted_images'
file_intern_cam = ''
#file_img_list = '/media/lucas/ntfs1t/3drecons/images/irchel070921/db_img_list.csv'

def read_img_list():
    file_csv = open(file_img_list, 'r')
    lines = file_csv.read().splitlines()
    image_names = []
    for line in lines:
        base,filename = path.split(line)
        print (filename)
        image_names.append(filename)

    return image_names



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

def load_pix4d(image_names):
    #offsetx, offsety, offsetz = 400302.000, 5232070.000, 466.000
    offsetx, offsety, offsetz = 2683905.000, 1249997.000, 457.000 #irchel mesh 1408
    #offsetx, offsety, offsetz = 2683900.000,    1249995.000,    298.000 #irchel mesh 0709
    points = []
    orientations = []
    filenames = []
    print(image_names)
    with open(file_extern_cam, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=' ')
        for row in csv_reader:
            # cam = CameraPose(row)
            # self.cameras[cam.img_name] = cam
            if image_names is None or row['imageName'] in image_names:
                filenames.append(row['imageName'])
                points.append((float(row['X'])-offsetx, float(row['Y'])-offsety, float(row['Z'])-offsetz))
                rotmat = angle_to_rotmat1(math.radians(float(row['Omega'])),
                                          math.radians(float(row['Phi'])),
                                          math.radians(float(row['Kappa'])))
                quat = quaternions.mat2quat(np.transpose(rotmat))
                orientations.append((quat[0], quat[1], quat[2], quat[3]))  # w, x, y, z
                # orientations.append((cam.x, cam.y, cam.z))

    return points, orientations, filenames


def publisher( points, orientations,filenames, rate=100):
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)  # pose_publisher/odometry
    image_pub = rospy.Publisher("/firefly/real_image", Image, queue_size=10)

    time.sleep(2)# needed for the recorder connect to the topic

    bridge = CvBridge()
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
        for i in tqdm(range(0, len(points))):
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


if __name__ == '__main__':
    print("circ!")
    sampling_rate = 10
    if len(sys.argv) >= 3:
        sampling_rate = 10 #int(sys.argv[3])

    height = 20 # int(sys.argv[1])
    pitch = 15 #int(sys.argv[2])

    image_names = read_img_list()

    points = []
    orientations = []
    filenames = []

    num_fixed = 100

    points_pix4d, orientation_pix4d, filename_pix4d = load_pix4d(image_names)
    points.extend(points_pix4d)
    orientations.extend(orientation_pix4d)
    filenames.extend(filename_pix4d)

    try:
        publisher(points, orientations, filenames, sampling_rate)
    except rospy.ROSInterruptException:
        pass

    print("Sampling done!")