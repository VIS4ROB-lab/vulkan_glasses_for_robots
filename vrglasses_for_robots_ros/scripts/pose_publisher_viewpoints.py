#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
import os
from transforms3d import quaternions
from tf.transformations import *


def fibonacci_sphere(samples,radius_sphere):

    points = []
    phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians

    for i in range(samples):
        z = 1 - (i / float(samples - 1))  # z goes from 1 to 0
        z = radius_sphere * z
        radius_circle = math.sqrt(radius_sphere*radius_sphere - z * z)  # radius at z
        #print radius_circle

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius_circle
        y = math.sin(theta) * radius_circle

        points.append((x, y, z))

    return points


def constant_height(height):
    points = []
    min_x = -150
    max_x =  151
    min_y = -150
    max_y =  151

    for x in range(min_x, max_x, 30):
        for y in range(min_y, max_y, 30):
            points.append((x,y,height))

    #print points
    return points

def star_positions2(center_x = 0.0, center_y = 0.0, height = 50.0):
    points = []
    yaw = []

    min_radius = 10;
    max_radius = 111;
    radius_step = 10;

    min_angle = 0;
    max_angle = 361;
    angle_step = 15;

    for a in range(min_angle, max_angle, angle_step):
        angle_rad = np.deg2rad(a);
        yaw_cam = - a  
        for r in range(min_radius, max_radius, radius_step):            
            x = center_x + (r * np.cos(angle_rad))  
            y = center_y + (r * np.sin(angle_rad))
            points.append((x,y,height))
            yaw.append(yaw_cam)

    return points, yaw

def star_positions(center_x = 0.0, center_y = 0.0, height = 50.0):
    points = []
    yaw = []

    min_radius = 10;
    max_radius = 111;
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
            yaw.append(a+ 180)

    return points, yaw


def frontal_plane():
    points = []
    x = -100
    min_y = -150
    max_y =  151
    min_z = -85
    max_z =  100

    for y in range(min_y, max_y, 25):
        for z in range(min_z, max_z, 25):
            points.append((x,y,z))

	return points


def compute_orientations(points):
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

        print vright
        print vup
        print vft

        transf_matrix = np.zeros((3, 3))
        transf_matrix[:, 0] = vft
        transf_matrix[:, 1] = vright
        transf_matrix[:, 2] = vup

        quat = quaternions.mat2quat(transf_matrix)
        orientations.append((quat[0], quat[1], quat[2], quat[3])) # w, x, y, z
       
    return orientations


def compute_downlook_orientations(points):
    orientations = []

    v = np.array([0.707, 0, 0.707, 0])
    v = v / np.linalg.norm(v)
    for pf in points:
        orientations.append((v[0], v[1], v[2], v[3])) # w, x, y, z
        #orientations.append((0.707, 0, 0.707, 0)) # w, x, y, z
        
    return orientations

# RPY to convert: 90deg, 0, -90deg
#q = quaternion_from_euler(1.5707, 0, -1.5707)

def compute_star_orientations(points, yaw_angles, pitch):
    orientations = []

    roll = 0
    nPoints = len(points)
    for i in range(nPoints):
        r_rad = np.deg2rad(roll);
        p_rad = np.deg2rad(pitch); 			# descent angle
        y_rad = np.deg2rad(yaw_angles[i]);  # direction of arrival (star line)
        print yaw_angles[i]
        quat = quaternion_from_euler(y_rad, p_rad, r_rad)
        orientations.append((quat[0], quat[1], quat[2], quat[3])) # w, x, y, z

    return orientations



def publisher(points, orientations):
    pub = rospy.Publisher('pose_publisher/odometry', Odometry, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(0.3) # Hz
    
    file_dir = '/home/fabiola/Desktop/star_poses.txt'
    with open(file_dir, 'w') as f:

        for i in xrange(1,len(points)):
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
    points = fibonacci_sphere(20,120)
    #points = constant_height(70)
    #points = frontal_plane()
    #points, yaw_angles = star_positions2()

    #orientations = compute_star_orientations(points, yaw_angles, -60)
    #orientations = compute_downlook_orientations(points)
    orientations = compute_orientations(points)

    try:
        publisher(points, orientations)
    except rospy.ROSInterruptException:
        pass
