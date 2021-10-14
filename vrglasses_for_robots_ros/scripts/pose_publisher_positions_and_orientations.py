#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
import os
from transforms3d import quaternions


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

    for pf in points:
        orientations.append((0.707, 0, 0.707, 0)) # w, x, y, z
        
    return orientations


def publisher(points, orientations):
    pub = rospy.Publisher('pose_publisher/odometry', Odometry, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(10) # Hz
    file_dir = '/home/fabiola/Desktop/sphere_poses3.txt'
    with open(file_dir, 'w+') as f:
       
    #for pt in points:
		for i in xrange(1,len(points)):
		    p = Odometry()
		    p.header.stamp = rospy.Time.now()
		    p.pose.pose.position.x = points[i][0]#-60
		    p.pose.pose.position.y = points[i][1]#-20
		    p.pose.pose.position.z = 10+ points[i][2]#15.0

			# Make sure the quaternion is valid and normalized
		    p.pose.pose.orientation.x = orientations[i][1]
		    p.pose.pose.orientation.y = orientations[i][2]
		    p.pose.pose.orientation.z = orientations[i][3]
		    p.pose.pose.orientation.w = orientations[i][0]
			#p.pose.pose.orientation.x = -0.000271862985499#0.0
		    #p.pose.pose.orientation.y = 0.612218839566#0.0
		    #p.pose.pose.orientation.z = -0.000340555828648#0.0
		    #p.pose.pose.orientation.w = 0.790688246145#1.0

		    np.savetxt(f, [points[i][0], points[i][1], points[i][2], orientations[i][0],orientations[i][1],orientations[i][2],orientations[i][3]], fmt='%f', newline=' ', delimiter=',') # w, x, y, z
		    f.write("\n")
			
			#f.write('"%f",%f,%f\n' % p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z)
			#print p
		    pub.publish(p)
		    if rospy.is_shutdown():
		        break
		    rate.sleep()

    f.close()


if __name__ == '__main__':
    points = fibonacci_sphere(100,70)
    orientations = compute_orientations(points) #compute_downlook_orientations(points)    
    try:
        publisher(points, orientations)
    except rospy.ROSInterruptException:
        pass
