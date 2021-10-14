#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
import os


def fibonacci_sphere(samples,radius_sphere):

    points = []
    phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians

    for i in range(samples):
        #y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        y = 1 - (i / float(samples - 1))  # y goes from 1 to 0
        y = radius_sphere * y
        radius_circle = math.sqrt(radius_sphere*radius_sphere - y * y)  # radius at y
        print radius_circle

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius_circle
        z = math.sin(theta) * radius_circle

        points.append((x, y, z))

    return points


def publisher(points):
    pub = rospy.Publisher('pose_publisher/odometry', Odometry, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(0.5) # Hz
    #while not rospy.is_shutdown():

    
    #file_dir = '/home/fabiola/Desktop/sphere_points.txt'
    #with open(file_dir, 'w+') as f:
      
    for pt in points:
	    p = Odometry()
	    p.header.stamp = rospy.Time.now()
	    p.pose.pose.position.x = pt[0]#-60
	    p.pose.pose.position.y = pt[2]#-20
	    p.pose.pose.position.z = 10+ pt[1]#15.0
	    # Make sure the quaternion is valid and normalized
	    p.pose.pose.orientation.x = 0.0
	    p.pose.pose.orientation.y = 0.0
	    p.pose.pose.orientation.z = 0.0
	    p.pose.pose.orientation.w = 1.0
	    #np.savetxt(f, [pt[0], pt[2], pt[1]], fmt='%f', newline=' ', delimiter=',')
	    #f.write("\n")
	    
	    #f.write('"%f",%f,%f\n' % p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z)
	    #print p
	    pub.publish(p)
	    if rospy.is_shutdown():
		break
	    rate.sleep()

    #f.close()


if __name__ == '__main__':
    points = fibonacci_sphere(1000,60)
    print points
    try:
        publisher(points)
    except rospy.ROSInterruptException:
        pass
