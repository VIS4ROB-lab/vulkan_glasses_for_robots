#!/usr/bin/env python

import sys
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import pywavefront
import numpy as np

def model_limits(obj_file):
    scene = pywavefront.Wavefront(obj_file, create_materials=True)
    for name, material in scene.materials.items():# assuming only one material

        if material.vertex_format == 'T2F_N3F_V3F':
            print(material.vertex_format)
            # extract all x coordinates (every 8th element starting with 6th)
            xVertices = material.vertices[5::8]

            # extract all y coordinates (every 8th element starting with 7th)
            yVertices = material.vertices[6::8]

            # extract all z coordinates (every 8th element sarting with 8th)
            zVertices = material.vertices[7::8]
        elif material.vertex_format == 'T2F_V3F':
            # extract all x coordinates (every 8th element starting with 6th)
            xVertices = material.vertices[2::8]

            # extract all y coordinates (every 8th element starting with 7th)
            yVertices = material.vertices[3::8]

            # extract all z coordinates (every 8th element sarting with 8th)
            zVertices = material.vertices[4::8]
        else:
            raise

        # calculate min/max of coordinates
        x_max = max(xVertices)
        x_min = min(xVertices)

        y_max = max(yVertices)
        y_min = min(yVertices)

        z_max = max(zVertices)
        z_min = min(zVertices)

        # print extrema
        print('min x:', x_min, 'max x:', x_max)
        print('min y:', y_min, 'max y:', y_max)
        print('min z:', z_min, 'max z:', z_max)

        diff_x = x_max - x_min
        diff_y = y_max - y_min

        diff_max = max(diff_x,diff_y)
        half_diff_max = diff_max / 2.0 # assumes a square output image

        x_cam = x_min + (diff_x / 2.0)
        y_cam = y_min + (diff_y / 2.0)
        z_cam = z_max+ 2.0   # z needs to be bigger than z_max
        l = - half_diff_max + y_cam
        r = half_diff_max + y_cam

        b = - half_diff_max + x_cam
        t = half_diff_max + x_cam

        print("coordinate center: u({})/v({})".format(x_cam, y_cam))
        print("scale(4096 resolution): {} units per pixel".format(diff_max/4096.0))
        print("z-range: {}/{}".format(z_cam-z_max, z_cam-z_min))
        return l, r, b, t, x_cam, y_cam, z_cam

def publisher(obj_file):
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('/firefly/odometry_ortho', Odometry, queue_size=1)
    pub_ortho_config = rospy.Publisher('/firefly/ortho_config', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(10) # Hz

    l, r, b, t, x, y, z = model_limits(obj_file)

    while not rospy.is_shutdown():
        p = Odometry()
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x =  x
        p.pose.pose.position.y =  y
        p.pose.pose.position.z =  z
        # Make sure the quaternion is valid and normalized
        p.pose.pose.orientation.x = 1.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 0.0

        ortho_config_msg = Float32MultiArray()
        ortho_config_msg.data = [l, r, b, t]

        pub.publish(p)
        pub_ortho_config.publish(ortho_config_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
