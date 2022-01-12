#!/usr/bin/env python

import sys
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

import pywavefront
import numpy as np

class OrthoMap:
    def __init__(self):
        return
    def color_map_cb(self,img):
        print('received color')
        return
    def semantic_map_cb(self,map):
        print('received semantics')
        return
    def depth_map_cb(self,map):
        print('received depth')
        return

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
            xVertices = material.vertices[2::5]

            # extract all y coordinates (every 8th element starting with 7th)
            yVertices = material.vertices[3::5]

            # extract all z coordinates (every 8th element sarting with 8th)
            zVertices = material.vertices[4::5]
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
        half_diff_max = diff_max / 8.0 # assumes a square output image

        print(diff_x)
        print(diff_y)
        print(diff_max)
        print(half_diff_max)

        x_cam = x_min + (diff_x / 2.0)
        y_cam = y_min + (diff_y / 2.0)
        z_cam = z_max + 2.0   # z needs to be bigger than z_max
        l = - half_diff_max
        r = half_diff_max

        b = - half_diff_max
        t = half_diff_max

        print("l({})/r({})/b({})/t({})".format(l, r, b, t))

        print("coordinate center: u({})/v({})".format(x_cam, y_cam))
        print("scale(4096 resolution): {} units per pixel".format(diff_max/4096.0))
        print("z-range: {}/{}".format(z_cam-z_max, z_cam-z_min))
        return l, r, b, t, x_cam, y_cam, z_cam

def subscriber():
    return

def publisher(obj_file):
    result_obj = OrthoMap()
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('/firefly/odometry_ortho', Odometry, queue_size=1)
    pub_ortho_config = rospy.Publisher('/firefly/ortho_config', Float32MultiArray, queue_size=1)

    sub_color_img = rospy.Subscriber("/firefly/vrglasses_for_robots_ros/ortho_color_map", Image, result_obj.color_map_cb,
                                          queue_size=1)
    sub_depth_img = rospy.Subscriber("/firefly/vrglasses_for_robots_ros/ortho_depth_map", Image, result_obj.depth_map_cb,
                                          queue_size=1)
    sub_semantic_img = rospy.Subscriber("/firefly/vrglasses_for_robots_ros/ortho_semantic_map", Image, result_obj.semantic_map_cb,
                                          queue_size=1)

    rate = rospy.Rate(1) # Hz

    l, r, b, t, x, y, z = model_limits(obj_file)

    while not rospy.is_shutdown():
        p = Odometry()
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.position.z = z
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
