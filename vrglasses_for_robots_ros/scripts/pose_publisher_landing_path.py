#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import *
from tqdm import tqdm

def build_position_query():
    init_pos = (-20.0,-13.0,36.0)
    end_pos = (-25.0,-15.0,-40.0)
    x1 = np.linspace(init_pos[0], end_pos[0], 20)
    y1 = np.linspace(init_pos[1], end_pos[1], 20)
    z1 = np.linspace(init_pos[2], end_pos[2], 20)

    X = np.dstack((x1, y1, z1))
    print(X[0])

    return X[0]

def build_position_db(c=(-25.0,-12.0,30.0)):

    points = []
    #top
    points.append(c)
    #l1
    ld = -20
    points.append((c[0], c[1], c[2]+ld))
    ost = 10
    points.append((c[0]-ost, c[1]-ost, c[2]+ld))
    points.append((c[0]-ost, c[1]+ost, c[2]+ld))
    points.append((c[0]+ost, c[1]+ost, c[2]+ld))
    points.append((c[0]+ost, c[1]-ost, c[2]+ld))

    #l2

    ld = -60
    points.append((c[0], c[1], c[2] + ld))
    ost = 5
    points.append((c[0] - ost, c[1] - ost, c[2] + ld))
    points.append((c[0] - ost, c[1] + ost, c[2] + ld))
    points.append((c[0] + ost, c[1] + ost, c[2] + ld))
    points.append((c[0] + ost, c[1] - ost, c[2] + ld))

    ost = 2
    points.append((c[0] - ost, c[1] - ost, c[2] + ld))
    points.append((c[0] - ost, c[1] + ost, c[2] + ld))
    points.append((c[0] + ost, c[1] + ost, c[2] + ld))
    points.append((c[0] + ost, c[1] - ost, c[2] + ld))

    return points


def compute_db_orientations(points, yaw_angles, pitch):

    orientations = []
    print('num points: {} / Pitch Angle: {}'.format(len(points), pitch))
    print(points)
    roll = 0
    nPoints = len(points)
    for i in range(nPoints):
        r_rad = np.deg2rad(roll);
        p_rad = np.deg2rad(pitch);  # descent angle
        y_rad = np.deg2rad(yaw_angles);  # direction of arrival (star line)

        quat = quaternion_from_euler(y_rad, p_rad, r_rad)
        orientations.append((quat[0], quat[1], quat[2], quat[3]))  # w, x, y, z

    return points, orientations

def compute_query_orientations(points, yaw_angles, pitch):
    points_array = []
    orientations = []
    print('num points: {} / Pitch Angle: {}'.format(len(points), pitch))
    print(points)
    roll = 0
    nPoints = len(points)
    for i in range(nPoints):
        r_rad = np.deg2rad(roll);
        p_rad = np.deg2rad(pitch);  # descent angle
        y_rad = np.deg2rad(yaw_angles);  # direction of arrival (star line)
        # print(yaw_angles[i])
        quat = quaternion_from_euler(y_rad, p_rad, r_rad)
        points_array.append((points[i,0],points[i,1],points[i,2]))
        orientations.append((quat[0], quat[1], quat[2], quat[3]))  # w, x, y, z

    return points_array, orientations

def publisher(points, orientations, rate=100):
    pub = rospy.Publisher('firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)  # pose_publisher/odometry
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(rate)  # Hz

    file_dir = '/home/lucas/Downloads/random_poses.txt'
    with open(file_dir, 'w') as f:
        print('Number of Points: {}'.format(len(points)))
        for i in tqdm(range(0, len(points))):
            p = Odometry()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = 'world'
            print(points[i])
            p.pose.pose.position.x = points[i][0]
            p.pose.pose.position.y = points[i][1]
            p.pose.pose.position.z = points[i][2]
            # Make sure the quaternion is valid and normalized
            print(orientations[i])
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


# def publisher():
#     pub = rospy.Publisher('/firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)
#     rospy.init_node('pose_publisher', anonymous=True)
#     rate = rospy.Rate(1) # Hz
#     while not rospy.is_shutdown():
#         p = Odometry()
#         p.header.stamp = rospy.Time.now()
#         p.pose.pose.position.x = -20.0
#         p.pose.pose.position.y = -13.0
#         p.pose.pose.position.z = 16.0
#         # Make sure the quaternion is valid and normalized
#
#         p.pose.pose.orientation.w = 0.0530938964657
#         p.pose.pose.orientation.x = -0.385775289178
#         p.pose.pose.orientation.y = 0.912428481139
#         p.pose.pose.orientation.z = -0.125828181362
#
#
#         pub.publish(p)
#         rate.sleep()

# 0.50904905  0.66936987 -0.42038654  0.34071719 no
# 0.1760578  0.11902859 0.60087547 0.77057415 no
# 0.50904905 -0.66936987  0.42038654 -0.34071719
# 0.         0.51861482 0.85500799 0.        
# 0.06769282 0.514178   0.84769328 0.11160094 no
# 0.06769282 -0.514178   -0.84769328  0.11160094
# 0.11160094 -0.84769328 -0.514178    0.06769282
#0.32719739 -0.78992439 -0.47913762  0.1984653 
# 0.738613 0.00291168 -0.658953 -0.142212


if __name__ == '__main__':
    try:

        points = build_position_db()
        points_array, orientation = compute_db_orientations(points, 0.0,180)
        publisher(points_array,orientation,1)

    except rospy.ROSInterruptException:
        pass

