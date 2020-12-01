#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def publisher():
    pub = rospy.Publisher('/firefly/vi_sensor/ground_truth/odometry', Odometry, queue_size=1)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        p = Odometry()
        p.header.stamp = rospy.Time.now()
        p.pose.pose.position.x = -60
        p.pose.pose.position.y = -30
        p.pose.pose.position.z = 30.0
        # Make sure the quaternion is valid and normalized
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 1.0
        pub.publish(p)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
