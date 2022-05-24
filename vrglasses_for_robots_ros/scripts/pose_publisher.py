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
        p.pose.pose.position.x = -0
        p.pose.pose.position.y = -60
        p.pose.pose.position.z = 250.0
        # Make sure the quaternion is valid and normalized
        p.pose.pose.orientation.x = 0.0
        p.pose.pose.orientation.y = 0.707
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = 0.707
        pub.publish(p)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
