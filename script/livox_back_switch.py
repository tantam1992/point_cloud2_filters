#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2

class LivoxBackPointCloudSwitcher:
    def __init__(self):
        rospy.init_node('livox_back_switch', anonymous=True)

        # Subscribers
        self.fold_state_sub = rospy.Subscriber('/fold_state', String, self.fold_state_callback)
        self.livox_back_empty_sub = rospy.Subscriber('/livox_back_empty/points', PointCloud2, self.livox_back_empty_callback)
        self.livox_back_with_cart_sub = rospy.Subscriber('/livox_back_with_cart/points', PointCloud2, self.livox_back_with_cart_callback)

        # Publisher
        self.livox_back_pub = rospy.Publisher('/livox_back/points', PointCloud2, queue_size=10)

        # State variables
        self.fold_state = "UNKNOWN"
        self.livox_back_empty_points = None
        self.livox_back_with_cart_points = None

    def fold_state_callback(self, msg):
        self.fold_state = msg.data
        self.publish_livox_back_points()

    def livox_back_empty_callback(self, msg):
        self.livox_back_empty_points = msg
        self.publish_livox_back_points()

    def livox_back_with_cart_callback(self, msg):
        self.livox_back_with_cart_points = msg
        self.publish_livox_back_points()

    def publish_livox_back_points(self):
        if (self.fold_state == "OPERATIONAL/LOADED" or self.fold_state == "OPERATIONAL/READY_PICKUP") and self.livox_back_with_cart_points is not None:
            self.livox_back_pub.publish(self.livox_back_with_cart_points)
        elif self.livox_back_empty_points is not None:
            self.livox_back_pub.publish(self.livox_back_empty_points)

if __name__ == '__main__':
    try:
        switcher = LivoxBackPointCloudSwitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass