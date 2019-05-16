#!/usr/bin/env python


import rospy
import numpy as np
from nav_msgs.msg import Odometry

from geometry_msgs.msg import (PoseWithCovarianceStamped, PointStamped, PoseWithCovariance, Point, TransformStamped,
                               Transform)

import piksi_rtk_msgs
from piksi_rtk_msgs.srv import DegradationLevel


class GNSSFixDegradation:

    def __init__(self):

        rospy.init_node("piksi")
        rospy.sleep(0.5)
        rospy.loginfo(rospy.get_name() + " start")

        stdev_spp = [2.0, 2.0, 5.0]
        stdev_spp_sbas = [1.0, 1.0, 2.0]
        stdev_rtk_float = [0.5, 0.5, 1.0]
        stdev_rtk_fix = [0.07, 0.07, 0.12]

        self.stdev = np.array([[stdev_rtk_fix],[stdev_rtk_float],[stdev_spp_sbas],[stdev_spp]])

        self.enu_frame_id = "enu"
        self.transform_child_frame_id = "gps_receiver"
        self.degradation_level = 1 # 1 - no degradation , 2 - rtk floatish , 3 - sppish

        self.subscribers = self.subscribe_topics()

        self.publishers = self.advertise_topics()

        self.service_servers = self.advertise_services()

        rospy.spin()

    def subscribe_topics(self):
        subscribers = {}

        subscribers["enu_pose_fix"] = rospy.Subscriber("enu_pose_fix", PoseWithCovarianceStamped,
                                                    self.degrade_signal)

        return subscribers
        

    def advertise_topics(self):
        publishers = {}

        publishers["enu_pose_best_fix"] = rospy.Publisher(rospy.get_name() + "/enu_pose_best_fix",
                                                          PoseWithCovarianceStamped, queue_size=10)

        publishers["odom_covariance"] = rospy.Publisher(rospy.get_name() + "/odom_covariance",
                                                 Odometry, queue_size=10)

        return publishers

    def advertise_services(self):
        servers = {}

        servers["change_degradation"] = rospy.Service(rospy.get_name() + "change_degradation",
                                                    DegradationLevel,
                                                    self.change_degradation_level)

        return servers

    def degrade_signal(self, pose_fix_msg):
        # for now only create odometry message with covariance
        odometry_msg = Odometry()
        odometry_msg.header.stamp = pose_fix_msg.header.stamp
        odometry_msg.header.frame_id = self.enu_frame_id
        odometry_msg.child_frame_id = self.transform_child_frame_id
        # odometry_msg.pose = pose_fix_msg.pose
        odometry_msg.pose.pose.position = pose_fix_msg.pose.pose.position
        odometry_msg.pose.pose.orientation = pose_fix_msg.pose.pose.orientation

        odometry_msg.pose.covariance[6 * 0 + 0] = 1
        odometry_msg.pose.covariance[6 * 1 + 1] = 2
        odometry_msg.pose.covariance[6 * 2 + 2] = 3
        odometry_msg.pose.covariance[6 * 3 + 3] = 4
        odometry_msg.pose.covariance[6 * 4 + 4] = 5
        odometry_msg.pose.covariance[6 * 5 + 5] = 6

        self.publishers["odom_covariance"].publish(odometry_msg)

    
    def change_degradation_level(self, request):
        response = DegradationLevel

        self.degradation_level = request.degradation_level
        response.message = "Degradation of signal changed to " + str(self.degradation_level)

        return response


