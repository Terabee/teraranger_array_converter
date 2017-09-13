#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
import tf2_ros

from sensor_msgs.msg import LaserScan, Range, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from teraranger_array.msg import RangeArray
import message_filters
# Import tools
from conversion_tools import *

######################################################################
# IMPORTANT: To improve readability "to_be_fused" and "to_fuse_into" #
# are respectively abbreviated "tbf" and "tfi"                       #
######################################################################


def correct_ranges(tbf_tf_list, tbf_array):
    # This function returns a list of ranges projected to one sensor frame
    corrected_ranges = []

    for i in range(len(tbf_tf_list)):
        trans = tbf_tf_list[i]
        current_range = tbf_array.ranges[i].range

        # Building point in the frame of the sensor
        if current_range != -1 and not math.isnan(current_range) and not math.isinf(current_range):
            x = current_range
            y = 0
            z = 0
        else:
            continue  # Skip out of range point

        # Transforming the point to the target_frame
        if trans is not None:
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            tz = trans.transform.translation.z
            rx = trans.transform.rotation.x
            ry = trans.transform.rotation.y
            rz = trans.transform.rotation.z
            w = trans.transform.rotation.w

            # Rotation matrix from quaternion
            # https://en.wikipedia.org/wiki/Rotation_matrix
            rot_mat = [
                [1-2*ry*ry-2*rz*rz, 2*rx*ry-2*rz*w, 2*rx*rz+2*ry*w],
                [2*rx*ry+2*rz*w, 1-2*rx*rx-2*rz*rz, 2*ry*rz-2*rx*w],
                [2*rx*rz-2*ry*w, 2*ry*rz+2*rx*w, 1-2*rx*rx-2*ry*ry]
            ]

            rot_mat = np.array(rot_mat)

            # Translation vector
            translation_vector = np.array([[tx], [ty], [tz]])

            point = np.array([[x], [y], [z]])
            transformed_point = np.matmul(rot_mat, point)  # Rotating point
            transformed_point += translation_vector  # Translating point

            corrected_ranges.append(transformed_point[0][0])  # We only keep x as we project

    return corrected_ranges


class RangeArrayCombinator(object):

    def __init__(self):
        rospy.init_node("range_array_combinator", anonymous=True)

        # Init of variables
        self.tf_list = []
        self.offsets = []
        self.number_of_sensors = 8
        self.subscribers = []

        # Getting sensor id to replace
        try:
            self.sensor_id = rospy.get_param("~sensor_id")
        except KeyError:
            self.sensor_id = 0
            rospy.logwarn("Private parameter 'sensor_id' is not set."
                          " Default value 0 will be used instead.")

        # Getting sensor_mask
        try:
            self.sensor_mask = rospy.get_param("~sensor_mask")

        except KeyError:
            self.sensor_mask = [True] * self.number_of_sensors
            rospy.logwarn("Private parameter 'sensor_mask' is not set."
                          " All sensors will be used for conversion")

        # Instanciating the publisher
        self.publisher = rospy.Publisher("ranges_fused",
                                         RangeArray,
                                         queue_size=1)

        # Wait for tf before launching conversion
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # rospy.loginfo('Waiting for tf to become available')
        # if not self.tf_buffer.can_transform('base_link', 'base_hub',
        #                                     rospy.Duration(0),
        #                                     rospy.Duration(60)):
        #     rospy.logerr('Tf timeout for transform between base_link and '
        #                  'base_hub')
        #     exit()
        # rospy.loginfo('Tf successfully received')

        # Instanciating subscribers
        self.to_be_fused_array_sub = message_filters.Subscriber("array_to_be_fused",
                                               RangeArray)

        self.to_fuse_into_array_sub = message_filters.Subscriber("array_to_fuse_into",
                                               RangeArray)

        self.subscribers.append(self.to_be_fused_array_sub)
        self.subscribers.append(self.to_fuse_into_array_sub)


        # Instanciating time synchronizer
        # We allow headerless while we implement header in rangearray
        time_sync = message_filters.ApproximateTimeSynchronizer(self.subscribers,
                                                                queue_size=1,
                                                                slop=0.1)
        time_sync.registerCallback(self.callback_range_array)


    def callback_range_array(self, tbf_array, tfi_array):
        #rospy.logwarn('Callback invoked...')
        # We make the assumption that sensors in tbf have the same x and y as the sensor to be replaced
        output_frame = tfi_array.ranges[self.sensor_id].header.frame_id

        tbf_tf_list = gather_sensor_tf(self.tf_buffer, output_frame, tbf_array, self.sensor_mask)

        # Project ranges and merge
        ranges_to_fuse = correct_ranges(tbf_tf_list, tbf_array)
        ranges_to_fuse.append(tfi_array.ranges[self.sensor_id].range)
        minimum_range = min(ranges_to_fuse)

        # Replace range value
        tfi_array.ranges[self.sensor_id].range = minimum_range

        # Publish the fused array
        self.publisher.publish(tfi_array)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
        rospy.logwarn("Node shutting down")


if __name__ == '__main__':
    fuser = RangeArrayCombinator()
    try:
        fuser.run()
    except rospy.ROSInterruptException:
        exit()
