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
# Import tools
from conversion_tools import *


def to_point_cloud(tf_list, rng_array, target_frame):
    # Initializing output structure
    header = Header()
    header.frame_id = target_frame
    header.stamp = rng_array.header.stamp

    nb_sensors = len(rng_array.ranges)  # Always 8 because of the parser

    # Transforming points to target frame
    points = []
    for i in range(nb_sensors):
        trans = tf_list[i]
        current_range = rng_array.ranges[i].range

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

            points.append(transformed_point)

    # Create pcl from points
    output_cloud = point_cloud2.create_cloud_xyz32(header, points)

    return output_cloud


def auto_build_scan(tf_list, rng_array, number_of_pos, offsets, scan_frame,
                    scan_time=1./100):

    corrected_range_array = [float('inf')]*number_of_pos
    number_of_sensor = 8  # The parser outputs 8 values anyway

    scan = LaserScan()
    scan.scan_time = scan_time
    scan.header.frame_id = scan_frame
    scan.angle_max = 2*math.pi
    scan.angle_min = 0.0
    scan.angle_increment = 2.0*math.pi / number_of_pos
    scan.time_increment = scan_time / number_of_pos
    scan.range_min = rng_array.ranges[0].min_range
    scan.range_max = rng_array.ranges[0].max_range

    for i in range(number_of_sensor):
        if tf_list[i] is not None:
            yaw = tf_to_z(tf_list[i].transform)
            yaw = wrap_to_2pi(yaw)
            new_pos = int(round(yaw / (2.0 * math.pi) * number_of_pos))

            if rng_array.ranges[i].range < 0:
                pass  # Out of range values (see parser for more info)
            else:
                corrected_range_array[new_pos] = rng_array.ranges[i].range + offsets[i]

    scan.ranges = corrected_range_array
    scan.header.stamp = rng_array.header.stamp
    return scan


def check_sensors_on_plane(tf_list):
    # This check implies that sensors should be  on an horizontal plane (
    # perpendicular to z-axis) in the frame of the tf list
    first_height = None
    plane = True
    for tf in tf_list:
        if tf is not None:
            if first_height is None:
                first_height = tf.transform.translation.z
            else:
                current_height = tf.transform.translation.z
                if not np.isclose(current_height, first_height, 0.0, 1.e-3):
                    plane = False

    return plane


def check_sensors_converging(tf_list):
    # This check implies that the tf list is done in the frame of the hub
    # and that the hub is the center of this frame
    converging = True

    for tf in tf_list:
        if tf is not None:
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = tf_to_z(tf.transform)
            computed_yaw = np.angle(complex(x, y))

            if not np.isclose(yaw, computed_yaw, 0.0, 1.e-2):
                converging = False

    return converging


class TrHubConverter(object):

    def __init__(self):
        rospy.init_node("tr_hub_converter", anonymous=True)

        # Init of variables
        self.tf_list = []
        self.offsets = []
        self.number_of_pos = 360
        self.scan_init = False
        self.pt_cld_init = False
        self.number_of_sensors = 8
        self.publishers = []

        # Getting mode param
        try:
            self.mode = rospy.get_param("~converter_mode")
        except KeyError:
            self.mode = "laser_scan"
            rospy.logwarn("Private parameter 'converter_mode' is not set."
                          " Default value 'laser_scan' will be used instead.")

        self.range_array_topic = "ranges"

        # Getting sensor_mask
        try:
            self.sensor_mask = rospy.get_param("~sensor_mask")
        except KeyError:
            self.sensor_mask = [True] * self.number_of_sensors
            rospy.logwarn("Private parameter 'sensor_mask' is not set."
                          " All sensors will be used for conversion")

        # Getting conversion frame
        try:
            self.conversion_frame = rospy.get_param("~conversion_frame")
        except KeyError:
            self.conversion_frame = "base_hub"
            rospy.logwarn("Private parameter 'conversion_frame' is not set."
                          " Default value 'base_hub' will be used instead.")

        # Getting refresh transform parameter
        try:
            self.force_tf_refresh = rospy.get_param("~force_tf_refresh")
            if self.force_tf_refresh:
                rospy.logwarn("WARNING: The transforms are going to be "
                              "refreshed for each conversion. This may impact "
                              "performance")
        except KeyError:
            self.force_tf_refresh = False
            rospy.logwarn("Private parameter 'force_tf_refresh' is not set."
                          " Default value 'False' will be used instead.")

        # Creating the right publisher
        if self.mode == "laser_scan":
            self.publisher = rospy.Publisher("scan", LaserScan, queue_size=1)
        elif self.mode == "point_cloud":
            self.publisher = rospy.Publisher("point_cloud", PointCloud2,
                                             queue_size=1)
        elif self.mode == "sequential_ranges":
            self.publisher = rospy.Publisher("sequential_ranges", Range, queue_size=8)
        elif self.mode == "individual_ranges":
            for i in range(self.number_of_sensors):
                if self.sensor_mask[i]:
                    self.publishers.append(rospy.Publisher("range_" + str(i), Range, queue_size=1))
                else:
                    self.publishers.append(None)
        else:
            rospy.logwarn("Incorrect value for the parameter 'converter_mode'."
                          " Default value 'laser_scan' will be used instead.")
            self.mode = "laser_scan"
            self.publisher = rospy.Publisher("scan", LaserScan, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Wait for tf before launching conversion
        rospy.loginfo('Waiting for tf to become available')
        if not self.tf_buffer.can_transform('base_link', 'base_link',
                                            rospy.Duration(0),
                                            rospy.Duration(60)):
            rospy.logerr('Tf timeout for transform between base_link and '
                         'base_link')
            exit()
        rospy.Rate(0.33).sleep()
        rospy.loginfo('Tf successfully received')

        # Initializing callback
        self.hub_subscriber = rospy.Subscriber(self.range_array_topic, RangeArray,
                                               self.parser_callback)
        self.data = RangeArray()
        self.data.ranges = [Range()] * self.number_of_sensors

    def init_auto_scan(self, tf_buffer, scan_frame, rng_array):
        self.tf_list = gather_sensor_tf(tf_buffer, scan_frame, rng_array,
                                        self.sensor_mask)

        # Checking sensor setup
        if not check_sensors_converging(self.tf_list):
            rospy.logerr("Sensors might not be converging to one point. The "
                         "resulting scan could be incorrect")
        if not check_sensors_on_plane(self.tf_list):
            rospy.logerr("Sensors are not on an horizontal plane. Ranges "
                         "might be overriding themselves and/or be incorrect")

        self.offsets = compute_offsets(self.tf_list)
        rospy.loginfo('Offsets list : %s', self.offsets)

        self.scan_init = True

    def init_point_cloud(self, tf_buffer, pt_cld_frame, rng_array):
        self.tf_list = gather_sensor_tf(tf_buffer, pt_cld_frame, rng_array,
                                        self.sensor_mask)
        self.pt_cld_init = True

    def parser_callback(self, msg):
        self.data = msg

        if self.mode == "laser_scan":
            if not self.scan_init:
                self.init_auto_scan(self.tf_buffer, self.conversion_frame, self.data)
            else:
                if self.force_tf_refresh:
                    self.tf_list = gather_sensor_tf(self.tf_buffer, self.conversion_frame, self.data,
                                                    self.sensor_mask)
                result = auto_build_scan(self.tf_list, self.data,
                                         self.number_of_pos, self.offsets,
                                         self.conversion_frame)
                self.publisher.publish(result)

        elif self.mode == "point_cloud":
            if not self.pt_cld_init:
                self.init_point_cloud(self.tf_buffer, self.conversion_frame, self.data)
            else:
                if self.force_tf_refresh:
                    self.tf_list = gather_sensor_tf(self.tf_buffer, self.conversion_frame, self.data,
                                                    self.sensor_mask)
                result = to_point_cloud(self.tf_list, self.data, self.conversion_frame)
                self.publisher.publish(result)

        elif self.mode == "sequential_ranges":
            for i in range(len(self.data.ranges)):
                if self.sensor_mask[i]:
                    current_range = self.data.ranges[i]
                    self.publisher.publish(current_range)

        elif self.mode == "individual_ranges":
            for i in range(len(self.data.ranges)):
                if self.publishers[i] is not None:
                    current_range = self.data.ranges[i]
                    self.publishers[i].publish(current_range)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
        rospy.logwarn("Node shutting down")


if __name__ == '__main__':
    converter = TrHubConverter()
    try:
        converter.run()
    except rospy.ROSInterruptException:
        exit()
