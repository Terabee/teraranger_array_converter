#!/usr/bin/python
# -*- coding: utf-8 -*-
import tf2_ros
import tf_conversions
import rospy
import math
from sensor_msgs.msg import Range
from teraranger_array.msg import RangeArray


def strip_backslash(frame_id):
    if frame_id != '' and not None:
        if frame_id[0] == '/':
            return frame_id[1:]
        else:
            return frame_id


def lookup_transform(tf_buffer, input_frame, target_frame):

    try:
        trans = tf_buffer.lookup_transform(strip_backslash(target_frame),
                                           strip_backslash(input_frame),
                                           rospy.Time(0))
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as exception:
        trans = None
        rospy.logwarn(exception)

    return trans


def gather_sensor_tf(tf_buffer, target_frame, rng_array, mask):
    tfs = []
    for i in range(len(rng_array.ranges)):
        sensor = rng_array.ranges[i]
        if mask[i]:
            trans = lookup_transform(tf_buffer,
                                     strip_backslash(sensor.header.frame_id),
                                     target_frame)
        else:
            trans = None
        tfs.append(trans)

    return tfs


def tf_to_z(transform):
    quaternion = (
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w)

    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # z_axis = math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))

    euler = tf_conversions.transformations.euler_from_quaternion(quaternion)

    return euler[2]

def tf_to_euler(transform):
    quaternion = (
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w)

    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # z_axis = math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))

    euler = tf_conversions.transformations.euler_from_quaternion(quaternion)

    return euler


def wrap_to_2pi(angle):
    if angle < 0:
        return angle + math.pi * 2.0
    else:
        return angle


def compute_offsets(tf_list):
    # In order to make this function work for a scan, it implies that the
    # provided tfs are between sensor and the hub_frame
    offsets = []

    for tf in tf_list:
        if tf is not None:
            x = tf.transform.translation.x
            y = tf.transform.translation.y

            distance = math.sqrt(x**2 + y**2)

            offsets.append(distance)
        else:
            offsets.append(0.0)
    return offsets
