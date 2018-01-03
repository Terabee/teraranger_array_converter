#!/usr/bin/python
import rospy
from teraranger_array.msg import RangeArray
from sensor_msgs.msg import Range
import message_filters
from std_msgs.msg import Header

def strip_backslash(frame_id):
    if frame_id != '' and not None:
        if frame_id.startswith("/"):
            frame_id = frame_id[1:]
        if frame_id.endswith("/"):
            frame_id = frame_id[:-1]
        return frame_id
    return frame_id

class RangeToArray(object):
    def __init__(self):
        rospy.init_node("ranges_to_range_array")

        self.number_of_sensors = 8
        subscribers = [None]*self.number_of_sensors

        for sensor_idx in range(self.number_of_sensors):
            topic_name = "gazebo/range_" + str(sensor_idx)
            subscribers[sensor_idx] = message_filters.Subscriber(topic_name,
                                                                 Range)
        pub_topic = "ranges"
        self.pub_array = rospy.Publisher(pub_topic, RangeArray,
                                         queue_size=self.number_of_sensors*2)

        time_sync = message_filters.ApproximateTimeSynchronizer(subscribers,
                                                                queue_size=1,
                                                                slop=0.1)
        time_sync.registerCallback(self.callback_range)

        hub_ns = strip_backslash(rospy.get_namespace())
        rospy.loginfo("name_space %s", hub_ns)
        if hub_ns == "":
            self.hub_frame = "base_hub"
        else:
            self.hub_frame = "base_" + hub_ns

    def callback_range(self, *args):

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.hub_frame
        ranges_array = RangeArray()
        ranges_array.header = header
        ranges_array.ranges = args[0:self.number_of_sensors]
        self.pub_array.publish(ranges_array)

if __name__ == '__main__':
    sensors_to_array = RangeToArray()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
