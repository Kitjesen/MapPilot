#!/usr/bin/env python3
# Taken from http://wiki.ros.org/rosbag/Cookbook

import argparse

import holoviews as hv
import numpy as np
import pandas as pd
import rosbag2_py
from allan_variance_msgs.msg import Imu9DoF
from rclpy.serialization import deserialize_message
from rosbag2_py._info import Info
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from std_msgs.msg import String

hv.extension('bokeh', logo=False)
import panel as pn
import rerun as rr

parser = argparse.ArgumentParser()

parser.add_argument("--input", type=str, default=None)
parser.add_argument("--topic", type=str, default=None)
parser.add_argument("--storage_id", type=str, default="mcap")

args = parser.parse_args()

def get_rosbag_options(path, storage_id, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(
        uri=path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

storage_options, converter_options = get_rosbag_options(args.input, args.storage_id)

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topics_with_message_count = Info().read_metadata(args.input, args.storage_id).topics_with_message_count

number_of_messages = 0
for topic_info in topics_with_message_count:
    if topic_info.topic_metadata.name == args.topic:
        print(f"{topic_info.message_count} message(s) available for topic '{args.topic}'")
        number_of_messages = topic_info.message_count

topic_types = reader.get_all_topics_and_types()

# Create a map for quicker lookup
type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

# Set filter for topic of string type
storage_filter = rosbag2_py.StorageFilter(topics=[args.topic])
reader.set_filter(storage_filter)

msg_counter = 0
# imu_data = np.random.rand(6, number_of_messages)
# timeline = np.random.rand(number_of_messages)

rr.init("plot_imu")
while reader.has_next():
    (topic, data, t) = reader.read_next()
    if type_map[topic]=="sensor_msgs/msg/Imu":
        msg = deserialize_message(data, Imu)
    else:
        msg = deserialize_message(data, Imu9DoF)
    if ((msg_counter/number_of_messages * 100) % 1) == 0:
        print(f"{msg_counter/number_of_messages * 100}% buffered", end="\r")
    # timeline[msg_counter] = msg.header.stamp.nanosec*10e-9+msg.header.stamp.sec
    # timeline[msg_counter] = t
    # imu_data[0, msg_counter] = msg.linear_acceleration.x
    # imu_data[1, msg_counter] = msg.linear_acceleration.y
    # imu_data[2, msg_counter] = msg.linear_acceleration.z
    # imu_data[3, msg_counter] = msg.angular_velocity.x
    # imu_data[4, msg_counter] = msg.angular_velocity.y
    # imu_data[5, msg_counter] = msg.angular_velocity.z
    # rr.set_time_sequence("step", msg_counter)
    rr.set_time_nanos("time", int(t))
    rr.log("acc/x", rr.Scalar(msg.linear_acceleration.x))
    rr.log("acc/y", rr.Scalar(msg.linear_acceleration.y))
    rr.log("acc/z", rr.Scalar(msg.linear_acceleration.z))
    # msg_type = get_message(type_map[topic])
    # msg = deserialize_message(data, msg_type)

    msg_counter += 1

print(f"{number_of_messages} IMU messages buffered!")
rr.connect()
# c = {}
# c["linear_acceleration_x"] = imu_data[0, :]
# c["linear_acceleration_y"] = imu_data[1, :]
# c["linear_acceleration_z"] = imu_data[2, :]
# c["angular_velocity_x"] = imu_data[3, :]
# c["angular_velocity_y"] = imu_data[4, :]
# c["angular_velocity_z"] = imu_data[5, :]
# c["Time"] = timeline[:]

# df = pd.DataFrame(c)

# hv_plot = hv.Curve(df, "Time", "linear_acceleration_x").opts(width=800)

# # display graph in browser
# # a bokeh server is automatically started
# bokeh_server = pn.Row(hv_plot).show(port=12345)

# # stop the bokeh server (when needed)
# bokeh_server.stop()



