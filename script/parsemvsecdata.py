#!/usr/bin/python

import argparse
import rosbag
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import os

sec = 1504645177
nsecs = 425404305

# arguments
parser = argparse.ArgumentParser()
parser.add_argument("bag", help="ROS bag file to extract")
parser.add_argument("--levent_topic", default="/davis/left/events", help="Event Topic")
parser.add_argument("--revent_topic", default="/davis/right/events", help="Event Topic")
parser.add_argument("--limage_topic", default="/davis/left/image_raw", help="Image Topic")
parser.add_argument("--rimage_topic", default="/davis/right/image_raw", help="Image Topic")
parser.add_argument("--limu_topic", default="/davis/left/imu", help="IMU Topic")
parser.add_argument("--rimu_topic", default="/davis/right/imu", help="IMU Topic")
parser.add_argument("--lviimage_topic", default="/visensor/left/image_raw", help="VIsensor Image Topic")
parser.add_argument("--rviimage_topic", default="/visensor/right/image_raw", help="VIsensor Image Topic")
parser.add_argument("--viimu_topic", default="/visensor/imu", help="IMU Topic")
parser.add_argument("--reset_time", help="Remove the Time offset")
args = parser.parse_args()


def get_lfilename(i):
    return "limages/frmae_" + str(i).zfill(8) + ".png"


def get_rfilename(i):
    return "rimages/frmae_" + str(i).zfill(8) + ".png"


def get_lvifilename(i):
    return "lviimages/frmae_" + str(i).zfill(8) + ".png"


def get_rvifilename(i):
    return "rviimages/frmae_" + str(i).zfill(8) + ".png"


def timestamp_str(ts):
    return str(ts.secs) + "." + str(ts.nsecs).zfill(9)


if not os.path_exists("limages"):
    os.makedirs("limages")

if not os.path_exists("rimages"):
    os.makedirs("rimages")

if not os.path_exists("lviimages"):
    os.makedirs("lviimages")

if not os.path_exists("rviimages"):
    os.makedirs("rviimages")

limage_index = 0
rimage_index = 0
lviimage_index = 0
rviimage_index = 0
levent_sum= 0
revent_sum= 0
limu_msg_sum = 0
rimu_msg_sum = 0
viimu_msg_sum = 0

levents_file = open('events_L.txt', 'w')
revents_file = open('events_R.txt', 'w')
limages_file = open('images_L.txt', 'w')
rimages_file = open('images_R.txt', 'w')
lviimages_file = open('viimages_L.txt', 'w')
rviimages_file = open('viimages_R.txt', 'w')
limu_file = open('limu.txt', 'w')
rimu_file = open('rimu.txt', 'w')
viimu_file = open('vi_imu.txt', 'w')
reset_time = rospy.Time()

with rosbag.Bag(args.bag, 'r') as bag:
    reset_time.secs = sec
    reset_time.nsecs = nsecs
    print("Reset time: " + timestamp_str(reset_time))


    for topic, msg, t in bag.read_messages():
        # Images

        if topic == args.limage_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print(e)
            limages_file.write(timestamp_str(msg.header.stamp -reset_time)+" ")
            limages_file.write(get_lfilename(limage_index)+"\n")

            cv2.imwrite(get_lfilename(limage_index),cv_image)
            limage_index = limage_index + 1

        if topic == args.rimage_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print(e)
            rimages_file.write(timestamp_str(msg.header.stamp -reset_time)+" ")
            rimages_file.write(get_rfilename(rimage_index)+"\n")

            cv2.imwrite(get_rfilename(rimage_index),cv_image)
            rimage_index = rimage_index + 1

        if topic == args.lviimage_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print(e)
            lviimages_file.write(timestamp_str(msg.header.stamp -reset_time)+" ")
            lviimages_file.write(get_lvifilename(lviimage_index)+"\n")

            cv2.imwrite(get_lvifilename(lviimage_index),cv_image)
            lviimage_index = lviimage_index + 1

        if topic == args.rviimage_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print(e)
            rviimages_file.write(timestamp_str(msg.header.stamp -reset_time)+" ")
            rviimages_file.write(get_rvifilename(rviimage_index)+"\n")

            cv2.imwrite(get_rvifilename(rviimage_index),cv_image)
            rviimage_index = rviimage_index + 1

        # Events

        if topic == args.levents_topic:
            for e in msg.events:
                levents_file.write(str(e.x)+" ")
                levents_file.write(str(e.y)+" ")
                levents_file.write(("1" if e.polarity else "0")+"\n")
                levent_sum = levent_sum + 1

        if topic == args.levents_topic:
            for e in msg.events:
                levents_file.write(str(e.x)+" ")
                levents_file.write(str(e.y)+" ")
                levents_file.write(("1" if e.polarity else "0")+"\n")
                levent_sum = levent_sum + 1

        # IMU

        if topic == args.limu_topic:
            limu_file.write(timestamp_str(msg.header.stamp-reset_time)+" ")
            limu_file.write(str(msg.linear_acceleration.x)+" ")
            limu_file.write(str(msg.linear_acceleration.y)+" ")
            limu_file.write(str(msg.linear_acceleration.z)+" ")
            limu_file.write(str(msg.angular_velocity.x)+" ")
            limu_file.write(str(msg.angular_velocity.y)+" ")
            limu_file.write(str(msg.angular_velocity.z)+" ")
            limu_msg_sum = limu_msg_sum + 1

        if topic == args.rimu_topic:
            rimu_file.write(timestamp_str(msg.header.stamp-reset_time)+" ")
            rimu_file.write(str(msg.linear_acceleration.x)+" ")
            rimu_file.write(str(msg.linear_acceleration.y)+" ")
            rimu_file.write(str(msg.linear_acceleration.z)+" ")
            rimu_file.write(str(msg.angular_velocity.x)+" ")
            rimu_file.write(str(msg.angular_velocity.y)+" ")
            rimu_file.write(str(msg.angular_velocity.z)+" ")
            rimu_msg_sum = limu_msg_sum + 1

        if topic == args.viimu_topic:
            viimu_file.write(timestamp_str(msg.header.stamp-reset_time)+" ")
            viimu_file.write(str(msg.linear_acceleration.x)+" ")
            viimu_file.write(str(msg.linear_acceleration.y)+" ")
            viimu_file.write(str(msg.linear_acceleration.z)+" ")
            viimu_file.write(str(msg.angular_velocity.x)+" ")
            viimu_file.write(str(msg.angular_velocity.y)+" ")
            viimu_file.write(str(msg.angular_velocity.z)+" ")
            viimu_msg_sum = viimu_msg_sum + 1

levents_file.close()
revents_file.close()
limages_file.close()
rimages_file.close()
lviimages_file.close()
rviimages_file.close()
limu_file.close()
rimu_file.close()
viimu_file.close()

