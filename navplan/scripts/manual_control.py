#!/usr/bin/env python3
import numpy as np
import rospy
from pynput.keyboard import Listener

from std_msgs.msg import Float64

rospy.init_node('manual_control')
long_drive_pub = rospy.Publisher('controls/throttle', Float64, queue_size=10)
steering_drive_pub = rospy.Publisher('controls/steer', Float64, queue_size=10)

manual_control_torque = 1
manual_control_steering_angle = 15


def on_press(key):
    key = str(key)[1]

    if key == 'd':
        steering_drive_pub.publish(Float64(-1 * manual_control_steering_angle))
    if key == 'a':
        steering_drive_pub.publish(Float64(manual_control_steering_angle))
    if key == 'w':
        long_drive_pub.publish(Float64(manual_control_torque))
    if key == 's':
        long_drive_pub.publish(Float64(-1 * manual_control_torque))


def on_release(key):
    key = str(key)[1]

    if key == 'd' or key == 'a':
        steering_drive_pub.publish(Float64(0))
    if key == 'w' or key == 's':
        long_drive_pub.publish(Float64(0))


with Listener(on_press=on_press, on_release=on_release) as listener:
    print("Listening for keys")
    listener.join()