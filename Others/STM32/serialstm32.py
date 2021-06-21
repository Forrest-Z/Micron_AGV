#!/usr/bin/env python

import rospy
import serial
from geometry_msgs.msg import Twist

port = serial.Serial("/dev/STM32_1", baudrate=115200, timeout=3.0)


def cmdVelCallback(data):
    desired_voltage_out = data.linear.x
    if (data.linear.y == 1.0):
        desired_voltage_out += 10.0
    message_out = str(desired_voltage_out) + "\r\n"
    port.write(message_out.encode())


def stm32Serial():
    rospy.init_node('stm32serial', anonymous=True)
    rospy.Subscriber("/cmd_vel_out", Twist, cmdVelCallback)
    rospy.spin()


if __name__ == '__main__':
    stm32Serial()
