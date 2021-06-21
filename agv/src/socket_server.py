#!/usr/bin/env python

import rospy
import socket
import sys
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

rospy.init_node('tx2_communication_server', anonymous=True)
# pub = rospy.Publisher('obstacle_distance', Float64, queue_size=1)
pub = rospy.Publisher('camera_obstacles', Float64MultiArray, queue_size=1)


class Server():
    def __init__(self, ip_addr, port_num):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip_addr, port_num)
        self.sock.bind(server_address)
        self.sock.listen(1)

    def run(self):
        print('waiting for a connection')
        self.connection, client_address = self.sock.accept()
        print('connected')
        while True:
            data = self.connection.recv(1024)
            if data:
                msg = data.decode("utf-8")
                print(msg)
                split_msg = msg.split(",")

                class_str = split_msg[1]
                confidence_str = split_msg[2]
                x_str = split_msg[3]
                y_str = split_msg[4]
                w_str = split_msg[3]
                h_str = split_msg[4]
                distance_str = split_msg[5]

                # pub.publish(distance_string)
                each = Float64MultiArray()
                each.data = (float(class_str),float(confidence_str),float(x_str),float(y_str),float(w_str),float(h_str),float(distance_str))
                pub.publish(each)
                # with open("/home/agv/catkin_ws/src/obstacles_log/socket.csv", "a") as log:
                #     log.write(msg)

    def stop(self):
        self.connection.close()
        self.shutdown.set()


if __name__ == '__main__':
    # if len(sys.argv) != 5:
    #     print('Invalid number of arguments')
    #     sys.exit()

    # ip_addr = sys.argv[1]
    # port_num = int(sys.argv[2])

    ip_addr = '10.169.53.208' #'localhost'
    port_num = 8889
    my_server = Server(ip_addr, port_num)
    my_server.run()
