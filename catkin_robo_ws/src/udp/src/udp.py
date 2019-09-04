#! /usr/bin/env python

import socket
import jsonpickle

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String

motorsPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def udpServer():
    UDP_IP = "0.0.0.0"
    UDP_PORT = 11123

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        print "received message:", data
        payload = jsonpickle.decode(data)
        lineX = float(payload['strength'])
        angleZ = float(payload['angle'])
        twist = Twist()
        twist.linear.x = lineX * 0.5
        twist.angular.z = angleZ
        motorsPublisher.publish(twist)


def publisher():
    rospy.init_node('udp', anonymous=True, log_level=rospy.DEBUG)
    udpServer()
    rospy.loginfo("Server is running")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
