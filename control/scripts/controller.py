#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    to_be_published = Twist()
    to_be_published.linear.x = 3 * data.axes[1]
    to_be_published.angular.z = 3 * data.axes[0]
    # Log what is being sent
    rospy.loginfo(f"Publishing: linear.x = {to_be_published.linear.x}, angular.z = {to_be_published.angular.z}")
    pub.publish(to_be_published)

def controller():
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('joy', Joy, callback)
    rospy.init_node('controller')
    rospy.loginfo("Joystick Controller Node Started")
    rospy.spin()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass