#!/usr/bin/env python
import tf
import math
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped

def get_euler(data):
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler

class converter():
    def __init__(self):
        self.jozi_pub = rospy.Publisher("/jozi_euler", Vector3, queue_size=1)
        self.jozi_sub = rospy.Subscriber("/jozi", PoseStamped, self.jozi_callback)

        self.leveler_pub = rospy.Publisher("/leveler_euler", Vector3, queue_size=1)
        self.leveler_sub = rospy.Subscriber("/leveler", PoseStamped, self.leveler_callback)

        self.base_sub = rospy.Subscriber("/base", PoseStamped, self.base_callback)

        # Create the filters
        window_size = 10
        self.jozi_window_x        = np.zeros(window_size)
        self.leveler_window_x     = np.zeros(window_size)
        self.base_window_x        = np.zeros(window_size)

        self.jozi_window_y        = np.zeros(window_size)
        self.leveler_window_y     = np.zeros(window_size)
        self.base_window_y        = np.zeros(window_size)

        self.jozi_window_z        = np.zeros(window_size)
        self.leveler_window_z     = np.zeros(window_size)
        self.base_window_z        = np.zeros(window_size)

    def jozi_callback(self, data):
        # Convert to euler
        euler = get_euler(data)

        # Save to filters
        self.jozi_window_x      = np.roll(self.jozi_window_x, 1)
        self.jozi_window_x[0]   = math.degrees(euler[0])
        self.jozi_window_y      = np.roll(self.jozi_window_y, 1)
        self.jozi_window_y[0]   = math.degrees(euler[1])
        self.jozi_window_z      = np.roll(self.jozi_window_z, 1)
        self.jozi_window_z[0]   = math.degrees(euler[2])

        # Get the result
        x = np.average(self.jozi_window_x)
        y = np.average(self.jozi_window_y)
        z = np.average(self.jozi_window_z)
        
        # Publish for plotting
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z
        self.jozi_pub.publish(msg)   

    def leveler_callback(self, data):
        # Convert to euler
        euler = get_euler(data)

        # Save to filters
        self.leveler_window_x      = np.roll(self.leveler_window_x, 1)
        self.leveler_window_x[0]   = math.degrees(euler[0]) - 9
        self.leveler_window_y      = np.roll(self.leveler_window_y, 1)
        self.leveler_window_y[0]   = math.degrees(euler[1])
        self.leveler_window_z      = np.roll(self.leveler_window_z, 1)
        self.leveler_window_z[0]   = math.degrees(euler[2])

        # Get the result
        x = np.average(self.leveler_window_x)
        y = np.average(self.leveler_window_y)
        z = np.average(self.leveler_window_z)

        # Publish for plotting
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z
        self.leveler_pub.publish(msg) 

    def base_callback(self, data):
        # Convert to euler
        euler = get_euler(data)

        self.base_window_x      = np.roll(self.base_window_x, 1)
        self.base_window_x[0]   = math.degrees(euler[0])
        self.base_window_y      = np.roll(self.base_window_y, 1)
        self.base_window_y[0]   = math.degrees(euler[1])
        self.base_window_z      = np.roll(self.base_window_z, 1)
        self.base_window_z[0]   = math.degrees(euler[2])

        # Get the result
        x = np.average(self.base_window_x)
        y = np.average(self.base_window_y)
        z = np.average(self.base_window_z)

        # Publish for plotting
        msg = Vector3()
        msg.x = x
        msg.y = y
        msg.z = z

if __name__ == '__main__':
    rospy.init_node('converted')
    tfb = converter()
    rospy.spin()