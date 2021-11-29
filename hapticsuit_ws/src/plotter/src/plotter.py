#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

class TFconverter():
    def __init__(self):
        self.jozi_pub = rospy.Publisher("/jozi", PoseStamped, queue_size=1)
        self.jozi_sub = rospy.Subscriber("/vicon/JOZI/JOZI", TransformStamped, self.jozi_callback)

        self.leveler_pub = rospy.Publisher("/leveler", PoseStamped, queue_size=1)
        self.leveler_sub = rospy.Subscriber("/vicon/LEVELER/LEVELER", TransformStamped, self.leveler_callback)

        self.base_pub = rospy.Publisher("/base", PoseStamped, queue_size=1)

        self.base_msg = PoseStamped()

    def jozi_callback(self, data):
        msg = PoseStamped()
        msg.pose.position.x = data.transform.translation.x
        msg.pose.position.y = data.transform.translation.y
        msg.pose.position.z = data.transform.translation.z
        
        msg.pose.orientation.x = data.transform.rotation.x
        msg.pose.orientation.y = data.transform.rotation.y
        msg.pose.orientation.z = data.transform.rotation.z
        msg.pose.orientation.w = data.transform.rotation.w

        msg.header.seq      = data.header.seq
        msg.header.stamp    = data.header.stamp
        msg.header.frame_id = data.header.frame_id

        self.base_msg.header.seq      = data.header.seq
        self.base_msg.header.stamp    = data.header.stamp
        self.base_msg.header.frame_id = data.header.frame_id

        self.jozi_pub.publish(msg)
        self.base_pub.publish(self.base_msg)

    def leveler_callback(self, data):
        msg = PoseStamped()
        msg.pose.position.x = data.transform.translation.x
        msg.pose.position.y = data.transform.translation.y
        msg.pose.position.z = data.transform.translation.z
        
        msg.pose.orientation.x = data.transform.rotation.x
        msg.pose.orientation.y = data.transform.rotation.y
        msg.pose.orientation.z = data.transform.rotation.z
        msg.pose.orientation.w = data.transform.rotation.w

        msg.header.seq      = data.header.seq
        msg.header.stamp    = data.header.stamp
        msg.header.frame_id = data.header.frame_id

        self.leveler_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('transform_to_TF_node')
    tfb = TFconverter()
    rospy.spin()