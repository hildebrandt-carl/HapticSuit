#!/usr/bin/python

import rospy
import sys
import time
import numpy as np

from freyja_msgs.msg import ReferenceState
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped

class FlightPathControllerNode:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('flight_path_node')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Define the height you want to hover at
        height = rospy.get_param('/{}/desired_height'.format(rospy.get_name()), 1)
        self.hover_height = height

        # Set the rate
        self.rate = 20 #Hz
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False

        # Init all the publishers and subscribers
        self.reference_pub = rospy.Publisher("/reference_state", ReferenceState, queue_size=10)

        # Used for debug
        self.drone_pos_sub = rospy.Subscriber("/vicon/JOZI/JOZI", TransformStamped, callback=self._GPS_callback)
        self.debug_reference = rospy.Publisher("/debug/reference_state", PointStamped, queue_size=10)
        self.debug_drone_position = rospy.Publisher("/debug/drone_position", PointStamped, queue_size=10)
        
        # Start the program
        time.sleep(2)
        self.start()

    def _GPS_callback(self, msg):
        new_msg = PointStamped()
        new_msg.header.frame_id = "world"
        new_msg.point.x = msg.transform.translation.x 
        new_msg.point.y = msg.transform.translation.y 
        new_msg.point.z = msg.transform.translation.z 
        self.debug_drone_position.publish(new_msg)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _send_waypoint(self, x, y, z, x1=0, y1=0, z1=0):
        msg = ReferenceState()
        msg.pn = x
        msg.pe = y
        msg.pd = -z
        msg.vn = x1
        msg.ve = y1
        msg.vd = z1
        msg.yaw = 0.0
        msg.an = 0.0
        msg.ae = 0.0
        msg.ad = 0.0

        self.reference_pub.publish(msg)
        self._log("Waypoint sent")

        # Create the debug msg
        new_msg = PointStamped()
        new_msg.header.frame_id = "world"
        new_msg.point.x = y
        new_msg.point.y = x
        new_msg.point.z = z
        self.debug_reference.publish(new_msg)

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        # Start with a 5 seconds wait
        counter = -self.rate * 5
        x_pos = 0
        y_pos = 0
        z_pos = 0.01
        x_dir = 0
        y_dir = 0

        # Used to control the speed (10 seconds per full rotation)
        time_array = np.linspace(0, 2*np.pi, 20 * self.rate)
        time_index = 0

        while not self._quit:
        
            # Send the waypoint
            self._send_waypoint(x=x_pos, y=y_pos, z=z_pos, x1=x_dir, y1=y_dir)

            if counter > 0:
                z_pos += 0.1 / self.rate
                z_pos = min(z_pos, self.hover_height)

            # This just helps wait 5 seconds
            counter += 1

            # Once we have reached the correct altitude
            if z_pos >= self.hover_height:
                radius = 0.1
                theta = time_array[time_index]
                x_pos = radius*np.cos(theta)
                y_pos = np.sin(theta)
                x_dir = -1 * radius * np.sin(theta)
                y_dir = np.cos(theta)
                time_index += 1
                if time_index >= len(time_array):
                    time_index = 0 

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = FlightPathControllerNode()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)
