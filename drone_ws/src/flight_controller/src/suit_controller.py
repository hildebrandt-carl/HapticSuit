#!/usr/bin/python

import rospy
import sys
import time
import numpy as np

from freyja_msgs.msg import WaypointTarget
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray

class SuitController:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('suit_controller')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Init the drone and program state
        self._quit = False

        # Set the rate
        self.rate = 10.0
        self.dt = 1.0 / self.rate

        self.current_throttle = 5

        # Set the drone current position
        self.drone_pos = np.zeros(2)
        self.current_waypoint = np.zeros(2)

        # Init all the publishers and subscribers
        self.waypoint_sub = rospy.Subscriber("/discrete_waypoint_target", WaypointTarget, callback=self._waypoint_callback)
        self.drone_pos_sub = rospy.Subscriber("/vicon/STARK/STARK", TransformStamped, callback=self._GPS_callback)
        self.suit_control_pub = rospy.Publisher("/haptic_suit/control_signal", Float64MultiArray, queue_size=10)
        
        # Start the program
        time.sleep(2)
        self.start()

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        rospy.loginfo(str(rospy.get_name()) + ": " + str(msg))

    def _GPS_callback(self, msg):
        self.drone_pos[0] = msg.transform.translation.x 
        self.drone_pos[1] = msg.transform.translation.y 

    def _waypoint_callback(self, msg):
        self.current_waypoint[0] = msg.terminal_pe
        self.current_waypoint[1] = msg.terminal_pn

    def _send_command(self, angle1, angle2, throttle1, throttle2):
        self._log("Message sent: a1:{}  a2:{}  t1:{}  t2:{}".format(angle1, angle2, throttle1, throttle2))
        msg = Float64MultiArray()
        msg.data = [angle1, angle2, throttle1, throttle2] 
        self.suit_control_pub.publish(msg)

    def _mainloop(self):
        # Set the rate
        r = rospy.Rate(self.rate)

        at_point = False

        # Holds the distance readings
        distance_window_filter = np.full(25, np.inf)

        # While we are running the program
        while not self._quit:
            
            # Only if the current waypoint is not all zeros
            if not np.all(self.current_waypoint==0):
                # Compute distance to goal point
                distance = np.linalg.norm(self.drone_pos - self.current_waypoint)

                # Save the distance
                distance_window_filter = np.roll(distance_window_filter, 1)
                distance_window_filter[0] = distance
            
                # If the distance is small, send a new throttle command:
                if np.average(distance_window_filter) < 0.55:
                    if not at_point:
                        self._log("Arrived at goal")
                        self._send_command(90,-90, self.current_throttle, self.current_throttle)
                        self.current_throttle += 5
                        at_point = True
                else:
                    at_point = False

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = SuitController()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)