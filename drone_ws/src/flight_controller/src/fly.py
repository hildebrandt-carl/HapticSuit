#!/usr/bin/python

import rospy
import sys
import time

from enum import Enum
from freyja_msgs.msg import WaypointTarget

class FlyTester:
    def __init__(self):
        # Init the ROS node
        rospy.init_node('tester_node')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 10.0
        self.dt = 1.0 / self.rate

        # Declare the control parameters
        self.moving_time = 5
        self.waiting_time = 2
        self.loops = 4

        # Init the drone and program state
        self._quit = False

        # Init all the publishers and subscribers
        self.attitude_pub = rospy.Publisher("/discrete_waypoint_target", WaypointTarget, queue_size=10)
        
        # Start the program
        time.sleep(2)
        self.start()

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _send_waypoint(self, x, y, z, time):
        msg = WaypointTarget()
        msg.terminal_pn = x
        msg.terminal_pe = y
        msg.terminal_pd = -z
        msg.terminal_vn = 0.0
        msg.terminal_ve = 0.0
        msg.terminal_vd = 0.0
        msg.terminal_yaw = 0.0

        msg.allocated_time = time
        msg.translational_speed = 1.0

        # Get there in a set amount of time
        msg.waypoint_mode = 0

        self.attitude_pub.publish(msg)
        self._log("Waypoint sent")

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        # Back and fourth counter
        back_fourth_counter = 0
        landing_counter = 0

        sent = False
        # Start with a 10 seconds wait
        counter = -self.rate * 10
        x_pos = 0
        y_pos = 0
        z_pos = 1

        while not self._quit:
        
            if back_fourth_counter < self.loops * 2:
                # Wait five seconds
                counter += 1
                if counter > self.rate * (self.waiting_time + self.moving_time):
                    counter = 0
                    if x_pos == 0:
                        x_pos = 1
                        sent = False
                    else:
                        x_pos = -1 * x_pos
                        sent = False

            # Landing mode
            else:
                landing_counter += 1
                self.moving_time = 3
                if landing_counter < self.rate * 10:
                    x_pos = 0
                    sent = False
                else:
                    x_pos = 0
                    z_pos = 0.5
                    sent = False

            # Send the waypoint
            if not sent:
                self._send_waypoint(x=x_pos,y=y_pos,z=z_pos,time=self.moving_time)
                sent = True
                back_fourth_counter += 1

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    try:
        x = FlyTester()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)
