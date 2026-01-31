#!/usr/bin/env python3
"""mux for /cmd_vel.

inputs:
  /cmd_vel_ai     
  /cmd_vel_avoid  
otput:
  /cmd_vel        

policy:
  if /cmd_vel_avoid received adn is not zero -> forward it.
  else forward /cmd_vel_ai.
"""

import rospy
from geometry_msgs.msg import Twist

class CmdVelMux:
    def __init__(self):
        self.avoid_timeout = float(rospy.get_param("~avoid_timeout", 0.25))

        self.last_ai = Twist()
        self.last_avoid = Twist()
        self.t_ai = rospy.Time(0)
        self.t_avoid = rospy.Time(0)

        rospy.Subscriber("/cmd_vel_ai", Twist, self.cb_ai, queue_size=1)
        rospy.Subscriber("/cmd_vel_avoid", Twist, self.cb_avoid, queue_size=1)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rate_hz = float(rospy.get_param("~rate", 30.0))
        self.rate = rospy.Rate(rate_hz)

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()

    def cb_ai(self, msg: Twist):
        self.last_ai = msg
        self.t_ai = rospy.Time.now()

    def cb_avoid(self, msg: Twist):
        self.last_avoid = msg
        self.t_avoid = rospy.Time.now()

    def step(self):
        now = rospy.Time.now()
        avoid_recent = (now - self.t_avoid).to_sec() < self.avoid_timeout
        use_avoid = avoid_recent and self._nonzero(self.last_avoid)
        self.pub.publish(self.last_avoid if use_avoid else self.last_ai)

    @staticmethod
    def _nonzero(t: Twist) -> bool:
        return abs(t.linear.x) > 1e-3 or abs(t.angular.z) > 1e-3

    def on_shutdown(self):
        self.pub.publish(Twist())

def main():
    rospy.init_node("cmd_vel_mux")
    CmdVelMux()

if __name__ == "__main__":
    main()
