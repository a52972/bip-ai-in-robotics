#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
import socket
import json

class UdpToCmdVelAI(object):
    def __init__(self):
        self.port = int(rospy.get_param("~port", 5005))
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.timeout_s = float(rospy.get_param("~timeout", 0.5))

        self.pub = rospy.Publisher("/cmd_vel_ai", Twist, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.05)

        self.last_msg_time = rospy.Time(0)
        self.last_cmd = Twist()

        rospy.loginfo("udp_to_cmdvel_ai listening on UDP port %d", self.port)

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                data, _addr = self.sock.recvfrom(2048)
                msg = json.loads(data.decode("utf-8"))

                cmd = Twist()
                cmd.linear.x = float(msg.get("linear_x", 0.0))
                cmd.angular.z = float(msg.get("angular_z", 0.0))

                self.last_cmd = cmd
                self.last_msg_time = rospy.Time.now()

            except socket.timeout:
                pass
            except Exception as e:
                rospy.logwarn_throttle(2.0, "UDP parse error: %s", str(e))

            # safety: if no packets recently, publish zero
            if (rospy.Time.now() - self.last_msg_time).to_sec() > self.timeout_s:
                safe = Twist()
                self.pub.publish(safe)
            else:
                self.pub.publish(self.last_cmd)

            r.sleep()

if __name__ == "__main__":
    rospy.init_node("udp_to_cmdvel_ai")
    UdpToCmdVelAI().spin()
