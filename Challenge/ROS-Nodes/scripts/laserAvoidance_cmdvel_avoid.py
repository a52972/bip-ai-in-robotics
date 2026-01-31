#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from jetbot_pro.cfg import laserAvoidanceConfig
from tf.transformations import euler_from_quaternion

class LaserFilter(object):
    def __init__(self):
        rospy.init_node("LidarFilter", anonymous=False)
        
        # --- Settings ---
        self.target_dist = 0.2
        self.max_linear = 0.1
        self.max_angular = 0.5 
        self.d_checkside = self.target_dist * 2
        
        # --- States ---
        self.state = 0      # 0: IDLE, 1: RIGHT_BYPASS, 2: LEFT_BYPASS
        self.substate = 0
        
        # --- Turn and Odometry Control ---
        self.robot_yaw = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Auxiliary variables to lock initial values of each maneuver
        self.target_yaw = 0.0
        self.turn_initialized = False
        self.start_move_x = 0.0
        self.start_move_y = 0.0
        self.dist_to_recover = 0.0 

        self.scan_data = None

        # --- ROS ---
        rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel_avoid", Twist, queue_size=10)
        
        Server(laserAvoidanceConfig, self.config_callback)   
        rospy.on_shutdown(self.cancel)

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.control_logic()
            r.sleep()

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, data):
        self.scan_data = data

    def get_dist(self, angle_deg):
        if self.scan_data is None or not self.scan_data.ranges: return 10.0
        index = int(angle_deg * (len(self.scan_data.ranges) / 360.0)) % len(self.scan_data.ranges)
        d = self.scan_data.ranges[index]
        return 10.0 if (math.isinf(d) or math.isnan(d) or d == 0.0) else d

    def check_sector(self, center_angle, width=20):
        start = int(center_angle - width/2)
        end = int(center_angle + width/2)
        min_d = 10.0
        for i in range(start, end):
            d = self.get_dist(i)
            if d < min_d: min_d = d
        return min_d

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    # AUXILIARY TURN FUNCTION
    def rotate_robot(self, cmd, angle_change):
        if not self.turn_initialized:
            self.target_yaw = self.robot_yaw + angle_change
            self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
            self.turn_initialized = True
            rospy.loginfo("Starting Turn: Target {:.2f}".format(self.target_yaw))

        yaw_error = self.angle_diff(self.target_yaw, self.robot_yaw)

        if abs(yaw_error) > 0.05:
            cmd.linear.x = 0.0
            speed = yaw_error * 2.0
            if speed > 0:
                cmd.angular.z = max(0.2, min(self.max_angular, speed))
            else:
                cmd.angular.z = min(-0.2, max(-self.max_angular, speed))
            return False
        else:
            cmd.angular.z = 0.0
            self.turn_initialized = False
            rospy.loginfo("Finishing turn.")
            return True

    
    # MAIN LOGIC
    def control_logic(self):
        if self.scan_data is None: return
        cmd = Twist()
        
        d_front = self.check_sector(0, 40)
        d_left = self.check_sector(90, 5)
        d_right = self.check_sector(270, 5)
        d_back_right = self.check_sector(260, 30) 
        d_back_left = self.check_sector(100, 30)

        # --- STATE 0: IDLE ---
        if self.state == 0:
            cmd.linear.x = 0
            
            if d_front < self.target_dist:
                rospy.loginfo("Obstacle Detected! Analyzing better path...")
                cmd.linear.x = 0.0
                self.substate = 0
                self.turn_initialized = False
                
                d_front_edge_left = self.check_sector(20, 15)
                d_front_edge_right = self.check_sector(340, 15)
                
                if d_front_edge_left < d_front_edge_right:
                    preference = 1 # Preference: Right
                else:
                    preference = 2 # Preference: Left

                can_go_left = (d_left > self.target_dist * 1.5)
                can_go_right = (d_right > self.target_dist * 1.5)

                # Final Decision Making
                if preference == 2: # Preference: Left
                    if can_go_left:
                        self.state = 2
                        rospy.loginfo("Decision: LEFT (Preference respected).")
                    elif can_go_right:
                        self.state = 1
                        rospy.loginfo("Decision: RIGHT (Left blocked).")
                    else:
                        self.state = 3
                        rospy.loginfo("Decision: NO WAY OUT. Initiating U-turn.")
                
                else: # preference == 1 (Preference: Right)
                    if can_go_right:
                        self.state = 1
                        rospy.loginfo("Decision: RIGHT (Preference respected).")
                    elif can_go_left:
                        self.state = 2
                        rospy.loginfo("Decision: LEFT (Right blocked).")
                    else:
                        self.state = 3
                        rospy.loginfo("Decision: NO WAY OUT. Initiating U-turn.")

        # --- STATE 1: DEVIATION TO THE RIGHT ---
        elif self.state == 1:
            
            # Sub 0: Rotate 90 to the Right
            if self.substate == 0:
                if self.rotate_robot(cmd, -math.pi/2): # -90 degrees
                    self.start_move_x = self.robot_x
                    self.start_move_y = self.robot_y
                    self.substate = 1
            
            # Sub 1: Move away from the object
            elif self.substate == 1:
                cmd.linear.x = self.max_linear
                dist = math.hypot(self.robot_x - self.start_move_x, self.robot_y - self.start_move_y)
                self.dist_to_recover = dist 
                
                # Moved away enough or sensor cleared
                if d_back_left > self.d_checkside and dist > 0.05:
                    self.substate = 2

            # Sub 2: Rotate 90 to the Left (Become Parallel)
            elif self.substate == 2:
                if self.rotate_robot(cmd, math.pi/2):
                    self.substate = 3
                    
            # Sub 3: Move forward until passing the object
            elif self.substate == 3:
                cmd.linear.x = self.max_linear
                rospy.loginfo("Front {:.2f} / Target {:.2f}".format(d_front, self.target_dist))
                if d_front < self.target_dist:
                    rospy.loginfo("ANOTHER OBJECT DETECTED! Returning to SUBSTATE 0")
                    self.substate = 0
                if d_back_left < self.d_checkside: # Object detected on the side
                    self.substate = 4
            
            # Sub 4: Continue moving forward until passing the object
            elif self.substate == 4:
                cmd.linear.x = self.max_linear
                rospy.loginfo("Front {:.2f} / Target {:.2f}".format(d_front, self.target_dist))
                if d_front < self.target_dist:
                    rospy.loginfo("ANOTHER OBJECT DETECTED! Returning to SUBSTATE 0")
                    self.substate = 0
                if d_back_left > self.d_checkside: # Object passed
                    self.substate = 5

            # Sub 5: Rotate 90 to the Left (Point to original route)
            elif self.substate == 5:
                if self.rotate_robot(cmd, math.pi/2):
                    self.start_move_x = self.robot_x
                    self.start_move_y = self.robot_y
                    self.substate = 6
            
            # Sub 6: Return to the original line
            elif self.substate == 6:
                cmd.linear.x = self.max_linear
                dist_now = math.hypot(self.robot_x - self.start_move_x, self.robot_y - self.start_move_y)
                
                if dist_now >= self.dist_to_recover * 0.9 and d_left < self.target_dist:
                    self.substate = 7

            # Sub 7: Rotate 90 to the Right (Final Alignment)
            elif self.substate == 7:
                if self.rotate_robot(cmd, -math.pi/2):
                    self.state = 0
                    self.substate = 0
                    
        # --- STATE 2: DEVIATION TO THE LEFT ---
        elif self.state == 2:
            
            # Sub 0: Rotate 90 to the Left (Start of deviation)
            if self.substate == 0:
                if self.rotate_robot(cmd, math.pi/2): # +90 degrees
                    self.start_move_x = self.robot_x
                    self.start_move_y = self.robot_y
                    self.substate = 1
            
            # Sub 1: Move away from the object
            elif self.substate == 1:
                cmd.linear.x = self.max_linear
                dist = math.hypot(self.robot_x - self.start_move_x, self.robot_y - self.start_move_y)
                self.dist_to_recover = dist 
                
                if d_back_right > self.d_checkside and dist > 0.05:
                    self.substate = 2

            # Sub 2: Rotate 90 to the Right (Become Parallel)
            elif self.substate == 2:
                if self.rotate_robot(cmd, -math.pi/2):
                    self.substate = 3
                    
            # Sub 3: Move forward until detecting the object on the side
            elif self.substate == 3:
                cmd.linear.x = self.max_linear
                rospy.loginfo("Front {:.2f} / Target {:.2f}".format(d_front, self.target_dist))
                if d_front < self.target_dist:
                    rospy.loginfo("ANOTHER OBJECT DETECTED! Returning to SUBSTATE 0")
                    self.substate = 0
                if d_back_right < self.d_checkside:
                    self.substate = 4
            
            # Sub 4: Move forward until passing the object
            elif self.substate == 4:
                cmd.linear.x = self.max_linear
                rospy.loginfo("Front {:.2f} / Target {:.2f}".format(d_front, self.target_dist))
                if d_front < self.target_dist:
                    rospy.loginfo("ANOTHER OBJECT DETECTED! Returning to SUBSTATE 0")
                    self.substate = 0
                if d_back_right > self.d_checkside:
                    self.substate = 5

            # Sub 5: Rotate 90 to the Right (Point to original route)
            elif self.substate == 5:
                if self.rotate_robot(cmd, -math.pi/2):
                    self.start_move_x = self.robot_x
                    self.start_move_y = self.robot_y
                    self.substate = 6
            
            # Sub 6: Return to the original line
            elif self.substate == 6:
                cmd.linear.x = self.max_linear
                dist_now = math.hypot(self.robot_x - self.start_move_x, self.robot_y - self.start_move_y)
                
                if dist_now >= self.dist_to_recover * 0.9 and d_right < self.target_dist:
                    self.substate = 7

            # Sub 7: Rotate 90 to the Left (Final Alignment)
            elif self.substate == 7:
                if self.rotate_robot(cmd, math.pi/2):
                    self.state = 0
                    self.substate = 0

        # --- STATE 3: U-TURN ---
        elif self.state == 3:
            if self.rotate_robot(cmd, math.pi):
                rospy.loginfo("U-turn complete. Returning to IDLE.")
                self.state = 0
                self.substate = 0

        self.cmd_pub.publish(cmd)

    def config_callback(self, config, level):
        self.target_dist = config['distance']
        self.max_linear = config['linear']  
        self.max_angular = config['angular']
        return config

    def cancel(self):
        self.cmd_pub.publish(Twist())

if __name__ == "__main__":
    LaserFilter()