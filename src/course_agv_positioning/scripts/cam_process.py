#!/usr/bin/env python

import rospy
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
import geometry_msgs.msg
import time
import math
from cv2 import cv2 as cv2
from cv_bridge import CvBridge, CvBridgeError


class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        self.receiver = rospy.Subscriber(
            "/course_agv/camera/camera_raw", Image, self.callback)
        self.vel_pub = rospy.Publisher(
            '/course_agv/velocity',
            geometry_msgs.msg.Twist, queue_size = 1)
        self.cmd = geometry_msgs.msg.Twist()
        self.ball_center = []
        self.findball = 0
        self.x_diff = 0
        self.y_diff = 0
        self.linear_speed = 0
        self.angular_speed = 0

    def callback(self, data):
        lower_grey = np.array([0, 0, 46]) 
        upper_grey = np.array([180, 43, 220])
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        # cv2.imshow("camera_raw", cv_image)
        # cv2.waitKey(3)
        
        # Guess Processing
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
        # cv2.imshow("blurred", blurred)

        # bgr to hsv
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # clear everything except the target
        mask = cv2.inRange(hsv, lower_grey, upper_grey)
        # cv2.imshow("hsv_raw", mask)

        # lvbo
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # show the ball in 2 bits pic
        cv2.bitwise_not(mask, mask)
        cv2.imshow("not mask", mask)
        cv2.waitKey(3)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        self.findball = len(cnts)

        if self.findball:
            c = max(cnts, key = cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            try:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                self.ball_center = center
                self.x_diff = self.ball_center[0] - 399
                self.y_diff = self.ball_center[1] - 500
                
                if self.y_diff > -20:
                    self.y_diff = 0
                if self.x_diff < 10 and self.x_diff > -10:
                    self.x_diff = 0
                if radius > 35:
                    radius = 0
                else :
                    radius = 1
                self.linear_speed = radius * 0.2
                self.angular_speed = - self.signal(self.x_diff) * 0.1
                # print radius
            except:
                pass


        else:
            self.linear_speed = 0
            self.angular_speed = 0.2
            # print "stop"
            # print center
            # print radius
        # if radius > 5:
        #     cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        #     cv2.circle(frame, center, 5, (0, 0, 255), -1)


    def signal(self, num):
        if num >= 0:
            return 1
        else:
            return -1

    
    def pub(self):
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = self.angular_speed
        self.vel_pub.publish(self.cmd)


    def stop_robot(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.vel_pub.publish(self.cmd)

        
def main():
    node_name = "Camera_test"
    print "node name =", node_name
    rospy.init_node(node_name)
    rate = rospy.Rate(rospy.get_param('vel_rate', 200))
    try:
        c = Camera()
        while not rospy.is_shutdown():
            c.pub()
            rate.sleep()
    except rospy.ROSInternalException:
        pass


if __name__ == '__main__':
    main()

