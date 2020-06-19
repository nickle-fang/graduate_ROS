#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import geometry_msgs.msg
import time
import math


angle_table = {0: 0.0, 1: 90.0, 2: 180.0, 3: 270.0}

class Position:
    distance = 0.0
    def __init__(self):
        self.receiver = rospy.Subscriber(
            "/course_agv/laser/scan", LaserScan, self.callback)
        self.vel_pub = rospy.Publisher(
            '/course_agv/velocity',
            geometry_msgs.msg.Twist, queue_size = 1)
        self.cmd = geometry_msgs.msg.Twist()
    
    def callback(self, data):
        # print("distance = ", sum(data.ranges) / 10)
        self.distance = sum(data.ranges) / 10

    def logdata(self):
        return self.distance

    def publish(self, speed):
        self.cmd.linear.x = speed
        self.cmd.angular.z = 0
        self.vel_pub.publish(self.cmd)
    
    def stop(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.vel_pub.publish(self.cmd)


class Orient:
    intergral = 0.0
    angle_now = 10.0
    angular_vel = 0.0
    def __init__(self):
        self.receiver = rospy.Subscriber(
            "/course_agv/imu", Imu, self.callback)
        self.vel_pub = rospy.Publisher(
            '/course_agv/velocity',
            geometry_msgs.msg.Twist, queue_size = 1)
        self.cmd = geometry_msgs.msg.Twist()

    def callback(self, data):
        
        # Read the quaternion of the robot imu
        imu_x = data.orientation.x
        imu_y = data.orientation.y
        imu_z = data.orientation.z
        imu_w = data.orientation.w

        # Read the angular velocity
        self.angular_vel = data.angular_velocity.z

        # Convert quaternions to Euler-Angles
        rpy_angle = [0, 0, 0]
        rpy_angle[0] = math.atan2(2 * (imu_w * imu_x + imu_y * imu_z), 1 - 2 * (imu_x**2 + imu_y**2))
        rpy_angle[1] = math.asin(2 * (imu_w * imu_y - imu_z * imu_x))
        rpy_angle[2] = math.atan2(2 * (imu_w * imu_z + imu_x * imu_y), 1 - 2 * (imu_y**2 + imu_z**2))
        # print(rpy_angle)
        self.angle_now = rpy_angle[2] * 180 / 3.14159 + 180

    def is_stop(self):
        if (self.angular_vel > -0.01 and self.angular_vel < 0.01):
            return 1

    def publish(self, aim_angle):
        pid_p = 20
        pid_i = 0.05
        
        difference = aim_angle - self.angle_now
        if (difference > 180):
            difference = difference - 360
        if (difference < -180):
            difference = difference + 360
        if (difference < 1 and difference > -1):
            difference = 0
        self.intergral = self.intergral + difference
        pid_out = pid_p * difference + pid_i * self.intergral
        pid_out = pid_out / 1000
        # print("raw_pid_out =", pid_out)
        # print("intergral = ", self.intergral)
        # print("angle now is ", self.angle_now)
        if (pid_out > 3):
            pid_out = 3
        if (pid_out < -3):
            pid_out = -3

        self.cmd.linear.x = 0
        # pid_out = 0 # emergency stop
        self.cmd.angular.z = pid_out
        self.vel_pub.publish(self.cmd)
        # print("diffenrence = ", difference)
        # print("pid_out = ",pid_out)
        return difference


def go_to_position(now_pos, des_pos, o, p, rate):
    aim_vector = [des_pos[0] - round(now_pos[2], 2),  des_pos[1] - round(now_pos[3], 2)]
    if aim_vector[0] == 0:
        aim_vector[0] = 0.01
    
    aim_theta = math.atan(aim_vector[1] / aim_vector[0])
    aim_theta = math.degrees(aim_theta)
    if (aim_vector[0] < 0 and aim_vector[1] < 0):
        aim_theta = aim_theta + 180
    if (aim_vector[0] < 0 and aim_vector[1] > 0):
        aim_theta = aim_theta + 180

    print "Adjusting the angle now......"
    while not rospy.is_shutdown():
        if o.publish(aim_theta) < 0.01 and o.publish(aim_theta) > -0.01 and o.is_stop():
            # print("Angle is set correctly")
            p.stop()
            time.sleep(2)
            break
        rate.sleep()

    # go to position
    print "Going to position now......"
    aim_distance = math.sqrt((aim_vector[0]**2 )+ (aim_vector[1]**2))
    current_speed = 0.01
    if aim_distance > 4.35:
        while current_speed <= 0.3:
            p.publish(current_speed)
            current_speed = current_speed + 0.01
            time.sleep(0.5)
        
        unchanged_vel_times = (aim_distance - 4.35) / 0.3
        for i in range(int(unchanged_vel_times)):
            p.publish(0.3)
            time.sleep(1)

        while current_speed > 0:
            p.publish(current_speed)
            current_speed = current_speed - 0.01
            time.sleep(0.5)
    else:
        # print "The distance is too short! The robot has to move slowly!"
        for i in range(int(aim_distance / 0.01)):
            p.publish(0.1)
            time.sleep(0.1)
        p.publish(0)
    # print "Position arrived"


def measure_position(pos_now, angle_flag, o, p, rate):
    print "Measuring the position now......"
    angle_flag = 0
    while not rospy.is_shutdown():
        difference = o.publish(angle_flag * 90.0)
        if (difference > -0.8 and difference < 0.8 and o.is_stop()):
            pos_now[angle_flag] = p.logdata()
            angle_flag = angle_flag + 1
        if (angle_flag == 4):
            # print "The position now is", (round(pos_now[2], 2), round(pos_now[3], 2))
            break
        rate.sleep()
    

def main():
    global aim_angle
    aim_angle = 0.0
    aim_pos = [3.0, 3.0]
    angle_flag = 0
    pos_now = [1.0, 1.0, 1.0, 1.0]

    node_name = "course_agv_positioning"
    print "node =", node_name
    try:
        rospy.init_node(node_name)
        o = Orient()
        p = Position()
        rate = rospy.Rate(rospy.get_param('vel_rate', 200))
        
        # measure the position now and print it
        measure_position(pos_now, angle_flag, o, p, rate)
        print "The position now is", (round(pos_now[2], 2), round(pos_now[3], 2))

        aim_pos = input("Please set the destination: \n")

        go_to_position(pos_now, aim_pos, o, p, rate)

        # test the aim position
        measure_position(pos_now, angle_flag, o, p, rate)

        # second go to position
        while math.sqrt(((aim_pos[0] - pos_now[2])**2 )+ ((aim_pos[1] - pos_now[3])**2)) >= 0.1:
            go_to_position(pos_now, aim_pos, o, p, rate)
            measure_position(pos_now, angle_flag, o, p, rate)

        p.stop()
        print "Destination arrived! The position now is", (round(pos_now[2], 2), round(pos_now[3], 2))
            

    except rospy.ROSInternalException:
        pass

if __name__ == '__main__':
    main()
