#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

Kp = 0.4  # Proportional parameter
Ki = 0.01  # Integral parameter
Kd = 0.7  # Differential parameter

e_i = []  # A list store fixed amount of errors
error_prev = 0  # Linear speed previous state error initialization
error_curr = 0  # Linear speed current state error initialization
error_integ = 0  # Linear speed integral error initialization
error_diff = 0  # Linear speed differential error initialization

last_right_sensor_reading = 0.5
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # creates publisher class


# function to control velocity of robot
def forwards(speed, turn):
    global pub
    vel_x = Twist()  # initiates twist message
    vel_x.linear.x = speed  # sets linear speed of robot
    vel_x.angular.z = turn * 1.0  # sets angle to turn to pid function output (turns left?)
    pub.publish(vel_x)  # publishes velocity


def pid(sensor_value):
    global e_i  # brings e_i list within function scope
    global error_prev
    global error_curr
    global error_integ
    global error_diff

    global Kp
    global Ki
    global Kd

    distance = 0.7  # Set the desire distance from the wall

    error_curr = distance - sensor_value
    error_diff = error_curr - error_prev
    error_prev = error_curr
    sum_e_i = 0

    e_i.append(error_curr)

    if len(e_i) > 5:

        for i in range(5):
            sum_e_i += e_i[5 - i]

            print("e_i: ", e_i)
            print("Sum: ", sum_e_i)

        del e_i[0]

    else:
        pass

    pid_value = Kp * error_curr + Ki * sum_e_i + Kd * error_diff

    print("error_prev: ", error_prev, "error_curr: ", error_curr)
    print("sum_e_i: ", sum_e_i)
    print("error_diff", error_diff)

    # print("PID Output: ", pid_value)

    return pid_value


def callback(msg):
    global last_right_sensor_reading

    # Get the minimun distance from right direction range
    right_values = min(msg.ranges[499:579])

    if 0 < right_values < 10:

        last_right_sensor_reading = right_values

    else:
        last_right_sensor_reading = 0.5

    # print("---------- test 1----------", last_right_sensor_reading)

    print("=============================")
    print("Distance to the Wall: ", last_right_sensor_reading)
    print("=============================")


def controller():
    global last_right_sensor_reading

    rate = rospy.Rate(10)  # sleep in loop at rate 25Hz

    base_speed = 0.3  # the base linear speed

    while not rospy.is_shutdown():

        pid_value = pid(last_right_sensor_reading)
        print("PID Output: ", pid_value)

        if pid_value > 0:
            print("===== Turning Left =====")
        if pid_value < 0:
            print("===== Turning Right =====")

        forwards(base_speed, pid_value)

        rate.sleep()  # pause rate


if __name__ == "__main__":

    try:
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        controller()

    except rospy.ROSInterruptException:
        pass
