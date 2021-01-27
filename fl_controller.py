#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub_ = None

points = []

regions_ = {
    'right': 0,
    'front_right': 0,
    'front': 0,
    'front_left': 0,
    'left': 0,
}

L1 = 0.0
L2 = 0.0
M1 = 0.0
M2 = 0.0
H1 = 0.0
H2 = 0.0


class MEMBERSHIP_FUNCTION:

    def __init__(self, points, label):

        self.points = points # critical points in the fuzzy diagram
        self.label = label   # labels of fuzzy diagram, near, med, far or extra

    def membership_value(self, laser_value):
        
        # Membership Function 
        
        if laser_value <= self.points[0] or laser_value >= self.points[3]:
            return 0.0
        elif laser_value >= self.points[1] and laser_value <= self.points[2]:
            return 1.0
        elif laser_value >= self.points[0] and laser_value < self.points[1]:
            return (laser_value - self.points[0]) / (self.points[1] - self.points[0])
        else:
            return (self.points[3] - laser_value) / (self.points[3] - self.points[2])

        # elif laser_value > self.points[2] and laser_value <= self.points[3]:
            #return (self.points[3] - laser_value) / (self.points[3] - self.points[2])
       # else:
        #    return 0.0


class Fuzzy_WF:

    
    def __init__(self):
        self.nouse = 0.0
        print("Fuzzy construction")
       

    
    # Get the distance between the robot and the Nearst object within the direction range
    # We are taking 5 directions of sensors here, right, left, front right and front left

    def callback_laser(self, msg):

        global regions_

        '''
        The laser subscriber returns a msg
            *the msg.ranges[] is an array of 720 distances
            *the laser is broken down into 0.5 degrees intervals in and anti-clockwise direction
        
        For wall following:
            we only take right and front_right sensor values
            which represent the distance from robot to the objects (wall)
            and we took the minimum value, the nearest object in this laser detecting range

            PROBLEM:
                how to represent front sensor range 

            In order to get a better reading, we can not set the range too small, because that will
            easily return a inf value (as we are only taking 2 senser here), so if the range too small
            it might miss the object, which will cause some trouble (membership value got 0)
            Vice versa, we can not set the range too large, hense we have to tune it and test to get a good one

            For making the distance seting easier, use the Pythagorean Theorem (Gou Gu, 5, 12, 13)
            in this case , the front_right laser angel is 64.87, in the laser range around [669]

            We first tried use range of 30, [519:549], not turning very well 


        '''
        regions_ = {
            'front': min(msg.ranges[0:40],msg.ranges[679:719]),

            'right': min(msg.ranges[499:579]),
            'front_right': min(msg.ranges[649:689]),

            'front_left': min(msg.ranges[30:100]),
            'left': min(msg.ranges[160:220]),
        }
        # print("=======================test_1=======================")
        

    def Rule_Base(self, a, b, c, d, e, f):

        global L1
        global L2
        global M1
        global M2
        global H1
        global H2

        '''
        Get the labels of the fuzzy diagram (Low, Medium and Far)

        L1 and L2 stands for Near to the object(wall)
        M1 and M2 stands for Medium to the object(wall)
        H1 and H2 stands for Far to the object(wall)
        '''

        # Distance to the wall
        #Near = 0.5, Medium = 1.5, Far = 2.0

        # Linear Speed
        Slow = 0.2
        Med = 0.5
        Fast = 1.0

        base_speed = 0.2
        
        # Angular Speed
        N = 0  # No steering
        L = 0.5  # turn left
        R = -0.5  # turn right

        # firing initialization 
        f1 = 0.0
        f2 = 0.0 
        f3 = 0.0
        f4 = 0.0
        f5 = 0.0
        f6 = 0.0 
        f7 = 0.0
        f8 = 0.0 
        f9 = 0.0

        # Rule Base
        if ((L1 > 0.0) and (L2 > 0.0)): f1 = min(L1, L2)
        if ((L1 > 0.0) and (M2 > 0.0)): f2 = min(L1, M2)
        if ((L1 > 0.0) and (H2 > 0.0)): f3 = min(L1, H2)

        if ((M1 > 0.0) and (L2 > 0.0)): f4 = min(M1, L2)
        if ((M1 > 0.0) and (M2 > 0.0)): f5 = min(M1, M2)
        if ((M1 > 0.0) and (H2 > 0.0)): f6 = min(M1, H2)

        if ((H1 > 0.0) and (L2 > 0.0)): f7 = min(H1, L2)
        if ((H1 > 0.0) and (M2 > 0.0)): f8 = min(H1, M2)
        if ((H1 > 0.0) and (H2 > 0.0)): f9 = min(H1, H2)


        firing_total = (f1 + f2 + f3 + f4 + f5 + f6 + f7 + f8 + f9)
        print('firing_total------:',firing_total) 

        #if f1 == 0 or f2 == 0:
        #    f9 == 1.0

        Linear = (f1*Slow) + (f2*Slow) + (f3*Med) + (f4*Med) + (f5*Med) + (f6*Med) + (f7*Slow) + (f8*Med) + (f9*Fast)

        Angular =(f1*L) + (f2*L) + (f3*R) + (f4*L) + (f5*N) + (f6*R) + (f7*L) + (f8*L) + (f9*R)


        print('Linear--:',Linear,'Angular---:',Angular)
        
        if Linear == 0 or firing_total == 0:

            Linear_output = (Linear + 0.2) / (firing_total + 1.0) # Linear_output >= 0.1  = 0.1

            Angular_output = (Angular - 0.5) / (firing_total + 1.0)
            
        else:
            Linear_output = Linear / firing_total

            Angular_output = Angular/ firing_total

        return Linear_output,Angular_output

if __name__ == '__main__':
    
    # cd = MF(0.0, 1.0, 2.0, 3.0)
    while not rospy.is_shutdown():
        global regions_
        global pub_
        # global points

        # print("test)

        pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
        sub = rospy.Subscriber('/scan', LaserScan,  Fuzzy_WF().callback_laser)

        rospy.init_node('reading_laser')

        regions = regions_
        msg = Twist()
        rate = rospy.Rate(25)

        near_right_distance = MEMBERSHIP_FUNCTION([0.0, 0.0, 0.5, 1.0], "Near")
        med_right_distance = MEMBERSHIP_FUNCTION([0.5, 1.0, 1.0, 1.5], "Med")
        far_right_distance = MEMBERSHIP_FUNCTION([1.0, 1.5, 20.0, 20.0], "Far")

        near_frontRight_distance = MEMBERSHIP_FUNCTION([0.0, 0.0, 0.7, 1.5], "Near")
        med_frontRight_distance = MEMBERSHIP_FUNCTION([0.7, 1.5, 1.5, 2.0], "Med")
        far_frontRight_distance = MEMBERSHIP_FUNCTION([1.5, 2.0, 20.0, 20.0], "Far")


        L1 = near_right_distance.membership_value(regions['right'])
        M1 = med_right_distance.membership_value(regions['right'])
        H1 = far_frontRight_distance.membership_value(regions['right'])

        L2 = near_frontRight_distance.membership_value(regions['front_right'])
        M2 = med_frontRight_distance.membership_value(regions['front_right'])
        H2 = far_frontRight_distance.membership_value(regions['front_right'])

        print("Right: ", regions['right'])
        print("Front Right: ", regions['front_right'])

        print("L1:", L1, "L2", L2)
        print("M1:", M1, "M2", M2)
        print("H1:", H1, "H2", H2)

        #output values of linear and angular velocity
        Output_Linear,Output_Angular = Fuzzy_WF().Rule_Base(L1, L2, M1, M2, H1, H2)
        
        msg.linear.x = Output_Linear
        msg.angular.z = Output_Angular

        pub_.publish(msg)

        rate.sleep()


        



