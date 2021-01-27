#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub_ = None

points = []

f = []

firing_total = 0.0

N1 = 0.0
N2 = 0.0
N3 = 0.0
M1 = 0.0
M2 = 0.0
M3 = 0.0
F1 = 0.0
F2 = 0.0
F3 = 0.0

regions_ = {
    'right': 0,
    'front_right': 0,
    'front': 0,
    'front_left': 0,
    'left': 0,
}

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
            return (laser_value - self.points[0]) / (self.points[1]) - self.points[0]
        elif laser_value > self.points[2] and laser_value <= self.points[3]:
            return (self.points[3] - laser_value) / (self.points[3] - self.points[2])
        else:
            return 0.0

class Fuzzy_OA:
    
    def __init__(self):
        print("Fuzzy construction")
        self.LOW = 0.0
        self.MED = 0.0
        self.HIGH = 0.0

    
    # Get the distance between the robot and the Nearst object within the direction range
    # We are taking 5 directions of sensors here, right, left, front right and front left

    def callback_laser(self, msg):

        global regions_
        regions_ = {
            'right': min(msg.ranges[519:549]),
            'front': min(msg.ranges[0:40],msg.ranges[679:719]),
            'front_right': min(msg.ranges[699:709]),
            'front_left': min(msg.ranges[30:100]),
            'left': min(msg.ranges[160:220]),
        }
        
        

    def Rule_Base(self, a, b, c, d, e, k, g, h, j):

        global N1
        global N2
        global N3
        global M1
        global M2
        global M3
        global F1
        global F2
        global F3
        global f

        f1 = 0.0
        f2 = 0.0
        f3 = 0.0
        f4 = 0.0
        f5 = 0.0
        f6 = 0.0
        f7 = 0.0
        f8 = 0.0
        f9 = 0.0
        f10 = 0.0
        f11 = 0.0
        f12 = 0.0
        f13 = 0.0
        f14 = 0.0
        f15 = 0.0
        f16 = 0.0
        f17 = 0.0
        f18 = 0.0
        f19 = 0.0
        f20 = 0.0
        f21 = 0.0
        f22 = 0.0
        f23 = 0.0
        f24 = 0.0
        f25 = 0.0
        f26 = 0.0
        f27 = 0.0

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
        
        # Angular Speed
        N = 0  # No steering
        L = 0.5  # turn left
        R = -0.5  # turn right
        

        # Rule Base
        if N3 > 0 and N1 > 0 and N2 > 0: f1 = min(N3, min(N1, N2))
        if N3 > 0 and N1 > 0 and M2 > 0: f2 = min(N3, min(N1, M2))
        if N3 > 0 and N1 > 0 and F2 > 0: f3 = min(N3, min(N1, F2))

        if N3 > 0 and M1 > 0 and N2 > 0: f4 = min(N3, min(M1, N2))
        if N3 > 0 and M1 > 0 and M2 > 0: f5 = min(N3, min(M1, M2))
        if N3 > 0 and M1 > 0 and F2 > 0: f6 = min(N3, min(M1, F2))

        if N3 > 0 and F1 > 0 and N2 > 0: f7 = min(N3, min(F1, N2))
        if N3 > 0 and F1 > 0 and M2 > 0: f8 = min(N3, min(F1, M2))
        if N3 > 0 and F1 > 0 and F2 > 0: f9 = min(N3, min(F1, F2))


        if M3 > 0 and N1 > 0 and N2 > 0: f10 = min(M3, min(N1, N2))
        if M3 > 0 and N1 > 0 and M2 > 0: f11 = min(M3, min(N1, M2))
        if M3 > 0 and N1 > 0 and F2 > 0: f12 = min(M3, min(N1, F2))

        if M3 > 0 and M1 > 0 and N2 > 0: f13 = min(M3, min(M1, N2))
        if M3 > 0 and M1 > 0 and M2 > 0: f14 = min(M3, min(M1, M2))
        if M3 > 0 and M1 > 0 and F2 > 0: f15 = min(M3, min(M1, F2))

        if M3 > 0 and F1 > 0 and N2 > 0: f16 = min(M3, min(F1, N2))
        if M3 > 0 and F1 > 0 and M2 > 0: f17 = min(M3, min(F1, M2))
        if M3 > 0 and F1 > 0 and F2 > 0: f18 = min(M3, min(F1, F2))


        if F3 > 0 and N1 > 0 and N2 > 0: f19 = min(F3, min(N1, N2))
        if F3 > 0 and N1 > 0 and M2 > 0: f20 = min(F3, min(N1, M2))
        if F3 > 0 and N1 > 0 and F2 > 0: f21 = min(F3, min(N1, F2))

        if F3 > 0 and M1 > 0 and N2 > 0: f22 = min(F3, min(M1, N2))
        if F3 > 0 and M1 > 0 and M2 > 0: f23 = min(F3, min(M1, M2))
        if F3 > 0 and M1 > 0 and F2 > 0: f24 = min(F3, min(M1, F2))

        if F3 > 0 and F1 > 0 and N2 > 0: f25 = min(F3, min(F1, N2))
        if F3 > 0 and F1 > 0 and M2 > 0: f26 = min(F3, min(F1, M2))
        if F3 > 0 and F1 > 0 and F2 > 0: f27 = min(F3, min(F1, F2))


        # print(f1)
        # i = 0
        firing_total = f1 + f2 + f3 + f4 + f5 + f6 + f7+ f8+ f9+ f10+ f11+ f12+ f13+ f14+ f15+ f16+ f17+ f18+ f19+ f20+ f21+ f22+ f23+ f24+ f25+ f26+ f27
        print('-- firing_total --:',firing_total)


        Linear = (f1*Slow) + (f2*Slow) + (f3*Slow) + (f4*Slow) + (f5*Slow) + (f6*Slow) + (f7*Slow) + (f8*Slow) + (f9*Slow) 
        + (f10*Slow) + (f11*Slow) + (f12*Slow) + (f13*Slow) + (f14*Med) + (f15*Med) + (f16*Slow) + (f17*Med) + (f18*Med)
        + (f19*Slow) + (f20*Slow) + (f21*Slow) + (f22*Slow) + (f23*Fast) + (f24*Fast) + (f25*Slow) + (f26*Fast) + (f27*Fast)

        Angular = (f1*L) + (f2*L) + (f3*L) + (f4*R) + (f5*L) + (f6*L) + (f7*R) + (f8*R) + (f9*L)
        + (f10*N) + (f11*L) + (f12*L) + (f13*R) + (f14*N) + (f15*N) + (f16*R) + (f17*N) + (f18*N)
        + (f19*N) + (f20*L) + (f21*L) + (f22*R) + (f23*N) + (f24*N) + (f25*R) + (f26*N) + (f27*N)


        print('--- Linear ---:',Linear,'--- Angular ---:',Angular)

        if Linear == 0 or firing_total == 0:

            Linear_output = (Linear + 0.2) / (firing_total + 1.0) # Linear_output >= 0.1  = 0.1

            Angular_output = Angular / (firing_total + 1.0)
            
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
    
        sub = rospy.Subscriber('/scan', LaserScan,  Fuzzy_OA().callback_laser)

        rospy.init_node('reading_laser')

        regions = regions_
        msg = Twist()
        rate = rospy.Rate(25)

        near_frontRight_distance = MEMBERSHIP_FUNCTION([0.0, 0.0, 0.7, 1.5], "Near")
        med_frontRight_distance = MEMBERSHIP_FUNCTION([0.7, 1.5, 1.5, 2.0], "Med")
        far_frontRight_distance = MEMBERSHIP_FUNCTION([1.5, 2.0, 20.0, 20.0], "Far")

        near_frontLeft_distance = MEMBERSHIP_FUNCTION([0.0, 0.0, 0.7, 1.5], "Near")
        med_frontLeft_distance = MEMBERSHIP_FUNCTION([0.7, 1.5, 1.5, 2.0], "Med")
        far_frontLeft_distance = MEMBERSHIP_FUNCTION([1.5, 2.0, 20.0, 20.0], "Far")

        near_front_distance = MEMBERSHIP_FUNCTION([0.0, 0.0, 1.0, 1.5], "Near")
        med_front_distance = MEMBERSHIP_FUNCTION([1.0, 1.5, 1.5, 2.0], "Med")
        far_front_distance = MEMBERSHIP_FUNCTION([1.5, 2.0, 20.0, 20.0], "Far")

        N1 = near_frontRight_distance.membership_value(regions['front_right'])
        M1 = med_frontRight_distance.membership_value(regions['front_right'])
        F1 = far_frontRight_distance.membership_value(regions['front_right'])

        N2 = near_frontLeft_distance.membership_value(regions['front_left'])
        M2 = med_frontLeft_distance.membership_value(regions['front_left'])
        F2 = far_frontLeft_distance.membership_value(regions['front_left'])

        N3 = near_front_distance.membership_value(regions['front'])
        M3 = med_front_distance.membership_value(regions['front'])
        F3 = far_front_distance.membership_value(regions['front'])

        print("Front Right: ", regions['front_right'])
        print("Front Left: ", regions['front_left'])

        print("N1:", N1, "N2: ", N2, "N3: ",N3)

        #output values of linear and angular velocity
        Output_Linear,Output_Angular = Fuzzy_OA().Rule_Base(N1, M1, F1, N2, M2, F2, N3, M3, F3)
        
        msg.linear.x = Output_Linear
        msg.angular.z = Output_Angular

        pub_.publish(msg)

        rate.sleep()

