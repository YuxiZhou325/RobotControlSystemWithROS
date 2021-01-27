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
        if N3 > 0 and N1 > 0 and N2 > 0: f[0] = min(N3, min(N1, N2))
        if N3 > 0 and N1 > 0 and M2 > 0: f[1] = min(N3, min(N1, M2))
        if N3 > 0 and N1 > 0 and F2 > 0: f[2] = min(N3, min(N1, F2))

        if N3 > 0 and M1 > 0 and N2 > 0: f[3] = min(N3, min(M1, N2))
        if N3 > 0 and M1 > 0 and M2 > 0: f[4] = min(N3, min(M1, M2))
        if N3 > 0 and M1 > 0 and F2 > 0: f[5] = min(N3, min(M1, F2))

        if N3 > 0 and F1 > 0 and N2 > 0: f[6] = min(N3, min(F1, N2))
        if N3 > 0 and F1 > 0 and M2 > 0: f[7] = min(N3, min(F1, M2))
        if N3 > 0 and F1 > 0 and F2 > 0: f[8] = min(N3, min(F1, F2))


        if M3 > 0 and N1 > 0 and N2 > 0: f[9] = min(M3, min(N1, N2))
        if M3 > 0 and N1 > 0 and M2 > 0: f[10] = min(M3, min(N1, M2))
        if M3 > 0 and N1 > 0 and F2 > 0: f[11] = min(M3, min(N1, F2))

        if M3 > 0 and M1 > 0 and N2 > 0: f[12] = min(M3, min(M1, N2))
        if M3 > 0 and M1 > 0 and M2 > 0: f[13] = min(M3, min(M1, M2))
        if M3 > 0 and M1 > 0 and F2 > 0: f[14] = min(M3, min(M1, F2))

        if M3 > 0 and F1 > 0 and N2 > 0: f[15] = min(M3, min(F1, N2))
        if M3 > 0 and F1 > 0 and M2 > 0: f[16] = min(M3, min(F1, M2))
        if M3 > 0 and F1 > 0 and F2 > 0: f[17] = min(M3, min(F1, F2))


        if F3 > 0 and N1 > 0 and N2 > 0: f[18] = min(F3, min(N1, N2))
        if F3 > 0 and N1 > 0 and M2 > 0: f[19] = min(F3, min(N1, M2))
        if F3 > 0 and N1 > 0 and F2 > 0: f[20] = min(F3, min(N1, F2))

        if F3 > 0 and M1 > 0 and N2 > 0: f[21] = min(F3, min(M1, N2))
        if F3 > 0 and M1 > 0 and M2 > 0: f[22] = min(F3, min(M1, M2))
        if F3 > 0 and M1 > 0 and F2 > 0: f[23] = min(F3, min(M1, F2))

        if F3 > 0 and F1 > 0 and N2 > 0: f[24] = min(F3, min(F1, N2))
        if F3 > 0 and F1 > 0 and M2 > 0: f[25] = min(F3, min(F1, M2))
        if F3 > 0 and F1 > 0 and F2 > 0: f[26] = min(F3, min(F1, F2))


        # print(f[0])
        # i = 0
        for i in range(26):
            global firing_total
            firing_total += f[i]
        print('-- firing_total --:',firing_total)


        Linear = (f[0]*Slow) + (f[1]*Slow) + (f[2]*Slow) + (f[3]*Slow) + (f[4]*Slow) + (f[5]*Slow) + (f[6]*Slow) + (f[7]*Slow) + (f[8]*Slow) 
        + (f[9]*Slow) + (f[10]*Slow) + (f[11]*Slow) + (f[12]*Slow) + (f[13]*Med) + (f[14]*Med) + (f[15]*Slow) + (f[16]*Med) + (f[17]*Med)
        + (f[18]*Slow) + (f[19]*Slow) + (f[20]*Slow) + (f[21]*Slow) + (f[22]*Fast) + (f[23]*Fast) + (f[24]*Slow) + (f[25]*Fast) + (f[26]*Fast)

        Angular = (f[0]*L) + (f[1]*L) + (f[2]*L) + (f[3]*R) + (f[4]*L) + (f[5]*L) + (f[6]*R) + (f[7]*R) + (f[8]*L)
        + (f[9]*N) + (f[10]*L) + (f[11]*L) + (f[12]*R) + (f[13]*N) + (f[14]*N) + (f[15]*R) + (f[16]*N) + (f[17]*N)
        + (f[18]*N) + (f[19]*L) + (f[20]*L) + (f[21]*R) + (f[22]*N) + (f[23]*N) + (f[24]*R) + (f[25]*N) + (f[26]*N)


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

