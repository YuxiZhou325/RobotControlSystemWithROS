#!/usr/bin/env python
"""
This file uses Fuzzy Logic implementing
right Edge following task
for the Rosbot in RVIZ and Gazebo environment
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

regions_ = {
    'right': 0,
    'front_right': 0,
    'front': 0,
    'front_left': 0,
    'left': 0,
}

''' 
Membership Function Class: It is used to determine the membership value of a given sensor value
with different membership functions.
'''


class MembershipFunctions:

    def __init__(self, mfPoints, label):
        self.points = mfPoints
        self.linguistic_label = label

    '''
    This function calculates the membership values of different sensors
    Parameters:
    [input-value]: Sensor value whose membership is to be determined
    '''

    def calculate_membership_value(self, laser_value):
        if laser_value < self.points[0] or laser_value > self.points[3]:
            return 0.0
        elif self.points[0] <= laser_value < self.points[1]:
            return (laser_value - self.points[0]) / (self.points[1] - self.points[0])
        elif self.points[1] <= laser_value <= self.points[2]:
            return 1.0
        elif self.points[2] < laser_value < self.points[3]:
            return (self.points[3] - laser_value) / (self.points[3] - self.points[2])

        return 0.0


# This class below stores different fuzzy rule base needed to calculate the firing strength
class Rule:
    """
    Object of this class is initialized with antecedents and consequent list, the rule is
    created with the instantiation of the class.
    Parameters:
    [antecedents]:list of antecedents, with index-0 1st sensor antecedent and index-1 with 2nd sensor value
    [consequents]:list of consequents, with index-0 as speed and index-1 as steer
    """

    def __init__(self, antecedents, consequents):
        self.antecedents = antecedents
        self.consequents = consequents

    '''
    This Function calculates the firing strength fo each rules with respect two sensor membership of both 
    the sensors.
    Parameters:
    [membership_values_dictionary]: List of dictionary of membership function name as key and it's respective
    membership value for both the sensors.
    Return Value: returns the minimum of membership value of sensors as firing strength for that particular rule.
    '''

    def calculate_firing_points(self, membership_values_dictionary):
        list_MemValues = []
        for i in range(len(self.antecedents)):
            list_MemValues.append(membership_values_dictionary[i][self.antecedents[i]])
        # returns the minimum of the two values as the firing strength of the rule.
        return min(list_MemValues)


# This class stores all  the rules needed for fuzzy logic and also gives the crisp output for speed and velocity.
class RuleBase:
    """
    An object of RuleBase class is initiated with list of rules objects passed to it.
    Parameters:
    [rules]: List of objects of class Rules
    """

    def __init__(self, rules):
        self.rules = rules

    ''' 
    This function gives the crisp output
    Parameters:
    [rule_firingStrength]:  List of firing strength of different rules
    [rule_speed]:           List of speed values w.r.t to firing strength
    [rule_steer]:           List of steer values w.r.t to firing strength
    '''

    def get_crisp_output(self, rule_firing_points, rule_speed, rule_steer):
        speed = steer = firing_point_sum = 0.0
        firing_point_length = len(rule_firing_points)
        ''' 
        At the beginning when the Robot is stationary, firing strength of 0 is 
        obtained, assigning a default velocity of 0.5 so that the robot starts moving 
        '''
        if firing_point_length == 0:
            speed = 0.5
            steer = 0.0
            return speed, steer
        # if firing strength is a list of one element, converting the list into float for calculation later.
        elif firing_point_length == 1:
            firing_point_sum = rule_firing_points[0]
        else:
            # calculating sum of firing strength, could have also used sum(rule_firing_points)
            for i in rule_firing_points:
                firing_point_sum += i

        print("firing_point_sum:", firing_point_sum)
        # Calculating product of firing strength and speed
        for firing_point, velocity in zip(rule_firing_points, rule_speed):
            speed += (firing_point * velocity)
        print(speed)

        # Calculating product of firing strength and steer
        for firing_point, turn in zip(rule_firing_points, rule_steer):
            steer += (firing_point * turn)

        print(steer)
        speed = speed / firing_point_sum
        steer = steer / firing_point_sum
        # returning crisp speed and turn values
        return speed, steer


'''This is the Parent class which implements fuzzy logic.
    It calculates membership value different sensors based on the their sensor reading.
'''


class FuzzySetREF(object):
    def __init__(self):
        # Right front sensor in  REF and top left sensor in OA
        self.sensor_1_distance = 0.0
        # Right back sensor in  REF and top right sensor in OA
        self.sensor_2_distance = 0.0
        self.front_sensor_distance = 0.0
        self.rule_base = []
        self.sensor_1_Membership = []
        self.sensor_2_Membership = []
        self.classspeed_centre_set = []
        self.classsteer_centre_set = []

        # output centre of set for speed
        self.speed_centre_set = {"Slow": 0.2, "Medium": 0.5, "Fast": 0.7}
        # output centre of set for steer
        self.steer_centre_set = {"Left": 0.8, "Forward": 0, "Right": -0.8}

        # Membership function for both obstacle avoidance and Right Edge Following.
        self.class_Membership = MembershipFunctions([1.0, 1.5, 2.0, 3], "REF")
        self.rule_base = self.create_rules()

        # Membership function for Right Edge following for both the sensors
        self.sensor_1_Membership = [MembershipFunctions([0.0, 0.0, 0.5, 1.0], "Near"),
                                    MembershipFunctions([0.5, 1.0, 1.0, 1.5], "Medium"),
                                    MembershipFunctions([1.0, 1.5, 20.0, 20.0], "Far")]
        self.sensor_2_Membership = [MembershipFunctions([0.0, 0.0, 0.7, 1.5], "Near"),
                                    MembershipFunctions([0.7, 1.5, 1.5, 2.0], "Medium"),
                                    MembershipFunctions([1.5, 2.0, 20.0, 20.0], "Far")]
        self.rule_base = self.create_rules()

        # ===================================================================================== #
        #                         Right Edge Wall Following Rule Base
        # ===================================================================================== #

    def create_rules(self):
        rule_base = []
        rule_1 = Rule(["Near", "Near"], ["Slow", "Left"])
        rule_2 = Rule(["Near", "Medium"], ["Slow", "Left"])
        rule_3 = Rule(["Near", "Far"], ["Medium", "Right"])
        rule_4 = Rule(["Medium", "Near"], ["Medium", "Left"])
        rule_5 = Rule(["Medium", "Medium"], ["Medium", "Forward"])
        rule_6 = Rule(["Medium", "Far"], ["Medium", "Right"])
        rule_7 = Rule(["Far", "Near"], ["Slow", "Left"])
        rule_8 = Rule(["Far", "Medium"], ["Medium", "Left"])
        rule_9 = Rule(["Far", "Far"], ["Fast", "Right"])
        rule_base = RuleBase([rule_1, rule_2, rule_3, rule_4, rule_5, rule_6, rule_7, rule_8, rule_9])
        return rule_base

    def set_sensor_values(self, sensor_1, sensor_2):
        self.sensor_1_distance = sensor_1
        self.sensor_2_distance = sensor_2

    def get_sensor_values(self):
        return self.sensor_1_distance, self.sensor_2_distance

    ''' This function create object rules and uses it to create Rule Base object.
        Return Value: object of RuleBase class'''

    # Calculates membership value of obstacle avoidance and right edge following w.r.t to fron sensor.
    # Return Value: list of dictionary.
    def class_fuzzyfication(self):
        front_sensor_distance = {"REF": self.class_Membership.calculate_membership_value(self.front_sensor_distance)}
        return [front_sensor_distance]

    # Calculates membership value of sensor 1 and sensor 2 w.r.t to respective membership functions.
    # This function is called both for obstacle avoidance and right edge following.
    # Return Value: list of dictionary.
    def fuzzification(self):
        sensor_1_distance = {"Near": self.sensor_1_Membership[0].calculate_membership_value(self.sensor_1_distance),
                             "Medium": self.sensor_1_Membership[1].calculate_membership_value(self.sensor_1_distance),
                             "Far": self.sensor_1_Membership[2].calculate_membership_value(self.sensor_1_distance)}
        print("sensor_1_distance", sensor_1_distance)
        sensor_2_distance = {"Near": self.sensor_2_Membership[0].calculate_membership_value(self.sensor_2_distance),
                             "Medium": self.sensor_2_Membership[1].calculate_membership_value(self.sensor_2_distance),
                             "Far": self.sensor_2_Membership[2].calculate_membership_value(self.sensor_2_distance)}
        print("sensor_2_distance", sensor_2_distance)

        return [sensor_1_distance, sensor_2_distance]

    # Calculates firing strength, gets the crisp output..
    # This function is called is only called when doing defuzzification between OA and REF.
    # Return Value: list of dictionary.
    def class_defuzzification(self, sensor_membership_values):
        rule_speed = []
        rule_steer = []
        rule_firing_points = []
        print("fuzzification Values: ", sensor_membership_values)
        # Iterating over RuleBase object to get the firing strength of each rule stored in it,
        for rule in self.rule_base.rules:
            fs = rule.calculate_firing_points(sensor_membership_values)
            # only store the firing strength of rule and its related consequents if the firing
            # strength of the rule is greater than 0.
            if fs > 0:
                rule_firing_points.append(fs)
                print("speed set", rule.consequents[0])
                print("steer set", rule.consequents[1])
                print("fs set", rule_firing_points)
                # storing and appending both the consequents of rules in two different lists
                rule_speed.append(self.classspeed_centre_set[rule.consequents[0]])
                rule_steer.append(self.classsteer_centre_set[rule.consequents[1]])
        print("Firing Strength: ", rule_firing_points)
        print("Rule Speed:", rule_speed)
        print("Rule Steer: ", rule_steer)
        # Gets crisp output for both steer and speed
        speed, steer = self.rule_base.get_crisp_output(rule_firing_points, rule_speed, rule_steer)
        return speed, steer

    # This function is called for both OA and REF
    # Calculates firing strength, gets the crisp output.
    # This function is called is only called when doing defuzzification between OA and REF.
    # Return Value: list of dictionary.
    def defuzzification(self, membership_values_dictionary):
        rule_speed = []
        rule_steer = []
        rule_firing_points = []
        print("fuzzification Values: ", membership_values_dictionary)
        # Iterating over RuleBase object to get the firing strength of each rule stored in it,
        for rule in self.rule_base.rules:
            fs = rule.calculate_firing_points(membership_values_dictionary)
            # only store the firing strength of rule and its related consequents if the firing
            # strength of the rule is greater than 0.
            if fs > 0:
                rule_firing_points.append(fs)
                print("speed set", rule.consequents[0])
                print("steer set", rule.consequents[1])
                print("fs set", rule_firing_points)
                # storing and appending both the consequents of rules in two different lists
                rule_speed.append(self.speed_centre_set[rule.consequents[0]])
                rule_steer.append(self.steer_centre_set[rule.consequents[1]])
        print("Firing Strength: ", rule_firing_points)
        print("Rule Speed:", rule_speed)
        print("Rule Steer: ", rule_steer)
        # Gets crisp output for both steer and speed
        speed, steer = self.rule_base.get_crisp_output(rule_firing_points, rule_speed, rule_steer)
        return speed, steer


# Declaring publisher to velocity and angular velocity to topic 'vmd_vel'
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # creates publisher class


# function to control velocity  and turn of the robo robot
def forwards(speed, turn):
    global pub
    print("Robot Speed: ", speed)
    print("Robot Turn: ", turn)
    vel_x = Twist()  # initiates twist message
    vel_x.linear.x = speed  # sets linear speed of robot
    vel_x.angular.z = turn  # sets angle to turn to pid function output (turns left?)
    pub.publish(vel_x)  # publishes velocity


# This function is called with new sensor values.
def callback(msg, fuzzy):
    global regions_

    regions_ = {
        'front': min(msg.ranges[0:40], msg.ranges[679:719]),
        'right': min(msg.ranges[499:579]),
        'left': min(msg.ranges[160:220]),

        'front_right': min(msg.ranges[649:689]),
        'front_left': min(msg.ranges[30:100]),
        'back_right': min(msg.ranges[449:539]),

        'min_right': min(min(msg.ranges[499:579]), min(msg.ranges[649:689])),
        'min_left': min(min(msg.ranges[0:40], msg.ranges[679:719]), msg.ranges[679:719], msg.ranges[30:100]),
    }
    # If the sensor value is 'inf' or greater than the membership function's range
    # then hard-code it a value so that it falls under it gets 'FAR' membership functions membership
    if str(regions_['front_right']) == 'inf' or regions_['front_right'] >= 2:
        regions_['front_right'] = 1.99
    if str(regions_['back_right']) == 'inf' or regions_['back_right'] >= 2:
        regions_['back_right'] = 1.99

    fuzzy_ref.set_sensor_values(regions_['right'], regions_['front_right'])


# This is the function which runs continuously and does the operations based on sensor values.
def controller(fuzzy):
    rate = rospy.Rate(10)  # sleep in loop at rate 10hz
    while not rospy.is_shutdown():
        # Control Architecture

        # Right Edge Wall Following
        print("REF")
        print("Right Distance Sensor Readings", fuzzy_ref.sensor_1_distance)
        print("Front Right Distance Sensor Readings", fuzzy_ref.sensor_2_distance)
        # Getting membership for REF for both sensor
        membership_values_dictionary = fuzzy_ref.fuzzification()
        # Getting Crisp output for both speed and steer
        speed_ref, steer_ref = fuzzy_ref.defuzzification(membership_values_dictionary)
        print("****speed****:", speed_ref)
        print("****steer****:", steer_ref)
        # Passing speed and steer to drive the robot
        forwards(speed_ref, steer_ref)
        rate.sleep()  # pauses rate


if __name__ == '__main__':

    # creating main class object and Fuzzy object for Right Edge Following
    fuzzy_ref = FuzzySetREF()

    try:
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback, fuzzy_ref)
        controller(fuzzy_ref)
    except rospy.ROSInterruptException:
        pass
