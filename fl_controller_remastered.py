#!/usr/bin/env python
"""
This file uses Fuzzy Logic implementing
right Edge following and obstacle avoidance task
for the Rosbot in RVIZ and Gazebo environment
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys

global behaviour_option

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
        speed = steer = firing_strength_sum = 0.0
        firing_strength_length = len(rule_firing_points)
        ''' 
        At the beginning when the Robot is stationary, firing strength of 0 is 
        obtained, assigning a default velocity of 0.5 so that the robot starts moving 
        '''
        if firing_strength_length == 0:
            speed = 0.5
            steer = 0.0
            return speed, steer
        # if firing strength is a list of one element, converting the list into float for calculation later.
        elif firing_strength_length == 1:
            firing_strength_sum = rule_firing_points[0]
        else:
            # calculating sum of firing strength, could have also used sum(rule_firing_points)
            for i in rule_firing_points:
                firing_strength_sum += i

        print("firing_strength_sum:", firing_strength_sum)
        # Calculating product of firing strength and speed
        for firing_point, velocity in zip(rule_firing_points, rule_speed):
            speed += (firing_point * velocity)
        print(speed)

        # Calculating product of firing strength and steer
        for firing_point, turn in zip(rule_firing_points, rule_steer):
            steer += (firing_point * turn)

        print(steer)
        speed = speed / firing_strength_sum
        steer = steer / firing_strength_sum
        # returning crisp speed and turn values
        return speed, steer


'''This is the Parent class which implements fuzzy logic.
    It calculates membership value different sensors based on the their sensor reading.
'''


class FuzzySet(object):
    def __init__(self):
        # Right front sensor in  REF and top left sensor in OA
        self.sensor_1_distance = 0.0
        # Right back sensor in  REF and top right sensor in OA
        self.sensor_2_distance = 0.0
        self.front_sensor_distance = 0.0
        self.rule_base = []
        self.sensor_1_Membership = []
        self.sensor_2_Membership = []
        self.classspeedCentreSet = []
        self.classsteerCentreSet = []

        # output centre of set for speed
        self.speedCentreSet = {"Slow": 0.1, "Medium": 0.8, "Fast": 1.0}
        # output centre of set for steer
        self.steerCentreSet = {"Left": 1.0, "Forward": 0, "Right": -0.8}

        # Membership function for both obstacle avoidance and Right Edge Following.
        self.class_Membership = [MembershipFunctions([0.1, 0.1, 1.0, 1.5], "OA"),
                                 MembershipFunctions([1.0, 1.5, 2.0, 3], "REF")]
        self.rule_base = self.create_rules()

    def set_sensor_values(self, sensor_1, sensor_2):
        self.sensor_1_distance = sensor_1
        self.sensor_2_distance = sensor_2

    def get_sensor_values(self):
        return self.sensor_1_distance, self.sensor_2_distance

    ''' This function create object rules and uses it to create Rule Base object.
        Return Value: object of RuleBase class'''

    def create_rules(self):
        r_base = []
        rule_1 = Rule(["OA"], ["Speed_OA", "Steer_OA"])
        rule_2 = Rule(["REF"], ["Speed_REF", "Steer_REF"])
        rule_base = RuleBase([rule_1, rule_2])
        return rule_base

    # Calculates membership value of obstacle avoidance and right edge following w.r.t to fron sensor.
    # Return Value: list of dictionary.
    def class_fuzzyfication(self):
        front_sensor_distance = {"OA": self.class_Membership[0].calculate_membership_value(self.front_sensor_distance),
                                 "REF": self.class_Membership[1].calculate_membership_value(self.front_sensor_distance)}
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
                rule_speed.append(self.classspeedCentreSet[rule.consequents[0]])
                rule_steer.append(self.classsteerCentreSet[rule.consequents[1]])
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
                rule_speed.append(self.speedCentreSet[rule.consequents[0]])
                rule_steer.append(self.steerCentreSet[rule.consequents[1]])
        print("Firing Strength: ", rule_firing_points)
        print("Rule Speed:", rule_speed)
        print("Rule Steer: ", rule_steer)
        # Gets crisp output for both steer and speed
        speed, steer = self.rule_base.get_crisp_output(rule_firing_points, rule_speed, rule_steer)
        return speed, steer


# This is the child class derived from FuzzySet class for Right Edge Following
# It reuses all the common functions from the prent class
class FuzzySetREF(FuzzySet):
    def __init__(self):
        super(FuzzySetREF, self).__init__()
        # Memebership function for Right Edge following for both the sensors
        self.sensor_1_Membership = [MembershipFunctions([0.1, 0.1, 0.2, 0.5], "Near"),
                                    MembershipFunctions([0.2, 0.5, 0.5, 0.7], "Medium"),
                                    MembershipFunctions([0.5, 0.7, 0.9, 2.0], "Far")]
        self.sensor_2_Membership = [MembershipFunctions([0.1, 0.1, 0.2, 0.5], "Near"),
                                    MembershipFunctions([0.2, 0.5, 0.5, 0.7], "Medium"),
                                    MembershipFunctions([0.5, 0.7, 0.9, 2.0], "Far")]
        self.rule_base = self.create_rules()

    # ===================================================================================== #
    #                         Right Edge Wall Following Rule Base
    # ===================================================================================== #

    def create_rules(self):
        rule_base = []
        rule_1 = Rule(["Near", "Near"], ["Slow", "Left"])
        rule_2 = Rule(["Near", "Medium"], ["Slow", "Left"])
        rule_3 = Rule(["Near", "Far"], ["Slow", "Left"])
        rule_4 = Rule(["Medium", "Near"], ["Slow", "Right"])
        rule_5 = Rule(["Medium", "Medium"], ["Slow", "Left"])
        rule_6 = Rule(["Medium", "Far"], ["Slow", "Left"])
        rule_7 = Rule(["Far", "Near"], ["Medium", "Forward"])
        rule_8 = Rule(["Far", "Medium"], ["Fast", "Forward"])
        rule_9 = Rule(["Far", "Far"], ["Medium", "Right"])
        rule_base = RuleBase([rule_1, rule_2, rule_3, rule_4, rule_5, rule_6, rule_7, rule_8, rule_9])
        return rule_base


# This is the child class derived from FuzzySet class for Obstacle Avoidance
# It reuses all the common functions from the prent class
class FuzzySetOA(FuzzySet):
    def __init__(self):
        super(FuzzySetOA, self).__init__()
        # Membership function for obstacle avoidance for both the sensors

        self.sensor_1_Membership = [MembershipFunctions([0.1, 0.1, 1.0, 1.5], "Near"),
                                    MembershipFunctions([1.0, 1.5, 1.5, 2.0], "Medium"),
                                    MembershipFunctions([1.5, 2.0, 2.5, 3.0], "Far")]
        self.sensor_2_Membership = [MembershipFunctions([0.1, 0.1, 1.0, 1.5], "Near"),
                                    MembershipFunctions([1.0, 1.5, 1.5, 2.0], "Medium"),
                                    MembershipFunctions([1.5, 2.0, 2.5, 3.0], "Far")]

        self.rule_base = self.create_rules()

    # ===================================================================================== #
    #                               Obstacle Avoidance Rule Base
    # ===================================================================================== #

    def create_rules(self):
        rule_base = []
        rule_1 = Rule(["Near", "Near"], ["Slow", "Left"])
        rule_2 = Rule(["Near", "Medium"], ["Slow", "Left"])
        rule_3 = Rule(["Near", "Far"], ["Slow", "Left"])
        rule_4 = Rule(["Medium", "Near"], ["Slow", "Left"])
        rule_5 = Rule(["Medium", "Medium"], ["Slow", "Forward"])
        rule_6 = Rule(["Medium", "Far"], ["Medium", "Forward"])
        rule_7 = Rule(["Far", "Near"], ["Slow", "Left"])
        rule_8 = Rule(["Far", "Medium"], ["Medium", "Forward"])
        rule_9 = Rule(["Far", "Far"], ["Fast", "Forward"])
        rule_base = RuleBase([rule_1, rule_2, rule_3, rule_4, rule_5, rule_6, rule_7, rule_8, rule_9])

        return rule_base


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
    front_sensor = min(min(msg.ranges[0:40]), min(msg.ranges[679:719]))
    right_sensor = min(msg.ranges[499:579])
    front_right_sensor = min(msg.ranges[649:689])
    front_left_sensor = min(msg.ranges[30:100])

    min_right_distance_sensor = min(min(msg.ranges[499:579]), min(msg.ranges[649:689]))
    min_left_distance_sensor = min(min(msg.ranges[0:40], msg.ranges[679:719]), msg.ranges[679:719], msg.ranges[30:100])

    fuzzy[0].front_sensor_distance = min_left_distance_sensor
    fuzzy[1].set_sensor_values(front_sensor, min_right_distance_sensor)
    fuzzy[2].set_sensor_values(min_left_distance_sensor, right_sensor)


# This is the function which runs continuously and does the operations based on sensor values.
def controller(fuzzy):
    global behaviour_option
    rate = rospy.Rate(10)  # sleep in loop at rate 10hz
    while not rospy.is_shutdown():
        # Control Architecture
        if behaviour_option == 0:
            print("BOTH")

            print("Right Front Sensor Readings", fuzzy[1].sensor_1_distance)
            print("Right Back Sensor Readings", fuzzy[1].sensor_2_distance)

            print("Top Left Sensor Readings", fuzzy[2].sensor_1_distance)
            print("Top Right Sensor Readings", fuzzy[2].sensor_2_distance)

            # Getting membership for REF for both sensor
            membership_values_dictionary = fuzzy[1].fuzzification()
            # Getting Crisp output for both speed and steer
            speed_ref, steer_ref = fuzzy[1].defuzzification(membership_values_dictionary)
            # Getting membership for AO for both sensor
            membership_values_dictionary = fuzzy[2].fuzzification()
            # Getting Crisp output for both speed and steer
            speed_oa, steer_oa = fuzzy[2].defuzzification(membership_values_dictionary)
            # Creating dynamic dictionary with REF speed, steer and OA speed,steer
            fuzzy[0].classspeedCentreSet = {"Speed_OA": speed_oa, "Speed_REF": speed_ref}
            fuzzy[0].classsteerCentreSet = {"Steer_OA": steer_oa, "Steer_REF": steer_ref}
            # Getting membership value for OA and REF
            membership_values_dictionary = fuzzy[0].class_fuzzyfication()
            # Getting Crisp output for both speed and steer
            speed, steer = fuzzy[0].class_defuzzification(membership_values_dictionary)
            print("****speed****:", speed)
            print("****steer****:", steer)
            # Passing speed and steer to drive the robot
            forwards(speed, steer)
        # REF
        if behaviour_option == 1:
            print("REF")
            print("Front Sensor Readings", fuzzy[1].sensor_1_distance)
            print("Right and Front Right Sensor Readings", fuzzy[1].sensor_2_distance)
            # Getting membership for REF for both sensor
            membership_values_dictionary = fuzzy[1].fuzzification()
            # Getting Crisp output for both speed and steer
            speed_ref, steer_ref = fuzzy[1].defuzzification(membership_values_dictionary)
            print("****speed****:", speed_ref)
            print("****steer****:", steer_ref)
            # Passing speed and steer to drive the robot
            forwards(speed_ref, steer_ref)
        # Obstacle Avoidance
        if behaviour_option == 2:
            print("OA")
            print("Top Left Sensor Readings", fuzzy[2].sensor_1_distance)
            print("Top Right Sensor Readings", fuzzy[2].sensor_2_distance)
            # Getting membership for AO for both sensor
            membership_values_dictionary = fuzzy[2].fuzzification()
            # Getting Crisp output for both speed and steer
            speed_oa, steer_oa = fuzzy[2].defuzzification(membership_values_dictionary)
            print("****speed****:", speed_oa)
            print("****steer****:", steer_oa)
            # Passing speed and steer to drive the robot
            forwards(speed_oa, steer_oa)
        rate.sleep()  # pauses rate


if __name__ == '__main__':
    global behaviour_option
    # Reading options passed through scrip to select the behaviour
    if str(sys.argv[1]) == 'Both':
        behaviour_option = 0
    if str(sys.argv[1]) == 'REF':
        behaviour_option = 1
    if str(sys.argv[1]) == 'OA':
        behaviour_option = 2

    # creating main class object
    fuzzySuper = FuzzySet()

    # Creating Fuzzy object for Right Edge Following
    fuzzyRef = FuzzySetREF()

    # # Creating Fuzzy object for Obstacle avoidance3
    fuzzyOA = FuzzySetOA()
    # creating list of fuzzy objects
    fuzzy = [fuzzySuper, fuzzyRef, fuzzyOA]
    try:
        rospy.init_node('script', anonymous=True)
        sub = rospy.Subscriber('/scan', LaserScan, callback, fuzzy)
        controller(fuzzy)
    except rospy.ROSInterruptException:
        pass
