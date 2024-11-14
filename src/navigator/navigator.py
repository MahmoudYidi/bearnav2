#!/usr/bin/env python3

from geometry_msgs.msg import Twist

class Navigator:

    def __init__(self):
        self.alignment = 0
        self.uncertainty = 0
        self.use_uncertainty = True
        #self.turn_gain = 0.0000000003  # turn 0.1 rad per each pixel of error
        self.turn_gain = 0.0000001
        self.velocity_gain = 1 # 1 is the same speed as thought map, less is slower more is faster #.93

    def process(self, msg):
        correction = self.alignment * self.turn_gain  # angle = px * angle/pixel
        #print(self.turn_gain)
        if self.use_uncertainty:
            correction = correction * (1 - self.uncertainty)
        out = Twist()
        out.linear.x = msg.linear.x * self.velocity_gain
        out.linear.y = msg.linear.y * self.velocity_gain
        out.linear.z = msg.linear.z * self.velocity_gain
        out.angular.x = msg.angular.x * self.velocity_gain 
        out.angular.y = msg.angular.y * self.velocity_gain
        out.angular.z = msg.angular.z * self.velocity_gain + correction
        return out

    def update_parameters(self, params):
        for param in params:
            if param.name == 'use_uncertainty':
                self.use_uncertainty = param.value
            elif param.name == 'turn_gain':
                self.turn_gain = param.value
            elif param.name == 'velocity_gain':
                self.velocity_gain = param.value

    def correction(self, msg):
        self.alignment = msg.alignment  # Px
        self.uncertainty = msg.uncertainty
