#!/usr/bin/env python

import numpy as np

"""
The class of the pid controller.
"""
class SinglePIDController:
    def __init__(self, Kp, Ki, Kd, max_value, min_value, angle = False):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = 0
        self.lastError = 0
        self.timestep = 0.1
        self.maximumValue = max_value
        self.minimumValue = min_value
        self.angle = angle

    def setTarget(self, target):
        """
        set the target pose.
        """
        self.target = target

    def getError(self, currentState):
        """
        return the different between two states
        """
        result = self.target - currentState
        return (result + np.pi) % (2 * np.pi) - np.pi if self.angle else result

    # def setMaximumUpdate(self, mv):
    #     """
    #     set maximum velocity for stability.
    #     """
    #     self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        
        if result > self.maximumValue:
            result = self.maximumValue
            # self.I = 0
        elif result < -self.maximumValue:
            result = -self.maximumValue
            # self.I = 0

        if 0 < result < self.minimumValue:
            result = self.minimumValue
        elif -self.minimumValue < result < 0:
            result = -self.minimumValue
        
        if self.angle:
            print ("*******************")
            print (result)
            print ("*******************")

        return result