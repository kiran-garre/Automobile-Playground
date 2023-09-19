import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy


class TorqueConverter:

    def __init__(self, k = 2, c = 0.03, a = 0.005, viscosity = 0.05) -> None:
        
        self.TIME_STEP = 0
        
        # constants that change TorqueConverter's behavior
        self.k = k
        self.c = c
        self.a = a
        self.visc = viscosity
        self.impeller_and_fluid_moment = 0.2 + k * viscosity * 4 * 0.05**2 # estimate

        self.driveshaft_moment = 0.3
        
        # stats for the impeller (input) and turbine (output)
        self.impeller_omega = 0
        self.input_torque = 0
        self.turbine_omega = 0
        self.output_torque = 0


    def update(self, shifting, drag_loss, mph):
        
        # calculate approxiate torque multiplication factor and apply it to output_torque
        mf = self.k * (1 - math.exp(-self.c * (self.impeller_omega - self.turbine_omega))) * (self.a * self.visc)
        if mph > 40:
            mf = 1 # locks at high speeds
        self.output_torque = mf * self.input_torque

        # disengage during shift
        if shifting:
            self.output_torque = -drag_loss

        # update values of turbine 
        turbine_alpha = self.output_torque / self.driveshaft_moment
        self.turbine_omega += turbine_alpha * self.TIME_STEP



        
