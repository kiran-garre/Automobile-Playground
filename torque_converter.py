
from re import I
from selectors import SelectorKey
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy

class TorqueConverter:

    def __init__(self, K = 2, C = 0.03, a = 0.005, viscosity = 0.05) -> None:
        
        self.TIME_STEP = 0
        
        self.K = K
        self.C = C
        self.a = a
        self.visc = viscosity
        self.impeller_and_fluid_moment = 0.2 + K * viscosity * 4 * 0.05**2

        self.driveshaft_moment = 0.3
        
        self.impeller_omega = 0
        self.input_torque = 0
        
        self.turbine_omega = 0
        self.output_torque = 0

    def update(self, gear_ratio):
        mf = self.K * (1 - math.exp(-self.C * (self.impeller_omega - self.turbine_omega))) * (1 + self.a * self.visc)
        
        self.output_torque = mf * self.input_torque

        # print(self.output_torque, self.driveshaft_moment)
        turbine_alpha = self.output_torque / self.driveshaft_moment
        # print(turbine_alpha)
        self.turbine_omega += turbine_alpha * self.TIME_STEP



        
