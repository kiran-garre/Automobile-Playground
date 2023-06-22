
from re import I
from selectors import SelectorKey
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy


class WheelSet:

    def __init__(self, mass_per_wheel=27, radius=0.4) -> None:
        
        self.TIME_STEP = 0
        
        self.moment = 0.5 * (mass_per_wheel * 4) * radius**2
        self.radius = radius
        self.final_drive_ratio = 3
        self.omega = 0
        self.force_on_ground = 0

        self.car_mass = 1000
        self.rolling_resistance = 0.01 * self.car_mass * self.radius

        self.linear_speed = 0
        self.drag = 0
    

    def update(self):
        self.linear_speed = self.radius * self.omega
        self.drag = 0.5 * 1.293 * self.linear_speed**2 * 0.2 * 2 # 0.5 * density of air (kg/m^3) * v^2 * drag coefficient * cross sectional area
        




        
        
        



    