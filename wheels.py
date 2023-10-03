
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy


class Wheels:

    def __init__(self, final_drive_ratio=3, drag_coef=0.2, cross_sectional_area=2, mass_per_wheel=27, radius=0.24) -> None:
        
        self.TIME_STEP = 0
        
        self.moment = 0.5 * (mass_per_wheel * 4) * radius**2
        self.radius = radius
        self.final_drive_ratio = final_drive_ratio
        self.omega = 0

        self.car_mass = 1000
        self.rolling_resistance = 0.01 * self.car_mass * self.radius # approximation of rolling resistance torque

        self.drag_coef = drag_coef
        self.cross_sectional_area = cross_sectional_area
        
        self.linear_speed = 0
        self.drag = 0
    

    def update(self):
        self.linear_speed = self.radius * self.omega
        self.drag = 0.5 * 1.293 * self.linear_speed**2 * self.drag_coef * self.cross_sectional_area 
        




        
        
        



    
