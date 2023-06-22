
from multiprocessing.dummy import current_process
from re import I
from selectors import SelectorKey
import sys
from tabnanny import check
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy


class Transmission:

    def __init__(self, number_of_gears, ratio_list) -> None:
        
        self.TIME_STEP = 0
        
        self.number_of_gears = number_of_gears

        # list of gear ratios (engine : driveshaft)
        self.ratio_list = ratio_list
        
        self.current_gear = 1
        self.gear_ratio = self.ratio_list[self.current_gear - 1]
        
        self.moment = 1
        self.omega = 0

        self.rpm = 0

        self.shift_delay = self.shift_delay_gen()
        self.shifting = False
        self.just_shifted = False


    def shift_delay_gen(self):
        number_of_steps = int(0.5 // self.TIME_STEP)
        while True:
            for _ in range(number_of_steps):
                yield False
            yield True
        

    def calculate_switch(self, throttle, engine_rpm):
        if (throttle < 0.7 and engine_rpm > 2500 and self.current_gear <= 5):
            return True
        elif (throttle >= 0.7 and engine_rpm > 7000 and self.current_gear <= 5):
            return True
        elif (engine_rpm < 1200 and self.current_gear >= 2 and throttle <= 0.10):
            return False
        else:
            return None

    
    def switch_gears(self, up):
        if up == None:
            pass
        elif up :
            self.current_gear += 1
        else:
            self.current_gear -= 1
        self.gear_ratio = self.ratio_list[self.current_gear - 1]


    def shifting_logic (self, throttle, engine_rpm, vehicle_speed):

        upshift_points = [
            (1000, 10),   
            (3000, 15),  
            (3000, 35),  
            (3200, 55),
            (3500, 65),
        ]


        downshift_points = [
            (1, 10),   
            (1, 20),  
            (1, 30),  
            (1, 45), 
            (1, 60),  
        ]

        if self.current_gear <= 5:
            for check_rpm, check_speed in upshift_points[self.current_gear - 1:]:
                if engine_rpm > check_rpm and vehicle_speed > check_speed:
                    return True
        
        elif self.current_gear >= 2:
            for check_rpm, check_speed in downshift_points:
                if engine_rpm < check_rpm and vehicle_speed < check_speed:
                    print("downshift")
                    return False
        
        return None

        


    def update(self, throttle, engine_rpm, vehicle_speed):
        
        self.rpm = self.omega / (2 * math.pi) * 60

        self.shifting = False
        step = 0

        where_shift = self.shifting_logic(throttle, engine_rpm, vehicle_speed)
        if where_shift != None or self.shifting:
            self.shifting = True
            # print(where_shift)
            step = next(self.shift_delay)
            if step:
                self.switch_gears(where_shift)
                self.gear_ratio = self.ratio_list[self.current_gear - 1]
                self.shifting = False
                self.just_shifted = True


        




        
        
        
      


