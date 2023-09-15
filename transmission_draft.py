import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy


class Transmission:

    def __init__(self, number_of_gears, ratio_list, shift_time) -> None:
        
        self.TIME_STEP = 0
        
        self.number_of_gears = number_of_gears
        self.ratio_list = ratio_list
        self.shift_time = shift_time
        
        self.current_gear = 1
        self.gear_ratio = self.ratio_list[self.current_gear - 1]
        
        self.moment = 1
        
        self.input_omega = 0
        self.output_omega = 0

        self.shift_delay = self.shift_delay_gen()
        self.shifting = False
        self.just_shifted = False


    def shift_delay_gen(self):

        """
        Generator function that returns False during shift delay and
        True when shift is complete
        """

        number_of_steps = int(self.shift_time // self.TIME_STEP)
        while True:
            for _ in range(number_of_steps):
                yield False
            yield True
    
    
    def switch_gears(self, up):
        if up == None:
            pass
        elif up :
            self.current_gear += 1
        else:
            self.current_gear -= 1
        self.gear_ratio = self.ratio_list[self.current_gear - 1]


    def shifting_logic (self, throttle, engine_rpm, vehicle_speed):

        """
        Defines conditions for shifting gears; returns True for upshift, 
        False for downshift, and None for no shift
        """
        
        upshift_points = [
            (6000 * throttle, 15 * throttle),   
            (9000 * throttle, 25 * throttle),  
            (9000 * throttle, 35 * throttle),  
            (9000 * throttle, 50 * throttle),
            (8000 * throttle, 70 * throttle),
            (8000 * throttle, 90 * throttle),
            (8000 * throttle, 100 * throttle),

        ]

        downshift_points = [
            (1, 10),   
            (1, 20),  
            (1, 30),  
            (1, 45), 
            (1, 60),  
        ]

        if self.current_gear <= self.number_of_gears - 1:
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
        
        """
        Update function creates a delay of approximately shift_time seconds using
        shift_delay generator; when shift == True, the transmission shifts gears
        """
        
        self.output_omega = self.input_omega / self.gear_ratio
        shift = 0

        where_shift = self.shifting_logic(throttle, engine_rpm, vehicle_speed)
        if where_shift != None or self.shifting:
            self.shifting = True
            shift = next(self.shift_delay)
            if shift:
                self.switch_gears(where_shift)
                self.gear_ratio = self.ratio_list[self.current_gear - 1]
                self.shifting = False
                self.just_shifted = True

        


        




        
        
        
      


