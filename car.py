import math
import time
import matplotlib.pyplot as plt
import numpy as np
import pygame
import sys
import ast

from engine import Engine
from torque_converter import TorqueConverter
from transmission import Transmission
from wheels import Wheels


class Car:
    
    def __init__(self) -> None:

        self.TIME_STEP = 1
        self.engine_rpm = 0
        self.mph = 0

        self.all_args = self.read_parameters("parameters.txt")
        self.initialize()

    
    # read parameters from parameters.txt and put all the properly typed data into arrays
    def read_parameters(self, file_name):

        default_engine_args = ["I", 6, 0.08, 0.0896, 9.3, 1.4, 5500]
        default_tc_args = [2, 0.03, 0.005, 0.05]
        default_transmission_args = [6, [3.2, 2.6, 1.8, 1.3, 1, 0.8], 0.3]
        default_wheel_args = [3.15, 0.33, 2, 175927, 0.24]
        default_list = [default_engine_args, default_tc_args, default_transmission_args, default_wheel_args]

        engine_args = []
        torque_converter_args = []
        transmission_args = []
        wheel_args = []
        
        arg_lists = [engine_args, torque_converter_args, transmission_args, wheel_args]
        
        with open(file_name, 'r') as file:
            current_list = 0
            counter = 0
            for line in file:
                line = line.strip()
                if line:
                    if line[0] == '#':
                        continue
                    elif line[0] == '*':
                        counter = 0
                        current_list += 1
                    else:
                        try:
                            data = ast.literal_eval(line.split("=")[-1]) # split at equals sign, choose type using ast
                            arg_lists[current_list].append(data)
                        except:
                            arg_lists[current_list].append(default_list[current_list][counter]) # if field is left blank, use default value
                        counter += 1

        return arg_lists

    
    def calibrate_time(self, override=0):
        
        if override <= 0:
            start = time.time()
            print("Calibrating...")
            end = time.time()
            printtime = end - start
            start = time.time()
            for _ in range(1000):
                self.update()
                time.sleep(printtime)
            end = time.time()
            override = (end - start) / 1000
        
        self.initialize()
        
        self.TIME_STEP = override
        self.engine.cs.set_cylinder_time_step(self.TIME_STEP)
        self.engine.cs.TIME_STEP = self.TIME_STEP
        self.torque_converter.TIME_STEP = self.TIME_STEP
        self.transmission.TIME_STEP = self.TIME_STEP
        self.wheels.TIME_STEP = self.TIME_STEP
        

    # sets all parameters
    def initialize(self):
        
        # set arguments from parameters.txt to corresponding engine part
        self.engine = Engine(*self.all_args[0])
        self.torque_converter = TorqueConverter(*self.all_args[1])
        self.transmission = Transmission(*self.all_args[2])
        self.wheels = Wheels(*self.all_args[3])
        
        # starting engine
        self.engine.cs.omega = 50

        # Linking
        self.engine.cs.moment += self.torque_converter.impeller_and_fluid_moment
        self.torque_converter.driveshaft_moment += self.transmission.moment + (self.wheels.moment / self.transmission.gear_ratio / self.wheels.final_drive_ratio)

    def update(self):

        self.throttle = 1
        
        # when shift is complete
        prev_gear_ratio = self.transmission.gear_ratio
        
        # set omega and torque of impeller equal to omega and torque of engine
        self.torque_converter.impeller_omega = self.engine.cs.omega
        self.torque_converter.input_torque = self.engine.cs.torque

        # set omega of transmission equal to omega of turbine
        self.transmission.input_omega = self.torque_converter.turbine_omega 
       
        # set omega of wheels equal to omega of transmission / final_drive_ratio
        self.wheels.omega = self.transmission.output_omega / self.wheels.final_drive_ratio

        # apply rolling resistance to turbine/driveshaft torque
        self.torque_converter.output_torque -= self.wheels.rolling_resistance * self.wheels.radius / self.wheels.final_drive_ratio
        drag_loss = (self.wheels.drag * self.wheels.radius / (self.transmission.gear_ratio * self.wheels.final_drive_ratio))

        # update every part
        self.engine.update(self.throttle, drag_loss)
        self.torque_converter.update(self.transmission.shifting, drag_loss)
        self.transmission.update(self.throttle, self.engine_rpm, self.mph)
        self.wheels.update()

        # update mph and engine rpm
        self.mph = self.wheels.linear_speed * 3600 / 1609.34
        self.engine_rpm = self.engine.cs.omega * (60 / (2 * math.pi))

        # use wheel speed to set the rotational speed of other parts (updating in opposite direction)
        if self.transmission.just_shifted:
                    
            self.transmission.output_omega = self.wheels.omega * self.wheels.final_drive_ratio

            self.torque_converter.turbine_omega = self.transmission.output_omega * self.transmission.gear_ratio
            self.torque_converter.driveshaft_moment = 0.3 + self.transmission.moment + (self.wheels.moment / self.transmission.gear_ratio / self.wheels.final_drive_ratio)

            self.engine.cs.omega *= self.transmission.gear_ratio / prev_gear_ratio
            self.engine.cs.moment += prev_gear_ratio / self.transmission.gear_ratio 
        
            # shifting process done; wait for next shift
            self.transmission.just_shifted = False


    def demo_run(self, duration, override=0, animate=False):
        
        self.calibrate_time(override)
        steps = int(duration // self.TIME_STEP)

        if animate:
            # pygame setup
            pygame.init()
            screen = pygame.display.set_mode((1000, 720))
            pixel_stroke = 200 / self.engine.cs.cylinders[0].stroke
            pixel_x_list = np.linspace(200, 800, self.engine.num_cylinders)
    
        for _ in range(steps):
            
            self.update()
            print("RPM: {}\nSPEED: {}\nGEAR: {}\nTIME: {}\n".format(self.engine_rpm, self.mph, self.transmission.current_gear, _ * self.TIME_STEP))
            
            # PYGAME ANIMATION #######################
            
            if animate:
                # poll for events
                # pygame.QUIT event means the user clicked X to close your window
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        sys.exit()

                # fill the screen with a color to wipe away anything from last frame
                screen.fill("black")

                for x, cyl in zip(pixel_x_list, self.engine.cs.cylinders):
                    color = "blue"
                    if cyl.current_stroke == 3:
                        color = "red"
                    pygame.draw.circle(screen, color, [x, 720 - (250 + pixel_stroke * cyl.x)], 40)

                # flip() the display to put your work on screen
                pygame.display.flip()
                
        plt.show()
    
        pygame.quit()






        
       




    

        




