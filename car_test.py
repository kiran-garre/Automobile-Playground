import math
import time
import matplotlib.pyplot as plt
import numpy as np
import pygame
import sys

from engine import Engine
from torque_converter import TorqueConverter
from transmission_draft import Transmission
from wheel_set import WheelSet


class Car:
    
    def __init__(self) -> None:

        self.TIME_STEP = 1
        
        # Engine
        self.configuration = "I"
        self.number_of_cylinders = 4
        self.stroke = 0.0995
        self.bore = 0.0875

        self.engine = Engine(self.configuration, self.number_of_cylinders, self.stroke, self.bore)

        # TorqueConverter
        self.K = 2
        self.C = 0.03
        self.alpha = 0.005
        self.viscosity = 0.05

        self.torque_converter = TorqueConverter(self.K, self.C, self.alpha, self.viscosity)

        # Transmission
        self.number_of_gears = 6
        self.ratio_list = [3.75, 2.25, 1.45, 1.15, 0.9, 0.7]
        self.shift_time = 0.5

        self.transmission = Transmission(self.number_of_gears, self.ratio_list, self.shift_time)

        # Wheels
        self.mass_per_wheel = 27
        self.wheel_radius = 0.4

        self.wheels = WheelSet(self.mass_per_wheel, self.wheel_radius)

        # Linking
        self.engine.cs.moment += self.torque_converter.impeller_and_fluid_moment
        self.torque_converter.driveshaft_moment += self.transmission.moment + self.wheels.moment

        self.mph = 0 
        self.engine_rpm = 0


    def setup_helper(self):
        pass

    def calibrate_time(self, override=0):
        
        if override <= 0:
            start = time.time()
            print("Calibrating...")
            end = time.time()
            printtime = end - start
            start = time.time()
            for _ in range(100):
                self.update()
                time.sleep(printtime)
            end = time.time()
            override = (end - start) / 110
        
        self.reset()
        
        self.TIME_STEP = override
        self.engine.cs.set_cylinder_time_step(self.TIME_STEP)
        self.engine.cs.TIME_STEP = self.TIME_STEP
        self.torque_converter.TIME_STEP = self.TIME_STEP
        self.transmission.TIME_STEP = self.TIME_STEP
        self.wheels.TIME_STEP = self.TIME_STEP
        

    def reset(self):
        # Engine
        self.configuration = "I"
        self.number_of_cylinders = 6
        self.stroke = 0.0946
        self.bore = 0.082
        self.compression_ratio = 10.2
        self.volumetric_efficiency = 1.2

        self.engine = Engine(self.configuration, self.number_of_cylinders, self.stroke, self.bore, self.compression_ratio, self.volumetric_efficiency)
        self.engine.cs.omega = 50

        # TorqueConverter
        self.K = 2
        self.C = 0.03
        self.alpha = 0.005
        self.viscosity = 0.05

        self.torque_converter = TorqueConverter(self.K, self.C, self.alpha, self.viscosity)

        # Transmission
        self.number_of_gears = 8
        self.ratio_list = [5.25, 3.36, 2.172, 1.72, 1.316, 1, 0.822, 0.64]
        self.shift_time = 0.5

        self.transmission = Transmission(self.number_of_gears, self.ratio_list, self.shift_time)

        # Wheels
        self.final_drive_ratio = 3.154
        self.drag_coefficient = 0.2
        self.cross_sectional_area = 2

        self.mass_per_wheel = 27
        self.wheel_radius = 0.4

        self.wheels = WheelSet(self.final_drive_ratio, self.drag_coefficient, self.cross_sectional_area, self.mass_per_wheel, self.wheel_radius)

        # Linking
        self.engine.cs.moment += self.torque_converter.impeller_and_fluid_moment
        self.torque_converter.driveshaft_moment = 0.3 + self.transmission.moment + (self.wheels.moment / self.transmission.gear_ratio / self.wheels.final_drive_ratio)

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
        self.torque_converter.update()
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
            self.engine.cs.moment *= prev_gear_ratio / self.transmission.gear_ratio
        
            # shifting process done; wait for next shift
            self.transmission.just_shifted = False


    def demo_run(self, duration, override=0, animate=False):
        torques_list = []
        rpms = []
        mphs = []
        pressures = []
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
            torques_list.append(self.engine.cs.torque)
            pressures.append(self.engine.cs.cylinders[1].pressure)
            mphs.append(self.mph)
            rpms.append(self.torque_converter.turbine_omega * 60 / (2 * math.pi))
            print("RPM: {}\nSPEED: {}\nTIME: {}\n".format(self.engine_rpm, self.mph, _ * self.TIME_STEP))
            
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
                       
        plt.plot(np.arange(_ + 1) * self.TIME_STEP, np.array(torques_list))
        plt.show()
    
        pygame.quit()

    











        
       




    

        





