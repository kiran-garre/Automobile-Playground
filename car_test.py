
import math
import time
import matplotlib.pyplot as plt
import numpy as np

from engine import Engine
from torque_converter import TorqueConverter
from transmission_draft import Transmission
from wheel_set import WheelSet
from frame import Frame



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
        self.shift_time_steps = 0.5 / self.TIME_STEP

        self.transmission = Transmission(self.number_of_gears, self.ratio_list)

        # Wheels
        self.mass_per_wheel = 27
        self.wheel_radius = 0.4

        self.wheels = WheelSet(self.mass_per_wheel, self.wheel_radius)

        # Frame
        self.mass = 1000
        self.drag_coefficient = 0.2

        self.frame = Frame(self.mass, self.drag_coefficient)

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
        self.number_of_cylinders = 4
        self.stroke = 0.0995
        self.bore = 0.0875

        self.engine = Engine(self.configuration, self.number_of_cylinders, self.stroke, self.bore)
        self.engine.cs.omega = 50

        # TorqueConverter
        self.K = 2
        self.C = 0.03
        self.alpha = 0.005
        self.viscosity = 0.05

        self.torque_converter = TorqueConverter(self.K, self.C, self.alpha, self.viscosity)

        # Transmission
        self.number_of_gears = 6
        self.ratio_list = [3.75, 2.25, 1.45, 1.15, 0.9, 0.7]
        self.shift_time_steps = 1

        self.transmission = Transmission(self.number_of_gears, self.ratio_list)

        # Wheels
        self.mass_per_wheel = 27
        self.wheel_radius = 0.4

        self.wheels = WheelSet(self.mass_per_wheel, self.wheel_radius)

        # Frame
        self.mass = 1000
        self.drag_coefficient = 0.2

        self.frame = Frame(self.mass, self.drag_coefficient)

        # Linking
        self.engine.cs.moment += self.torque_converter.impeller_and_fluid_moment
        self.torque_converter.driveshaft_moment = 0.3 + self.transmission.moment + (self.wheels.moment / self.transmission.gear_ratio / self.wheels.final_drive_ratio)


    def update(self):

        throttle = 0.5

        prev_gear_ratio = self.transmission.gear_ratio
        
        # set omega and torque of impeller equal to omega and torque of engine
        self.torque_converter.impeller_omega = self.engine.cs.omega
        self.torque_converter.input_torque = self.engine.cs.torque
        # print(self.engine.cs.torque)

        # set omega of transmission equal to omega of turbine
        self.transmission.omega = self.torque_converter.turbine_omega 

       
        # set omega of wheels equal to omega of transmission / gear_ratio
        self.wheels.omega = self.transmission.omega / self.transmission.gear_ratio / self.wheels.final_drive_ratio

        # apply rolling resistance to turbine/driveshaft torque
        self.torque_converter.output_torque -= self.wheels.rolling_resistance
        torque_loss = (self.wheels.rolling_resistance * self.transmission.gear_ratio) + (self.wheels.drag / self.wheels.radius * self.transmission.gear_ratio)

        
        # update every part
        self.engine.update(throttle, self.transmission.gear_ratio, torque_loss)
        self.torque_converter.update(self.transmission.gear_ratio)
        self.transmission.update(throttle, self.engine_rpm, self.mph)
        self.wheels.update()

        # update mph and engine rpm
        self.mph = self.wheels.linear_speed * 3600 / 1609.34
        self.engine_rpm = self.engine.cs.omega * (60 / (2 * math.pi))

        # decrease torque while shifting due to slippage
        if self.transmission.shifting:
            self.torque_converter.output_torque *= 1

        # when shift is complete
        if self.transmission.just_shifted:
            # print("HEEEEEEEEEEEEE")
            print(self.transmission.current_gear)
            
            self.transmission.omega = self.wheels.omega * self.wheels.final_drive_ratio

            self.torque_converter.turbine_omega = self.transmission.omega * self.transmission.gear_ratio

            self.engine.cs.omega *= self.transmission.gear_ratio / prev_gear_ratio
            self.torque_converter.impeller_omega = self.engine.cs.omega

            #self.engine.cs.omega = self.engine.cs.omega * 1 / (prev_gear_ratio / self.transmission.gear_ratio)
            

            # self.engine.cs.omega = self.torque_converter.turbine_omega * self.transmission.gear_ratio
            self.transmission.just_shifted = False
            prev_gear_ratio = self.transmission.gear_ratio

        self.torque_converter.driveshaft_moment = 0.3 + self.transmission.moment + (self.wheels.moment / self.transmission.gear_ratio / self.wheels.final_drive_ratio)

    

    def run(self, duration):
        torques_list = []
        rpms = []
        mphs = []
        self.calibrate_time()
        steps = int(duration // self.TIME_STEP)
        for _ in range(steps):
            self.update()
            torques_list.append(self.engine.cs.torque)
            mphs.append(self.mph)
            rpms.append(self.torque_converter.turbine_omega * 60 / (2 * math.pi))
            print("RPM: {}\nSPEED: {}\nTIME: {}\n".format(self.engine_rpm, self.mph, _ * self.TIME_STEP))
            

            
            # print(self.mph)
            # if self.transmission.shifting:
            #     print("jj")
            #     plt.axvline(x=_)

            # print("TIME: ", _ * self.TIME_STEP)
            # print(self.transmission.gear_ratio, self.torque_converter.output_torque)
            # rpms.append(self.engine_rpm)
            # time.sleep(0.01)
            
        plt.plot(np.arange(_ + 1) * self.TIME_STEP, torques_list)
        # plt.plot(rpms, torques_list)
        # plt.plot(range(_ + 1), mphs)
        # plt.plot(range(_ + 1), rpms)
        plt.show()

    def display(self):
        speed = "Speed (MPH):\t|"
        revs =  "Revolutions:\t|"
        for _ in range(int(60 * self.mph / 300)):
            speed += "/"
        for _ in range(int(60 * self.engine_rpm / 9000)):
            revs += "/"
        print(speed, end="\r")
        print(revs, end="\r")
        

    

car = Car()
car.run(20)
        







        
       




    

        





