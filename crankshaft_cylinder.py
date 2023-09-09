
from re import I
from selectors import SelectorKey
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy

   
R = 8.3145
engine_temp = 20 + 273.15 # K
AIR_FLOW_RATE = 0.001 * 1.3 * 0.8 # displacement * air density * volumetric efficiency (assume constant for now)
Y = 1.4
    


class Cylinder:

    def __init__(self, bore, stroke, start_x, initial_stroke, compression_ratio) -> None:

        global AIR_FLOW_RATE

        self.TIME_STEP = 0
        
        # cylinder stats
        self.radius = bore / 2 # m
        self.stroke = stroke # m
        self.cyl_vol = stroke * self.radius**2 * math.pi # m^3 
        self.cyl_vol += self.cyl_vol / (compression_ratio / 1) # avoids div by 0 error
        self.mass = 1 # (math.pi * self.radius**2 * (stroke / 4.5) * 7850) # mass of piston in kg (density of carbon steel ~ 7850 kg/m^3) -> m^3 * kg/m^3 = kg
        
        AIR_FLOW_RATE = 0 # bore * stroke * 1.35 # 0.0797591 # (((self.cyl_vol * 4 * 61023.7) * (1.29) * 733 / 3456) * 0.000471947) # displacement * air density with mols * volumetric efficiency * max expected omega

        # m^3 * kg/m^3

        self.x = start_x # m
        self.force = 0 # N = kg*m/s^2
        self.friction = 0

        # realtime stats
        self.mols = 0 # mols
        self.pressure = 0 # N/m^2
        self.temp = engine_temp # K
        self.available_volume = self.cyl_vol - (self.radius**2 * math.pi * self.x) # m^3
        self.current_stroke = initial_stroke

        # generator function for keeping track of stroke
        self.stroke_gen = self.next_stroke()
        next(self.stroke_gen) # need to send None first to use generator

    def inject(self, throttle, rpm, gear_ratio):
        estimated_air = 0
        k = 3e-5
        c = 1.0008
        if rpm < 3000:
            estimated_air = k * throttle * 0.9817425815 * math.sqrt(rpm)
        else:
            estimated_air = k * throttle * (math.sqrt(3000) - (c**(gear_ratio / 2))**(rpm - 3000))
        self.mols = estimated_air + estimated_air / 14.7 # air to fuel ratio
    
    def spark(self):

        "PV = nRT -> P = nRT/V"
        
        if self.mols != 0:
            fuel_mass_fraction = 1 / (1 + 14.7)
            air_mass_fraction = 14.7 * fuel_mass_fraction
            Q = self.mols * (44e6 * fuel_mass_fraction) / (2 * fuel_mass_fraction)
            Cp = 1005
            self.temp = Q / (self.mols * Cp) # K

    def exhaust(self):
        self.mols = 0

    def next_stroke(self):
        i = self.current_stroke
        while True:
            i = i % 4 + 1
            spark = False
            throttle = yield
            gear_ratio = yield
            omega = yield
            match i:
                case 1:
                    self.inject(throttle, omega, gear_ratio)
                case 3:
                    self.spark()
                    spark = True
                case 4:
                    self.exhaust()

            self.update(spark)
            
            yield i


    def update(self, sparkbool):
        old_pressure = self.pressure
        self.available_volume = self.cyl_vol - (self.radius**2 * math.pi * self.x) # m^3

        if True:
            self.available_volume = self.cyl_vol / 2

        self.pressure = self.mols * R * (self.temp + engine_temp) / (self.available_volume) # N/m^2
        

        if (not sparkbool):
            if (old_pressure != 0): 
                self.temp = self.temp * (self.pressure / old_pressure)**((Y - 1)/Y) # K (Charles' Law)
            else:
                self.temp = engine_temp

        self.force = self.pressure * (math.pi * self.radius**2) # N/m^2 * m^2 = N
    

class Crankshaft:

    def __init__(self, num_cylinders, configuration, stroke, bore, compression_ratio) -> None:

        self.TIME_STEP = 0.00015
        
        self.mass = 23 # kg
        self.bearing_length = stroke / 2
        self.thickness = 0.05 # m

        self.num_cylinders = num_cylinders
        self.cylinders = [None] * num_cylinders
        self.angle_offset = np.radians(720 / num_cylinders)

        self.configuration_setup(configuration, num_cylinders)
        print(self.angles)


        for i in range(num_cylinders):
            start_x = self.bearing_length * np.cos(self.angles[i]) + self.bearing_length
            self.cylinders[i] = Cylinder(bore, stroke, start_x, self.start_stroke[i], compression_ratio)

        self.theta = 0 # rad 
        self.omega = 0 # rad/s
        self.alpha = 0 # rad/s^2
        self.torque = 0
        self.multiple = 0

        self.rpm = 0

        self.moment = (self.cylinders[0].mass * self.bearing_length**2) * self.num_cylinders + (0.5 * self.mass * (self.thickness / 2)**2) + (0.5 * 13.6 * 0.2**2) # estimation of moment of inertia?

        self.throttle = 0

        self.torque_list = np.zeros((num_cylinders,))

    def configuration_setup(self, config, num_cylinders):
        

        
        if config == "I" and num_cylinders == 4: # firing order = 1, 3, 4, 2
            self.original_angles = np.array([0, -3 * math.pi, -math.pi, -2 * math.pi])
            
            self.angles = np.array([0, -3 * math.pi, -math.pi, -2 * math.pi])
            self.verticals = np.array([0, math.pi, math.pi, 0])
            self.start_stroke = np.array([3, 4, 2, 1])

        
        elif config == "I" and num_cylinders == 6: # firing order = 1, 5, 3, 6, 2, 4
            
            self.original_angles = np.radians([0, -480, -240, -600, -120, -360])
            self.angles = np.radians([0, -480, -240, -600, -120, -360])
            print(self.angles)
            
            self.start_stroke = np.array([3, 3, 1, 2, 2, 4])

            self.verticals = np.array([0, -480, -240, -600, -120, -360])


        

    def update(self, gear_ratio, torque_loss):

        def update_properties():
            self.torque = np.sum(self.torque_list) - torque_loss
            self.alpha = self.torque / self.moment
            self.omega += self.alpha * self.TIME_STEP
            self.theta += self.omega * self.TIME_STEP

            self.angles = (self.original_angles + self.theta)
            for i in range(len(self.cylinders)):
                self.cylinders[i].x = self.bearing_length * np.cos(self.angles[i]) + self.bearing_length
            
            for i in range(len(self.cylinders)):
                self.torque_list[i] = self.bearing_length * self.cylinders[i].force * np.sin(self.angles[i]) # sine is omitted due to error caused by discrete time steps
                print(self.angles[i])
        
            
        

        update_properties()

        stroke_list = [0, 0, 0, 0, 0, 0]
        for i in range(len(self.cylinders)):
            stroke_list[i] = self.cylinders[i].current_stroke
        print(stroke_list, self.theta)



        
        
        # for i in range(len(stroke_list)): # should always be false
        #     match stroke_list[i]:
        #         case 1:
        #             print(self.torque_list[i] < 0, stroke_list[i])
        #         case 2:
        #             print(self.torque_list[i] > 0, stroke_list[i])
        #         case 3:
        #             print(self.torque_list[i] < 0, stroke_list[i])
        #         case 4:
        #             print(self.torque_list[i] != 0, stroke_list[i])
        
        strokes_passed = int(self.theta // self.angle_offset)
        for i in range(strokes_passed):
            # sys.exit()
            # this checks if it is time for a new stroke for each cylinder independently; the first loop until theta == angle_offset handles the angle offset
            for j in range(len(self.cylinders)):
                check_angle = self.angles[j] - ((strokes_passed - i - 1) * math.pi) - self.original_angles[j]
                
                # addition for second condition accounts for 1 iteration error
                if check_angle >= 0 and check_angle < self.angle_offset + (self.omega * self.TIME_STEP):
                    print(j)
                    cyl = self.cylinders[j]
                    cyl.stroke_gen.send(self.throttle)
                    cyl.stroke_gen.send(gear_ratio)
                    cyl.current_stroke = cyl.stroke_gen.send(self.rpm)
                    next(cyl.stroke_gen)

                    self.original_angles[j] = -math.pi

            # necessary for if power stroke gets skipped
            update_properties()

            self.angles %= 2 * math.pi
                
        self.multiple = self.theta // math.pi
        self.theta %= 2 * math.pi



        self.rpm = self.omega / (2 * math.pi) * 60


    def set_cylinder_time_step(self, t):
        for cyl in self.cylinders:
            cyl.TIME_STEP = t  
        








