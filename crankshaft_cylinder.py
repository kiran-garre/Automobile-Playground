
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


class Cylinder:

    def __init__(self, bore, stroke, start_x, initial_stroke, compression_ratio) -> None:

        global AIR_FLOW_RATE

        self.TIME_STEP = 0
        
        # cylinder stats
        self.radius = bore / 2 # m
        self.stroke = stroke # m
        self.cyl_vol = stroke * self.radius**2 * math.pi # m^3 --> displacement volume
        self.cyl_vol += self.cyl_vol / (compression_ratio - 1) # --> clearance volume
        self.mass = 1 # kg (estimate)
        
        AIR_FLOW_RATE = 0 

        self.x = start_x # m
        self.force = 0 # N
        self.friction = 0

        # realtime stats
        self.total_mols = 0 # moles
        self.grams_of_gas = 0
        self.pressure = 0 # N/m^2
        self.temp = engine_temp # K
        self.available_volume = self.cyl_vol - (self.radius**2 * math.pi * self.x) # m^3
        self.current_stroke = initial_stroke

        # generator function for keeping track of stroke
        self.stroke_gen = self.next_stroke()
        next(self.stroke_gen) # need to send None first to use generator

    def inject(self, throttle, rpm):
        peak_rpm = 5000
        estimated_air = throttle * self.cyl_vol / max(1, (rpm / peak_rpm)) * 1225 # find air in m^3, then convert to grams using density of air
        self.grams_of_gas = estimated_air / 14.7 # use air:fuel ratio to get grams of gas
        self.total_mols = (estimated_air / 28.96) + (self.grams_of_gas / 100) # convert air and gas to moles (gas is approximately 100 g/mol)

    
    def spark(self):

        "PV = nRT -> P = nRT/V"
        if self.total_mols != 0:
            self.temp = 2411 # adiabatic flame temperature of gasoline

    def exhaust(self):
        self.total_mols = 0

    def next_stroke(self):
        i = self.current_stroke
        while True:
            i = i % 4 + 1
            spark = False
            throttle = yield
            omega = yield
            match i:
                case 1:
                    self.inject(throttle, omega)
                case 3:
                    self.spark()
                    spark = True
                case 4:
                    self.exhaust()

            self.update(spark)
            
            yield i

    def stroke_behavior(self, stroke, throttle, omega):
        
        self.current_stroke = stroke
        spark = False
        match stroke:
            case 1:
                self.inject(throttle, omega)
            case 3:
                self.spark()
                spark = True
            case 0: # stroke 4
                self.exhaust()

        self.update(spark)
        


    def update(self, sparkbool):
        old_pressure = self.pressure
        self.available_volume = self.cyl_vol - (self.radius**2 * math.pi * self.x) # m^3
        self.pressure = self.total_mols * R * (self.temp + engine_temp) / (self.available_volume) # N/m^2
        
        if (not sparkbool):
            if (old_pressure != 0): 
                self.temp = self.temp * (self.pressure / old_pressure) # K (Gay Lussac's Law)
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
        
        self.stroke_list = None
        self.configuration_setup(configuration, num_cylinders)

        self.check_angles = np.zeros(num_cylinders)

        


        for i in range(num_cylinders):
            start_x = self.bearing_length * np.cos(self.angles[i]) + self.bearing_length
            self.cylinders[i] = Cylinder(bore, stroke, start_x, self.stroke_list[i], compression_ratio)

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
            
            self.angles = np.array([0, -3 * math.pi, -math.pi, -2 * math.pi])
            self.stroke_list = np.array([3.0, 3.0, 3.0, 3.0])
            self.stroke_list += (self.angles) / (math.pi)

        
        elif config == "I" and num_cylinders == 6: # firing order = 1, 5, 3, 6, 2, 4
            
            self.angles = np.radians([0, -480, -240, -600, -120, -360])
            
            self.stroke_list = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])
            
            self.stroke_list += (self.angles) / (math.pi)
            print(self.stroke_list)
            # sys.exit()



        

    def update(self, torque_loss):

        def update_properties():
            self.torque = np.sum(self.torque_list) - torque_loss
            # print(torque_loss)
            self.alpha = self.torque / self.moment
            self.omega += self.alpha * self.TIME_STEP
            self.theta += self.omega * self.TIME_STEP
            
            self.angles += self.omega * self.TIME_STEP


            for i in range(len(self.cylinders)):
                self.cylinders[i].x = self.bearing_length * np.cos(self.angles[i]) + self.bearing_length
            
            for i in range(len(self.cylinders)):
                self.torque_list[i] = self.bearing_length * self.cylinders[i].force * np.sin(self.angles[i]) # sine is omitted due to error caused by discrete time steps

            self.stroke_list += (self.omega * self.TIME_STEP) / (math.pi)
            self.stroke_list %= 4
    
 


        for j in range(len(self.cylinders)):

            if self.angles[j] + (self.omega * self.TIME_STEP) >= self.check_angles[j]:
                rpm = 60 * self.omega / (2 * math.pi)
                self.cylinders[j].stroke_behavior(round(self.stroke_list[j]) % 4, self.throttle, rpm)
                # self.cylinders[j].current_stroke += 1
                

                self.check_angles[j] += math.pi

        # print(self.torque_list)
        update_properties()


        print(self.torque_list)
        print(self.cylinders[0].grams_of_gas)
        # for cyl in self.cylinders:
        #     print(cyl.temp)
                
        self.multiple = self.theta // math.pi
        self.theta %= self.angle_offset



        self.rpm = self.omega / (2 * math.pi) * 60


    def set_cylinder_time_step(self, t):
        for cyl in self.cylinders:
            cyl.TIME_STEP = t  
        










