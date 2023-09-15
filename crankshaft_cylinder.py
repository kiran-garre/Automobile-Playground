import sys
import numpy as np
import matplotlib.pyplot as plt
import time
from math import pi, floor


R = 8.3145
ENGINE_TEMP = 20 + 273.15 # K


class Cylinder:

    def __init__(self, bore, stroke, start_x, initial_stroke, compression_ratio, volumetric_efficiency) -> None:

        self.TIME_STEP = 0
        
        # cylinder stats
        self.radius = bore / 2 
        self.stroke = stroke 
        self.cyl_vol = stroke * self.radius**2 * pi # displacement volume
        self.cyl_vol += self.cyl_vol / (compression_ratio - 1) # add clearance volume
        self.mass = 1 
        self.ve = volumetric_efficiency 

        self.x = start_x # m
        
        # realtime stats
        self.total_mols = 0 # moles
        self.kg_of_gas = 0
        self.kg_of_air = 0
        self.pressure = 0
        self.force = 0 
        self.temp = ENGINE_TEMP
        self.available_volume = self.cyl_vol - (self.radius**2 * pi * self.x) 
        self.current_stroke = initial_stroke

    def inject(self, throttle, rpm):
        peak_rpm = 5000
        self.kg_of_air = throttle * self.ve * self.cyl_vol / max(1, (rpm / peak_rpm))**0.9 * 1.293 # kg
        self.kg_of_gas = self.kg_of_air / 14.7 # use air:fuel ratio to get kg of gas
        self.total_mols = (self.kg_of_air / 0.029) + (self.kg_of_gas / 0.114) # convert air and gas to moles (air is 0.029 kg/mol, gas is 0.114 kg/mol)

    
    def spark(self):
       
        "Q = mcT"

        if self.total_mols != 0:
            specific_heat = 1005 * (1 + 0.22 * (self.pressure - 1e5) / 1e5) # specific heat changes with pressure
            energy_released = 48000000 * self.kg_of_gas # 48 MJ per kg of gas * heat efficiency of engine (~20%)
            total_mass = self.kg_of_air + self.kg_of_gas
            self.temp = energy_released / (total_mass * specific_heat) 
            self.total_mols *= 17 / 12.5

    def exhaust(self):
        self.kg_of_air = 0
        self.kg_of_gas = 0
        self.total_mols = 0
        self.temp = ENGINE_TEMP


    # chooses correct function based on stroke number
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
        
        old_available_vol = self.available_volume
        self.available_volume = self.cyl_vol - (self.radius**2 * pi * self.x) # m^3

        # prevents intake stroke from adding significant torque
        if self.current_stroke == 1:
            self.available_volume = self.cyl_vol

        # polytropic process equation only works if pressure and moles are not 0
        if not (sparkbool or self.total_mols == 0 or self.pressure == 0):
            y = 1.4
            n = 1 / (1 - y)
            self.pressure = (self.pressure * old_available_vol**n) / self.available_volume**n # polytropic process equation: PV^n = constant
            self.temp = self.pressure * self.available_volume / self.total_mols / R # ideal gas law
        
        # handles power strokes and when previous iteration's pressure is 0
        else:
            self.pressure = self.total_mols * R * (self.temp) / (self.available_volume) * 0.2 
        
        self.force = self.pressure * (pi * self.radius**2) # N/m^2 * m^2 = N


class Crankshaft:

    def __init__(self, num_cylinders, configuration, stroke, bore, compression_ratio, volumetric_efficiency) -> None:

        self.TIME_STEP = 0
        
        self.mass = 23 
        self.crank_length = stroke / 2 # approximate

        self.num_cylinders = num_cylinders

        self.cylinders = [None] * num_cylinders 
        self.angle_offset = 2 * pi / num_cylinders # angle offset between each piston/crank
        
        self.stroke_list = None 
        self.configuration_setup(configuration, num_cylinders)

        self.check_angles = np.zeros(num_cylinders) # angles that must be passed for each cylinder to move to the next stroke

        
        # initialize starting positions of pistons
        for i in range(num_cylinders):
            start_x = self.crank_length * np.cos(self.angles[i]) + self.crank_length
            self.cylinders[i] = Cylinder(bore, stroke, start_x, self.stroke_list[i], compression_ratio, volumetric_efficiency)

        self.theta = 0 
        self.omega = 0 
        self.alpha = 0 
        self.torque = 0 

        self.moment = 0.3 # estimation of moment of inertia

        self.throttle = 0 

        self.torque_list = np.zeros(num_cylinders) 
        
    
    # sets up angles and strokes for each cylinder
    def configuration_setup(self, config, num_cylinders):
        
        if config == "I" and num_cylinders == 4: # firing order = 1, 3, 4, 2
            
            self.angles = np.array([0, -3 * pi, -pi, -2 * pi])
            self.stroke_list = np.array([3.0, 3.0, 3.0, 3.0])
            self.stroke_list += (self.angles) / (pi)

        
        elif config == "I" and num_cylinders == 6: # firing order = 1, 5, 3, 6, 2, 4
            
            self.angles = np.radians([0, -480, -240, -600, -120, -360])
            self.stroke_list = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])
            self.stroke_list += (self.angles) / (pi)

        
    def update(self, torque_loss):

        def update_properties():

            """
            Updates each value: 
            - torque alpha, omega, and angles 
            - positions and new torques for each cylinder
            - strokes for each cylinder
            """

            self.torque = np.sum(self.torque_list) - torque_loss
            self.alpha = self.torque / self.moment
            self.omega += self.alpha * self.TIME_STEP
            self.angles += self.omega * self.TIME_STEP

            for i in range(len(self.cylinders)):
                self.cylinders[i].x = self.crank_length * np.cos(self.angles[i]) + self.crank_length
            
            for i in range(len(self.cylinders)):
                self.torque_list[i] = self.crank_length * self.cylinders[i].force * np.sin(self.angles[i])

            self.stroke_list += (self.omega * self.TIME_STEP) / pi
            self.stroke_list %= 4

        # handles stroke changes
        for j in range(len(self.cylinders)):

            if self.angles[j] >= self.check_angles[j]:
                rpm = 60 * self.omega / (2 * pi)
                self.cylinders[j].stroke_behavior(floor(self.stroke_list[j]), self.throttle, rpm)
                self.check_angles[j] += pi

        update_properties()


    def set_cylinder_time_step(self, t):
        for cyl in self.cylinders:
            cyl.TIME_STEP = t  
        










