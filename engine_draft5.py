
from re import I
from selectors import SelectorKey
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy

   

TIME_STEP = 0.0015
GAS_MOLAR = 120 # g/mol
ENERGY_DENSITY = 46000 
AVG_EFFICIENCY = 0.25 
R = 8.3145
engine_temp = 20 + 273.15 # K
AIR_FLOW_RATE = 0.001 * 1.3 * 0.8 # displacement * air density * volumetric efficiency (assume constant for now)
Y = 1.4
omega = 75
theta = 0


class Engine:

    def __init__(self, configuration, num_cylinders, stroke, bore) -> None:
        self.configuration = configuration
        self.num_cylinders = num_cylinders
        self.omega = 0

        self.cylinder_list = [None] * num_cylinders

        self.c0 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0.0995, initial_stroke=3)
        self.c1 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0, initial_stroke=4)
        self.c2 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0, initial_stroke=2)
        self.c3 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0.0995, initial_stroke=1)

        self.cs = Crankshaft(4, c0, c1, c2, c3)
        
    def update(self):
        for cyl in self.cs.cylinders:
            spark = False

            match cyl.current_stroke:
            #     # case 1:
            #     #     cyl.inject(cs.throttle, cs.omega)
                case 3:
                    cyl.spark()
                    spark = True
                case 4:
                    cyl.exhaust()
            cyl.update(spark)
        cs.update()

    


class Cylinder:

    def __init__(self, bore, stroke, start_x, initial_stroke) -> None:

        global AIR_FLOW_RATE
        
        # cylinder stats
        self.radius = bore / 2 # m
        self.stroke = stroke # m
        self.cyl_vol = stroke * self.radius**2 * math.pi + 5e-5 # m^3 (addition for clearance volume to avoid div by 0 error)
        self.mass = 1 # (math.pi * self.radius**2 * (stroke / 4.5) * 7850) # mass of piston in kg (density of carbon steel ~ 7850 kg/m^3) -> m^3 * kg/m^3 = kg
        
        AIR_FLOW_RATE = (self.cyl_vol * 4) * (1.3 / 0.028) * 0.8 * 733 / 3456 # displacement * air density * volumetric efficiency * max expected omega

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


    # def inject_(self, throttle):

    #     estimated_air_per_step = throttle * AIR_FLOW_RATE * TIME_STEP
    #     self.mols += estimated_air_per_step / 14.7 # air to fuel ratio
    #     # self.pressure = self.mols * R * self.temp / self.available_volume # mols * J/(mol * K) * K / m^3 = N/m^2
    #     # self.update()

    def inject(self, throttle, omega):
        estimated_air = math.pi / omega * throttle * AIR_FLOW_RATE 
        # print(estimated_air)
        self.mols = estimated_air + estimated_air / 14.7 # air to fuel ratio
        # self.pressure = self.mols * R * self.temp / self.available_volume # mols * J/(mol * K) * K / m^3 = N/m^2
        # self.update()
    
    def spark(self):

        "PV = nRT -> P = nRT/V"

        self.temp = 1773.15 # K
        # self.update(True)

    def exhaust(self):
        # print(self.mols)
        self.mols = 0
        self.temp = engine_temp
        # self.update()

    def next_stroke(self):
        i = self.current_stroke
        while True:
            i = i % 4 + 1
            spark = False
            throttle = yield
            omega = yield
            match self.current_stroke:
                case 1:
                    print(self.mols == 0)
                    self.inject(throttle, omega)
                    # print(self.mols == 0)
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
        
        # if (not sparkbool):
        #     self.temp = self.available_volume * self.temp / old_vol # K (Charles' Law)
        
        self.pressure = self.mols * R * self.temp / (self.available_volume) # N/m^2
        print(self.mols)
        
        if (not sparkbool):
            if (old_pressure != 0): 
                self.temp = self.temp * (self.pressure / old_pressure)**((Y - 1)/Y) # K (Charles' Law)
            else:
                self.temp = engine_temp


        self.force = self.pressure * (math.pi * self.radius**2) # N/m^2 * m^2 = N


        

    def display(self):
        print("Gas in cylinder: ", self.mols * 1000 * 120)
        print("Temperature: ", self.temp)
        print("Pressure:", self.pressure)
        print("Available Volume: ", self.available_volume)

        print("Force: ", self.force)
        print("Acceleration: ", self.a)
        print("Velocity: ", self.v)
        print("Position:", self.x)

        print()
    

class Crankshaft:

    def __init__(self, num_cylinders, cyl0, cyl1, cyl2, cyl3) -> None:

        self.num_cylinders = num_cylinders

        self.cyl0 = cyl0
        self.cyl1 = cyl1
        self.cyl2 = cyl2
        self.cyl3 = cyl3

        self.mass = 23 # kg
        self.bearing_length = cyl0.stroke / 2
        self.thickness = 0.05 # m

        self.cylinders = [self.cyl0, self.cyl1, self.cyl2, self.cyl3]
        
        self.angle0 = 0 # radians from the vertical
        self.angle1 = math.pi
        self.angle2 = math.pi
        self.angle3 = 0

        self.angles = np.array([self.angle0, self.angle1, self.angle2, self.angle3])
        
        self.cyl0.x = self.bearing_length * math.cos(self.angle0) + self.bearing_length
        self.cyl1.x = self.bearing_length * math.cos(self.angle1) + self.bearing_length
        self.cyl2.x = self.bearing_length * math.cos(self.angle2) + self.bearing_length
        self.cyl3.x = self.bearing_length * math.cos(self.angle3) + self.bearing_length
        
        self.joint0 = [cyl0.x, self.angles[0]]
        self.joint1 = [cyl1.x, self.angles[1]]
        self.joint2 = [cyl2.x, self.angles[2]]
        self.joint3 = [cyl3.x, self.angles[3]]

        
        self.theta = 0 # rad 
        self.omega = 0 # rad/s
        self.alpha = 0 # rad/s^2
        self.multiple = 0 
        self.rotational_ke = 0

        self.moment = (cyl0.mass * self.bearing_length**2) * self.num_cylinders + (0.5 * self.mass * (self.thickness / 2)**2) # estimation of moment of inertia?

        self.force_list = []
        self.stroke_list = []
        self.torque_list = []

        self.throttle = 0

    def update(self):
        
        

        
        cyl0_torque = self.bearing_length * self.cyl0.force * (np.sin(self.angle0))
        cyl1_torque = self.bearing_length * self.cyl1.force * (np.sin(self.angle1)) 
        cyl2_torque = self.bearing_length * self.cyl2.force * (np.sin(self.angle2))
        cyl3_torque = self.bearing_length * self.cyl3.force * (np.sin(self.angle3))        



        self.force_list = [self.cyl0.force, self.cyl1.force, self.cyl2.force, self.cyl3.force]
        self.stroke_list = [self.cyl0.current_stroke, self.cyl1.current_stroke, self.cyl2.current_stroke, self.cyl3.current_stroke]
        self.torque_list = [cyl0_torque, cyl1_torque, cyl2_torque, cyl3_torque]

        # for i in range(len(self.stroke_list)): # should always be false
        #     match self.stroke_list[i]:
        #         case 1:
        #             print(self.torque_list[i] < 0)
        #         case 2:
        #             print(self.torque_list[i] > 0)
        #         case 3:
        #             print(self.torque_list[i] < 0)
        #         case 4:
        #             print(self.torque_list[i] > 0)

        

        # print(self.cyl0.current_stroke)
        self.alpha = (cyl0_torque + cyl1_torque + cyl2_torque + cyl3_torque) / self.moment
        print(self.alpha)
        self.omega += self.alpha * TIME_STEP
        self.theta += (self.omega * TIME_STEP) 
        print(self.theta)

        
        if (self.theta // (math.pi)) > self.multiple:
            # print("h")
            for _ in range(int((self.theta // (math.pi)) - self.multiple)):
                
                self.cyl0.stroke_gen.send(self.throttle)
                self.cyl1.stroke_gen.send(self.throttle)
                self.cyl2.stroke_gen.send(self.throttle)
                self.cyl3.stroke_gen.send(self.throttle)

                self.cyl0.current_stroke = self.cyl0.stroke_gen.send(self.omega)
                self.cyl1.current_stroke = self.cyl1.stroke_gen.send(self.omega)
                self.cyl2.current_stroke = self.cyl2.stroke_gen.send(self.omega)
                self.cyl3.current_stroke = self.cyl3.stroke_gen.send(self.omega)
                
                next(self.cyl0.stroke_gen)
                next(self.cyl1.stroke_gen)
                next(self.cyl2.stroke_gen)
                next(self.cyl3.stroke_gen)

                
            self.multiple = self.theta // (math.pi)

        self.angle0 = self.theta + 0
        self.angle1 = self.theta + math.pi
        self.angle2 = self.theta + math.pi
        self.angle3 = self.theta + 0

        self.cyl0.x = self.bearing_length * math.cos(self.angle0) + self.bearing_length
        self.cyl1.x = self.bearing_length * math.cos(self.angle1) + self.bearing_length
        self.cyl2.x = self.bearing_length * math.cos(self.angle2) + self.bearing_length
        self.cyl3.x = self.bearing_length * math.cos(self.angle3) + self.bearing_length

        self.rotational_ke = 0.5 * self.moment * self.omega**2
        
    def display(self):
        print("Omega: ", self.omega)
        print("Theta: ", self.theta)








c0 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0.0995, initial_stroke=3)
c1 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0, initial_stroke=4)
c2 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0, initial_stroke=2)
c3 = Cylinder(stroke=0.0995, bore=0.0875, start_x=0.0995, initial_stroke=1)

cs = Crankshaft(4, c0, c1, c2, c3)

c0_xs = []
c1_xs = []
c2_xs = []
c3_xs = []
omegas = []
thetas = []
strokes = [2]

i = 0
cs.omega = omega
injection_counter_3 = 0
cs.throttle = 0.05
for t in range(0, 10000, 1):
    for cyl in cs.cylinders:
        spark = False
        match cyl.current_stroke:
        #     # case 1:
        #     #     cyl.inject(cs.throttle, cs.omega)
            case 3:
                cyl.spark()
                spark = True
            case 4:
                cyl.exhaust()
        cyl.update(spark)

    # print(c3.mols)

    
    c0_xs.append(c0.x)
    # c1_xs.append(c1.x)
    # c2_xs.append(c2.x)
    # c3_xs.append(c3.x)
    omegas.append(cs.omega)
    print()
    # print(theta)
    # thetas.append(cs.theta)
    # strokes.append(c0.current_stroke)
    
    i += 1

    # print(cs.force_list)
    print(cs.stroke_list)
    # print(theta)
    # print(omega)
    # print(c3.current_stroke)
    # print(cs.torque_list)
    # print(c2.current_stroke, c2.force, 180*cs.angle2/math.pi % 360, np.cos(cs.angle2), np.sin(cs.angle2))
    # print(180*cs.theta/math.pi % 360)
    # print(sum(cs.torque_list))

    if t > 3000:
        cs.throttle = 0.05
 
    # print(cs.angle0)
  
    
    # print(cs.alpha)


    # print(cs.theta)
    # print(t * TIME_STEP)
    # cs.display()
    before = cs.rotational_ke
    cs.update()
    after = cs.rotational_ke
    theta = cs.theta
    omega = cs.omega
    # print((after - before) / TIME_STEP)

# plt.plot(range(10000), c0_xs)
# plt.scatter(range(100), c1_xs)
# plt.scatter(range(10), c2_xs)
# plt.scatter(range(10), c3_xs)
# plt.plot(range(i), c0_xs)
plt.plot(range(i), omegas)
plt.show()
    

        



    






# contains_gas = [False, False, False, False]
# to_be_injected = -1
# move_to_next = True
# for t in range(0, 1000, 1):
#     if not contains_gas.any(): # if no cylinder contains gas
#         if move_to_next:
#             to_be_injected = injection_order()
#             move_to_next = False
#         else:
#             if cs.angles[to_be_injected] == math.pi:
#                 cs.cylinders[to_be_injected].inject()
#                 contains_gas[to_be_injected] = True
    
#     else:
#         to_be_sparked = contains_gas.index(True)
#         if cs.angles[to_be_sparked] == 0:
#             cs.cylinders[to_be_sparked].spark()
#             contains_gas[to_be_sparked] = False



    










    

class Gear:
    def __init__(self, radius, teeth) -> None:
        self.radius = radius
        self.teeth = teeth

        self.alpha = 0
        self.omega = 0

        mass = (math.pi * radius**2) * 7.85 # mass of gear (density ~ 7.85)
        self.moment = mass * radius**2

        self.energy = 0

    def spin(self, applied_torque):
        self.alpha = applied_torque

    def update(self):
        self.omega += self.alpha * TIME_STEP
        self.energy = 0.5 * self.moment * self.omega**2
