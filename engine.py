

from crankshaft_cylinder import Cylinder, Crankshaft

from re import I
from selectors import SelectorKey
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy

max_torques = []

class Engine:

    def __init__(self, configuration="I", num_cylinders=4, stroke=0.0995, bore=0.0875) -> None:
        self.configuration = configuration
        self.num_cylinders = num_cylinders

        self.hp = 0

        self.cs = Crankshaft(num_cylinders, stroke, bore)

        self.output_torque = 0
        
    def update(self, throttle, gear_ratio, torque_loss):
        
        self.cs.throttle = throttle
        for cyl in self.cs.cylinders:
            spark = False
            match cyl.current_stroke:
                # case 1:
                #     cyl.inject(self.cs.throttle, self.cs.omega)
                case 3:
                    spark = True
        #         # case 4:
        #         #     cyl.exhaust()
            cyl.update(spark)
            
            
        self.cs.update(gear_ratio, torque_loss)
        # max_torques.append(sum(self.cs.torque_list))

        self.hp = self.cs.torque * self.cs.omega / 745.7


# e = Engine()
# # print(e.cs.moment)
# omegas = []
# c0_xs = []
# hps = []
# e.cs.omega = 75
# for t in range(10000):
#     e.update(throttle=1)
#     # print(e.cs.torque)
#     # print(e.cs.cylinders[0].temp, e.cs.cylinders[0].current_stroke, e.cs.cylinders[0].mols)
    
#     omegas.append(e.cs.omega)
#     hps.append(e.hp)
#     c0_xs.append(e.cs.cylinders[3].x)
#     # time.sleep(0.001)
#     # print(e.hp)


# # plt.plot(np.arange(t + 1) * e.cs.TIME_STEP, omegas)
# plt.plot(np.arange(t + 1) * e.cs.TIME_STEP, max_torques)
# # plt.plot(omegas, max_torques)
# # plt.plot(range(t + 1), c0_xs)
# plt.show()