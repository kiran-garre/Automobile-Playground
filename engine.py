from crankshaft_cylinder import Crankshaft

import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy


class Engine:

    def __init__(self, configuration="I", num_cylinders=4, stroke=0.0995, bore=0.0875, compression_ratio = 11, volumetric_efficiency=1, peak_rpm = 5000) -> None:

        self.num_cylinders = num_cylinders
        
        # create crankshaft
        self.cs = Crankshaft(num_cylinders, configuration, stroke, bore, compression_ratio, volumetric_efficiency, peak_rpm)

        # engine stats
        self.hp = 0
        self.torque = 0
        
    def update(self, throttle, torque_loss):
        
        # update crankshaft
        self.cs.throttle = throttle
        self.cs.update(torque_loss)
        
        # update each cylinder; spark is True when current_stroke == 3 (power stroke)
        for cyl in self.cs.cylinders:
            spark = False
            match cyl.current_stroke:
                case 3:
                    spark = True
            cyl.update(spark)

        self.hp = self.cs.torque * self.cs.omega / 745.7
        self.torque = self.cs.torque

