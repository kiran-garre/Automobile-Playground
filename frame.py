
from re import I
from selectors import SelectorKey
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import copy

class Frame:

    def __init__(self, mass, drag_coefficient) -> None:
        self.mass = mass
        self.drag_coefficient = drag_coefficient