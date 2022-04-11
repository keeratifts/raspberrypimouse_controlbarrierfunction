#! /usr/bin/env python

from importlib.resources import path
import numpy as np
from math import *
import matplotlib.pyplot as plt

import sys

i = [1, 2, 3, 4]

empty_array = np.empty((0, 4), int)
empty_array = np.append(empty_array, np.array([[16, 26, 36], [17, 27, 37]]), axis=0)
print('2D Numpy array:')
print(empty_array)