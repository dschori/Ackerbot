#!/usr/bin/env python3
import scipy.integrate as integrate
import math
import matplotlib.pyplot as plt
import numpy as np

def euler_spiral(l, A):
    factor = A * math.sqrt(math.pi)
    return (factor * integrate.quad(lambda t: math.cos(math.pi * t * t / 2), 0, l)[0],
        factor * integrate.quad(lambda t: math.sin(math.pi * t * t / 2), 0, l)[0])

def draw():
    x = []
    y = []
    for l in np.arange(-4, 4, 0.01):
        px, py = euler_spiral(l, 1)
        x.append(px)
        y.append(py)
    plt.plot(x, y)
    plt.show()

draw()
