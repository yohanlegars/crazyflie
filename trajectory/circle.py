import math
import numpy as np

def circle_trajectory(t):

    T = 25

    if t > T:
        pos = [radius, 0, 2.5]
        vel = [0, 0, 0]
        acc = [0, 0, 0]

    else:
        pos = [5 * np.cos(2 * np.pi * t / T)-5, 5 * np.sin(2 * np.pi * t / T), 2.5 * t / T]
        vel = [-5 * 2 * np.pi / T * np.sin(2 * np.pi * t / T), 5 * 2 * np.pi / T * np.cos(2 * np.pi * t / T), 2.5 / T]
        acc = [-5 * 4 * np.pi**2 / T**2 * np.cos(2 * np.pi * t / T), -5 * 4 * np.pi**2 / T**2 * np.sin(2 * np.pi * t / T), 0]

    s = np.zeros(11)
    s[0] = pos[0]
    s[1] = pos[1]
    s[2] = pos[2]
    s[3] = vel[0]
    s[4] = vel[1]
    s[5] = vel[2]
    s[6] = acc[0]
    s[7] = acc[1]
    s[8] = acc[2]
    s[9] = 0
    s[10] = 0

    state = {'x': s[0:3], 'x_dot': s[3:6], 'x_ddot': s[6:9], 'yaw': s[9], 'yaw_dot': s[10]}
    return state




