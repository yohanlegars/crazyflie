import numpy as np

def diamond_trajectory(t):

    T = 25


    if t<0:
        pos = np.array([0, 0, 0])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    elif t<T/4:
        pos =  np.array([0,np.sqrt(2),np.sqrt(2)])*t/(T/4)
        vel = np.array([0, np.sqrt(2), np.sqrt(2)]) / (T / 4)
        acc = np.array([0, 0, 0])
    elif t < T / 2:
        pos =  np.array([0, np.sqrt(2), np.sqrt(2)]) * (2 - 4 * t / T) + np.array([0, 0, 2 * np.sqrt(2)]) * ((4 * t / T) - 1)
        vel =  np.array([0, np.sqrt(2), np.sqrt(2)]) * (-4 / T) +  np.array([0, 0, 2 * np.sqrt(2)]) * (4 / T)
        acc = np.array([0, 0, 0])
    elif t < 3 * T / 4:
        pos =  np.array([0, 0, 2 * np.sqrt(2)]) * (3 - (4 * t / T)) +  np.array([0, -np.sqrt(2), np.sqrt(2)]) * ((4 * t / T) - 2)
        vel =  np.array([0, 0, 2 * np.sqrt(2)]) * (-4 / T) +  np.array([0, -np.sqrt(2), np.sqrt(2)]) * (4 / T)
        acc =  np.array([0, 0, 0])
    elif t < T:
        pos =  np.array([0, -np.sqrt(2), np.sqrt(2)]) * (4 - (4 * t / T)) +  np.array([1, 0, 0]) * ((4 * t / T) - 3)
        vel =  np.array([0, -np.sqrt(2), np.sqrt(2)]) * (-4 / T) +  np.array([1, 0, 0]) * (4 / T)
        acc =  np.array([0, 0, 0])
    else:
        pos =  np.array([1, 0, 0])
        vel =  np.array([0, 0, 0])
        acc =  np.array([0, 0, 0])

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