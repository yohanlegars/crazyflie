import numpy as np


def tj_from_line(start_pos, end_pos, time_ttl, t_c):

    v_max = (end_pos - start_pos) * 2 / time_ttl

    if t_c >= 0 and t_c < time_ttl / 2:

        vel = v_max * t_c / (time_ttl / 2)
        pos = start_pos + t_c * vel / 2
        acc = [0, 0, 0]
    else:
        vel = v_max * (time_ttl - t_c) / (time_ttl / 2)
        pos = end_pos - (time_ttl - t_c) * vel / 2
        acc = [0, 0, 0]

    return [pos, vel, acc]


def hover_trajectory(t):
    T = 25

    spos = np.array([0, 0, 1])
    epos = np.array([0, 0, 2])

    if t < T / 4:
        [pos, vel, acc] = tj_from_line(spos, epos, T / 4, t)

    elif t > T / 4:
        [pos, vel, acc] = tj_from_line(epos, epos, 3 * T / 4, t)

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


