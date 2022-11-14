import gym
import env
import numpy as np
from linear_controller import PDcontrolller
from nonlinear_controller import GeometricNonlinearController
from matplotlib import pyplot as plt
from hover import *
from circle import *
from diamond import *
from matplotlib import animation

env = gym.make('Quadrotor-v0')

current_state = env.reset(position=[0, 0, 0])


print("current:", current_state)
dt = 0.01
t = 0

#controller = PDcontrolller()
controller = GeometricNonlinearController()
plan_trajectory = {'x': [], 'y': [], 'z': []}
real_trajectory = {'x': [], 'y': [], 'z': []}
actions = []

while(t < 25):
    desired_state = diamond_trajectory(t)
    control_variable = controller.control(desired_state, current_state)
    action = control_variable['cmd_motor_speeds']
    obs, reward, done, info = env.step(action)
    print("--------------------------")
    print("desired:", desired_state['x'])
    print("current:", obs['x'])

    actions.append(action)

    plan_trajectory['x'].append(desired_state['x'][0])
    plan_trajectory['y'].append(desired_state['x'][1])
    plan_trajectory['z'].append(desired_state['x'][2])
    
    real_trajectory['x'].append(obs['x'][0])
    real_trajectory['y'].append(obs['x'][1])
    real_trajectory['z'].append(obs['x'][2])
    current_state = obs
    t += dt


fig = plt.figure()
ax1 = plt.axes(projection='3d')

ax1.plot3D(plan_trajectory['x'], plan_trajectory['y'], plan_trajectory['z'], 'gray')
ax1.plot3D(real_trajectory['x'], real_trajectory['y'], real_trajectory['z'], 'red')

plt.show()

x_des = np.array(plan_trajectory['x'])
y_des = np.array(plan_trajectory['y'])
z_des = np.array(plan_trajectory['z'])

x = np.array(real_trajectory['x'])
y = np.array(real_trajectory['y'])
z = np.array(real_trajectory['z'])

errorx = np.sum((x-x_des)**2)
errory = np.sum((y-y_des)**2)
errorz = np.sum((z-z_des)**2)

Tracking_performance = errorx + errory + errorz

motor_speed = np.array(actions)
motor_speed_squared = motor_speed**2
Total_battery_storage = np.sum(motor_speed_squared)

print('\nThe tracking performance: ', Tracking_performance)
print('\nThe total time spent to finish the task: ', t)
print('\nThe total battery storage:', Total_battery_storage)












