import numpy as np
from scipy.spatial.transform import Rotation
from numpy.linalg import norm


class GeometricNonlinearController:
    def __init__(self):
        self.Kd = np.diag([33, 33, 32])
        self.Kp = np.diag([83, 83, 242])
        self.Kp_t = np.diag([2700, 2700, 540])
        self.Kd_t = np.diag([200, 200, 40])


        m = 0.030  # weight (in kg) with 5 vicon markers (each is about 0.25g)
        g = 9.81  # gravitational constant
        I = np.array([[1.43e-5, 0, 0],
                      [0, 1.43e-5, 0],
                      [0, 0, 2.89e-5]])  # inertial tensor in m^2 kg
        L = 0.046  # arm length in m
        self.rotor_speed_min = 0
        self.rotor_speed_max = 2700
        self.mass = m
        self.inertia = I
        self.invI = np.linalg.inv(I)
        self.arm_length = L
        self.k_thrust = 2.3e-08
        self.k_drag = 7.8e-11
        self.g = g


    def control(self, desired_state, state):
        """
        :param desired state: pos, vel, acc, yaw, yaw_dot
        :param current state: x, v, q, w
        :return:
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_moment = np.zeros((3,))

        acc = desired_state['x_ddot'] + self.Kp @ (desired_state['x'] - state['x']) + \
              self.Kd @ (desired_state['x_dot'] - state['v'])  # obtain commanded acceleration
        Fc = self.mass * acc + np.array([0, 0, self.mass * self.g])  # obtain commanded force
        R = Rotation.from_quat(state['q']).as_matrix()  # obtain rotation matrix of current orientation

        # Calculation of commanded force u1
        b3 = R[:, 2]  # (3, 1)
        u1 = b3.T @ Fc.reshape([3, 1])

        # Calculation of commanded torque u2
        b3_c = Fc / norm(Fc)  # desired b3 (body-frame Z)

        psi = desired_state['yaw']
        a_psi = np.array([np.cos(psi), np.sin(psi), 0])
        cross_b3c_apsi = np.cross(b3_c, a_psi)
        b2_c = cross_b3c_apsi / norm(cross_b3c_apsi)

        r_des = np.vstack((np.cross(b2_c, b3_c), b2_c, b3_c)).T  # desired rotation matrix
        ss_matrix = r_des.T @ R - R.T @ r_des  # orientation error
        vimap = np.array([-ss_matrix[1, 2], ss_matrix[0, 2], -ss_matrix[0, 1]])
        e_r = 0.5 * vimap  # orientation error

        w_des = np.array([0, 0, desired_state['yaw']])
        e_w = state['w'] - w_des  # angular velocity

        u2 = self.inertia @ (-self.Kp_t @ e_r - self.Kd_t @ e_w)

        gama = self.k_drag / self.k_thrust
        cof_temp = np.array([1, 1, 1, 1, 0, self.arm_length, 0, -self.arm_length, -self.arm_length, 0, self.arm_length,
                             0, gama, -gama, gama, -gama]).reshape(4, 4)

        u = np.array([u1, u2[0], u2[1], u2[2]])
        F_i = np.matmul(np.linalg.inv(cof_temp), u)

        for i in range(4):
            if F_i[i] < 0:
                F_i[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = np.sqrt(F_i[i] / self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max

        cmd_thrust = u1
        cmd_moment[0] = u2[0]
        cmd_moment[1] = u2[1]
        cmd_moment[2] = u2[2]

        cmd_q = Rotation.as_quat(Rotation.from_matrix(r_des))
        cmd_q = cmd_q / norm(cmd_q)

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}

        return control_input