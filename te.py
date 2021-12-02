import time
import struct
import numpy as np
import serial

motor_reductions = np.array([
                             [ 1/24.,  -1/28.8,     1/48.],
                             [ 0,      1/28.8,     -1/24.],
                             [ 0,           0,     1/48.]])

motor_sprs = np.array([200, 200, 200]).T

degrees_per_step = motor_reductions * (360. / motor_sprs)[:,None]

# port = serial.Serial("/dev/cu.usbmodem14101", 112500)
# time.sleep(2)

def generate_step_instructions(delta_theta, dps=[9, 9, 9]):
    dps = np.sign(delta_theta) * np.abs(dps)
    dps[delta_theta == 0] = 0
    delta_theta = np.array(delta_theta)
    dps = np.array(dps)
    print(delta_theta, dps)
    instructions = []
    for _ in range(6):
        # min_travel_time = np.inf
        # for i in range(6):
        #     travel_time = delta_theta[i]/dps[i]
        #     if delta_theta[i] != 0:
        #         if travel_time < min_travel_time:
        #             min_travel_time = travel_time
        #     else:
        #         dps[i] = 0

        # if min_travel_time == np.inf:
        #     break
        pass
    steps_per_second = np.linalg.lstsq(degrees_per_step.T, dps.T)
    print(degrees_per_step, steps_per_second)
generate_step_instructions([90,0,0])