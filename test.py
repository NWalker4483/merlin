import time
import struct
import numpy as np
import serial

motor_reductions = np.array([[1/48., 0,     0,      0,          0,        0],
                             [0,     1/48., 0,  0,          0,       0],
                             [0,     -1/48.,     1/48.,  0,          0,        0],
                             [0,     0,     0,      1/24.,    0,    0],
                             [0,     0,     0,      -1/28.8,         1/28.8,     0],
                             [0,     0,     0,      -1/12,           1/24.,        1/24.]])
print(motor_reductions)
motor_sprs = 200.

radians_per_step = (360. / motor_sprs) * motor_reductions 
# port = serial.Serial("/dev/cu.usbmodem1422201", 112500)
# time.sleep(1.5)

def generate_step_instructions(delta_theta, dps=[9, 9, 9, 9, 9, 9]):

    dps = np.sign(delta_theta) * np.abs(dps)
    dps[delta_theta == 0] = 0

    delta_theta = np.array(delta_theta)
    delta_theta.shape = (6,1)

    dps = np.array(dps)
    dps.shape = (6,1)
    
    instructions = []
    for _ in range(6):
        min_travel_time = np.inf
        for i in range(6):
            if delta_theta[i][0] != 0 and dps[i][0] != 0:
                travel_time = delta_theta[i][0]/dps[i][0]
                if travel_time < min_travel_time:
                    min_travel_time = travel_time

        if min_travel_time == np.inf:
            break

        steps_per_second = np.linalg.inv(radians_per_step.T) @ dps
        # Compute which Joint to a complete first and then re-calculate motor speeds
        steps_to_take = min_travel_time * steps_per_second
        print(f"Steps to Take Shape: {steps_to_take.shape}")
        sub_delta_theta = radians_per_step.T @ steps_to_take
        print(f"Sub Delta Shape: {sub_delta_theta.shape} {sub_delta_theta}")
        delta_theta = delta_theta - sub_delta_theta

        instructions.append(
            (steps_to_take, steps_per_second))
    return instructions


def send_instructions(port, instructions):
    port.write('T'.encode())
    port.write(str(len(instructions)).encode())
    for in_set in instructions:
        for val in in_set[0]:
            port.write(bytearray(struct.pack("f", val)))
        for val in in_set[1]:
            port.write(bytearray(struct.pack("f", abs(val))))

speeds = [18, 9, 18, 18, 18, 18]

#[[30, 30, 30, 30, 30, 30],[-30, -30, -30, -30, -30, -30],[15, 0, 0, 0, 0, 0],[-15, 0, 0, 0, 0, 0],[15, 0, 0, 0, 0, 0],[-15, 0, 0, 0, 0, 0],[0, -0, -90, -0, -0, -0]]

animation = \
[[15, 0, 0, 0, 0, 0],
 [0, 15, 0, 0, 0, 0], 
 [0, 0, 15, 0, 0, 0],  
 [0, 0, 0, 15, 0, 0],  
 [0, 0, 0, 0, 15, 0],  
 [0, 0, 0, 0, 0, 15], [-15,-15, -15, -15, -15, -15]]
input("Start:\n")
for frame in animation:
    # send_instructions(port,
    (generate_step_instructions(frame, speeds))
    sec = 5 
    input()

    # for _ in range(sec * 10):
    #     time.sleep(1/10)
    #     port.write('R'.encode())
    #     port.read(12)

    


