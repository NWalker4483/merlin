import time
import struct
import numpy as np
import serial

motor_reductions = np.array([[1/48., 0,     0,      0,          0,        0],
                             [0,     1/48., -1/48.,  0,          0,       0],
                             [0,     0,     1/48.,  0,          0,        0],
                             [0,     0,     0,      1/24.,    -1/28.8,    -1/12],
                             [0,     0,     0,      0,         1/28.8,     1/24.],
                             [0,     0,     0,      0,           0,        1/24.]])

motor_sprs = 200

degrees_per_step = (360. / motor_sprs) * motor_reductions 

port = serial.Serial("/dev/cu.usbmodem14201", 112500)
time.sleep(1.5)

def generate_step_instructions(delta_theta, dps=[9, 9, 9, 9, 9, 9]):

    dps = np.sign(delta_theta) * np.abs(dps)
    dps[delta_theta == 0] = 0
    delta_theta = np.array(delta_theta)
    dps = np.array(dps)
    
    instructions = []
    for _ in range(6):
        min_travel_time = np.inf
        for i in range(6):
            if delta_theta[i] != 0 and dps[i] != 0:
                travel_time = delta_theta[i]/dps[i]
                if travel_time < min_travel_time:
                    min_travel_time = travel_time
            else:
                dps[i] = 0

        if min_travel_time == np.inf:
            break
    
        steps_per_second = np.linalg.solve(degrees_per_step.T, dps)
        assert(np.allclose(np.dot(degrees_per_step.T, steps_per_second), dps))
        # print("\n", steps_per_second, "\n")
        # Compute which Joint to a complete first and then re-calculate motor speeds
        steps_to_take = min_travel_time * steps_per_second
        steps_to_take = steps_to_take.astype(int)
        sub_delta_theta = np.dot(steps_to_take, degrees_per_step)

        delta_theta = delta_theta - sub_delta_theta

        instructions.append(
            (steps_to_take.tolist(), steps_per_second.tolist()))
    return instructions


def send_instructions(port, instructions):
    port.write('W'.encode())
    port.write(str(len(instructions)).encode())
    for in_set in instructions:
        for val in in_set[0]:
            port.write(bytearray(struct.pack("f", val)))
        for val in in_set[1]:
            port.write(bytearray(struct.pack("f", abs(val))))

speeds = [18, 18, 18, 18, 18, 18]

animation = \
[[15, -15, 0, 0, 0, 0], [-15, 15, 0, 0, 0, 0],
 [0, 15, 0, 0, 0, 0], [0, -15, 0, 0, 0, 0], 
 [0, 0, 15, 0, 0, 0], [0,0, -15, 0, 0, 0], 
 [0, 0, 0, 15, 0, 0], [0,0, 0, -15, 0, 0], 
 [0, 0, 0, 0, 15, 0], [0,0, 0, 0, -15, 0], 
 [0, 0, 0, 0, 0, 15], [0,0, 0, 0, 0, -15]]

input("Start")
for frame in animation:
    send_instructions(port,
    generate_step_instructions(frame, speeds))
    sec = 5 

    # for _ in range(10):
    #     time.sleep(1/10)
    #     port.write('R'.encode())
    #     pass

    input()# time.sleep(1.5)


