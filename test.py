import time
import struct
import numpy as np
import serial

def array(*args, **kwargs):
    kwargs.setdefault("dtype", np.float32)
    return np.array(*args, **kwargs)

motor_reductions = np.array([[1/48., 0,     0,      0,          0,        0],
                             [0,     1/48., 0,  0,          0,       0],
                             [0,     -1/48.,     1/48.,  0,          0,        0],
                             [0,     0,     0,      1/24.,    0,    0],
                             [0,     0,     0,      -1/28.8,         1/28.8,     0],
                             [0,     0,     0,      -1/12.,           1/24.,        1/24.]], dtype=np.float32)

FULL_ROTATION = 360.
SPR = 200.

degrees_per_step = motor_reductions * (FULL_ROTATION / SPR)
degrees_per_step_inv = np.linalg.inv(degrees_per_step)

target_speeds = array([9, 9, 9, 9, 0, 0])
target_speeds.shape = (6,1)

current_poses = array([0,0,0,0,0,0])
current_poses.shape = (6,1)

def update_arm_state():
    pass

def solve_motor_speeds():
    pass
{
  ();
  Matrix<6, 1> delta_theta
  // Compute Appropriate Errors
  for (int joint = 0; joint < 6; joint++)
  {
    float d_theta = target_poses(joint) - current_poses(joint);
    // Ensure that an exact solution exists by rounding to the nearest multiple of degrees_per_step(joint, joint))
    float result = abs(d_theta) + (degrees_per_step(joint, joint) / 2.);
    result -= fmod(result, degrees_per_step(joint, joint));
    result *= d_theta > 0 ? 1 : -1;
    d_theta = result;
    
    delta_theta(joint) = d_theta;
    if (d_theta == 0) target_speeds(joint) = 0;
    target_speeds(joint) = abs(target_speeds(joint)) * (d_theta > 0 ? 1 : -1);
  }
   Serial.print("\nFirst Delta Theta:");
    Serial << delta_theta;
    Serial.println("\n");

  for (int stage = 0; stage < 6; stage++)
  {
    float min_travel_time = BIG_NUMBER;
    for (int joint = 0; joint < 6; joint++)
    {
      if ((abs(delta_theta(joint)) >= degrees_per_step(joint, joint)) and (target_speeds(joint) != 0))
      {
        float travel_time = delta_theta(joint) / target_speeds(joint);
        if (travel_time < min_travel_time)
          min_travel_time = travel_time;
      } 
      else {
        target_speeds(joint) = 0;
        }
    }
    if (min_travel_time == BIG_NUMBER) break;

    Matrix<6, 1> steps_per_second = degrees_per_step_inv * target_speeds;
    Matrix<6, 1> steps_to_take = steps_per_second * abs(min_travel_time);
    Matrix<6, 1> sub_delta_theta = degrees_per_step * steps_to_take;
    delta_theta -= sub_delta_theta;

    if(1){
    Serial.print("\nStage: ");
    Serial.print(stage + 1);
    Serial.print("\nSub Delta Theta: ");
    Serial << sub_delta_theta;
    Serial.println("\nCurrent Poses: ");
    Serial << current_poses;
    Serial.println(" ");
    }
    
    for (int i = 0; i < 6; i++)
      cmd_buffer[stage][0][i] = steps_to_take(i);
    for (int i = 0; i < 6; i++)
      cmd_buffer[stage][1][i] = steps_per_second(i);

    cmd_len = stage + 1;
    interrupt = true;
  }
  cmd_idx = -1;
}


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

        steps_per_second = np.linalg.inv(degrees_per_step) @ dps
        # Compute which Joint to a complete first and then re-calculate motor speeds
        steps_to_take = min_travel_time * steps_per_second
        # print(f"Steps to Take Shape: {steps_to_take}")
        sub_delta_theta = degrees_per_step @ steps_to_take
        # print(f"Sub Delta Shape: {sub_delta_theta.shape} {sub_delta_theta}")
        delta_theta = delta_theta - sub_delta_theta

        instructions.append(
            (steps_to_take.tolist(), steps_per_second.tolist()))
    return instructions


def send_instructions(port, instructions):
    port.write('T'.encode())
    port.write(str(len(instructions)).encode())
    for in_set in instructions:
        for val in in_set[0]:
            port.write(bytearray(struct.pack("f", val)))
        for val in in_set[1]:
            port.write(bytearray(struct.pack("f", abs(val))))



print()
# port = serial.Serial("/dev/cu.usbmodem1422201", 112500)
# time.sleep(1.5)

#[[30, 30, 30, 30, 30, 30],[-30, -30, -30, -30, -30, -30],[0.261799, 0, 0, 0, 0, 0],[-0.261799, 0, 0, 0, 0, 0],[0.261799, 0, 0, 0, 0, 0],[-0.261799, 0, 0, 0, 0, 0],[0, -0, -90, -0, -0, -0]]

animation = \
[[0.261799, 0.261799, 0.261799, 0.261799, 0, 0], [-0.261799, -0.261799, -0.261799, -0.261799, 0, 0]]
input("Start:\n")
for frame in animation:
    # send_instructions(port,
    print(generate_step_instructions(frame, speeds))
    sec = 5 
    input()

    # for _ in range(sec * 10):
    #     time.sleep(1/10)
    #     port.write('R'.encode())
    #     port.read(12)

    



