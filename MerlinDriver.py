from robodkdriver import RoboDK, RobotSerial
import struct
import numpy as np

import numpy as np

motor_reductions = np.array([[1/48., 0,     0,      0,          0,      0], 
                             [0,     1/48., 1/48.,  0,          0,      0], 
                             [0,     0,     1/48.,  0,          0,      0], 
                             [0,     0,     0,      1/24., -1/28.8,     1/24.], 
                             [0,     0,     0,      0,      1/28.8,     -1/48.], 
                             [0,     0,     0,      0,           0,     1/24.]])

motor_sprs = np.array([200, 200, 200, 200, 200, 200]).T

degrees_per_step = (360. / motor_sprs) * motor_reductions.T

def generate_step_instructions(delta_theta, dps = [9, 9, 9, 9, 9, 9]):
    assert((np.sign(delta_theta) == np.sign(dps)).all())
    delta_theta = np.array(delta_theta)
    dps = np.array(dps)
    instructions = [] 
    for _ in range(6):
        min_travel_time = np.inf
        for i in range(6):
u            if delta_theta[i] != 0:
                if travel_time < min_travel_time:
                    min_travel_time = travel_time
            else:
                dps[i] = 0

        if min_travel_time == np.inf:
            break

        steps_per_second = np.linalg.solve(degrees_per_step, dps)
                
        # Compute which Joint to a complete first and then re-calculate motor speeds
        steps_to_take = min_travel_time * steps_per_second
        steps_to_take = steps_to_take.astype(int)
        sub_delta_theta = np.dot(steps_to_take, degrees_per_step.T)

        delta_theta = delta_theta - sub_delta_theta

        instructions.append((steps_to_take.tolist(), steps_per_second.tolist()))
    return instructions

print(*generate_step_instructions([90, 90, 90, -90, 90, 90], [1, 1, 1, -3, 1, 1]),sep="\n")

class MerlinRobot(RobotSerial):

    def run_command(self, cmd, args):
        RoboDK.update_status('working')

        if cmd == 'CONNECT':
            connected = self._connect(port=args[0], baud_rate=int(args[1]))

            if not connected:
                return 'connection_problems'

            return 'ready'

        if cmd == 'DISCONNECT' or cmd == 'STOP':
            disconnected = self._disconnect()

            if disconnected:
                return 'disconnected'

            return 'ready'

        if cmd == 'MOVJ':
            self._send_message('M')
            for i in range(6):
                self._serial.write(bytearray(struct.pack("f", float(args[i]))))
    
            if self._get_message('DONE'):
                return 'ready'

            return 'error'

        if cmd == "MOVL":
            pass # Use the move group planner to figure out joint trajectory

        if cmd == 'CJNT':
            self._send_message("C")

            joints = self._get_message()
            RoboDK.update_status(joints)

            return 'ready'

if __name__ == '__main__':
    RoboDK(MerlinRobot()).run_driver()
