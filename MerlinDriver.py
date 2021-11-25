from robodkdriver import RoboDK, RobotSerial
import struct
import numpy as np

class MerlinRobot(RobotSerial):
    def calculate_motor_speeds(target_dps):
        assert(len(target_dps) == 6)

        motor_sprs = np.array([200, 200, 200, 200, 200, 200]).T

        target_dps = np.array(target_dps)

        motor_reductions = np.array([[1/48., 0,     0,      0,          0,      0], 
                                    [0,     1/48., 1/48.,  0,          0,      0], 
                                    [0,     0,     1/48.,  0,          0,      0], 
                                    [0,     0,     0,      1/24., -1/28.8,     1/24.], 
                                    [0,     0,     0,      0,      1/28.8,     -1/48.], 
                                    [0,     0,     0,      0,           0,     1/24.]])

        degrees_per_step =  np.divide(motor_reductions.T, 360/motor_sprs)

        steps_per_second = np.linalg.solve(degrees_per_step, target_dps)

        delays = np.array(1000000/steps_per_second, dtype = int)

        return delays

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
