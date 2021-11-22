import sys
from robodkdriver import RoboDK, RobotSerial
import struct
import time
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
            self._send_message('M') #{angle}'.format(cmd=cmd, angle=args[0]))
            for i in range(6):
                self._serial.write(bytearray(struct.pack("f", float(args[i]))))
    
            if self._get_message('DONE'):
                return 'ready'

            return 'error'

        if cmd == 'CJNT':
            self._send_message("C")

            joints = self._get_message()
            RoboDK.update_status(joints)

            return 'ready'

if __name__ == '__main__':
    RoboDK(MerlinRobot()).run_driver()
