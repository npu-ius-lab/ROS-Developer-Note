#!/usr/bin/env python3

from __future__ import print_function
import sys
import time
from argparse import ArgumentParser

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("\nYou may need to install it with:\npip3 install --user pymavlink\n")
    sys.exit(1)

try:
    import serial
except ImportError as e:
    print("Failed to import pyserial: " + str(e))
    print("\nYou may need to install it with:\npip3 install --user pyserial\n")
    sys.exit(1)


class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        if isinstance(b, str):
            b = b.encode('utf-8')
        
        self.debug("sending '%s' of len %u\n" % (b, len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70: n = 70
            chunk = b[:n]
            buf = list(chunk)
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0, 0, len(chunk), buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True, timeout=0.1)
        if m is not None:
            data_bytes = bytes(m.data[:m.count])
            try:
                self.buf += data_bytes.decode('utf-8', errors='replace')
            except Exception as e:
                print(f"Error decoding received data: {e}")

    def read_response(self, timeout=1.0):
        """Read all available output from the shell for a given duration."""
        output = ""
        start_time = time.time()
        while time.time() - start_time < timeout:
            self._recv() # Try to receive new data
            if self.buf:
                output += self.buf
                self.buf = '' # Clear buffer after reading
            time.sleep(0.05)
        return output

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('port', metavar='PORT', nargs='?', default='tcp:127.0.0.1:5760',
                        help='Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT.')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                        help="Mavlink port baud rate (default=57600)", default=57600)
    args, unknown = parser.parse_known_args()


    if args.port is None:
        # Autodetection logic here...
        print("Error: No port specified and autodetection not implemented in this snippet.")
        return

    mav_serialport = None
    try:
        print("Connecting to MAVLINK on {}...".format(args.port))
        mav_serialport = MavlinkSerialPort(args.port, args.baudrate, devnum=10)
        
        # --- ROBUST DELAY-BASED EXECUTION ---

        # 1. "Wake up" the shell and clear any old buffers.
        print("\n--- Initializing shell... ---")
        mav_serialport.write('\n\n')
        time.sleep(1)
        initial_output = mav_serialport.read_response(timeout=1.0)
        if initial_output:
            print("--- Initial shell output: ---")
            sys.stdout.write(initial_output)
            sys.stdout.flush()
            print("-----------------------------")

        # 2. Send the 'ekf2 stop' command.
        print("\n>>> Sending command: ekf2 stop")
        mav_serialport.write('ekf2 stop\n')
        
        # Read the immediate response from the stop command.
        response = mav_serialport.read_response(timeout=1.0)
        if response:
            sys.stdout.write(response)
            sys.stdout.flush()

        # 3.Wait for a sufficient amount of time for EKF2 to fully stop.
        print("\n--- Waiting 10 seconds for EKF2 to stop completely... ---")
        time.sleep(10)

        # 4. Send the 'ekf2 start' command.
        print("\n>>> Sending command: ekf2 start")
        mav_serialport.write('ekf2 start\n')
        
        # Read the response from the start command.
        response = mav_serialport.read_response(timeout=1.0)
        if response:
            sys.stdout.write(response)
            sys.stdout.flush()

        print("\n--- All commands sent successfully. ---")

    except Exception as e:
        print(f"\nAn error occurred: {e}")

    finally:
        if mav_serialport:
            mav_serialport.close()
            print("Connection closed.")

if __name__ == '__main__':
    main()
