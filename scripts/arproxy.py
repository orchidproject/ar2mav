#!/usr/bin/python
import csv
import sys
import os
import socket
import errno
from optparse import OptionParser
import struct
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil


parser = OptionParser()
parser.add_option("-f", "--file", dest="file", help="Csv file with mapping", metavar="FILE", default="map.csv")
parser.add_option("-p", "--port", dest="port", help="Incoming port for ARDrones", metavar="PORT", default="14550")
parser.add_option("-l", "--local", dest="local", help="Local Host Address", metavar="HOST", default="127.0.0.1")
parser.add_option("-v", "--verbose", dest="verbose", help="Verbose", metavar="VERBOSE", default=False)
(options, args) = parser.parse_args()

VIDEO_PORT = 5555
COMMAND_PORT = 5556
NAVDATA_PORT = 5554

COMMAND_MASK = 0b00010001010101 << 18
TAKEOFF_MASK = 1 << 9
EMERGENCY_MASK = 1 << 8

NAVDATA_MESSAGE = "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r"


class ARProxyConnection:

    def __init__(self, connection, sdk, host, verbose=False, control=1.0, repeat=3):
        self.connection = connection
        self.sdk = sdk
        self.host = host
        self.drone = 0
        self.manual = -1
        self.verbose = verbose
        # Manual Control variables
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.seq = 1
        self.control = control
        self.repeat = repeat

    def process_from_drone(self, msg):
        if self.verbose:
            print_msg("From UAV:", msg)
        # Since AUTO_MODE does not get set on the Ar Drone 2.0 this is not feasible
        # a.k.a you can not switch the base_mode MAV_MODE_FLAG_MANUAL_INPUT_ENABLED off
        # if self.manual == -1 and msg.get_type() == "HEARTBEAT":
        # self.manual = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED > 0
        # if self.manual:
        #         print "UAV in MANUAL"
        #     else:
        #         print "UAV not in MANUAL"
        if self.manual == -1:
            self.manual = 0
        self.connection.port.sendto(msg._msgbuf, self.host)
        self.drone = self.connection.last_address

    def process_from_sdk(self, data):
        if not data["drone_state"]["command_mask"]:
            print "Proper stream not on, sending AT_REF"
            self.sdk.sendto(NAVDATA_MESSAGE % self.seq, (self.drone[0], COMMAND_PORT))
            self.seq += 1
        else:
            print "SDK MESSAGE", data

    def process_from_host(self, msg):
        if self.verbose:
            print_msg("From Ground(%d):" % self.manual, msg)
        if self.manual == -1:
            print "Have not connected to drone yet"
            return
        elif msg.get_type() == "SET_MODE":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                self.manual = True
                if self.verbose:
                    print "IN MANUAL MODE"
            else:
                self.manual = False
                if self.verbose:
                    print "MANUAL MODE OFF"
            self.connection.port.sendto(msg._msgbuf, self.drone)
        elif self.manual:
            self.send_manual_command(msg)
        else:
            self.connection.port.sendto(msg._msgbuf, self.drone)

    def send_manual_command(self, msg):
        if msg.get_type() == "COMMAND_LONG":
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                send = (("AT*REF={},%d\r" % (COMMAND_MASK | TAKEOFF_MASK)) * self.repeat).format(
                    *range(self.seq, self.seq + self.repeat + 1))
            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                send = (("AT*REF={},%d\r" % COMMAND_MASK) * self.repeat).format(
                    *range(self.seq, self.seq + self.repeat + 1))
            else:
                print "Unsupported command in Manual Mode: %d" % msg.command
                return
        elif msg.get_type() == "RC_CHANNELS_OVERRIDE":
            send = (self.rc_channels_encode(msg) * self.repeat).format(
                *range(self.seq, self.seq + self.repeat + 1))
        else:
            return
        #self.sdk.sendto("\x01\x00\x00\x00", (self.drone, NAVDATA_PORT))
        self.sock.sendto(send, (self.drone[0], COMMAND_PORT))
        self.seq += self.repeat
        if self.verbose:
            print send


    def rc_channels_encode(self, msg):
        transmit_values = struct.unpack('>iiiiii', struct.pack('>ffffff',
                                                               self.control * (msg.chan1_raw - 1500) / 500,
                                                               self.control * (msg.chan2_raw - 1500) / 500,
                                                               self.control * (msg.chan3_raw - 1500) / 500,
                                                               self.control * (msg.chan4_raw - 1500) / 500,
                                                               self.control * (msg.chan5_raw - 1500) / 500,
                                                               self.control * (msg.chan6_raw - 1500) / 500, ))
        return "AT*PCMD_MAG={},1," + ",".join([str(i) for i in transmit_values]) + "\r"


def print_msg(prefix, msg):
    skip = ["VFR_HUD", "GPS_RAW_INT", "ATTITUDE", "LOCAL_POSITION_NED", "RAW_IMU",
            "SYS_STATUS", "GLOBAL_POSITION_INT", "NAV_CONTROLLER_OUTPUT"]
    type = msg.get_type()
    if type not in skip:
        print "%s %s[%s]" % (prefix, type, ", ".join("%s:%s" % (i, str(msg.__dict__[i])) for i in msg._fieldnames))


def load_map(path):
    f = open(path, mode='r')
    content = csv.reader(f, delimiter=',')
    ip_map = {}
    for row in content:
        ip_map[row[1]] = int(row[2])
    return ip_map


def run_proxy(port, csv_map, host="127.0.0.1", verbose=False):
    mavlink_connection = mavutil.mavlink_connection(host + ":" + port)
    sdk_connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sdk_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sdk_connection.bind((host, NAVDATA_PORT))
    sdk_connection.setblocking(0)
    ip_map = {}
    port_map = {}
    for key in csv_map:
        if verbose:
            print(key + " mapped to " + str(csv_map[key]))
        ip_map[key] = ARProxyConnection(mavlink_connection, sdk_connection,
                                        (host, int(csv_map[key])), verbose)
        port_map[csv_map[key]] = ip_map[key]
    # Main loop
    while True:
        # Receive MAVLink messages
        msg = mavlink_connection.recv_match()
        if msg:
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            elif mavlink_connection.last_address[0] not in ip_map.keys() and mavlink_connection.last_address[0] != host:
                print("Unregistered AUV with IP: " + mavlink_connection.last_address[0])
            elif mavlink_connection.last_address[0] != host:
                ip_map[mavlink_connection.last_address[0]].process_from_drone(msg)
            else:
                port_map[mavlink_connection.last_address[1]].process_from_host(msg)
        # Receive SDK messages
        try:
            packet, address = sdk_connection.recvfrom(65535)
            if address not in ip_map.keys():
                print "Unregistered AUV with IP: ", address
            else:
                port_map[address].process_from_sdk(decode_navdata(packet))
        except socket.error as e:
            if e.errno not in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                raise


# def establish_navdata():
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     sock.bind(("192.168.1.2", 5554))
#     sock.setblocking(0)
#     cmd = "AT*CONFIG=%d,\"general:navdata_demo\",\"%s\"\r"
#     cmd2 = "AT*CTRL=%d,0\r"
#     stream = False
#     nav_data = False
#     t = time.clock();
#     seq = 1
#     while True:
#         if time.clock() - t > 2:
#             print "SSR"
#             t = time.clock()
#         if not stream:
#             print "Init stream"
#             sock.sendto("\x01\x00\x00\x00", ("192.168.1.1", 5554))
#             time.sleep(0.1)
#
#         if not stream:
#             print "Stream on"
#         stream = True
#         data = decode_navdata(packet)
#         if not data["drone_state"]["command_mask"]:
#             print "Send general:navdata_demo"
#             sock.sendto(cmd % (seq, "TRUE"), ("192.168.1.1", 5556))
#             seq += 1
#             continue
#         elif not nav_data:
#             print "Command mask on"
#             sock.sendto(cmd2 % seq, ("192.168.1.1", 5556))
#             seq += 1
#             nav_data = data["drone_state"]["navdata_demo_mask"]
#             if nav_data:
#                 print "Nav data on"
#         if nav_data:
#             print data
#             seq += 1
#         if seq > 100:
#             print "STOP"
#             sock.sendto(cmd % (seq, "FALSE"), ("192.168.1.1", 5556))


def decode_navdata(packet):
    """Decode a navdata packet."""
    offset = 0
    _ = struct.unpack_from("IIII", packet, offset)
    drone_state = dict()
    drone_state['fly_mask'] = _[1] & 1  # FLY MASK : (0) ardrone is landed, (1) ardrone is flying
    drone_state['video_mask'] = _[1] >> 1 & 1  # VIDEO MASK : (0) video disable, (1) video enable
    drone_state['vision_mask'] = _[1] >> 2 & 1  # VISION MASK : (0) vision disable, (1) vision enable */
    drone_state['control_mask'] = _[1] >> 3 & 1  # CONTROL ALGO (0) euler angles control, (1) angular speed control */
    drone_state['altitude_mask'] = _[
                                       1] >> 4 & 1  # ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
    drone_state['user_feedback_start'] = _[1] >> 5 & 1  # USER feedback : Start button state */
    drone_state['command_mask'] = _[1] >> 6 & 1  # Control command ACK : (0) None, (1) one received */
    drone_state['fw_file_mask'] = _[1] >> 7 & 1  # Firmware file is good (1) */
    drone_state['fw_ver_mask'] = _[1] >> 8 & 1  # Firmware update is newer (1) */
    drone_state['fw_upd_mask'] = _[1] >> 9 & 1  # Firmware update is ongoing (1) */
    drone_state['navdata_demo_mask'] = _[1] >> 10 & 1  # Navdata demo : (0) All navdata, (1) only navdata demo */
    drone_state['navdata_bootstrap'] = _[
                                           1] >> 11 & 1  # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
    drone_state['motors_mask'] = _[1] >> 12 & 1  # Motor status : (0) Ok, (1) Motors problem */
    drone_state['com_lost_mask'] = _[1] >> 13 & 1  # Communication lost : (1) com problem, (0) Com is ok */
    drone_state['vbat_low'] = _[1] >> 15 & 1  # VBat low : (1) too low, (0) Ok */
    drone_state['user_el'] = _[1] >> 16 & 1  # User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
    drone_state['timer_elapsed'] = _[1] >> 17 & 1  # Timer elapsed : (1) elapsed, (0) not elapsed */
    drone_state['angles_out_of_range'] = _[1] >> 19 & 1  # Angles : (0) Ok, (1) out of range */
    drone_state['ultrasound_mask'] = _[1] >> 21 & 1  # Ultrasonic sensor : (0) Ok, (1) deaf */
    drone_state['cutout_mask'] = _[1] >> 22 & 1  # Cutout system detection : (0) Not detected, (1) detected */
    drone_state['pic_version_mask'] = _[
                                          1] >> 23 & 1  # PIC Version number OK : (0) a bad version number, (1) version number is OK */
    drone_state['atcodec_thread_on'] = _[1] >> 24 & 1  # ATCodec thread ON : (0) thread OFF (1) thread ON */
    drone_state['navdata_thread_on'] = _[1] >> 25 & 1  # Navdata thread ON : (0) thread OFF (1) thread ON */
    drone_state['video_thread_on'] = _[1] >> 26 & 1  # Video thread ON : (0) thread OFF (1) thread ON */
    drone_state['acq_thread_on'] = _[1] >> 27 & 1  # Acquisition thread ON : (0) thread OFF (1) thread ON */
    drone_state['ctrl_watchdog_mask'] = _[
                                            1] >> 28 & 1  # CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
    drone_state['adc_watchdog_mask'] = _[
                                           1] >> 29 & 1  # ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
    drone_state['com_watchdog_mask'] = _[1] >> 30 & 1  # Communication Watchdog : (1) com problem, (0) Com is ok */
    drone_state['emergency_mask'] = _[1] >> 31 & 1  # Emergency landing : (0) no emergency, (1) emergency */
    data = dict()
    data['drone_state'] = drone_state
    data['header'] = _[0]
    data['seq_nr'] = _[2]
    data['vision_flag'] = _[3]
    offset += struct.calcsize("IIII")
    while 1:
        try:
            id_nr, size = struct.unpack_from("HH", packet, offset)
            offset += struct.calcsize("HH")
        except struct.error:
            break
        values = []
        for i in range(size - struct.calcsize("HH")):
            values.append(struct.unpack_from("c", packet, offset)[0])
            offset += struct.calcsize("c")
        # navdata_tag_t in navdata-common.h
        if id_nr == 0:
            values = struct.unpack_from("IIfffIfffI", "".join(values))
            values = dict(
                zip(['ctrl_state', 'battery', 'theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz', 'num_frames'],
                    values))
            # convert the millidegrees into degrees and round to int, as they
            # are not so precise anyways
            for i in 'theta', 'phi', 'psi':
                values[i] = int(values[i] / 1000)
                # values[i] /= 1000
        data[id_nr] = values
    return data


if __name__ == "__main__":
    csv_map = load_map(options.file)
    # run_proxy(options.port, csv_map, options.local, options.verbose)
    establish_navdata()
