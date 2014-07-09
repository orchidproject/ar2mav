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

from tools import *

parser = OptionParser()
parser.add_option("-f", "--file", dest="file", help="Csv file with mapping", metavar="FILE", default="map.csv")
parser.add_option("-p", "--port", dest="port", help="Incoming port for ARDrones", metavar="PORT", default="14550")
parser.add_option("-l", "--local", dest="local", help="Local Host Address", metavar="HOST", default="127.0.0.1")
parser.add_option("-v", "--verbose", dest="verbose", help="Verbose", metavar="VERBOSE", default=False)
(options, args) = parser.parse_args()

NAVDATA_MESSAGE = "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r"
NAVDATA_OPTIONS = 1 << NAVDATA_OPTIONS_STR["DEMO"] | 1 << NAVDATA_OPTIONS_STR["VISION_DETECT"] | \
    1 << NAVDATA_OPTIONS_STR["GAMES"] | 1 << NAVDATA_OPTIONS_STR["MAGNETO"] | \
    1 << NAVDATA_OPTIONS_STR["HDVIDEO_STREAM"] | 1 << NAVDATA_OPTIONS_STR["WIFI"] | \
    1 << NAVDATA_OPTIONS_STR["GPS"]

# Messages
# HEARTBEAT - sanitised X
# ATTITUDE - sanitised
# CONTROLLER_OUTPUT - not use by AR Drone 2.0
# CURRENT_MISSION -  sanitised X
# FILTERED_POSITION = GLOBAL_POSITION_INT ?
# GPS = GPS_RAW_INT ?
# MISSION_ITEM - not used by AR Drone 2.0 during manual
# RAW_IMU - not used by AR Drone 2.0
# RC = RC_CHANNELS_RAW - not used by AR Drone 2.0
# STATE - sanitised
# STATUS - sanitised
# VFR_HUD - not used by AR Drone 2.0

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
        self.cmd_seq = 1
        self.control = control
        self.repeat = repeat
        # MAVLink meta data variables
        self.base_mode = None
        self.custom_mode = None
        self.status = None
        self.mission_seq = 0
        self.time = 0

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
        self.manual = 0
        if msg.get_type() == "HEARTBEAT":
            self.base_mode = msg.base_mode
            self.custom_mode = msg.custom_mode
            self.status = msg.status
        if msg.get_type() == "MISSION_CURRENT":
            self.mission_seq = msg.seq
        self.connection.port.sendto(msg._msgbuf, self.host)
        self.drone = self.connection.last_address

    def process_from_sdk(self, data):
        if not data["drone_state"]["command_mask"]:
            print "Proper stream not on, sending AT_REF"
            self.sdk.sendto(NAVDATA_MESSAGE % self.cmd_seq, (self.drone[0], PORTS["AT"]))
            self.cmd_seq += 1
        elif time.clock() - self.time > 1:
            self.connection.port.sendto(mavutil.mavlink.MAVLink_heartbeat_message(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                self.base_mode,
                self.custom_mode,
                self.status
            ), self.host)
            self.connection.port.sendto(mavutil.mavlink.MAVLink_mission_current_message(
                self.mission_sequence
            ), self.host)
            # TODO
            # SYS_STATUS
            # ATTITUDE
            print "SDK MESSAGE", data
            self.time = time.clock()

    def process_from_host(self, msg):
        if self.verbose:
            print_msg("From Ground(%d):" % self.manual, msg)
        if self.manual == -1:
            print "Have not connected to drone yet"
            return
        elif msg.get_type() == "SET_MODE":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                self.manual = True
                print "MANUAL MODE ON"
            else:
                self.manual = False
                print "MANUAL MODE OFF"
            self.connection.port.sendto(msg._msgbuf, self.drone)
        elif self.manual:
            self.send_manual_command(msg)
        else:
            self.connection.port.sendto(msg._msgbuf, self.drone)

    def send_manual_command(self, msg):
        if msg.get_type() == "COMMAND_LONG":
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                send = (("AT*REF={},%d\r" % COMMAND_TAKEOFF) * self.repeat).format(
                    *range(self.cmd_seq, self.cmd_seq + self.repeat + 1))
            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                send = (("AT*REF={},%d\r" % COMMAND_LAND) * self.repeat).format(
                    *range(self.cmd_seq, self.cmd_seq + self.repeat + 1))
            else:
                print "Unsupported command in Manual Mode: %d" % msg.command
                return
        elif msg.get_type() == "RC_CHANNELS_OVERRIDE":
            send = (self.rc_channels_encode(msg) * self.repeat).format(
                *range(self.cmd_seq, self.cmd_seq + self.repeat + 1))
        else:
            return
        self.sdk.sendto("\x01\x00\x00\x00", (self.drone[0], PORTS["NAVDATA"]))
        self.sock.sendto(send, (self.drone[0], PORTS["AT"]))
        self.cmd_seq += self.repeat
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
    print time.clock()
    skip = ["HEARTBEAT",
            "MISSION_CURRENT", "SYS_STATUS", "ATTITUDE"
            "GPS_RAW_INT", "GLOBAL_POSITION_INT",
            "LOCAL_POSITION_NED", "RAW_IMU", "NAV_CONTROLLER_OUTPUT","VFR_HUD"]
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
    sdk_connection.bind((host, PORTS["NAVDATA"]))
    sdk_connection.setblocking(0)
    # Construct maps between IP Addresses, ports and ARProxyConnection
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
                print("Unregistered AUV with IP(MAV): " + mavlink_connection.last_address[0])
            elif mavlink_connection.last_address[0] != host:
                ip_map[mavlink_connection.last_address[0]].process_from_drone(msg)
            else:
                port_map[mavlink_connection.last_address[1]].process_from_host(msg)
        # Receive SDK messages
        try:
            packet, address = sdk_connection.recvfrom(65535)
            if address[0] not in ip_map.keys():
                print "Unregistered AUV with IP(SDK): ", address[0]
            else:
                ip_map[address[0]].process_from_sdk(decode_navdata(packet))
        except socket.error as e:
            if e.errno not in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                raise


def establish_navdata():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("192.168.1.3", 5554))
    sock.setblocking(0)
    cmd = "AT*CONFIG=%d,\"general:navdata_demo\",\"%s\"\r"
    cmd1 = "AT*CONFIG=%d,\"general:navdata_options\",\"%d\"\r"
    cmd2 = "AT*CTRL=%d,0\r"
    stream = False
    nav_data = False
    t = time.clock();
    seq = 1
    while True:
        if time.clock() - t > 2:
            print "SSR", NAVDATA_OPTIONS
            t = time.clock()
        if not stream:
            print "Init stream"
            sock.sendto("\x01\x00\x00\x00", ("192.168.1.1", 5554))
            sock.sendto(cmd1 % (seq, NAVDATA_OPTIONS), ("192.168.1.1", 5556))
            seq += 1
            time.sleep(0.2)
        try:
            packet, address = sock.recvfrom(65535)
        except socket.error:
            continue
        if not stream:
            print "Stream on"
        stream = True
        data = decode_navdata(packet)
        if not data["ARDRONE_STATE"]["COMMAND_MASK"]:
            print "Send general:navdata_demo"
            sock.sendto(cmd % (seq, "TRUE"), ("192.168.1.1", 5556))
            seq += 1
            continue
        elif not nav_data:
            print "Command mask on"
            sock.sendto(cmd2 % seq, ("192.168.1.1", 5556))
            seq += 1
            nav_data = data["ARDRONE_STATE"]["NAVDATA_DEMO_MASK"]
            if nav_data:
                print "Nav data on"
            else:
                sock.sendto(cmd % (seq, "TRUE"), ("192.168.1.1", 5556))
                seq += 1
        if nav_data:
            if "GPS" not in data.keys():
                sock.sendto(cmd1 % (seq, NAVDATA_OPTIONS), ("192.168.1.1", 5556))
                seq += 1
            else:
                print data
                seq += 1
        if seq > 100:
            print "STOP"
            sock.sendto(cmd % (seq, "FALSE"), ("192.168.1.1", 5556))


if __name__ == "__main__":
    csv_map = load_map(options.file)
    #run_proxy(options.port, csv_map, options.local, options.verbose)
    establish_navdata()
