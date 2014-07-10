#!/usr/bin/python
import csv
import sys
import os
import socket
import errno
from optparse import OptionParser
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil
from tools import *

parser = OptionParser()
parser.add_option("-f", "--file", dest="file", help="Csv file with mapping", metavar="FILE", default="map.csv")
parser.add_option("-p", "--port", dest="port", help="Incoming port for ARDrones", metavar="PORT", default="14550")
parser.add_option("-l", "--local", dest="local", help="Local Host Address", metavar="HOST", default="127.0.0.1")
parser.add_option("-v", "--verbose", dest="verbose", type="int", help="Verbose Level", metavar="VERBOSE", default=0)
parser.add_option("-t", "--test", action="store_true", dest="test", help="Test SDK", metavar="TEST", default=False)
(options, args) = parser.parse_args()

# Constants
NAVDATA_OPTIONS = 1 << NAVDATA_OPTIONS_STR["DEMO"] | 1 << NAVDATA_OPTIONS_STR["GPS"] | \
                  1 << NAVDATA_OPTIONS_STR["REFERENCES"] | 1 << NAVDATA_OPTIONS_STR["TIME"]
SDK_COMMAND = 0
SDK_ACK = 1
SDK_RC = 2
SDK_NAVDATA_REQUEST = 10
SDK_NAVDATA_COMMAND = 11
SDK_NAVDATA_OPTIONS = 12

SKIP_TYPES = ["SYS_STATUS", "ATTITUDE", "GPS_RAW_INT", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "RAW_IMU",
              "NAV_CONTROLLER_OUTPUT", "VFR_HUD"]
# Messages
# HEARTBEAT - sanitised X
# ATTITUDE - sanitised X
# CONTROLLER_OUTPUT - not use by AR Drone 2.0
# CURRENT_MISSION -  sanitised X
# FILTERED_POSITION = GLOBAL_POSITION_INT - sanitised X
# GPS = GPS_RAW_INT - sanitised X
# MISSION_ITEM - not used by AR Drone 2.0 during manual
# RAW_IMU - not used by AR Drone 2.0
# RC = RC_CHANNELS_RAW - not used by AR Drone 2.0
# SYS_STATUS - sanitised X
# VFR_HUD - not used by AR Drone 2.0


class ARProxyConnection:
    # Verbose levels are 4:
    # 0 - Nothing printed
    # 1 - Only information about manual control is printed
    # 2 - Manual Control Information + some extra
    # 3 - All incoming data is printed - detailed messages
    def __init__(self, connection, sdk, host, verbose=0, repeat=1):
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
        self.repeat = repeat
        # SDK variables
        self.request_navdata_time = 0
        self.mav_last = 0
        self.mav_interval = 0.25
        self.sdk_call = 0
        # MAVLink meta data variables
        self.base_mode = None
        self.custom_mode = None
        self.status = None
        self.mission_seq = 1

    def process_from_drone(self, msg):
        if self.verbose > 2:
            print_msg("From UAV:", msg)
        if time.clock() - self.request_navdata_time > 1:
            self.manual = 0
        self.mav_last = time.clock()
        if msg.get_type() == "HEARTBEAT":
            if self.verbose == 1 or self.verbose == 2:
                print "UAV HB"
            self.base_mode = msg.base_mode
            self.custom_mode = msg.custom_mode
            self.status = msg.system_status
        if msg.get_type() == "MISSION_CURRENT":
            self.mission_seq = msg.seq
        self.connection.port.sendto(msg._msgbuf, self.host)
        self.drone = self.connection.last_address

    def process_from_sdk(self, data):
        if time.clock() - self.request_navdata_time < 0.2:
            return
        elif data["ARDRONE_STATE"]["NAVDATA_DEMO_MASK"]:
            if not all(flag in data.keys() for flag in ("TIME", "REFERENCES", "DEMO", "GPS")):
                if self.verbose > 0:
                    print "No NAVDATA"
                if self.verbose > 2:
                    print data.keys()
                self.invoke_sdk(SDK_NAVDATA_COMMAND)
                self.invoke_sdk(SDK_NAVDATA_OPTIONS)
                self.invoke_sdk(SDK_ACK)
            elif time.clock() - self.mav_last > self.mav_interval:
                if self.verbose > 0:
                    print "Make MAVLink"
                msgs = self.construct_mavlink_messages(data)
                for key in msgs.keys():
                    self.connection.port.sendto(msgs[key].pack(self.connection.mav), self.host)
                self.mav_last = time.clock()
        else:
            if self.verbose > 0:
                print "NAVDATA DEMO"
            self.invoke_sdk(SDK_NAVDATA_COMMAND)
            self.invoke_sdk(SDK_ACK)

    def process_from_host(self, msg):
        if self.verbose > 2:
            print_msg("From Ground(%d):" % self.manual, msg)
        if self.manual == -1:
            if self.verbose > 0:
                print "No drone"
            return
        elif msg.get_type() == "SET_MODE":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                self.manual = True
                self.invoke_sdk(SDK_NAVDATA_REQUEST)
                self.invoke_sdk(SDK_NAVDATA_OPTIONS)
                if self.verbose > 0:
                    print "MANUAL MODE ON"
            else:
                self.manual = False
                self.connection.port.sendto(msg._msgbuf, self.drone)
                if self.verbose > 0:
                    print "MANUAL MODE OFF"
        elif self.manual:
            self.send_manual_command(msg)
        else:
            self.connection.port.sendto(msg._msgbuf, self.drone)

    def send_manual_command(self, msg):
        if msg.get_type() == "COMMAND_LONG":
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.invoke_sdk(SDK_COMMAND, COMMAND_TAKEOFF)
            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                self.invoke_sdk(SDK_COMMAND, COMMAND_LAND)
            else:
                if self.verbose > 0:
                    print "Unsupported manual command: %d" % msg.command
        elif msg.get_type() == "RC_CHANNELS_OVERRIDE":
            self.invoke_sdk(SDK_RC,
                            (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw))

    def invoke_sdk(self, command, extra=0):
        msg = None
        if command == SDK_NAVDATA_REQUEST:
            self.sdk.sendto("\x01\x00\x00\x00", (self.drone[0], PORTS["NAVDATA"]))
            self.request_navdata_time = time.clock()
            return
        elif command == SDK_NAVDATA_COMMAND:
            msg = "AT*CONFIG={},\"general:navdata_demo\",\"TRUE\"\r"
        elif command == SDK_NAVDATA_OPTIONS:
            msg = "AT*CONFIG={},\"general:navdata_options\",\"%d\"\r" % NAVDATA_OPTIONS
        elif command == SDK_COMMAND:
            msg = "AT*REF={},%d\r" % extra
        elif command == SDK_RC:
            if len(extra) == 6:
                rc = struct.unpack('iiiiii', struct.pack('ffffff',
                                                         (extra[0] - 1500) / 500,
                                                         (extra[1] - 1500) / 500,
                                                         (extra[2] - 1500) / 500,
                                                         (extra[3] - 1500) / 500,
                                                         (extra[4] - 1500) / 500,
                                                         (extra[5] - 1500) / 500))
                msg = "AT*PCMD_MAG={},1," + ",".join([str(i) for i in rc]) + "\r"
            elif len(extra) == 4:
                rc = struct.unpack('iiii', struct.pack('ffff',
                                                       (extra[0] - 1500) / 500,
                                                       (extra[1] - 1500) / 500,
                                                       (extra[2] - 1500) / 500,
                                                       (extra[3] - 1500) / 500))
                msg = "AT*PCMD={},1," + ",".join([str(i) for i in rc]) + "\r"
        elif command == SDK_ACK:
            msg = "AT*CTRL={},0,0\r"
        for i in range(self.repeat):
            self.sdk.sendto(msg.format(self.cmd_seq + i), (self.drone[0], PORTS["AT"]))
            if self.verbose > 2:
                print msg.format(self.cmd_seq + i)
        self.cmd_seq += self.repeat

    def construct_mavlink_messages(self, data):
        messages = dict()
        messages["HEARTBEAT"] = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            self.base_mode | mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
            self.custom_mode, self.status, 3)
        messages["MISSION_CURRENT"] = mavutil.mavlink.MAVLink_mission_current_message(self.mission_seq)
        messages["ATTITUDE"] = mavutil.mavlink.MAVLink_attitude_message(data["TIME"],
                                                                        data["REFERENCES"]["ROLL"],
                                                                        data["REFERENCES"]["PITCH"],
                                                                        data["REFERENCES"]["YAW"], 0, 0, 0)
        messages["SYS_STATUS"] = mavutil.mavlink.MAVLink_sys_status_message(
            1, 0, 0, 0, 0, 0, data["DEMO"]["BATTERY"], 0, 0, 0, 0, 0, 0)
        messages["GLOBAL_POSITION_INT"] = mavutil.mavlink.MAVLink_global_position_int_message(
            data["TIME"], data["GPS"]["LATITUDE"], data["GPS"]["LONGITUDE"], data["GPS"]["ELEVATION"],
            data["DEMO"]["ALTITUDE"], data["DEMO"]["VX"], data["DEMO"]["VY"], data["DEMO"]["VZ"],
            0)
        messages["GPS_RAW_INT"] = mavutil.mavlink.MAVLink_gps_raw_int_message(
            data["TIME"], 0, data["GPS"]["LATITUDE"] * 1E7, data["GPS"]["LONGITUDE"] * 1E7,
            data["GPS"]["ELEVATION"] * 1E3, data["GPS"]["HDOP"] * 100, data["GPS"]["VDOP"] * 100,
            data["GPS"]["SPEED"] * 100, data["GPS"]["PHI_P"], data["GPS"]["NB_STABILITIES"])
        return messages


def print_msg(prefix, msg):
    if msg.get_type() not in SKIP_TYPES:
        print "%s %s[%s]" % (
            prefix, msg.get_type(), ", ".join("%s:%s" % (i, str(msg.__dict__[i])) for i in msg._fieldnames))


def load_map(path):
    f = open(path, mode='r')
    content = csv.reader(f, delimiter=',')
    ip_map = {}
    for row in content:
        ip_map[row[1]] = int(row[2])
    return ip_map


def run_proxy(port, csv_map, host="127.0.0.1", verbose=0):
    mavlink_connection = mavutil.mavlink_connection(host + ":" + port)
    sdk_connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sdk_connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sdk_connection.bind((host, PORTS["NAVDATA"]))
    sdk_connection.setblocking(0)
    # Construct maps between IP Addresses, ports and ARProxyConnection
    ip_map = {}
    port_map = {}
    for key in csv_map:
        if verbose > 1:
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
                if verbose > 0:
                    print("Unregistered AUV with IP(MAV): " + mavlink_connection.last_address[0])
            elif mavlink_connection.last_address[0] != host:
                ip_map[mavlink_connection.last_address[0]].process_from_drone(msg)
            else:
                port_map[mavlink_connection.last_address[1]].process_from_host(msg)
        # Receive SDK messages
        try:
            packet, address = sdk_connection.recvfrom(65535)
            if address[0] not in ip_map.keys():
                if verbose > 0:
                    print "Unregistered AUV with IP(SDK): ", address[0]
            else:
                ip_map[address[0]].process_from_sdk(decode_navdata(packet))
        except socket.error as e:
            if e.errno not in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                raise


def establish_navdata(local):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((local, 5554))
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
            print cmd1 % (seq, NAVDATA_OPTIONS)
            sock.sendto(cmd1 % (seq, NAVDATA_OPTIONS), ("192.168.1.1", 5556))
            seq += 1
            time.sleep(0.2)
        try:
            packet, address = sock.recvfrom(65535)
        except socket.error:
            continue
        if not stream:
            print "Stream on "
        stream = True
        data = decode_navdata(packet)
        print "BT:", data["ARDRONE_STATE"]["NAVDATA_BOOTSTRAP"]
        if not data["ARDRONE_STATE"]["COMMAND_MASK"]:
            print "Send general:navdata_demo ", data["ARDRONE_STATE"]["NAVDATA_BOOTSTRAP"]
            sock.sendto(cmd % (seq, "TRUE"), ("192.168.1.1", 5556))
            seq += 1
            continue
        elif not nav_data:
            print "Command mask on ", data["ARDRONE_STATE"]["NAVDATA_BOOTSTRAP"]
            # sock.sendto("AT*CTRL=0\r", ("192.168.1.1", 5556))
            seq += 1
            nav_data = data["ARDRONE_STATE"]["NAVDATA_DEMO_MASK"]
            if nav_data:
                print "Nav data on", data["ARDRONE_STATE"]["NAVDATA_BOOTSTRAP"]
            else:
                sock.sendto(cmd % (seq, "TRUE"), ("192.168.1.1", 5556))
                print "No nav data", data["ARDRONE_STATE"]["NAVDATA_BOOTSTRAP"]
                # time.sleep(0.1)
                seq += 1
        if nav_data:
            if "GPS" not in data.keys():
                sock.sendto(cmd1 % (seq, NAVDATA_OPTIONS), ("192.168.1.1", 5556))
                print "No GPS", data["ARDRONE_STATE"]["NAVDATA_BOOTSTRAP"]
                # time.sleep(0.1)
                seq += 1
            else:
                print "data"
                # sock.sendto(cmd % (seq, "TRUE"), ("192.168.1.1", 5556))
                sock.sendto(cmd1 % (seq, NAVDATA_OPTIONS), ("192.168.1.1", 5556))
                seq += 1
        if seq > 100:
            print "STOP"
            sock.sendto(cmd % (seq, "FALSE"), ("192.168.1.1", 5556))


if __name__ == "__main__":
    csv_map = load_map(options.file)
    if options.test:
        establish_navdata(options.local)
    else:
        run_proxy(options.port, csv_map, options.local, options.verbose)