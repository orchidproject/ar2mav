#!/usr/bin/python
import sys
import os
import socket
import errno
import time
from math import pi
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil
from tools import *

# Constants
REQUIRED_NAVDATA = ("DEMO", "GPS", "TIME", "GYROS_OFFSETS")
NAVDATA_OPTIONS = 0
for data in REQUIRED_NAVDATA:
    NAVDATA_OPTIONS |= 1 << NAVDATA_OPTIONS_STR[data]

SDK_COMMAND = 0
SDK_ACK = 1
SDK_RC = 2
SDK_CAMERA = 3
SDK_EMERGENCY = 4
SDK_NAVDATA_REQUEST = 10
SDK_NAVDATA_COMMAND = 11
SDK_NAVDATA_OPTIONS = 12
SDK_NAVDATA_CORRECTION = 13
PING_TIMES = 5
PING_TIMEOUT = 1

SKIP_TYPES = ["SYS_STATUS", "ATTITUDE", "GPS_RAW_INT", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "RAW_IMU",
              "NAV_CONTROLLER_OUTPUT", "VFR_HUD"]

QGC_PORT = 14555
EMERGENCY_CODE = 100
KILL_CODE = 255
ADHOC_MANUAL = 99
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
    def __init__(self, name, ip, in_port, destination, nav_data_port, timeout=10, verbose=0, repeat=1, qgc=False):
        # General input variables required
        self.drone = (ip, PORTS["MAVLINK"])  # IP address and port for communication with the drone
        self.name = name  # Name of the drone, mainly for logging purposes
        self.ip = ip  # Ip of the drone, used to change the system ID to the last digits of the ip
        self.port = in_port  # Host port to use to request the MAVLink packets to be received on
        self.host = destination  # Host (ip, port) to which to reroute the MAVLink packages.
        self.nav_data_port = nav_data_port  # The NAVDATA port on the drone. Required for communication
        self.connection = None  # This is the MAVLink connection from mavutil.py
        self.target_system = None   # System ID of the drone. Required to be able to send commands to it.
        self.target_component = None    # Component ID of the drone. Required to be able to send commands to it.
        self.sdk = None     # Represents the socket connection for sending packets trough the SDK
        self.timeout = timeout  #
        self.alive = True

        self.manual = -1    # Indicator if Manual Mode is on
        self.verbose = verbose  # Verbosity level
        self.qgc = qgc  # Indicator if packets should be re routed to QGC

        # Manual Control variables
        self.cmd_seq = 1    # Counter for the SDK commands send
        self.repeat = repeat    # Variable for how many time single command should be repeated
        # SDK variables
        self.request_navdata_time = 0   # Last time of request for NAVDATA
        self.mav_last = 0   # Last time a MAVLink message was sent to host
        self.mav_interval = 0.25    # Interval on which to send MAVLink messages when using the SDK
        self.sdk_call = 0   # The first recent time when we invoked the SDK
        self.change_mode = 0    # Counter for how many times we changed the MODE, used for MANUAL/AUTO indication

        # MAVLink meta data variables used for creating or alternating MAVLink messages
        self.base_mode = None
        self.custom_mode = None
        self.status = None
        self.mission_seq = 1
        self.camera_value = 1.0
        self.relative_alt = 0.24

    def start(self):
        print("[AR2MAV]%s: ADDRESS: %s" % (self.name, str(self.drone)))
        self.connection = mavutil.mavlink_connection(self.host[0] + ":" + str(self.port))
        self.sdk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sdk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sdk.bind((self.host[0], self.nav_data_port))
        self.sdk.setblocking(0)
        while True:
            # Ping the ArdDrone until we receive a heartbeat
            while not self.target_system:
                print("[AR2MAV]%s: Waiting Heartbeat" % self.name)
                for i in range(PING_TIMES):
                    self.connection.port.sendto(
                        mavutil.mavlink.MAVLink_ping_message(time.time() * 1E6, 1, 0, 0).pack(self.connection.mav),
                        self.drone)
                    time.sleep(PING_TIMEOUT)
                msg = self.connection.recv_match(type="HEARTBEAT", blocking=False, timeout=self.timeout)
                if msg:
                    self.process_from_drone(msg)
            # Receive MAVLink messages
            msg = self.connection.recv_match(blocking=False)
            if msg:
                if msg.get_type() == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        sys.stdout.write(msg.data)
                        sys.stdout.flush()
                # Received a message from other address than our drone, or from the host
                elif self.connection.last_address != self.drone and self.connection.last_address != self.host and \
                        ((not self.qgc) or self.connection.last_address != (self.host[0], QGC_PORT)):

                    if self.verbose > 0:
                        print(
                            "[AR2MAV]%s: Unregistered AUV with IP(MAV): %s" % (
                                self.name, self.connection.last_address[0]))
                elif self.connection.last_address != self.drone:
                    self.process_from_host(msg)
                else:
                    self.process_from_drone(msg)
            # If we have not received any MAVLink messages recently we have to ping the drone again
            if time.clock() - self.mav_last > self.timeout:
                self.target_system = None
            # Receive SDK messages
            try:
                packet, address = self.sdk.recvfrom(65535)
                if address[0] != self.drone[0]:
                    self.sdk.sento(msg._msgbuf, (self.host[0], self.nav_data_port + 1))
                else:
                    self.process_from_sdk(decode_navdata(packet))
            except socket.error as e:
                if e.errno not in [errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED]:
                    raise

    def process_from_drone(self, msg):
        if self.verbose > 2:
            print_msg("From %s:" % self.name, msg)
        if time.clock() - self.request_navdata_time > 1:
            self.manual = 0
        self.mav_last = time.clock()
        if msg.get_type() == "HEARTBEAT":
            if self.verbose > 0:
                print str(self.name) + ": Heartbeat (" + str(msg.base_mode) + "," + str(msg.custom_mode) + ")"
            # Set the MAV target and component, and retrieve the modes
            self.target_system = msg.get_srcSystem()
            self.target_component = msg.get_srcComponent()
            self.base_mode = msg.base_mode
            # Modify modes landed
            if self.relative_alt < 0.25:
                self.custom_mode = 9
                msg.custom_mode = 9
            else:
                self.custom_mode = msg.custom_mode
            self.status = msg.system_status
        if msg.get_type() == "MISSION_CURRENT":
            self.mission_seq = msg.seq
        if msg.get_type() == "GLOBAL_POSITION_INT":
            self.relative_alt = msg.relative_alt / 1E3
        # Modify the mav source system to the last digits of the IP address
        self.connection.mav.srcSystem = int(self.ip.split(".")[3])
        self.connection.port.sendto(msg.pack(self.connection.mav), self.host)
        # Also reroute the message to QGC
        if self.qgc:
            self.connection.port.sendto(msg._msgbuf, (self.host[0], QGC_PORT))

    def process_from_sdk(self, data):
        if time.clock() - self.request_navdata_time < 0.2:
            return
        # If NAVDATA is on
        if data["ARDRONE_STATE"]["NAVDATA_DEMO_MASK"]:
            self.sdk_call = 0
            # If not all required NAVDATA is on request again
            if not all(flag in data.keys() for flag in REQUIRED_NAVDATA):
                if self.verbose > 0:
                    print("[AR2MAV]%s: No NAVDATA" % self.name)
                if self.verbose > 2:
                    print self.name, data.keys()
                self.invoke_sdk(SDK_NAVDATA_COMMAND)
                self.invoke_sdk(SDK_NAVDATA_OPTIONS)
                self.invoke_sdk(SDK_ACK)
            # Else we create and send MAVLink messages
            elif time.clock() - self.mav_last > self.mav_interval:
                if self.verbose > 0:
                    print("[AR2MAV]%s: Make MAVLink" % self.name)
                # If we are landed and we don't need to acknowledge change to MANUAL mode
                if data["DEMO"]["CONTROL_STATE"] < 3 < self.change_mode:
                    self.custom_mode = 9
                # If we need to acknowledge change to MANUAL mode
                elif self.manual:
                    self.custom_mode = 99
                    self.change_mode += 1
                # Otherwise we are in AUTO mode
                else:
                    self.custom_mode = 3
                    self.change_mode += 1
                # Construct all MAVLink messages and send them
                msgs = self.construct_mavlink_messages(data)
                for key in msgs.keys():
                    self.connection.mav.srcSystem = int(self.ip.split(".")[3])
                    self.connection.port.sendto(msgs[key].pack(self.connection.mav), self.host)
                self.mav_last = time.clock()
        else:
            # Request NAVDATA
            if self.sdk_call == 0:
                self.sdk_call = time.clock()
            if time.clock() - self.sdk_call > 5:
                print("[AR2MAV]%s: NAVDATA DEMO GONE WRONG for more than 5 seconds" % self.name)
                print("[AR2MAV]%s: Switching back to MANUAL" % self.name)
                self.manual = False
            else:
                if self.verbose > 0:
                    print("[AR2MAV]%s: NAVDATA DEMO GONE WRONG" % self.name)
                self.invoke_sdk(SDK_NAVDATA_COMMAND)
                self.invoke_sdk(SDK_NAVDATA_OPTIONS)
                self.invoke_sdk(SDK_NAVDATA_CORRECTION)
                self.invoke_sdk(SDK_ACK)

    def process_from_host(self, msg):
        if self.verbose > 2:
            print_msg("From Ground(%s[%d]):" % (self.name, self.manual), msg)
        if self.manual == -1:
            if self.verbose > 0:
                print("[AR2MAV]%s: No drone" % self.name)
            return
        elif msg.get_type() == "SET_MODE":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED > 0:
                # Switch to MANUAL mode
                if msg.custom_mode == ADHOC_MANUAL:
                    self.manual = 1
                    self.custom_mode = ADHOC_MANUAL
                    self.change_mode = 0
                    self.invoke_sdk(SDK_NAVDATA_REQUEST)
                    self.invoke_sdk(SDK_NAVDATA_OPTIONS)
                    if self.verbose > 0:
                        print("[AR2MAV]%s: MANUAL MODE ON" % self.name)
                            # Switch to AUTO mode
                elif msg.custom_mode == 3:
                    self.manual = 0
                    self.custom_mode = 3
                    self.change_mode = 0
                    self.connection.port.sendto(msg._msgbuf, self.drone)
                    if self.verbose > 0:
                        print("[AR2MAV]%s: MANUAL MODE OFF" % self.name)
        # Turn on/off emergency mode
        elif msg.get_type() == "COMMAND_LONG" and msg.command == EMERGENCY_CODE:
            self.invoke_sdk(SDK_EMERGENCY)
        # If we are in manual
        elif self.manual:
            self.send_manual_command(msg)
        # Otherwise reroute the message to the drone
        else:
            if hasattr(msg, 'target_system'):
                msg.target_system = self.target_system
                msg.target_component = self.target_component
            self.connection.port.sendto(msg.pack(self.connection.mav), self.drone)

    def send_manual_command(self, msg):
        # Accepted manual commands are only TAKEOFF and LAND
        if msg.get_type() == "COMMAND_LONG":
            if msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.invoke_sdk(SDK_COMMAND, COMMAND_TAKEOFF)
            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                self.invoke_sdk(SDK_COMMAND, COMMAND_LAND)
            elif self.verbose > 0:
                print("[AR2MAV]%s Unsupported manual command: %d" % (self.name, msg.command))
        # For toggling the cameras
        elif msg.get_type() == "PARAM_SET" and "CAM-RECORD_HORI" in msg.param_id:
            self.invoke_sdk(SDK_CAMERA, extra=msg.param_value)
            self.camera_value = msg.param_value
        # If we are in MANUAL and the host have requested a parameter. Sends back ONLY CAMERA.
        elif msg.get_type() == "PARAM_REQUEST_READ":
            message = mavutil.mavlink.MAVLink_param_value_message(msg.param_id, self.camera_value,
                                                                  mavutil.mavlink.MAVLINK_TYPE_FLOAT,
                                                                  14, 12)
            self.connection.port.sendto(message.pack(self.connection.mav), self.host)
        # For manual control
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
        elif command == SDK_NAVDATA_CORRECTION:
            msg = "AT*CONFIG={},\"general:video_enable\",\"TRUE\"\r"
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
        elif command == SDK_CAMERA:
            msg = "AT*CONFIG={},\"video:video_channel\",\"%d\"\r" % (extra > 0)
        elif command == SDK_EMERGENCY:
            msg = "AT*REF={},%d\r" % COMMAND_EMERGENCY
            for i in range(self.repeat):
                self.sdk.sendto(msg.format(self.cmd_seq + i), (self.drone[0], PORTS["AT"]))
            self.cmd_seq += self.repeat
            time.sleep(self.mav_interval)
            msg = "AT*REF={},%d\r" % COMMAND_LAND
        for i in range(self.repeat):
            self.sdk.sendto(msg.format(self.cmd_seq + i), (self.drone[0], PORTS["AT"]))
            if self.verbose > 2:
                print("[AR2MAV]" + str(self.name) + str(msg.format(self.cmd_seq + i)))
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
                                                                        data["DEMO"]["PHI"] * pi / 180000,
                                                                        data["DEMO"]["THETA"] * pi / 180000,
                                                                        data["DEMO"]["PSI"] * pi / 180000,
                                                                        # TODO No Idea which is ROLL, PITCH and YAW angular speed ?
                                                                        0, 0, 0)
        messages["SYS_STATUS"] = mavutil.mavlink.MAVLink_sys_status_message(
            # TODO How to get the voltage and the current battery in milliamperes ?
            (1 << 17) - 1, (1 << 17) - 1, (1 << 17) - 1, 0, 0, -1,
            struct.unpack('h', struct.pack('h', data["DEMO"]["BATTERY"]))[0], 0, 0, 0, 0, 0, 0)
        messages["GLOBAL_POSITION_INT"] = mavutil.mavlink.MAVLink_global_position_int_message(
            data["TIME"],
            struct.unpack("i", struct.pack("i", round(data["GPS"]["LATITUDE"] * 1E7)))[0],
            struct.unpack("i", struct.pack("i", round(data["GPS"]["LONGITUDE"] * 1E7)))[0],
            struct.unpack("i", struct.pack("i", round(data["GPS"]["ELEVATION"] * 1E3)))[0],
            struct.unpack("i", struct.pack("i", round(data["DEMO"]["ALTITUDE"])))[0],
            # TODO Not sure Vx, vY amd Vz are in GPS frame and also which is the heading ?
            struct.unpack("h", struct.pack("h", round(data["DEMO"]["VX"] / 10)))[0],
            struct.unpack("h", struct.pack("h", round(data["DEMO"]["VY"] / 10)))[0],
            struct.unpack("h", struct.pack("h", round(data["DEMO"]["VZ"] / 10)))[0],
            0)
        self.relative_alt = round(data["DEMO"]["ALTITUDE"]) / 1E3
        messages["GPS_RAW_INT"] = mavutil.mavlink.MAVLink_gps_raw_int_message(
            struct.unpack("Q", struct.pack("Q", round(data["GPS"]["LAST_FRAME_TIME"] * 1E3)))[0], 0,
            struct.unpack("i", struct.pack("i", round(data["GPS"]["LATITUDE"] * 1E7)))[0],
            struct.unpack("i", struct.pack("i", round(data["GPS"]["LONGITUDE"] * 1E7)))[0],
            struct.unpack("i", struct.pack("i", round(data["GPS"]["ELEVATION"] * 1E3)))[0],
            struct.unpack("H", struct.pack("H", round(data["GPS"]["HDOP"] * 100)))[0],
            struct.unpack("H", struct.pack("H", round(data["GPS"]["VDOP"] * 100)))[0],
            struct.unpack("H", struct.pack("H", round(data["GPS"]["SPEED"] * 100)))[0],
            # TODO Not sure if this is Course Over Ground ?
            struct.unpack("H", struct.pack("H", round(data["GPS"]["DEGREE"] * 100)))[0],
            # TODO data["GPS"]["NB_SATELLITES"] is out of the 255 range ?
            struct.unpack("B", struct.pack("B", 0))[0])
        return messages

def print_msg(prefix, msg):
    if msg.get_type() not in SKIP_TYPES:
        print("[AR2MAV]%s %s[%s]" % (
            prefix, msg.get_type(), ", ".join("%s:%s" % (i, str(msg.__dict__[i])) for i in msg._fieldnames)))


def load_file(path):
    mapping = dict()
    if path.endswith(".yaml"):
        import yaml

        stream = open(path, "r")
        content = yaml.load(stream)
        stream.close()
        for name in content["drones_active"]:
            mapping[name] = content["drones"][name]
    elif path.endswith(".csv"):
        import csv

        stream = open(path, "r")
        content = csv.reader(stream)
        i = 1
        for row in content:
            if len(row) == 4:
                mapping[row[0]] = dict()
                mapping[row[0]]["ip"] = row[1]
                mapping[row[0]]["in_port"] = int(row[2])
                mapping[row[0]]["out_port"] = int(row[3])
            else:
                print("[AR2MAV]Skipping row %i, not of length 4" % i)
            i += 1
        stream.close()
    else:
        print("[AR2MAV]File not yaml or csv, using default settings")
        mapping["Parrot"] = dict()
        mapping["Parrot"]["ip"] = "192.168.1.1"
        mapping["Parrot"]["in_port"] = 57001
        mapping["Parrot"]["out_port"] = 58001
    return mapping


# #******************************************************************************
# Parse any arguments that follow the node command
# *******************************************************************************
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-f", "--file", dest="file", help="File with mapping of drone names,ips,ports", metavar="FILE",
                  default="../drones.yaml")
parser.add_option("-p", "--port", dest="port", help="Incoming port for ARDrones", metavar="PORT", default="14550")
parser.add_option("-l", "--local", dest="local", help="Local Host Address", metavar="HOST", default="127.0.0.1")
parser.add_option("-v", "--verbose", dest="verbose", type="int", help="Verbose Level", metavar="VERBOSE", default=0)
parser.add_option("-q", "--qgc", action="store_true", dest="qgc", help="Route to QGC?", metavar="QGC", default=False)
(options, args) = parser.parse_args()

if __name__ == "__main__":
    drones = load_file(options.file)
    proxies = list()
    for key, data in drones.iteritems():
        if options.verbose > 0:
            print("[AR2MAV]%s with IP:%s mapped from port %d to port %d" % (
                key, data["ip"], data["in_port"], data["out_port"]))
        proxies.append(ARProxyConnection(name=key, ip=data["ip"], in_port=data["in_port"],
                                         destination=(options.local, data["out_port"]),
                                         nav_data_port=PORTS["NAVDATA"] + len(proxies),
                                         verbose=options.verbose, qgc=options.qgc))
        t = threading.Thread(name="Thread-" + key, target=proxies[-1].start)
        t.setDaemon(True)
        t.start()
    time.sleep(30)
    while proxies[0].alive:
        time.sleep(30)
