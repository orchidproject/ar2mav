#!/usr/bin/python
import csv
import sys
import os
import socket
from optparse import OptionParser
from struct import pack, unpack

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink/pymavlink'))
import mavutil
from pymavlink.dialects.v10 import ardupilotmega

parser = OptionParser()
parser.add_option("-f", "--file", dest="file", help="Csv file with mapping", metavar="FILE", default="map.csv")
parser.add_option("-p", "--port", dest="port", help="Incoming port for ARDrones", metavar="PORT", default="14550")
parser.add_option("-l", "--local", dest="local", help="Local Host Address", metavar="HOST", default="127.0.0.1")
parser.add_option("-v", "--verbose", dest="verbose", help="Verbose", metavar="VERBOSE", default=False)
(options, args) = parser.parse_args()


class ARProxyConnection:


    def __init__(self, connection, host, verbose=False, control=1.0, repeat=3):
        self.connection = connection
        self.host = host
        self.drone = 0
        self.manual = -1
        self.verbose = verbose
        # Manual Control variables
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.seq = 1
        self.control = control
        self.repeat = repeat


    def process_from_drone(self, msg):
        if self.verbose:
            print_msg("From UAV:", msg)
        # Since AUTO_MODE does not get set on the Ar Drone 2.0 this is not feasible
        # a.k.a you can not switch the base_mode MAV_MODE_FLAG_MANUAL_INPUT_ENABLED off
        # if self.manual == -1 and msg.get_type() == "HEARTBEAT":
        #     self.manual = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED > 0
        #     if self.manual:
        #         print "UAV in MANUAL"
        #     else:
        #         print "UAV not in MANUAL"
        if self.manual == -1:
            self.manual = 0
        self.connection.port.sendto(msg._msgbuf, self.host)
        self.drone = self.connection.last_address


    def process_from_host(self, msg):
        if self.verbose:
            print_msg("From Ground(%d):" % self.manual, msg)
        if self.manual == -1:
            print "Have not connected to drone yet"
            return
        elif msg.get_type() == "SET_MODE":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED > 0:
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
                send = ("AT*REF={},290718208\r" * self.repeat).format(
                    *range(self.seq, self.seq+self.repeat+1))
                self.sock.sendto(send, (self.drone[0], 5556))
                self.seq += self.repeat
                if self.verbose:
                    print send
            elif msg.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
                send = ("AT*REF={},290717696\r" * self.repeat).format(
                    *range(self.seq, self.seq+self.repeat+1))
                self.sock.sendto(send, (self.drone[0], 5556))
                self.seq += self.repeat
                if self.verbose:
                    print send
            else:
                print "Unsupported command in Manual Mode: %d" % msg.command
        elif msg.get_type() == "RC_CHANNELS_OVERRIDE":
            send = (self.rc_channels_encode(msg) * self.repeat).format(
                *range(self.seq, self.seq+self.repeat+1))
            self.sock.sendto(send, (self.drone[0], 5556))
            self.seq += self.repeat
            if self.verbose:
                print send

    def rc_channels_encode(self,msg):
        transmit_values = unpack('>iiiiii', pack('>ffffff',
                                                 self.control * (msg.chan1_raw - 1500) / 500,
                                                 self.control * (msg.chan2_raw - 1500) / 500,
                                                 self.control * (msg.chan3_raw - 1500) / 500,
                                                 self.control * (msg.chan4_raw - 1500) / 500,
                                                 self.control * (msg.chan5_raw - 1500) / 500,
                                                 self.control * (msg.chan6_raw - 1500) / 500,))
        return "AT*PCMD_MAG={},1," + ",".join([str(i) for i in transmit_values]) + "\r"


def print_msg(prefix, msg):
    skip = ["VFR_HUD", "GPS_RAW_INT", "ATTITUDE", "LOCAL_POSITION_NED","RAW_IMU",
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


def receive_mavlink(master):
    msg = master.recv_match()
    if not msg:
        return
    if msg.get_type() == "BAD_DATA":
        if mavutil.all_printable(msg.data):
            sys.stdout.write(msg.data)
            sys.stdout.flush()
    elif master.last_address[0] not in ip_map.keys() and master.last_address[0] != host:
        print("Unregistered AUV with IP: " + master.last_address[0])
    elif master.last_address[0] != host:
        ip_map[master.last_address[0]].process_from_drone(msg)
    else:
        port_map[master.last_address[1]].process_from_host(msg)


def receive_sdk(sock):
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data, " from ", addr


def run_proxy(port, csv_map, host="127.0.0.1", verbose=False):
    master = mavutil.mavlink_connection(host + ":" + port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, 5554))
    ip_map = {}
    port_map = {}
    for key in csv_map:
        if verbose:
            print(key + " mapped to " + str(csv_map[key]))
        ip_map[key] = ARProxyConnection(master, (host, int(csv_map[key])), verbose)
        port_map[csv_map[key]] = ip_map[key]
    # Main loop
    while True:
        receive_mavlink(master)
        receive_sdk(sock)





if __name__ == "__main__":
    csv_map = load_map(options.file)
    run_proxy(options.port, csv_map, options.local, options.verbose)
