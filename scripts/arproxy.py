#!/usr/bin/python
import csv, sys, os
from optparse import OptionParser

sys.path.insert(0, '/home/botev/auv/src/roscopter/mavlink/pymavlink')
import mavutil

parser = OptionParser()
parser.add_option("-f", "--file", dest="file", help="Csv file with mapping", metavar="FILE", default="map.csv")
parser.add_option("-p", "--port", dest="port", help="Incoming port for ARDrones", metavar="PORT", default="14550")
parser.add_option("-l", "--local", dest="local", help="Local Host Address", metavar="HOST", default="127.0.0.1")
parser.add_option("-i", "--in", dest="inc", help="Incoming Local Host Address", metavar="INC", default="127.0.0.1")
parser.add_option("-s", "--sim", dest="sim", help="Use simulator mode", metavar="SIM", default=False)
(options, args) = parser.parse_args()


# Format of csv file should be uav_id,ip_address,remapped_port
def load_map(path):
    f = open(path, mode='r')
    content = csv.reader(f, delimiter=',')
    ip_map = {}
    for row in content:
        ip_map[row[1]] = int(row[2])
    return ip_map


def run_proxy_sim(port, ip_map, host="127.0.0.1", inc="127.0.0.1"):
    master = mavutil.mavlink_connection(host + ":" + port)
    port_map = {}
    for key in ip_map:
        print(key + " mapped to " + str(ip_map[key]))
        port_map[ip_map[key]] = 0
    while True:
        # Check master connection
        msg = master.recv_match()
        if msg:
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            else:
                print(str(master.last_address) + " - " + msg.get_type())
                if master.last_address[1] not in ip_map.values():
                    master.port.sendto(msg._msgbuf, (host, int(ip_map[master.last_address[0]])))
                    port_map[ip_map[master.last_address[0]]] = master.last_address
                else:
                    if port_map[master.last_address[1]]:
                        master.port.sendto(msg._msgbuf, port_map[master.last_address[1]])


def run_proxy(port, ip_map, host="127.0.0.1", inc="127.0.0.1"):
    master = mavutil.mavlink_connection(inc + ":" + port)
    print(host + ":" + port)
    port_map = {}
    for key in ip_map:
        print(key + " mapped to " + str(ip_map[key]))
        port_map[ip_map[key]] = 0
    while True:
        # Check master connection
        msg = master.recv_match()
        if msg:
            if msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
            elif master.last_address[0] not in ip_map.keys() and master.last_address[0] != host:
                print("Unregistered AUV with IP: " + master.last_address[0])
            else:
                if master.last_address[0] != host:
                    if msg.get_type() == "HEARTBEAT":
                        print "From UAV(H) - ", msg.base_mode
                    if msg.get_type() == "MISSION_CURRENT":
                        print "From UAV(MC) - ", msg.seq
                    if msg.get_type() == "MISSION_ITEM":
                        print "From UAV(MI) - ", " ".join(msg.seq, msg.current, msg.autocontinue, msg.param1, msg.param2, msg.param3, msg.param4)
                    if msg.get_type() == "MISSION_ACK":
                        print "From UAV(ACK) - ", msg.type
                    if msg.get_type() == "COMMAND_ACK":
                        print "From UAV(CACK) - ", msg.result
                    if msg.get_type() == "MISSION_REQUEST":
                        print "From UAV(MR) - ", msg.target_system, " ", msg.target_component, " ", msg.seq
                    master.port.sendto(msg._msgbuf, (host, ip_map[master.last_address[0]]))
                    port_map[ip_map[master.last_address[0]]] = master.last_address
                else:
                    if port_map[master.last_address[1]]:
                        print "From Ground - ", msg._type
                        master.port.sendto(msg._msgbuf, port_map[master.last_address[1]])


if __name__ == "__main__":
    ip_map = load_map(options.file)
    if options.sim:
        run_proxy_sim(options.port, ip_map, options.local, options.inc)
    else:
        run_proxy(options.port, ip_map, options.local, options.inc)
