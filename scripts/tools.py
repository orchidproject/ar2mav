# Converting https://github.com/felixge/node-ar-drone/blob/master/lib/navdata/parseNavdata.js
# to Python
# Thank god someone did this before. Thanks to John Wiseman - https://github.com/wiseman

# see ARDroneLIb/Soft/Common/config.h
PORTS = {
    "FTP": 5551,
    "AUTH": 5552,
    "VIDEO_RECORDER": 5553,
    "NAVDATA": 5554,
    "VIDEO": 5555,
    "AT": 5556,
    "RAW_CAPTURE": 5557,
    "PRINTF": 5558,
    "CONTROL": 5559
}

# see ARDroneLib/Soft/Common/config.h
STATES = {
    "FLY_MASK": 1 << 0,  # FLY MASK : (0) ardrone is landed, (1) ardrone is flying
    "VIDEO_MASK": 1 << 1,  # VIDEO MASK : (0) video disable, (1) video enable
    "VISION_MASK": 1 << 2,  # VISION MASK : (0) vision disable, (1) vision enable
    "CONTROL_MASK": 1 << 3,  # CONTROL ALGO : (0) euler angles control, (1) angular speed control
    "ALTITUDE_MASK": 1 << 4,  # ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active
    "USER_FEEDBACK_START": 1 << 5,  # USER feedback : Start button state
    "COMMAND_MASK": 1 << 6,  # Control command ACK : (0) None, (1) one received
    "CAMERA_MASK": 1 << 7,  # Camera enable : (0) Camera enable, (1) camera disable
    "TRAVELLING_MASK": 1 << 8,  # Travelling enable : (0) disable, (1) enable
    "USB_MASK": 1 << 9,  # USB key : (0) usb key not ready, (1) usb key ready
    "NAVDATA_DEMO_MASK": 1 << 10,  # Navdata demo : (0) All navdata, (1) only navdata demo
    "NAVDATA_BOOTSTRAP": 1 << 11,
    # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent
    "MOTORS_MASK": 1 << 12,  # Motors status : (0) Ok, (1) Motors problem
    "COM_LOST_MASK": 1 << 13,  # Communication Lost : (1) com problem, (0) Com is ok
    "SOFTWARE_FAULT": 1 << 14,  # <DEPRECATED> Software fault detected - user should land as quick as possible (1)
    "VBAT_LOW": 1 << 15,  # VBat low : (1) too low, (0) Ok
    "USER_EL": 1 << 16,  # User Emergency Landing : (1) User EL is ON, (0) User EL is OFF
    "TIMER_ELAPSED": 1 << 17,  # Timer elapsed : (1) elapsed, (0) not elapsed
    "MAGNETO_NEEDS_CALIB": 1 << 18,
    # Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed
    "ANGLES_OUT_OF_RANGE": 1 << 19,  # Angles : (0) Ok, (1) out of range
    "WIND_MASK": 1 << 20,  # WIND MASK: (0) ok, (1) Too much wind
    "ULTRASOUND_MASK": 1 << 21,  # Ultrasonic sensor : (0) Ok, (1) deaf
    "CUTOUT_MASK": 1 << 22,  # Cutout system detection : (0) Not detected, (1) detected
    "PIC_VERSION_MASK": 1 << 23,  # PIC Version number OK : (0) a bad version number, (1) version number is OK
    "ATCODEC_THREAD_ON": 1 << 24,  # ATCodec thread ON : (0) thread OFF (1) thread ON
    "NAVDATA_THREAD_ON": 1 << 25,  # Navdata thread ON : (0) thread OFF (1) thread ON
    "VIDEO_THREAD_ON": 1 << 26,  # Video thread ON : (0) thread OFF (1) thread ON
    "ACQ_THREAD_ON": 1 << 27,  # Acquisition thread ON : (0) thread OFF (1) thread ON
    "CTRL_WATCHDOG_MASK": 1 << 28,
    # CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled
    "ADC_WATCHDOG_MASK": 1 << 29,  # ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good
    "COM_WATCHDOG_MASK": 1 << 30,  # Communication Watchdog : (1) com problem, (0) Com is ok
    "EMERGENCY_MASK": 1 << 31  # Emergency landing : (0) no emergency, (1) emergency
}

# see ARDroneLib/Soft/Common/config.h
AT_COMMAND_BITS = {
    "AG": 1 << 0,
    "AB": 1 << 1,
    "AD": 1 << 2,
    "AH": 1 << 3,
    "L1": 1 << 4,
    "R1": 1 << 5,
    "L2": 1 << 6,
    "R2": 1 << 7,
    "SELECT": 1 << 8,
    "START": 1 << 9,
    "TRIM_THETA": 1 << 18,
    "TRIM_PHI": 1 << 20,
    "TRIM_YAW": 1 << 22,
    "X": 1 << 24,
    "Y": 1 << 28,
}

COMMAND_LAND = AT_COMMAND_BITS["TRIM_THETA"] | AT_COMMAND_BITS["TRIM_PHI"] | AT_COMMAND_BITS["TRIM_YAW"] | \
               AT_COMMAND_BITS["X"] | AT_COMMAND_BITS["Y"]
COMMAND_TAKEOFF = COMMAND_LAND | AT_COMMAND_BITS["START"]
COMMAND_EMERGENCY = COMMAND_LAND | AT_COMMAND_BITS["SELECT"]

# see ARDroneLib/Soft/Common/navdata_keys.h
NAVDATA_OPTIONS_CODE = {
    0: "DEMO",
    1: "TIME",
    2: "RAW_MEASURES",
    3: "PHYS_MEASURES",
    4: "GYROS_OFFSETS",
    5: "EULER_ANGLES",
    6: "REFERENCES",
    7: "TRIMS",
    8: "RC_REFERENCES",
    9: "PWM",
    10: "ALTITUDE",
    11: "VISION_RAW",
    12: "VISION_OF",
    13: "VISION",
    14: "VISION_PERF",
    15: "TRACKERS_SEND",
    16: "VISION_DETECT",
    17: "WATCHDOG",
    18: "ADC_DATA_FRAME",
    19: "VIDEO_STREAM",
    20: "GAMES",
    21: "PRESSURE_RAW",
    22: "MAGNETO",
    23: "WIND",
    24: "KALMAN_PRESSURE",
    25: "HDVIDEO_STREAM",
    26: "WIFI",
    27: "GPS",
    65535: "CKS"
}

NAVDATA_OPTIONS_STR = {
    "DEMO": 0,
    "TIME": 1,
    "RAW_MEASURES": 2,
    "PHYS_MEASURES": 3,
    "GYROS_OFFSETS": 4,
    "EULER_ANGLES": 5,
    "REFERENCES": 6,
    "TRIMS": 7,
    "RC_REFERENCES": 8,
    "PWM": 9,
    "ALTITUDE": 10,
    "VISION_RAW": 11,
    "VISION_OF": 12,
    "VISION": 13,
    "VISION_PERF": 14,
    "TRACKERS_SEND": 15,
    "VISION_DETECT": 16,
    "WATCHDOG": 17,
    "ADC_DATA_FRAME": 18,
    "VIDEO_STREAM": 19,
    "GAMES": 20,
    "PRESSURE_RAW": 21,
    "MAGNETO": 22,
    "WIND": 23,
    "KALMAN_PRESSURE": 24,
    "HDVIDEO_STREAM": 25,
    "WIFI": 26,
    "GPS": 27,
    "CKS": 65535
}
# see ARDroneLib/Soft/Common/control_states.h
CONTROL_STATES = {
    0: "CTRL_DEFAULT",
    1: "CTRL_INIT",
    2: "CTRL_LANDED",
    3: "CTRL_FLYING",
    4: "CTRL_HOVERING",
    5: "CTRL_TEST",
    6: "CTRL_TRANS_TAKEOFF",
    7: "CTRL_TRANS_GOTOFIX",
    8: "CTRL_TRANS_LANDING",
    9: "CTRL_TRANS_LOOPING"
}

# see ARDroneLib/Soft/Common/control_states.h
FLY_STATES = {
    0: "FLYING_OK",
    1: "FLYING_LOST_ALT",
    2: "FLYING_LOST_ALT_GO_DOWN",
    3: "FLYING_ALT_OUT_ZONE",
    4: "FLYING_COMBINED_YAW",
    5: "FLYING_BRAKE",
    6: "FLYING_NO_VISION"
}

# see ARDroneLib/Soft/Common/navdata_common.h
# NAVDATA_HEADER = 0x55667788
# Actually I always receive this from the drone so changed
NAVDATA_HEADER = 1432778632
import struct


def decode_navdata(packet):
    # Decode a navdata packet
    # see ARDroneLib/Soft/Common/navdata_common.h
    offset = 0
    data = dict()
    temp = struct.unpack_from("IIII", packet, offset)
    data["HEADER"] = temp[0]
    data["SEQUENCE"] = temp[2]
    data["VISION_DEFINED"] = temp[3]
    temp_dict = dict()
    for key, value in STATES.iteritems():
        temp_dict[key] = temp[1] & STATES[key] > 0
    data["ARDRONE_STATE"] = temp_dict
    offset += struct.calcsize("IIII")
    while True:
        try:
            option_id, size = struct.unpack_from("HH", packet, offset)
            offset += struct.calcsize("HH")
        except struct.error:
            break
        if option_id not in NAVDATA_OPTIONS_CODE.keys():
            pass
        elif NAVDATA_OPTIONS_CODE[option_id] == "CKS":
            temp = struct.unpack_from("B" * (len(packet) - 8), packet)
            checksum = struct.unpack_from("I", packet, struct.calcsize("B" * (len(packet) - 8)) + 4)
            data["CHECKSUM"] = checksum[0] == sum(temp)
            break
        elif NAVDATA_OPTIONS_CODE[option_id] == "DEMO":
            temp_dict = dict()
            temp = struct.unpack_from("HHIfffifffI", packet, offset)
            temp_dict["FLY_STATE"] = temp[0]
            temp_dict["CONTROL_STATE"] = temp[1]
            temp_dict["BATTERY"] = temp[2]
            temp_dict["THETA"] = temp[3]
            temp_dict["PHI"] = temp[4]
            temp_dict["PSI"] = temp[5]
            temp_dict["ALTITUDE"] = temp[6]
            temp_dict["VX"] = temp[7]
            temp_dict["VY"] = temp[8]
            temp_dict["VZ"] = temp[9]
            temp_dict["FRAME_INDEX"] = temp[10]
            data["DEMO"] = temp_dict
            # Skip rest of packet as everything except detection_camera_type is deprecated
        elif NAVDATA_OPTIONS_CODE[option_id] == "TIME":
            temp = struct.unpack_from("I", packet, offset)
            data["TIME"] = drone_time_to_milliseconds(temp[0])
        elif NAVDATA_OPTIONS_CODE[option_id] == "RAW_MEASURES":
            temp_dict = dict()
            temp = struct.unpack_from("H" * 3 + "h" * 5 + "I" + "H" * 9 + "Iih", packet, offset)
            temp_dict["RAW_ACCS"] = [temp[0], temp[1], temp[2]]
            temp_dict["RAW_GYROS"] = [temp[3], temp[4], temp[5]]
            temp_dict["RAW_GYROS110"] = [temp[6], temp[7]]
            temp_dict["VBAT_RAW"] = temp[8]
            # No idea also
            temp_dict["US_ECHO"] = [temp[9], temp[10], temp[11], temp[12]]
            temp_dict["US_COURBE"] = [temp[13], temp[14], temp[15]]
            temp_dict["ECHO"] = [temp[16], temp[17], temp[18]]
            temp_dict["ALT_TEMP"] = temp[19]
            temp_dict["GRADIENT"] = temp[20]
            data["RAW_MEASURES"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "PHYS_MEASURES":
            temp_dict = dict()
            temp = struct.unpack_from("fH" + "f" * 6 + "I" * 3, packet, offset)
            temp_dict["ACCS_TEMP"] = temp[0]
            temp_dict["GYRO_TEMP"] = temp[1]
            temp_dict["PHYS_ACCS"] = [temp[2], temp[3], temp[4]]
            temp_dict["PHYS_GYROS"] = [temp[5], temp[6], temp[7]]
            temp_dict["ALIM3V3"] = temp[8]
            temp_dict["VREF_EPSON"] = temp[9]
            temp_dict["VREF_IDG"] = temp[10]
            data["PHYS_MEASURES"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "GYROS_OFFSETS":
            temp = struct.unpack_from("f" * 3, packet, offset)
            data["GYROS_OFFSETS"] = [temp[0], temp[1], temp[2]]
        elif NAVDATA_OPTIONS_CODE[option_id] == "EULER_ANGLES":
            temp = struct.unpack_from("f" * 2, packet, offset)
            data["EULER_ANGLES"] = [temp[0], temp[1]]
        elif NAVDATA_OPTIONS_CODE[option_id] == "REFERENCES":
            temp_dict = dict()
            temp = struct.unpack_from("i" * 8 + "f" * 6 + "I" + "f" * 5 + "i", packet, offset)
            temp_dict["THETA"] = temp[0]
            temp_dict["PHI"] = temp[1]
            temp_dict["THETA_I"] = temp[2]
            temp_dict["PHI_A"] = temp[3]
            temp_dict["PITCH"] = temp[4]
            temp_dict["ROLL"] = temp[5]
            temp_dict["YAW"] = temp[6]
            temp_dict["PSI"] = temp[7]
            temp_dict["VX"] = temp[8]
            temp_dict["VY"] = temp[9]
            temp_dict["THETA_MOD"] = temp[10]
            temp_dict["PSI_MOD"] = temp[11]
            temp_dict["K_VX"] = temp[12]
            temp_dict["K_VY"] = temp[13]
            temp_dict["K_MODE"] = temp[14]
            temp_dict["UI_TIME"] = temp[15]
            temp_dict["UI_THETA"] = temp[16]
            temp_dict["UI_PHI"] = temp[17]
            temp_dict["UI_PSI"] = temp[18]
            temp_dict["UI_PSI_ACC"] = temp[19]
            temp_dict["UI_SEQ"] = temp[20]
            data["REFERENCES"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "TRIMS":
            temp = struct.unpack_from("f" * 3, packet, offset)
            data["TRIMS"] = [temp[0], temp[1], temp[2]]
        elif NAVDATA_OPTIONS_CODE[option_id] == "RC_REFERENCES":
            temp_dict = dict()
            temp = struct.unpack_from("i" * 5, packet, offset)
            temp_dict["PTICH"] = temp[0]
            temp_dict["ROLL"] = temp[1]
            temp_dict["YAW"] = temp[2]
            temp_dict["GAZ"] = temp[3]
            temp_dict["AG"] = temp[4]
            data["RC_REFERENCES"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "PWM":
            temp_dict = dict()
            # # TODO
            temp = struct.unpack_from("iiiii", packet, offset)
            data["PWM"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "ALTITUDE":
            temp_dict = dict()
            # # TODO
            temp = struct.unpack_from("iiiii", packet, offset)
            data["ALTITUDE"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "VISION_RAW":
            temp = struct.unpack_from("f" * 3, packet, offset)
            data["VISION_RAW"] = [temp[0], temp[1], temp[2]]
        elif NAVDATA_OPTIONS_CODE[option_id] == "VISION_OF":
            temp_dict = dict()
            temp = struct.unpack_from("f" * 10, packet, offset)
            temp_dict["DX"] = [temp[0], temp[1], temp[2], temp[3], temp[4]]
            temp_dict["DY"] = [temp[5], temp[6], temp[7], temp[8], temp[9]]
            data["VISION_OF"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "VISION":
            temp_dict = dict()
            # # TODO
            temp = struct.unpack_from("iiiii", packet, offset)
            data["VISION"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "VISION_PERF":
            temp_dict = dict()
            temp = struct.unpack_from("f" * 6, packet, offset)
            temp_dict["TIME_SZO"] = temp[0]
            temp_dict["TIME_CORNERS"] = temp[1]
            temp_dict["TIME_COMPUTE"] = temp[2]
            temp_dict["TIME_TRACKING"] = temp[3]
            temp_dict["TIME_TRANS"] = temp[4]
            temp_dict["TIME_UPDATE"] = temp[5]
            temp_dict["TIME_CUSTOM"] = struct.unpack_from("f" * 20, packet, offset)
            data["VISION_PERF"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "TRACKERS_SEND":
            temp_dict = dict()
            num = (size - struct.calcsize("HH")) / 12
            temp = struct.unpack_from("i" * num, packet, offset)
            temp_dict["LOCKED"] = list(temp)
            temp = struct.unpack_from("i" * 2 * num, packet, offset)
            temp_dict["POINTS"] = list(temp)
            data["TRACKERS_SEND"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "VISION_DETECT":
            temp_dict = dict()
            # # TODO
            temp = struct.unpack_from("i", packet, offset)
            data["VISION_DETECT"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "WATCHDOG":
            temp = struct.unpack_from("i", packet, offset)
            data["WATCHDOG"] = temp[0]
        elif NAVDATA_OPTIONS_CODE[option_id] == "ADC_DATA_FRAME":
            temp_dict = dict()
            temp = struct.unpack_from("I", packet, offset)
            temp_dict["VERSION"] = temp[0]
            temp = struct.unpack_from("H" * 32, packet, offset)
            temp_dict["VERSION"] = list(temp)
            data["ADC_DATA_FRAME"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "VIDEO_STREAM":
            temp_dict = dict()
            # # TODO
            temp = struct.unpack_from("i", packet, offset)
            data["VIDEO_STREAM"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "GAMES":
            temp = struct.unpack_from("II", packet, offset)
            data["GAMES"] = [temp[0], temp[1]]
        elif NAVDATA_OPTIONS_CODE[option_id] == "PRESSURE_RAW":
            temp = struct.unpack_from("ihii", packet, offset)
            data["PRESSURE_RAW"] = list(temp)
        elif NAVDATA_OPTIONS_CODE[option_id] == "MAGNETO":
            temp = struct.unpack_from("ihii", packet, offset)
            # # TODO
            data["MAGNETO"] = list(temp)
        elif NAVDATA_OPTIONS_CODE[option_id] == "WIND":
            temp = struct.unpack_from("ihii", packet, offset)
            # # TODO
            data["WIND"] = list(temp)
        elif NAVDATA_OPTIONS_CODE[option_id] == "KALMAN_PRESSURE":
            temp = struct.unpack_from("ihii", packet, offset)
            # # TODO
            data["KALMAN_PRESSURE"] = list(temp)
        elif NAVDATA_OPTIONS_CODE[option_id] == "HDVIDEO_STREAM":
            temp_dict = dict()
            temp = struct.unpack_from("I" * 7, packet, offset)
            temp_dict["STATE"] = temp[0]
            temp_dict["STORAGE_PACKETS"] = temp[1]
            temp_dict["STORAGE_SIZE"] = temp[2]
            temp_dict["USB_SIZE"] = temp[3]
            temp_dict["USB_FREE"] = temp[4]
            temp_dict["FRAME_NUMBER"] = temp[5]
            temp_dict["USB_REMAINING_TIME"] = temp[6]
            data["HDVIDEO_STREAM"] = temp_dict
        elif NAVDATA_OPTIONS_CODE[option_id] == "WIFI":
            temp = struct.unpack_from("I", packet, offset)
            data["WIFI"] = temp[0]
        elif NAVDATA_OPTIONS_CODE[option_id] == "GPS":
            # see https://github.com/felixge/node-ar-drone/issues/75
            temp_dict = dict()
            temp = struct.unpack_from(
                "d" * 4 + "iii" + "d" * 4 + "I" + "f" * 10 + "ddfI" + "f" * 5 + "I" + "B" * 24 + "iIffI", packet,
                offset)
            temp_dict["LATITUDE"] = temp[0]
            temp_dict["LONGITUDE"] = temp[1]
            temp_dict["ELEVATION"] = temp[2]
            temp_dict["HDOP"] = temp[3]
            temp_dict["DATA_AVAILABLE"] = temp[4]
            temp_dict["ZERO_VALIDATED"] = temp[5]
            temp_dict["WPT_VALIDATED"] = temp[6]
            temp_dict["LAT_0"] = temp[7]
            temp_dict["LON_0"] = temp[8]
            temp_dict["LAT_FUSE"] = temp[9]
            temp_dict["LON_FUSE"] = temp[10]
            temp_dict["STATE"] = temp[11]
            temp_dict["X_TRAJ"] = temp[12]
            temp_dict["X_REF"] = temp[13]
            temp_dict["Y_TRAJ"] = temp[14]
            temp_dict["Y_REF"] = temp[15]
            temp_dict["THETA_P"] = temp[16]
            temp_dict["PHI_P"] = temp[17]
            temp_dict["THETA_I"] = temp[18]
            temp_dict["PHI_I"] = temp[19]
            temp_dict["THETA_D"] = temp[20]
            temp_dict["PHI_D"] = temp[21]
            temp_dict["VDOP"] = temp[22]
            temp_dict["PDOP"] = temp[23]
            temp_dict["SPEED"] = temp[24]
            temp_dict["LAST_FRAME_TIME"] = drone_time_to_milliseconds(temp[25])
            temp_dict["DEGREE"] = temp[26]
            temp_dict["DEGREE_MAG"] = temp[27]
            temp_dict["EHPE"] = temp[28]
            temp_dict["EHVE"] = temp[29]
            temp_dict["C_N0"] = temp[30]
            temp_dict["NB_SATELLITES"] = temp[31]
            temp_dict["SAT_CHANNELS"] = temp[32:56]
            temp_dict["PLUGGED"] = temp[56]
            temp_dict["EPH_METER_STATUS"] = temp[57]
            temp_dict["VX_TRAJ"] = temp[58]
            temp_dict["VY_TRAJ"] = temp[59]
            temp_dict["FRIMWARE_STATUS"] = temp[60]
            data["GPS"] = temp_dict
        offset += size - struct.calcsize("HH")
    return data


def drone_time_to_milliseconds(time):
    # 32 bit int value, 11 most significant bits are seconds, rest 21 - microseconds
    return (time >> 21) * 1000 + ((time << 11) >> 11) / 1000



