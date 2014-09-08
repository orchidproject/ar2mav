#!/usr/bin/python
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo

class CalibrateHelper:
    def __init__(self, width=640, height=360):
        self.info = CameraInfo()
        self.info.width = width
        self.info.height = height
        self.info_pub = None

    def run(self):
        rospy.init_node("calibrate_helper")
        self.info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=100)
        rospy.Service("camera/set_camera_info", SetCameraInfo, self.set_info_cb)
        while not rospy.is_shutdown():
            self.info.header.stamp = rospy.Time.now()
            self.info_pub.publish(self.info)
            rospy.sleep(2)

    def set_info_cb(self, data):
        print data.camera_info
        self.info = data.camera_info
        return True, "Hi"

if __name__ == "__main__":
    try:
        cal = CalibrateHelper()
        cal.run()
    except KeyboardInterrupt:
        print cal.info
        pass