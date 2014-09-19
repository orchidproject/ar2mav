#include<ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>

#ifndef AR2MAV_H
#define AR2MAV_H

namespace ar2mav{
    /**
     * @brief The ARDroneVideo class
     * Class which is designed to receive the raw ARDrone2.0 stream and extraxt the x264 frames
     */
    class ARDroneVideo{
        private:
            volatile bool active;
            std::string name;
            std::string drone_ip;
            int drone_port;
            int buffer_size;
            struct timeval timeout;
            ros::Publisher pub;
        public:
            ARDroneVideo(){}
            ~ARDroneVideo(){this->active = false;}
            /**
             * @brief ARDroneVideo initialises the class from the parameters in the node handle
             * @param nh
             */
            ARDroneVideo(ros::NodeHandle nh);
            /**
             * @brief fetch_video continously listens to any incoming UDP packets and extraxts the x264 frames
             */
            void fetch_video();
    };

    /**
     * @brief The ARDroneDriver class
     * Class for republishing the raw image together with the CameraInfo for ArDrone2.0
     */
    class ARDroneDriver{
        private:
            sensor_msgs::CameraInfoPtr bottom_camera;
            sensor_msgs::CameraInfoPtr front_camera;
            std::string name;
            image_transport::CameraPublisher pub;
            image_transport::Subscriber sub;

        public:
            ARDroneDriver(){}
            ~ARDroneDriver(){}
            /**
             * @brief ARDroneDriver initialises the class from the parameters in the node handle
             * @param nh
             */
            ARDroneDriver(ros::NodeHandle nh);
            /**
             * @brief republish_callback republishes the image together with the CameraInfo
             * @param msg
             */
            void republish_callback(const sensor_msgs::ImageConstPtr& msg);
    };

}

#endif
