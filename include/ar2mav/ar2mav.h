#include<ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>

#ifndef AR2MAV_H
#define AR2MAV_H

namespace ar2mav{
    int fetch_video(ros::NodeHandle nh, std::string drone_ip, int drone_port,
        int buffer_size, struct timeval timeout, std::string name);

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
            ARDroneVideo(ros::NodeHandle nh);
            void fetch_video();
    };

    class ARDroneDriver{
        private:
            sensor_msgs::CameraInfo bottom_camera;
            sensor_msgs::CameraInfo front_camera;
            std::string name;
            boost::shared_ptr<image_transport::PublisherPlugin> image_pub;
            image_transport::Subscriber image_sub;
            ros::Publisher info_pub;

        public:
            ARDroneDriver(){}
            ~ARDroneDriver(){}
            ARDroneDriver(ros::NodeHandle nh, std::string name, std::string in_transport, std::string out_transport);
            void republish_callback(const sensor_msgs::ImageConstPtr& msg);
    };

}

#endif
