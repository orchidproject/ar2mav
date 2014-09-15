#include <ar2mav/ar2mav.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>
#include <pluginlib/class_loader.h>

sensor_msgs::CameraInfoPtr loadCameraInfo(const ros::NodeHandle& nh, const std::string prefix){
        sensor_msgs::CameraInfoPtr camera = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo());
        int temp;
        std::string str_temp;
        std::vector<double> vec_temp;
        nh.param<int>(prefix + "/height", temp, 360);
        camera->height = temp;
        nh.param<int>(prefix + "/width", temp, 640);
        camera->width = temp;
        nh.param<std::string>(prefix + "/distortion_model", str_temp, "blob");
        camera->distortion_model = str_temp;
        nh.param<std::vector<double> >(prefix + "/D", vec_temp, std::vector<double>(5,0));
        for(int i=0;i<vec_temp.size();i++)
            camera->D.push_back(vec_temp[i]);
        nh.param<std::vector<double> >(prefix + "/K", vec_temp, std::vector<double>(9,0));
        for(int i=0;i<vec_temp.size();i++)
            camera->K[i] = vec_temp[i];
        nh.param<std::vector<double> >(prefix + "/R", vec_temp, std::vector<double>(9,0));
        for(int i=0;i<vec_temp.size();i++)
            camera->R[i] = vec_temp[i];
        nh.param<std::vector<double> >(prefix + "/P", vec_temp, std::vector<double>(12,0));
        for(int i=0;i<vec_temp.size();i++)
            camera->P[i] = vec_temp[i];
        nh.param<int>(prefix + "/binning_x", temp, 0);
        camera->binning_x = temp;
        nh.param<int>(prefix + "/binning_y", temp, 0);
        camera->binning_y = temp;
        nh.param<int>(prefix + "/roi/x_offset", temp, 0);
        camera->roi.x_offset = temp;
        nh.param<int>(prefix + "/roi/y_offset", temp, 0);
        camera->roi.y_offset = temp;
        nh.param<int>(prefix + "/roi/height", temp, 0);
        camera->roi.height = temp;
        nh.param<int>(prefix + "/roi/width", temp, 0);
        camera->roi.width = temp;
        nh.param<int>(prefix + "/roi/do_rectify", temp, 0);
        camera->roi.do_rectify = temp == 0 ? 0 : 1;
        camera->header.frame_id = prefix + "/bottom_camera";
        //ROS_INFO("%s,%f,%f,%f,%f,%f" ,prefix.c_str(), camera->D[0],camera->D[1],camera->D[2],camera->D[3],camera->D[4]);
        return camera;
    }

namespace ar2mav{


    void ARDroneDriver::republish_callback(const sensor_msgs::ImageConstPtr& msg){
            this->bottom_camera->header.stamp = msg->header.stamp;
            this->info_pub.publish(this->bottom_camera);
            this->image_pub->publish(msg);
        }

    ARDroneDriver::ARDroneDriver(ros::NodeHandle nh, std::string name, std::string in_transport, std::string out_transport){
            this->name = name;
            this->bottom_camera = loadCameraInfo(nh,"/" + name + "/bottom_camera");
            this->front_camera = loadCameraInfo(nh,"/" + name + "/front_camera");
            ROS_INFO("Loaded camera info's complete %s", in_transport.c_str());
            this->info_pub = nh.advertise<sensor_msgs::CameraInfo>("/" + name + "/video/camera_info",5);
            image_transport::ImageTransport it(nh);
            std::string in_topic = "/" + name + "/video";
            std::string out_topic = "/" + name + "/video/image_raw";
            pluginlib::ClassLoader<image_transport::PublisherPlugin> loader("image_transport", "image_transport::PublisherPlugin");
            std::string lookup_name = image_transport::PublisherPlugin::getLookupName(out_transport);
            this->image_pub = loader.createInstance(lookup_name);
            this->image_pub->advertise(nh, out_topic, 1, image_transport::SubscriberStatusCallback(),
                               image_transport::SubscriberStatusCallback(), ros::VoidPtr(), false);
            this->image_sub = it.subscribe(in_topic, 1, &ARDroneDriver::republish_callback, this, in_transport);
        }
}

	
