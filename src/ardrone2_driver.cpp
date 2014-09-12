#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/publisher_plugin.h>
#include <pluginlib/class_loader.h>

class ARDrone2Driver{

private:
    sensor_msgs::CameraInfo bottom_camera;
    sensor_msgs::CameraInfo front_camera;
    std::string name;
    boost::shared_ptr<image_transport::PublisherPlugin> image_pub;
    image_transport::Subscriber image_sub;
    ros::Publisher info_pub;

    sensor_msgs::CameraInfo loadCameraInfo(const ros::NodeHandle& nh, const std::string prefix){
        sensor_msgs::CameraInfo camera = sensor_msgs::CameraInfo();
        int temp;
        std::string str_temp;
        std::vector<double> vec_temp;
        nh.param<int>(prefix + "/height", temp, 360);
        camera.height = temp;
        nh.param<int>(prefix + "/width", temp, 640);
        camera.width = temp;
        nh.param<std::string>(prefix + "/distortion_model", str_temp, "blob");
        camera.distortion_model = str_temp;
        nh.param<std::vector<double> >(prefix + "/D", vec_temp, std::vector<double>(5,0));
        for(int i=0;i<vec_temp.size();i++)
            camera.D.push_back(vec_temp[i]);
        nh.param<std::vector<double> >(prefix + "/K", vec_temp, std::vector<double>(9,0));
        for(int i=0;i<vec_temp.size();i++)
            camera.K[i] = vec_temp[i];
        nh.param<std::vector<double> >(prefix + "/R", vec_temp, std::vector<double>(9,0));
        for(int i=0;i<vec_temp.size();i++)
            camera.R[i] = vec_temp[i];
        nh.param<std::vector<double> >(prefix + "/P", vec_temp, std::vector<double>(12,0));
        for(int i=0;i<vec_temp.size();i++)
            camera.P[i] = vec_temp[i];
        nh.param<int>(prefix + "/binning_x", temp, 0);
        camera.binning_x = temp;
        nh.param<int>(prefix + "/binning_y", temp, 0);
        camera.binning_y = temp;
        nh.param<int>(prefix + "/roi/x_offset", temp, 0);
        camera.roi.x_offset = temp;
        nh.param<int>(prefix + "/roi/y_offset", temp, 0);
        camera.roi.y_offset = temp;
        nh.param<int>(prefix + "/roi/height", temp, 0);
        camera.roi.height = temp;
        nh.param<int>(prefix + "/roi/width", temp, 0);
        camera.roi.width = temp;
        nh.param<int>(prefix + "/roi/do_rectify", temp, 0);
        camera.roi.do_rectify = temp == 0 ? 0 : 1;
        return camera;
    }

public:
    void republish_callback(const sensor_msgs::ImageConstPtr& msg){
        this->bottom_camera.header.stamp = ros::Time::now();
        this->info_pub.publish(this->bottom_camera);
        this->image_pub->publish(msg);
    }

    ARDrone2Driver(std::string name, std::string in_transport, std::string out_transport){
        this->name = name;
        ros::NodeHandle nh("~");
        bottom_camera = this->loadCameraInfo(nh,name + "/bottom_camera");
        front_camera = this->loadCameraInfo(nh,name + "/front_camera");
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
        this->image_sub = it.subscribe(in_topic, 1, &ARDrone2Driver::republish_callback, this, in_transport);
    }

    ~ARDrone2Driver(){}
};

int main(int argc, char **argv){
    ros::init(argc, argv, "ardrone2_driver");
    ros::NodeHandle nh("~");
    std::string name, in_transport, out_transport;
    nh.param<std::string>("name", name, "drone");
    nh.param<std::string>("in_transport", in_transport, "x264");
    nh.param<std::string>("in_transport", out_transport, "raw");
    ARDrone2Driver driver =  ARDrone2Driver(name,in_transport, out_transport);
    ROS_INFO("Starting ArDrone2.0 Driver node...");
    ros::spin();
}

	
