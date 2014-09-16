#include <ar2mav/ar2mav.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

/**
 * @brief Loads a CameraInfo from the parameter server
 * @param[in] nh - the node handle to be used
 * @param[in] prefix - for retrieving the correct parameters from the server
 * @returns A CameraInfoPtr with the fill in data
 */
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
        return camera;
    }

namespace ar2mav{

    /**
     * See header file
     */
    void ARDroneDriver::republish_callback(const sensor_msgs::ImageConstPtr& msg){
            this->bottom_camera->header.stamp = msg->header.stamp;
            this->info_pub.publish(this->bottom_camera);
            this->image_pub.publish(msg);
        }

    /**
     * See header file
     */
    ARDroneDriver::ARDroneDriver(ros::NodeHandle nh){
            nh.param<std::string>("name", this->name, "drone");
            this->bottom_camera = loadCameraInfo(nh,"/" + name + "/bottom_camera");
            this->front_camera = loadCameraInfo(nh,"/" + name + "/front_camera");
            this->info_pub = nh.advertise<sensor_msgs::CameraInfo>("/" + name + "/video/camera_info",5);
            image_transport::ImageTransport it(nh);
            this->image_pub = it.advertise("/" + name + "/video/image_raw",1);
            std::string in_transport;
            nh.param<std::string>("in_transport", in_transport, "x264");
            this->image_sub = it.subscribe("/" + name + "/video", 1, &ARDroneDriver::republish_callback, this, in_transport);
        }
}

	
