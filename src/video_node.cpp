#include<ros/ros.h>
#include<nodelet/loader.h>
#include<ar2mav/ar2mav.h>

int main(int argc, char **argv)
{
    //***************************************************************************
    //	Initialise this ROS Node and read ROS parameters
    //***************************************************************************
    ros::init(argc, argv, "drone_video");
    ros::NodeHandle nh("~");

    /*
    nodelet::Loader manager(true);
    nodelet::M_string remappings;
    nodelet::V_string my_argv;

    XmlRpc::XmlRpcValue shared_params;
    std::string name;
    if (nh.getParam("name", name))
        shared_params["name"] = name;

    ros::param::set(ros::this_node::getName() + "_video",shared_params);
    ros::param::set(ros::this_node::getName() + "_driver",shared_params);
    manager.load(ros::this_node::getName() + "_video", "ar2mav/ARDroneVideoNodelet", remappings, my_argv);
    manager.load(ros::this_node::getName() + "_driver", "ar2mav/ARDroneDriverNodelet", remappings, my_argv);
    ros::spin();
    */
    ar2mav::ARDroneVideo video = ar2mav::ARDroneVideo(nh);
    video.fetch_video();
    return 0;
}
