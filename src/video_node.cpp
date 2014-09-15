#include<ros/ros.h>
#include<ar2mav/ar2mav.h>

int main(int argc, char **argv)
{
    //***************************************************************************
    //	Initialise this ROS Node and read ROS parameters
    //***************************************************************************
    ros::init(argc, argv, "x264_test_publisher");
    ros::NodeHandle nh("~");
    ar2mav::ARDroneVideo video = ar2mav::ARDroneVideo(nh);
    video.fetch_video();
    return 0;
}
