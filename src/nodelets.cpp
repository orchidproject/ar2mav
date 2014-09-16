#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <ar2mav/ar2mav.h>

namespace ar2mav{
    class ARDroneDriverNodelet : public nodelet::Nodelet{
        public:
            ARDroneDriverNodelet(){}
            ~ARDroneDriverNodelet(){}
        private:
            boost::shared_ptr<ar2mav::ARDroneDriver> driver;
            virtual void onInit();
    };

    void ARDroneDriverNodelet::onInit(){
        ros::NodeHandle nh(getMTPrivateNodeHandle());
        this->driver = boost::shared_ptr<ar2mav::ARDroneDriver>(new ar2mav::ARDroneDriver(nh));
    }

    class ARDroneVideoNodelet : public nodelet::Nodelet{
        public:
            ARDroneVideoNodelet(){}
            ~ARDroneVideoNodelet(){this->video.~ARDroneVideo();}
        private:
            boost::shared_ptr<boost::thread> worker;
            ar2mav::ARDroneVideo video;
            virtual void onInit();
    };


    void ARDroneVideoNodelet::onInit(){
        ros::NodeHandle nh(getMTPrivateNodeHandle());
        this->video = ar2mav::ARDroneVideo(nh);
        this->worker = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ar2mav::ARDroneVideo::fetch_video, this->video)));
    }
} //namespace ar2mav

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(ar2mav, ARDroneVideoNodelet, ar2mav::ARDroneVideoNodelet, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS(ar2mav, ARDroneDriverNodelet, ar2mav::ARDroneDriverNodelet, nodelet::Nodelet);

