#include<ros/ros.h>
#include<nodelet/nodelet.h>
#include<boost/thread.hpp>

class ARDrone2DriverNodelet: public nodelet::Nodelet {

private:
	void onInit(){
		is_active = true;
		driverThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&ARDrone2DriverNodelet::republish, this)));
	}
	void republish(){
		NODELET_INFO("spinning");
	}
	volatile bool is_active;
	boost::shared_ptr<boost::thread> driverThread;

public:
	ARDrone2DriverNodelet():
	is_active(false)
	{}

	~ARDrone2DriverNodelet(){
		if(is_active){
			NODELET_INFO("Shutting down driver thread.");
			is_active = false;
			driverThread->join();
			NODELET_INFO("Driver thread stopped.");
        }
    }
}

	
