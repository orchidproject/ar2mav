#include <arpa/inet.h>
#include <image_transport/image_transport.h>
#include <x264_image_transport/x264Packet.h>

/**
 * Receives ArDrone 2.0 video stream and publishes it on x264_image_transport
 */

int establish_socket(const char* drone_ip, const int* drone_port, const struct timeval* timeout){
	//***************************************************************************
	//  Set up sockets
	//***************************************************************************
	int socketNumber;
	
	sockaddr_in myAddr;
	sockaddr_in droneAddr;

	myAddr.sin_family = AF_INET;
	myAddr.sin_addr.s_addr = INADDR_ANY;
	myAddr.sin_port = htons(*drone_port);

	droneAddr.sin_family = AF_INET;
	droneAddr.sin_addr.s_addr = inet_addr(drone_ip);
	droneAddr.sin_port = htons(*drone_port);

	socketNumber = socket(AF_INET, SOCK_STREAM, 0);
	while(ros::ok() && bind(socketNumber, (sockaddr*) &myAddr, sizeof(sockaddr_in)) < 0) {
		ROS_INFO("Failed to bind socket");
		ros::Duration((*timeout).tv_sec).sleep();
	}
	while(ros::ok() && connect(socketNumber, (sockaddr*) &droneAddr, sizeof(sockaddr_in)) != 0) {
		ROS_INFO("Did not manage to establish connection");
		ros::Duration((*timeout).tv_sec).sleep();
	}
	setsockopt(socketNumber, SOL_SOCKET, SO_RCVTIMEO, (char *)timeout,sizeof(struct timeval));
	return socketNumber;
}

int fetch_video(ros::NodeHandle nh,int drone_port, std::string drone_ip, 
		int buffer_size, struct timeval timeout, std::string topic_name){
	//***************************************************************************
	//   Helper variables
	//***************************************************************************
	int index, read, result, i;
	const uint16_t* header_size;
	const uint32_t* payloadsize;
	unsigned char part[buffer_size];
	int partLength;
	bool check;
	x264_image_transport::x264Packet message;
	index = 0;
	//***************************************************************************
	//   Initialise connection and publisher
	//***************************************************************************
	int socketNumber = establish_socket(drone_ip.c_str(), &drone_port, &timeout);
	ros::Publisher pub = nh.advertise<x264_image_transport::x264Packet>(topic_name, 1000);
	//***************************************************************************
	//   Decode PaVE packet and send the encoded video stream
	//***************************************************************************
	ROS_INFO("***** START VIDEO STREAM *****");
	while (ros::ok()) {
		if(index == 0) {
			partLength = recv(socketNumber, part, buffer_size,0);
			if (partLength <= 0) {
				ROS_INFO("Did not receive video data, trying to recover.");
				close(socketNumber);
				ros::Duration(timeout.tv_sec).sleep();
				socketNumber = establish_socket(drone_ip.c_str(), &drone_port, &timeout);
				index = 0;
				continue;
			}
		}
		if (strncmp((const char*) (part+index),"PaVE", 4) != 0) {
			ROS_INFO("PaVE not synchronized, trying to rebind");
			for(i = 0;i<buffer_size-index-3;i++)
				if(strncmp((const char*) (part+index+i),"PaVE", 4) == 0){
					index += i;
					break;
				}
			if(i == buffer_size-index-3)
				index = 0;
			continue;
		}
		header_size = (const uint16_t*) (part + index + 6);
		payloadsize = (const uint32_t*) (part + index + 8);
		message.img_width = *(const uint16_t*) (part + index + 16);
		message.img_height = *(const uint16_t*) (part + index + 18);
		message.header.stamp.fromSec(*(const uint32_t*) (part + index + 24) / 1000.0);
		// This packet did not contain all the data
		if(partLength - index -  *header_size < *payloadsize) {
			read = partLength - index - *header_size;
			check = false;
			while (read < *payloadsize) {
				partLength = recv(socketNumber, part+index+*header_size+read, *payloadsize - read,0);
				if(partLength <= 0){
					check = true;
					break;
				}
				read += partLength;
			}
			if(check){
				ROS_INFO("Timedout while waiting extra packets.");
				index = 0;
				continue;
			}		
			partLength = index + *header_size + *payloadsize;
		}

		message.data.assign(part+index+*header_size, part+index+*header_size+*payloadsize);
		pub.publish(message);
		//Received more than one packet in the buffer
		if(partLength - index - *header_size > *payloadsize)
			index += *header_size + *payloadsize;
		else
			index = 0;
	}
	ROS_INFO("Closing socket.");
	close(socketNumber);
	return 0;
}

int main(int argc, char **argv)
{
	//***************************************************************************
	//	Initialise this ROS Node and read ROS parameters
	//***************************************************************************
	ros::init(argc, argv, "x264_test_publisher");
	ros::NodeHandle nh("~");

	int drone_port;
	nh.param<int>("drone_port", drone_port, 5555);
	std::string drone_ip;
	nh.param<std::string>("drone_ip", drone_ip, "192.168.1.1");
	int buffer_size;
	nh.param<int>("buffer_size", buffer_size, 65536);
	struct timeval timeout;
	timeout.tv_usec = 0;
	int temp;
	nh.param<int>("timeout", temp, 1);
	timeout.tv_sec = temp;
	std::string topic_name;
	nh.param<std::string>("topic_name", topic_name, "/image/x264");
	
	image_transport::ImageTransport it(nh);
	return fetch_video(nh, drone_port, drone_ip, buffer_size, timeout, topic_name);
}

/*
typedef struct { //PaVE
    uint8_t signature[4]; // "PaVE" - used to identify the start of frame
    uint8_t version; // Version code
    uint8_t video_codec; // Codec of the following frame
    uint16_t header_size; // Size of the parrot_video_encapsulation_t
    uint32_t payload_size; // Amount of data following this PaVE
    uint16_t encoded_stream_width; // ex: 640
    uint16_t encoded_stream_height; // ex: 368
    uint16_t display_width; // ex: 640
    uint16_t display_height; // ex: 360
    uint32_t frame_number; // Frame position inside the current stream
    uint32_t timestamp; // In milliseconds
    uint8_t total_chuncks; // Number of UDP packets containing the current decodable payload - currently unused
    uint8_t chunck_index; // Position of the packet - first chunk is #0 - currenty unused
    uint8_t frame_type; // I-frame, P-frame - parrot_video_encapsulation_frametypes_t
    uint8_t control; // Special commands like end-of-stream or advertised frames
    uint32_t stream_byte_position_lw; // Byte position of the current payload in the encoded stream - lower 32-bit word
    uint32_t stream_byte_position_uw; // Byte position of the current payload in the encoded stream - upper 32-bit word
    uint16_t stream_id; // This ID indentifies packets that should be recorded together
    uint8_t total_slices; // number of slices composing the current frame
    uint8_t slice_index; // position of the current slice in the frame
    uint8_t header1_size; // H.264 only : size of SPS inside payload - no SPS present if value is zero
    uint8_t header2_size; // H.264 only : size of PPS inside payload - no PPS present if value is zero
    uint8_t reserved2[2]; // Padding to align on 48 bytes
    uint32_t advertised_size; // Size of frames announced as advertised frames
    uint8_t reserved3[12]; // Padding to align on 64 bytes
    uint8_t reserved4[4]; // padding -- added b/c it was in the KIPR library code
} __attribute__ ((packed)) parrot_video_encapsulation_t;

typedef enum {
    //PaVE codec IDs
    CODEC_UNKNNOWN = 0,
    CODEC_VLIB,
    CODEC_P264,
    CODEC_MPEG4_VISUAL,
    CODEC_MPEG4_AVC
} parrot_video_encapsulation_codecs_t;

typedef enum {
	//PaVE frame types
    FRAME_TYPE_UNKNNOWN = 0, FRAME_TYPE_IDR_FRAME, // headers followed by I-frame
    FRAME_TYPE_I_FRAME, FRAME_TYPE_P_FRAME, FRAME_TYPE_HEADERS
} parrot_video_encapsulation_frametypes_t;

*/
