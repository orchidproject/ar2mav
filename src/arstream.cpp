//Found that

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>

#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif
 
extern "C" {
#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
}


#define DRONE_IP_ADDR "192.168.1.1"
#define DRONE_VID_STREAM_PORT "5555"



#define VIDEO_BUFFER_SIZE 40000

// Time collection globals
double payload_time = 0;
double decoder_time = 0;
double converter_time = 0;
double gui_time = 0;


typedef struct { //PaVE
    uint8_t signature[4]; /* "PaVE" - used to identify the start of
     frame */
    uint8_t version; /* Version code */
    uint8_t video_codec; /* Codec of the following frame */
    uint16_t header_size; /* Size of the parrot_video_encapsulation_t */
    uint32_t payload_size; /* Amount of data following this PaVE */
    uint16_t encoded_stream_width; /* ex: 640 */
    uint16_t encoded_stream_height; /* ex: 368 */
    uint16_t display_width; /* ex: 640 */
    uint16_t display_height; /* ex: 360 */
    uint32_t frame_number; /* Frame position inside the current stream
     */
    uint32_t timestamp; /* In milliseconds */
    uint8_t total_chuncks; /* Number of UDP packets containing the
     current decodable payload - currently unused */
    uint8_t chunck_index; /* Position of the packet - first chunk is #0
     - currenty unused*/
    uint8_t frame_type; /* I-frame, P-frame -
     parrot_video_encapsulation_frametypes_t */
    uint8_t control; /* Special commands like end
            -of-stream or
            advertised frames */
    uint32_t stream_byte_position_lw; /* Byte position of the current payload in
             the encoded stream - lower 32-bit word */
    uint32_t stream_byte_position_uw; /* Byte position of the current payload in
             the encoded stream - upper 32-bit word */
    uint16_t stream_id; /* This ID indentifies packets that should be
             recorded together */
    uint8_t total_slices; /* number of slices composing the current
             frame */
    uint8_t slice_index; /* position of the current slice in the frame
             */
    uint8_t header1_size; /* H.264 only : size of SPS inside payload -
             no SPS present if value is zero */
    uint8_t header2_size; /* H.264 only : size of PPS inside payload -
             no PPS present if value is zero */
    uint8_t reserved2[2]; /* Padding to align on 48 bytes */
    uint32_t advertised_size; /* Size of frames announced as advertised
             frames */
    uint8_t reserved3[12]; /* Padding to align on 64 bytes */
    uint8_t reserved4[4]; // padding -- added b/c it was in the KIPR library code
} __attribute__ ((packed)) parrot_video_encapsulation_t;

typedef enum { //PaVE codec IDs
    CODEC_UNKNNOWN = 0,
    CODEC_VLIB,
    CODEC_P264,
    CODEC_MPEG4_VISUAL,
    CODEC_MPEG4_AVC
} parrot_video_encapsulation_codecs_t;

typedef enum { //PaVE frame types
    FRAME_TYPE_UNKNNOWN = 0, FRAME_TYPE_IDR_FRAME, /* headers followed by I-frame */
    FRAME_TYPE_I_FRAME, FRAME_TYPE_P_FRAME, FRAME_TYPE_HEADERS
} parrot_video_encapsulation_frametypes_t;

void printPaVE(parrot_video_encapsulation_t PaVE) {
    printf("\n---------------------------\n");

    printf("Codec : %s\n",
           (PaVE.video_codec == CODEC_MPEG4_VISUAL) ?
           "MP4" :
           ((PaVE.video_codec == CODEC_MPEG4_AVC) ? "H264" : "Unknown"));

    printf("StreamID : %d \n", PaVE.stream_id);
    printf("Timestamp : %d ms\n", PaVE.timestamp);
    printf("Encoded dims : %d x %d\n", PaVE.encoded_stream_width,
           PaVE.encoded_stream_height);
    printf("Display dims : %d x %d\n", PaVE.display_width, PaVE.display_height);
    ////printf ("Header size  : %d (PaVE size : %d)\n", PaVE.header_size, sizeof (parrot_video_encapsulation_t));
    printf("Header size : %d\n", PaVE.header_size);
    printf("Payload size : %d\n", PaVE.payload_size);
    printf("Size of SPS inside payload : %d\n", PaVE.header1_size);
    printf("Size of PPS inside payload : %d\n", PaVE.header2_size);
    printf("Slices in the frame : %d\n", PaVE.total_slices);
    printf("Frame Type / Number : %s : %d : slide %d/%d\n",
           (PaVE.frame_type == FRAME_TYPE_P_FRAME) ?
           "P-Frame" :
           ((PaVE.frame_type == FRAME_TYPE_I_FRAME) ?
            "I-Frame" : "IDR-Frame"), PaVE.frame_number,
           PaVE.slice_index + 1, PaVE.total_slices);

    printf("---------------------------\n\n");
}

int receive(int socketNumber, unsigned char *buffer, int requestSize) {
    int lengthReceived = -1;
    lengthReceived = recv(socketNumber, buffer, requestSize, 0);
    if (lengthReceived < 0) {
        printf("failed to receive assumed PaVE packet\n");
    } /*else {
        printf("asked for %i bytes, received packet of %i bytes\n", requestSize,
               lengthReceived);
    }*/
    return lengthReceived;
}

void fetch_and_decode(int socketNumber, int destinationSocket) {
	int index = 0;
	double t,lastTime;
	int partLength;
	int payloadLength;
	uint32_t read;
	unsigned char payload[VIDEO_BUFFER_SIZE];
	unsigned char part[VIDEO_BUFFER_SIZE];
	parrot_video_encapsulation_t PaVE;
	while(true){
		t = clock();
		if(index == 0){
			partLength = -1;
			partLength = receive(socketNumber, part, VIDEO_BUFFER_SIZE);
			//printf("1\n");
			if (partLength < 0) {
			    printf("did not receive video data\n");
			    return;
			}
		}
		memcpy(&PaVE, part + index, sizeof(parrot_video_encapsulation_t));
		//printf("2\n");
		if (strncmp((const char*) PaVE.signature, "PaVE", 4) != 0) {
		    printf("PaVE not synchronized, skipping iteration\n");
		    index = 0;
		    continue;
		} 
		else {
		    //printf("PaVE synchronized. YIPEEEEEEEEEEEEEEEEEEEEEEEE\n");
		    //printPaVE(PaVE);
		}
		//Received more than one packet in buffer
		//printf("(%d,%d,%d,%d)\n", partLength, index, read,PaVE.payload_size);
		if(partLength - index -  PaVE.header_size > PaVE.payload_size){
			memcpy(payload, part + index + PaVE.header_size, PaVE.payload_size);
			index += PaVE.header_size + PaVE.payload_size;
		}
		else{
			read = 0;
			if(memcpy(payload, part + index + PaVE.header_size, partLength - index - PaVE.header_size));
			read += partLength - index - PaVE.header_size;
			if(read > PaVE.payload_size){
				printf("%d > %d \n",read, PaVE.payload_size);
			}
			lastTime = clock();

			payloadLength = -1;
			while (read < PaVE.payload_size && (clock() - lastTime) < 0.1) {
				//printf("gathering payload...\n");
				payloadLength = receive(socketNumber, payload + read, PaVE.payload_size - read);
				read += payloadLength;
				lastTime = clock();
			}

			if (read != PaVE.payload_size) {
				printf("FAIL %d %d \n", read, PaVE.payload_size );
				continue; // if the while loop times out before a full frame, need to exit
			}

			payload_time += clock() - t;
			index = 0;
		}

//		printf("payload complete, attempting to decode frame\n");
	/*
			AVFrame *picture = NULL;
			AVCodec *codec = NULL;
			avcodec_register_all();
		    av_register_all();
		    av_log_set_level(AV_LOG_DEBUG);
		    codec = avcodec_find_decoder(CODEC_ID_H264);
		    codec->id = CODEC_ID_H264;
		    
			AVCodecContext *codecContext = NULL;
			codecContext = avcodec_alloc_context3(codec);
		    avcodec_open2(codecContext, codec, 0);
		    codecContext->width = 640;
		    codecContext->height = 360;
		    codecContext->pix_fmt = PIX_FMT_YUV420P;
		    codecContext->skip_frame = AVDISCARD_DEFAULT;
		    codecContext->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
		    codecContext->skip_loop_filter = AVDISCARD_DEFAULT;
		    codecContext->workaround_bugs = FF_BUG_AUTODETECT;
		    codecContext->debug = true;
		    
		AVPacket avPkt;
		av_init_packet(&avPkt);
		avPkt.data = NULL;
		avPkt.size = 0;
		avPkt.data = payload;
		avPkt.size = PaVE.payload_size;
		printf("avPkt.size = %d\n",avPkt.size);
		int done = 0;
		int ret = -1;
		t = clock();
		ret = avcodec_decode_video2(codecContext, picture, &done, &avPkt);
		decoder_time += clock() - t;
		avPkt.datag = NULL;
		avPkt.size = 0;
		av_free_packet(&avPkt);
		if (done == 0 || ret < 0) {
		    printf("could not decode frame\n");
		    return;
		}
	*/
		// send one packet of "some bytes" to drone
		int badCheck = -1;
		badCheck = write(destinationSocket, payload, PaVE.payload_size);
		if (badCheck < 0) {
		    printf("Failed to send basic packet\n");
		}
	}
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
 
int main() {
    printf("\n\n*********************** START ***********************\n\n");

    int socketNumber;
    int writer;
    socklen_t targ;
    sockaddr_in myAddr;
    sockaddr_in droneAddr;
    sockaddr_in writers;
    sockaddr_in target;

    // my sockaddr_in
    myAddr.sin_family = AF_INET;
    myAddr.sin_addr.s_addr = INADDR_ANY; // my IP address
    myAddr.sin_port = htons(atoi(DRONE_VID_STREAM_PORT));

    // the drone's sockaddr_in
    droneAddr.sin_family = AF_INET;
    droneAddr.sin_addr.s_addr = inet_addr(DRONE_IP_ADDR);
    droneAddr.sin_port = htons(atoi(DRONE_VID_STREAM_PORT));

    // player socket
    writers.sin_family = AF_INET;
    writers.sin_addr.s_addr = INADDR_ANY;
    writers.sin_port = htons(atoi("57810"));
    // player socket
    //target.sin_family = AF_INET;
    //target.sin_addr.s_addr = inet_addr("127.0.0.1");
    //target.sin_port = htons(atoi("5781"));

    socketNumber = socket(AF_INET, SOCK_STREAM, 0);
    writer = socket(AF_INET, SOCK_STREAM, 0);
    targ = socket(AF_INET, SOCK_STREAM, 0);

    // bind the socket
    if (bind(socketNumber, (sockaddr*) &myAddr, sizeof(sockaddr_in)) < 0) {
        printf("failed to bind socket\n");
    }
    if (bind(writer, (sockaddr*) &writers, sizeof(writers)) < 0) {
        printf("failed to bind socket2\n");
    }
    int result = -1;
    //result = bind(writer, (struct sockaddr*)&writers, sizeof(sockaddr_in));

    result = connect(socketNumber, (sockaddr*) &droneAddr, sizeof(sockaddr_in));
    if (result != 0) {
        printf("connection NOT established\n");
    }

    //result = connect(writer, (sockaddr*) &target, sizeof(sockaddr_in));
    listen(writer,5);
    int newsockfd = accept(writer, (struct sockaddr *)&target, &targ);
    if (newsockfd < 0)
    {
        perror("ERROR on accept");
        exit(1);
    }

        fetch_and_decode(socketNumber,newsockfd);

    return 1;
}


