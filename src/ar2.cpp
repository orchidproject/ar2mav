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

//#define FF_API_SWS_GETCONTEXT = 1 
extern "C" {
#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
}


#define DRONE_IP_ADDR "192.168.1.1"
#define DRONE_VID_STREAM_PORT "5555"



#define VIDEO_BUFFER_SIZE 40000

void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame) {
  FILE *pFile;
  char szFilename[32];
  int  y;
  
  // Open file
  sprintf(szFilename, "frame%d.ppm", iFrame);
  pFile=fopen(szFilename, "wb");
  if(pFile==NULL)
    return;
  
  // Write header
  fprintf(pFile, "P6\n%d %d\n255\n", width, height);
  
  // Write pixel data
  for(y=0; y<height; y++)
    fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);
  
  // Close file
  fclose(pFile);
}

/*
// Time collection globals
double payload_time = 0;
double decoder_time = 0;
double converter_time = 0;
double gui_time = 0;


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
    uint8_t header1_size; / H.264 only : size of SPS inside payload - no SPS present if value is zero 
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

void printPaVE(parrot_video_encapsulation_t PaVE) {
    printf("\n---------------------------\n");
    printf("Codec : %s\n",
           (PaVE.video_codec == CODEC_MPEG4_VISUAL) ?
           "MP4" :
           ((PaVE.video_codec == CODEC_MPEG4_AVC) ? "H264" : "Unknown"));
    printf("StreamID : %d \n", PaVE.stream_id);
    printf("Timestamp : %d ms\n", PaVE.timestamp);
    printf("Encoded dims : %d x %d\n", PaVE.encoded_stream_width, PaVE.encoded_stream_height);
    printf("Display dims : %d x %d\n", PaVE.display_width, PaVE.display_height);
    printf ("Header size  : %d (PaVE size : %d)\n", PaVE.header_size, sizeof (parrot_video_encapsulation_t));
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
    } else {
        printf("asked for %i bytes, received packet of %i bytes\n", requestSize,
               lengthReceived);
    }
    return lengthReceived;
}
*/
void fetch_and_decode(int socketNumber, int destinationSocket, bool skip) {
	int index,badCheck,read;
	double lastTime;
	const uint16_t* header_size;
	const uint32_t* payloadsize;
	const uint32_t* timestamp;
	unsigned char part[VIDEO_BUFFER_SIZE];
	FILE * pFile = fopen("wtv.txt","w");
	int partLength;
	int cou = 0;
	index = 0;
				avcodec_register_all();
			avformat_network_init();
			av_log_set_level(AV_LOG_DEBUG);
			
			//AVCodec *codecMP4 = NULL;
			AVCodec *codecH264 = NULL;
			AVCodecContext *contextMP4 = NULL;
			AVCodecContext *contextH264 = NULL;
		    /*
		    codecMP4 = avcodec_find_decoder(CODEC_ID_MPEG4);
		    contextMP4 = avcodec_alloc_context3(codecMP4);
		    contextMP4->pix_fmt = PIX_FMT_YUV420P;
			contextMP4->skip_frame = AVDISCARD_DEFAULT;
			contextMP4->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
			//contextMP4->error_recognition = FF_ER_CAREFUL;
			contextMP4->skip_loop_filter = AVDISCARD_DEFAULT;
			contextMP4->workaround_bugs = FF_BUG_AUTODETECT;
			contextMP4->codec_type = AVMEDIA_TYPE_VIDEO;
			contextMP4->codec_id = CODEC_ID_MPEG4;
			contextMP4->skip_idct = AVDISCARD_DEFAULT;
			contextMP4->width = 640;
			contextMP4->height = 360;
		    avcodec_open2(contextMP4, codecMP4, 0);
		    */
		    codecH264 = avcodec_find_decoder(CODEC_ID_H264);
		    contextH264 = avcodec_alloc_context3(codecH264);
		    contextH264->pix_fmt = PIX_FMT_YUV420P;
			contextH264->skip_frame = AVDISCARD_DEFAULT;
			contextH264->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
			//contextH264->error_recognition = FF_ER_CAREFUL;
			contextH264->skip_loop_filter = AVDISCARD_DEFAULT;
			contextH264->workaround_bugs = FF_BUG_AUTODETECT;
			contextH264->codec_type = AVMEDIA_TYPE_VIDEO;
			contextH264->codec_id = CODEC_ID_H264;
			contextH264->skip_idct = AVDISCARD_DEFAULT;
			contextH264->width = 640;
			contextH264->height = 360;
			avcodec_open2(contextH264, codecH264, 0);
			
			SwsContext *sws_ctx = NULL;
			sws_ctx = sws_getContext(contextH264->width, contextH264->height,
			contextH264->pix_fmt,contextH264->width,contextH264->height,PIX_FMT_RGB24,SWS_BILINEAR,
        NULL, NULL, NULL);
        
        			AVFrame *picture = NULL;
			AVFrame *prs = NULL;
			AVPacket avPkt;
	while(true){
		cou ++;
		//printf("H\n");
		if(index == 0){
			partLength = -1;
			partLength = recv(socketNumber, part, VIDEO_BUFFER_SIZE,0);
			if (partLength < 0) {
			    printf("did not receive video data\n");
			    return;
			}
		}
		if (strncmp((const char*) (part+index),"PaVE", 4) != 0) {
		    printf("PaVE not synchronized, skipping iteration\n");
		    index = 0;
		    continue;
		} 
		const uint16_t* header_size = (const uint16_t*) (part + index + 6);
		const uint32_t* payloadsize = (const uint32_t*) (part + index + 8);
		const uint32_t* timestamp = (const uint32_t*) (part + index + 24);
		//printf("(%d,%d,%d,%d)\n",*header_size,*payloadsize,index,partLength);
		badCheck = -1;
		read = 0;
		if(partLength - index -  *header_size < *payloadsize){
			//printf("H2\n");
			read = partLength - index - *header_size;
			lastTime = clock();
			while (read < *payloadsize && (clock() - lastTime) < 0.1) {
				partLength = recv(socketNumber, part+index+*header_size+read, *payloadsize - read,0);
				read += partLength;
				lastTime = clock();
				//printf("W: %d,%d\n",read,partLength);
			}
			partLength = index + *header_size + *payloadsize;
		}

		//Transmit frame
		badCheck = write(destinationSocket, part+index+*header_size, *payloadsize);
		if (badCheck < 0) {
		    printf("Failed to send basic packet\n");
		}
		
		//Received more than one packet in the buffer
		if(partLength - index - *header_size > *payloadsize){
			//printf("(%d,%d,%d,%d,%d)\n",*header_size,*payloadsize,index,partLength,read);
			if(!skip)
				index += *header_size + *payloadsize;
		}
		else 
			index = 0;
			
			
//		printf("payload complete, attempting to decode frame\n");
//			avcodec_init();

			

			picture = avcodec_alloc_frame();
		    
		    prs = avcodec_alloc_frame();
		    /*
		    avcodec_open2(codecContext, codec, 0);
		    codecContext->width = 640;
		    codecContext->height = 360;
		    codecContext->pix_fmt = PIX_FMT_YUV420P;
		    codecContext->skip_frame = AVDISCARD_DEFAULT;
		    codecContext->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
		    codecContext->skip_loop_filter = AVDISCARD_DEFAULT;
		    codecContext->workaround_bugs = FF_BUG_AUTODETECT;
		    codecContext->debug = true;
		    */
		
		av_init_packet(&avPkt);
		avPkt.data = (unsigned char*)(part+index+*header_size);
		avPkt.size = *payloadsize;
		///printf("avPkt.size = %d\n",avPkt.size);
		int done = 0;
		int ret = -1;
//		t = clock();
		ret = avcodec_decode_video2(contextH264, picture, &done, &avPkt);
		//AVPicture m_Rgb;
		//m_Rgb.linesize[0] = contextH264->width * 4;
		//m_Rgb.data[0] = (uint8_t*)malloc( m_Rgb.linesize[0] * contextH264->height );
		//sws_scale(sws_ctx, picture->data, picture->linesize, 0, contextH264->height, m_Rgb.data, m_Rgb.linesize );
		//sws_scale(sws_ctx, picture->data, picture->linesize,contextH264->width,contextH264->height, prs->data,prs->linesize);
		//img_convert((AVPicture *)prs, PIX_FMT_RGB24, 
        //            (AVPicture*)picture, contextH264->pix_fmt, contextH264->width, 
        //            contextH264->height);
		//img_convert((AVPicture*)prs, PIX_FMT_RGB24, (AVPicture*)picture, PIX_FMT_YUV420P, 640, 360);
//		decoder_time += clock() - t;
		//avPkt.data = NULL;
		//avPkt.size = 0;
	    av_free_packet(&avPkt);
		if (done == 0 || ret < 0) {
		    printf("could not decode frame\n");
		    //return;
		}
		else{
			///printf("Picture size:%dx%d\n",picture->width,picture->height); 
			if(cou == 500){
				printf("WAWWWW\n");
				for(int x=0;x<(picture->width);x++)
					for(int y=0; y<(picture->height); y++){
						int offset = 3 * (x + y * picture->width);
						//fprintf(pFile,"%d, ",m_Rgb.data[0][offset]);
					}
					fprintf(pFile,"\n");
			}
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
    writers.sin_port = htons(atoi("57819"));
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
    printf("Waiting\n");
    listen(writer,5);
    int newsockfd = accept(writer, (struct sockaddr *)&target, &targ);
    if (newsockfd < 0)
    {
        perror("ERROR on accept");
        exit(1);
    }
    fetch_and_decode(socketNumber,newsockfd, false);

    return 1;
}


