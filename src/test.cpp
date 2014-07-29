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
#include <stdint.h>

int main(int argc, char *argv[]){
    int port = 5556;
	uint32_t takeoff = 290718208;
	uint32_t land = 290717696;
	char buff[200];
//  	sprintf(buff, "AT*REF=%d,%d\rAT*REF=%d,%d\rAT*REF=%d,%d\r",i+1,command,i+2,command,i+3,command);
//  	printf("%li\n",strlen(buff));
//	printf("%s\n",buff);
	int sock;
	//Structure for address of server
	sockaddr_in myAddr;
	sockaddr_in droneAddr;
	//Create the socket
	if((sock=socket(AF_INET, SOCK_DGRAM, 0))<0)
	{
	perror("Failed to create socket");
	exit(EXIT_FAILURE);
	}

	myAddr.sin_family = AF_INET;
    myAddr.sin_addr.s_addr = INADDR_ANY; // my IP address
    myAddr.sin_port = htons(atoi("5556"));
    
    // the drone's sockaddr_in
    droneAddr.sin_family = AF_INET;
    droneAddr.sin_addr.s_addr = inet_addr("192.168.1.1");
    droneAddr.sin_port = htons(atoi("5556"));
    
    if(bind(sock,( struct sockaddr *) &myAddr, sizeof(myAddr))<0)
	{
	perror("bind failed");
	exit(EXIT_FAILURE);
	}	
	int i = 0;
	for(;i<1000;i++){
		sprintf(buff, "AT*CONFIG=%d,\"video:video_channel\",\"%s\"\r",i+1,argv[1]);
		printf("%li\n",strlen(buff));
		printf("%s\n",buff);
		if (sendto(sock, buff, strlen(buff), 0, (struct sockaddr *)&droneAddr, sizeof(droneAddr)) < 0) { perror("sendto failed"); return 0; }
	}	
    return 0;
}
