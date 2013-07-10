/*
  g++ cata_voice_festival.cpp -o cata_voice_festival -I/usr/include/festival -I/usr/include/speech_tools -lFestival -leststring -lestools -lestbase
  Install female voices: http://www.cstr.ed.ac.uk/projects/festival/mbrola.html
*/

#include "stdio.h"
#include "festival/festival.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#define BUFFSIZE 255

void Die(char *mess) 
{ 
	perror(mess); 
	exit(1); 
}


bool SayText(const EST_String &text)
{
	festival_say_text(text);
	festival_wait_for_spooler();	
	return true;
}

int main(int argc, char** argv)
{
	system("clear");
	printf("Cata Voice Festival\nCCNY Robotics Lab 2011\n\n");
	
	printf("-> initializing UDP server ...\n");
	int sock;
	struct sockaddr_in echoserver;
	struct sockaddr_in echoclient;
	char buffer[BUFFSIZE];
	unsigned int echolen, clientlen, serverlen;
        int received = 0;

	printf("... Create the UDP socket\n");
	if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
            Die("Failed to create socket");

        memset(&echoserver, 0, sizeof(echoserver));       /* Clear struct */
	echoserver.sin_family = AF_INET;                  /* Internet/IP */
	echoserver.sin_addr.s_addr = htonl(INADDR_ANY);   /* Any IP address */
	echoserver.sin_port = htons(12345);       /* server port */
	
	printf("... Bind the socket\n");
	serverlen = sizeof(echoserver);
	if (bind(sock, (struct sockaddr *) &echoserver, serverlen) < 0)
		Die("Failed to bind server socket");

	printf("... UDP service is ready\n\n");	

	printf("-> initializing festival API ...\n");
	int heap_size=210000;
	int load_init_files=1;
	festival_initialize(load_init_files,heap_size);

	printf("... Cata Voice is ready\n");
	SayText("Cata Voice is ready");

	while (1) 
	{
		/* Receive a message from the client */
		clientlen = sizeof(echoclient);
		if ((received = recvfrom(sock, buffer, BUFFSIZE, 0, (struct sockaddr *) &echoclient, &clientlen)) < 0) 
                	Die("Failed to receive message");
              	fprintf(stderr, "Message Received from: %s\n", inet_ntoa(echoclient.sin_addr));
		buffer[received] = NULL;	
		printf(buffer);
		
		SayText(buffer);
       }

	
	return 0;
}
