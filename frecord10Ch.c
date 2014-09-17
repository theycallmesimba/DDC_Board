/* frecord10Ch.c  Record data from DDC through all 10 channels
 * Includes function calls to PulseShapeDisc.ipf IGOR file
 * (C) 2014, Brandon Buonacorsi & James Morad <bbuonacorsi@ucdavis.edu>
 * Based off of frecord_events.c (C) 2012, Wojtek Skulski <info@skutek.com>
 * This program reads the data from the DDC through a server/client connection
 * port number to connect to server with is 8888
 *
 *   bfin-linux-uclibc-gcc -Wall -o frecord10Ch -mfdpic  frecord10Ch.c
 *   cp  frecord10Ch ~/uClinux-dist/romfs/bin
 *   sudo cp frecord10Ch /mnt/win
 * 
 * Pseudocode:
 *	  Wait for client to connect
 *    Assign handler
 *    ievent := 0;
 *    DO WHILE ( ievent < nevent)
 * (* read FPGA status register *)
 * IF acquisition is not running THEN 
 *     (* read data *)
 *     (* write data in ASCII to event struct *)
 *     (* send event struct over socket to client *)
 *     ievent := ievent + 1;
 * END IF;
 *   END; (* DO WHILE *)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>  //write
#include <time.h>
//#include <pthread.h> //for threading, link with lpthread
// #include <errno.h>
// #include <signal.h>
// #include <fcntl.h>
// #include <ctype.h>
// #include <termios.h>
// #include <sys/types.h>
// #include <sys/mman.h>
#include "bitmasks.h" /*useful bitmasks*/
#include "fpga_4futils.h" /*FPGA addresses*/
#include "fcommon.c" /*FPGA utility functions*/  
#define TOO_MANY_EVENTS 100001 /* avoid trashing the disk*/
#define N_SAMPLES 8192 /* waveforms contain 8192 samples*/
#define HISTOGRAM_SIZE 4096 /*In-situ performance measurement, extended 16-fold*/
#define ADBL0 -55.386
#define ADBL1 -41.403
#define ADBL2 51.872
#define ADCBL3 11.477
#define ADCBL4 27.727
#define ADCBL5 91.186
#define ADCBL6 -7.831
#define ADCBL7 -46.319
#define ADCBL8 -11.396
#define ADCBL9 22.708
#define ADCTOV 0.0001220703125

struct event_packet{

	unsigned int first_time;
	unsigned int second_time;
	unsigned int ch0_lo;
	unsigned int ch0_hi;
	unsigned int ch1_lo;
	unsigned int ch1_hi;
	unsigned int ch2_lo;
	unsigned int ch2_hi;
	unsigned int ch3_lo;
	unsigned int ch3_hi;
	unsigned int ch4_lo;
	unsigned int ch4_hi;
	unsigned int ch5_lo;
	unsigned int ch5_hi;
	unsigned int ch6_lo;
	unsigned int ch6_hi;
	unsigned int ch7_lo;
	unsigned int ch7_hi;
	unsigned int ch8_lo;
	unsigned int ch8_hi;
	unsigned int ch9_lo;
	unsigned int ch9_hi;
};

int main(int argc, char **argv) {
	
	unsigned int run_time_ms; /* total run time in ms*/
	int  nsmpls, i;
	long nevts, ievent, ntrials;
	unsigned int time = 0;
	unsigned int time_send;
	unsigned long  wave0_start, wave1_start, wave2_start, wave3_start, wave4_start, wave5_start, wave6_start, wave7_start, wave8_start, wave9_start; 
	unsigned long  *base0_ptr, *base1_ptr, *base2_ptr, *base3_ptr, *base4_ptr, *base5_ptr, *base6_ptr, *base7_ptr, *base8_ptr, *base9_ptr;  /* +4 pointer math*/
	unsigned long  *lptr0, *lptr1, *lptr2, *lptr3, *lptr4, *lptr5, *lptr6, *lptr7, *lptr8, *lptr9;  /* +4 pointer math*/ 
	unsigned long  lword0, lword1, lword2, lword3, lword4, lword5, lword6, lword7, lword8, lword9; /* long  for 32-bit  */
	unsigned int lo0, hi0, lo1, hi1, lo2, hi2, lo3, hi3, lo4, hi4, lo5, hi5, lo6, hi6, lo7, hi7, lo8, hi8, lo9, hi9; /* low and high parts of 32-bit word*/
	double vlo0, vhi0, vlo1, vhi1, vlo2, vhi2, vlo3, vhi3, vlo4, vhi4, vlo5, vhi5, vlo6, vhi6, vlo7, vhi7, vlo8, vhi8, vlo9, vhi9; /*low and high parts of baseline adjusted ADC output*/
	volatile unsigned long bits; /* aux variable for bit test*/
	long clk_ticks; /* FPGA  stop watch*/
	long histogram [HISTOGRAM_SIZE];
	
	struct event_packet this_event;
	
	//Socket variables
	int socket_desc, new_socket, c;
	struct sockaddr_in server, client;
	
	//Create socket
	socket_desc = socket(AF_INET, SOCK_STREAM, 0);
	if(socket_desc == -1)
	{
		puts("Could not create socket");
	}
	
	//Get sock descriptor
	int sock = socket_desc;
	int read_size;
	char client_message[2000];
	char *token; //to tokenize client string
	
	//Prepare sockaddr
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(8888); //port num = 8888
	
	//Bind the socket
	if(bind(socket_desc,(struct sockaddr *)&server, sizeof(server)) < 0)
	{
		puts("Bind failed");
		return 1;
	}
	
	//Listen for incoming client calls
	listen(socket_desc, 3);
	
	//Accept incoming client connections
	puts("Waiting for incoming connections...");
	c = sizeof(struct sockaddr_in);
	while((new_socket = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)))
	{
		puts("Connection accepted");
		
		//Get client IP and port number
		char *client_ip = inet_ntoa(client.sin_addr);
		//int client_port = ntohs(client.sin_port);
		puts(client_ip);
  
		printf("preparing to read message from client\n");
		//Receive message from client (i.e. #samples, #events)
		read_size = recv(sock, client_message, 2000, 0);
		
		//initialize token and then tokenize client_message
		token = strtok(client_message, ", ");
		nsmpls = strtoul(token, 0, 0);  /* how many samples each wave*/
		token = strtok(NULL, ", ");
		nevts  = strtoul(token, 0, 0);  /* how many events*/
		printf("received request to gather %ld events with %i samples per event\n", nevts, nsmpls);
	
		run_time_ms = Get_milliseconds (); /* start ms stop watch*/
	
		if ((nevts < 0) || (nevts >= TOO_MANY_EVENTS)) {
			fprintf(stderr, "Invalid num of events: %ld\n", nevts);
			return 0;
		} /* sanity check: invalid number of events*/
		if ((nsmpls < 0) || (nsmpls > N_SAMPLES)) {
			fprintf(stderr, "Invalid num of ADC samples: %d\n", nsmpls);
			return 0;
		} /* sanity check: invalid number of ADC samples*/
  
		/* wave_start address and pointer to FPGA waveforms */
		wave0_start = FPGA_WAVE_START + 0 * FPGA_WAVE_GAP;
		base0_ptr   = (unsigned long *) wave0_start; /* ptr to 32-bit words*/
		wave1_start = FPGA_WAVE_START + 1 * FPGA_WAVE_GAP;
		base1_ptr   = (unsigned long *) wave1_start; /* ptr to 32-bit words*/
		wave2_start = FPGA_WAVE_START + 2 * FPGA_WAVE_GAP;
		base2_ptr   = (unsigned long *) wave2_start; /* ptr to 32-bit words*/
		wave3_start = FPGA_WAVE_START + 3 * FPGA_WAVE_GAP;
		base3_ptr   = (unsigned long *) wave3_start; /* ptr to 32-bit words*/
		wave4_start = FPGA_WAVE_START + 4 * FPGA_WAVE_GAP;
		base4_ptr   = (unsigned long *) wave4_start; /* ptr to 32-bit words*/
		wave5_start = FPGA_WAVE_START + 5 * FPGA_WAVE_GAP;
		base5_ptr   = (unsigned long *) wave5_start; /* ptr to 32-bit words*/
		wave6_start = FPGA_WAVE_START + 6 * FPGA_WAVE_GAP;
		base6_ptr   = (unsigned long *) wave6_start; /* ptr to 32-bit words*/
		wave7_start = FPGA_WAVE_START + 7 * FPGA_WAVE_GAP;
		base7_ptr   = (unsigned long *) wave7_start; /* ptr to 32-bit words*/
		wave8_start = FPGA_WAVE_START + 8 * FPGA_WAVE_GAP;
		base8_ptr   = (unsigned long *) wave8_start; /* ptr to 32-bit words*/
		wave9_start = FPGA_WAVE_START + 9 * FPGA_WAVE_GAP;
		base9_ptr   = (unsigned long *) wave9_start; /* ptr to 32-bit words*/
	
		for (i = 0; i < HISTOGRAM_SIZE; i++) {
		histogram [i] = 0; 
		} /* for i*/
	
		/* **************** */
		/* BEGIN OUTER LOOP */
		/* **************** */
		ievent  = 0;
		ntrials = 0; /*how many times did we try the FPGA?*/
		while ( ievent < nevts) {
	
			/* ------------------------------------------------ 
			* Check if FPGA is running the waveform capture. 
			* If so then do not attempt reading the waveform. 
			* ------------------------------------------------*/
			ntrials ++;
			bits = FPGA_is_running();   /*1 == running; 0 == not running*/
		
			if ( bits == 0 ) {
			/*FPGA capture not running --> waveform is waiting*/
			/*We will use clock counter in the FPGA for profiling*/
		
			Stop_CLK_ticks ();  Clear_CLK_ticks();
			clk_ticks = 0;
			Start_CLK_ticks();
		
			//printf("WAVES/o %s\n",argv[3]); /* IGOR wave name*/
			//printf("BEGIN\n"); /* IGOR wave begin marker*/
			//printf("Time\tCh0\tCh1\tCh2\tCh3\tCh4\tCh5\tCh6\tCh7\tCh8\tCh9\n");
			/* inner loop acquires ADC samples; 2 samples per mem access*/
		
			for (i = 0; i < nsmpls/2; i++) { 
					/* every 32-bit read returns two 16-bit samples  */
					/* i is indexing two-sample words, 32-bits each  */
					/* 32 bit needs unpacking into two 16-bit shorts */
		
				lptr0 = (unsigned long *) (base0_ptr + i);
				lword0 = *( lptr0);
				lo0 = (unsigned int) (lword0 & 0x0000FFFF);
				vlo0 = ADCTOV*(lo0 - ADBL0);
				lword0 =      (lword0 & 0xFFFF0000);
				hi0 = (unsigned int) (lword0 >> 16);
				vhi0 = ADCTOV*(hi0-ADBL0);
		   
				lptr1 = (unsigned long *) (base1_ptr + i);
				lword1 = *( lptr1);
				lo1 = (unsigned int) (lword1 & 0x0000FFFF);
				vlo1 = ADCTOV*(lo1-ADBL1);
				lword1 =      (lword1 & 0xFFFF0000);
				hi1 = (unsigned int) (lword1 >> 16);
				vhi1 = ADCTOV*(hi1-ADBL1);
	   
				lptr2 = (unsigned long *) (base2_ptr + i);
				lword2 = *( lptr2);
				lo2 = (unsigned int) (lword2 & 0x0000FFFF);
				vlo2 = ADCTOV*(lo2-ADBL2);
				lword2 =      (lword2 & 0xFFFF0000);
				hi2 = (unsigned int) (lword2 >> 16);
				vhi2 = ADCTOV*(hi2-ADBL2);
		   
				lptr3 = (unsigned long *) (base3_ptr + i);
				lword3 = *( lptr3);
				lo3 = (unsigned int) (lword3 & 0x0000FFFF);
				vlo3 = ADCTOV*(lo3-ADCBL3);
				lword3 =      (lword3 & 0xFFFF0000);
				hi3 = (unsigned int) (lword3 >> 16);
				vhi3 = ADCTOV*(hi3-ADCBL3);
		   
				lptr4 = (unsigned long *) (base4_ptr + i);
				lword4 = *( lptr4);
				lo4 = (unsigned int) (lword4 & 0x0000FFFF);
				vlo4 = ADCTOV*(lo4-ADCBL4);
				lword4 =      (lword4 & 0xFFFF0000);
				hi4 = (unsigned int) (lword4 >> 16);
				vhi4 = ADCTOV*(hi4-ADCBL4);
	   
				lptr5 = (unsigned long *) (base5_ptr + i);
				lword5 = *( lptr5);
				lo5 = (unsigned int) (lword5 & 0x0000FFFF);
				vlo5 = ADCTOV*(lo5-ADCBL5);
				lword5 =      (lword5 & 0xFFFF0000);
				hi5 = (unsigned int) (lword5 >> 16);
				vhi5 = ADCTOV*(hi5-ADCBL5);
		   
				lptr6 = (unsigned long *) (base6_ptr + i);
				lword6 = *( lptr6);
				lo6 = (unsigned int) (lword6 & 0x0000FFFF);
				vlo6 = ADCTOV*(lo6-ADCBL6);
				lword6 =      (lword6 & 0xFFFF0000);
				hi6 = (unsigned int) (lword6 >> 16);
				vhi6 = ADCTOV*(hi6-ADCBL6);
	   
				lptr7 = (unsigned long *) (base7_ptr + i);
				lword7 = *( lptr7);
				lo7 = (unsigned int) (lword7 & 0x0000FFFF);
				vlo7 = ADCTOV*(lo7-ADCBL7);
				lword7 =      (lword7 & 0xFFFF0000);
				hi7 = (unsigned int) (lword7 >> 16);
				vhi7 = ADCTOV*(hi7-ADCBL7);
		   
				lptr8 = (unsigned long *) (base8_ptr + i);
				lword8 = *( lptr8);
				lo8 = (unsigned int) (lword8 & 0x0000FFFF);
				vlo8 = ADCTOV*(lo8-ADCBL8);
				lword8 =      (lword8 & 0xFFFF0000);
				hi8 = (unsigned int) (lword8 >> 16);
				vhi8 = ADCTOV*(hi8-ADCBL8);
	   
				lptr9 = (unsigned long *) (base9_ptr + i);
				lword9 = *( lptr9);
				lo9 = (unsigned int) (lword9 & 0x0000FFFF);
				vlo9 = ADCTOV*(lo9-ADCBL9);
				lword9 =      (lword9 & 0xFFFF0000);
				hi9 = (unsigned int) (lword9 >> 16);
				vhi9 = ADCTOV*(hi9-ADCBL9);
		   
				time +=10;
		   
				time_send = htonl(time);
		
				//Ensure correct bit order before sending
				lo0 = htons(lo0);
				lo1 = htons(lo1);
				lo2 = htons(lo2);
				lo3 = htons(lo3);
				lo4 = htons(lo4);
				lo5 = htons(lo5);
				lo6 = htons(lo6);
				lo7 = htons(lo7);
				lo8 = htons(lo8);
				lo9 = htons(lo9);
				hi0 = htons(hi0);
				hi1 = htons(hi1);
				hi2 = htons(hi2);
				hi3 = htons(hi3);
				hi4 = htons(hi4);
				hi5 = htons(hi5);
				hi6 = htons(hi6);
				hi7 = htons(hi7);
				hi8 = htons(hi8);
				hi9 = htons(hi9);
		   
				this_event.first_time = time_send;
			   
				time+=10;
			   
				time_send = htonl(time);
			   
				this_event.second_time = time_send;
			   
				this_event.ch0_lo = lo0;
				this_event.ch0_hi = hi0;
				this_event.ch1_lo = lo1;
				this_event.ch1_hi = hi1;
				this_event.ch2_lo = lo2;
				this_event.ch2_hi = hi2;
				this_event.ch3_lo = lo3;
				this_event.ch3_hi = hi3;
				this_event.ch4_lo = lo4;
				this_event.ch4_hi = hi4;
				this_event.ch5_lo = lo5;
				this_event.ch5_hi = hi5;
				this_event.ch6_lo = lo6;
				this_event.ch6_hi = hi6;
				this_event.ch7_lo = lo7;
				this_event.ch7_hi = hi7;
				this_event.ch8_lo = lo8;
				this_event.ch8_hi = hi8;
				this_event.ch9_lo = lo9;
				this_event.ch9_hi = hi9;
			   
				int struct_size = sizeof(this_event);
				struct_size = htonl(struct_size);
				printf("sending size of struct...\n");
				send(sock,&struct_size,sizeof(int),0);
				printf("sending struct...\n");
				send(sock,&this_event,sizeof(this_event),0);
				printf("sent struct of size %i\n", ntohl(struct_size));
				printf("size of time is %ld\n",sizeof(unsigned int));
				printf("size of unsigned int is %ld\n",sizeof(unsigned int));
				printf("lo0 is %i\n",lo0);
		   
			}
			
		
	
			clk_ticks = clk_ticks / 1000; /*use 10 us step*/
			if (clk_ticks < 0) { 
				histogram[0] ++; /*underflow*/
			} else if (clk_ticks >= HISTOGRAM_SIZE) {
				histogram[HISTOGRAM_SIZE-1] ++; /*overflow*/
			} else {
				histogram[clk_ticks] ++; /*in histo*/
			} 
	
			ievent++; /*inc event counter*/
	
		} /* if FPGA data was waiting */
	
		} /* end of outer WHILE (loop over events)*/
	
		/* ****************/
		/* END OUTER LOOP */
		/* ****************/
  
		fflush(stdout);
  
		if(read_size == 0)
		{
			puts("Client disconnected");
			fflush(stdout);
		}
		else if(read_size == -1)
		{
			perror("recv failed");
		}
  
		if(new_socket < 0)
		{
			perror("Accept failed");
			return 1;
		}
	
	}
	return 0;
} /* END OF UTILITY*/
