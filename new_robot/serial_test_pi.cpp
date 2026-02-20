/*
//#include <lccv.hpp>
//#include <iostream>
#include <string.h>	// c
//#include <string>	// c++
//#include <errno.h>
//#include <unistd.h>
//#include <chrono>
#include <thread>
#include <termios.h>                                                         
//#include <stdio.h>
//#include <stdlib.h>	                                                     
#include <fcntl.h>                                                                                                               
//#include <sys/types.h> 
//#include <stdint.h>
#include <sys/signal.h>
//#include <time.h>
//#include <stdbool.h>	

//#include <opencv2/opencv.hpp>
#include <sstream> 

//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui.hpp>
*/


#include <string.h>	// c
#include <thread>
#include <termios.h>                                                         
#include <fcntl.h>                                                                                                               
#include <sys/signal.h>
#include <sstream> 



using namespace std::chrono_literals;
using namespace std;



#define BAUDRATE B115200                                                      
#define MODEMDEVICE "/dev/ttyAMA0"



int uart0_filestream = -1;
void init(){
	// connect to the arduino device
	uart0_filestream = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
		exit(-1);
	}
	
	// config stuff
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
}	


int main(){
	
	init();
	
	// wait until pico starts up and is ready
	//this_thread::sleep_for(1000ms);
	
	
	// Read up to 255 characters from the port if they are there
	unsigned char rx_buffer[256];
	int rx_length = 0;
		
	printf("Waiting for pico to start...\n");
		
	while (rx_length <=0) 				 								//remove the while to make this non-blocking
		rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		
	// data recieved, meaning the pico is ready to go
	rx_buffer[rx_length] = '\0';
	printf("Start data recieved: %s\n", rx_buffer);
	
	/*
	if (rx_length < 0)
	{
		//An error occured (will occur if there are no bytes)
	}
	else if (rx_length == 0)
	{
		//No data waiting - if we are non-blocking
	}
	else
	{
		//Bytes received
		rx_buffer[rx_length] = '\0';
		printf("%i bytes read : %s", rx_length, rx_buffer);
	}
	*/

	printf("Sending data\n");
	
	int FL = 0, FR = 0, BL = 0, BR = 0;
	char tx_buffer[100];
	while(1){
		// send data to pico
		
		//sprintf(tx_buffer, "%d, %d, %d, %d", FL++, FR--, BL++, BR--);
		sprintf(tx_buffer, "[%d,%d,%d,%d]", FL, FR, BL, BR);
		printf("DATA SENT:\n%s\n\n", tx_buffer);
		/*
		tx_buffer[0] = FL++;
		tx_buffer[1] = FR--; 
		tx_buffer[2] = BL++;
		tx_buffer[3] = BR--;
		printf("FL : %d\n", tx_buffer[0]);
		printf("FR : %d\n", tx_buffer[1]);
		printf("BL : %d\n", tx_buffer[2]);
		printf("BR : %d\n", tx_buffer[3]);
		*/
		
		int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		printf("COUNT: %d\n\n", count);
		if (count < 0)
		{
			printf("UART TX error\n");
		}
		
		FL++;
		FR--;
		BL++;
		BR--;
		
		
		if(FL > 100)
			FL = 0;
		
		if(FR < -100)
			FR = 0;
		
		if(BL > 100)
			BL = 0;
		if(BR < -100)
			BR = 0;
		
		
		//this_thread::sleep_for(200ms);
		this_thread::sleep_for(20ms);	
	}
	
	
}
