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
	while (rx_length <=0) {
		this_thread::sleep_for(300ms);				 							
		rx_length = read(uart0_filestream, (void*)rx_buffer, 255);	
	}	
		
	// data recieved, meaning the pico is ready to go
	rx_buffer[rx_length] = '\0';
	printf("Start data received: %s\n", rx_buffer);
	
	
	
	int ct = 0;
	char tx_buffer[100];
	while(1){
		// send data to pico
		sprintf(tx_buffer, "%c message from the pi", 'a' + (ct % 26));
		ct++;
		ct %= 26000;
		
		int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
		if (count < 0)
		{
			printf("UART TX error\n");
		}
		
		printf("DATA SENT: %s\n", tx_buffer);
		
		// wait for the response
		rx_length = 0; // reset rx_length
		while (rx_length <=0)
		{
			this_thread::sleep_for(300ms);
			rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
		}
			
		rx_buffer[rx_length] = '\0';
		printf("Response received: %s\n\n", rx_buffer);
		
		
		this_thread::sleep_for(500ms);	
	}
	
	
}
