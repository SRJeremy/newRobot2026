// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
/*
FL = FR = BL = BR = 0;
sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
cameras[ROOM1_CAM].getVideoFrame(img, 1000);
resize(img, img, Size(640/2, 480/2));
flip(img, img, -1); // flip so facing right way

imshow("DEBUG", img);
input = waitKey(0);
* */
// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP


// Line trace with pi camera
#define TESTING 1

#define DEBUG_GREEN_TURN 0

// includes for opencv stuff
// includes for the serial communication and other stuff
#include <lccv.hpp>
#include <iostream>
#include <bitset> // for binary display 
#include <string.h>	// c
#include <string>	// c++
#include <errno.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <termios.h>                                                         
#include <stdio.h>
#include <stdlib.h>	                                                     
#include <fcntl.h>                                                                                                               
#include <sys/types.h> 
#include <stdint.h>
#include <sys/signal.h>
#include <time.h>
#include <stdbool.h>	

#include <opencv2/opencv.hpp>
#include <sstream> 

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <cmath> // Required for M_PI constant and math functions
#include <iomanip> // Optional: for setting output precision



#define BAUDRATE B115200                                                      
#define MODEMDEVICE "/dev/ttyAMA0"

#define DEBUG_IMSHOW(name, frame) imshow(name, frame);
//#define DEBUG_IMSHOW(name,frame)

#define ROOM1_CAM 0
#define ROOM2_CAM 1


#define GREEN_BOTTOM 0
#define GREEN_TOP 1
#define GREEN_RIGHT 2
#define GREEN_LEFT 3

#define COMMAND_LINE_TRACE 0
#define COMMAND_LEFT_GREEN 1
#define COMMAND_RIGHT_GREEN 2
#define COMMAND_DOUBLE_GREEN 3
#define COMMAND_DEBUG_STOP 4
#define COMMAND_ENCODER_MOVE 5
#define COMMAND_IMU_TURN 6

// if this is 0 green will be checked for like normal and handled 
// when green is detected and handled this is set to 1 UNTIL green is no longer detected at all, then it is set back to 0
int Green_Already = 0;







using namespace std::chrono_literals;
using namespace std;
using namespace cv;

int resolution_width = 1640;
int resolution_height = 1232;



char tx_buffer[256];
int stop_motors = 1;
int FL = 0, FR = 0, BL = 0, BR = 0;
unsigned char rx_buffer[256];
int rx_length = 0;
int input;





// GLOBAL CAM OBJECT JUST IN CASE ITS NEEDED IN FUNCTIONS ?? or maybe make the whole thing with an init_cam() function and then a get_img() function that returns a mat...?
lccv::PiCamera cameras[2];



// used to sort contours
// contour compare function
// void contour_compare(contoura contourb)
	// i = get area of one
	// j = get area of the other
	// return (i<j)
bool contour_compare(const vector<Point> &a, const vector<Point> &b) {
	return contourArea(a) < contourArea(b);
}



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
	setprecision(4); // set precision for float calculations   (not sure if i need this anymore.....not doing radians to degree calculations anymore.......)
	
	
	
	
    int index = 0;
    for (auto& cam : cameras){
        cam.options->camera = index++;
        cam.options->video_width=resolution_width;
        cam.options->video_height=resolution_height;
        cam.options->framerate=30;
        cam.options->verbose=true;
        cam.startVideo();
        //auto title = camTitle(cam.options->camera);
        //std::cout << title << " Started:"<< std::endl;
        //cv::namedWindow(title,cv::WINDOW_NORMAL);
    }



	

	
	// init serial and wait for arduino side to be ready
	init();
	

	
	
	rx_length = read(uart0_filestream, (void*)rx_buffer, 255); // clear buffer???

	printf("Waiting for pico to start...\n");
	
	if(!TESTING){
		while (rx_length <=0) 				 								//remove the while to make this non-blocking
			rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
			
		// clear the buffer?
		//while(rx_length > 0)
		//	rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
	}
	// data recieved, meaning the pico is ready to go
	rx_buffer[rx_length] = '\0';
	printf("Start data recieved: %s\n", rx_buffer);
	this_thread::sleep_for(100ms);
	
	
	
	
	
	Mat img;
	char input;
	do{
		
		if (!cameras[ROOM2_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM2_CAM].stopVideo();
			break;
		}
		// resize image that contains full FOV but will be small for fast compute later
		resize(img, img, Size(640/2, 480/2));
		
		
		imshow("room2", img);
		input = waitKey(1);

	} while(input != 'q');
	
	
	
	
	return 0;
}

