
// Line trace with pi camera


// includes for opencv stuff
// includes for the serial communication and other stuff
#include <lccv.hpp>
#include <iostream>
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



#define BAUDRATE B115200                                                      
#define MODEMDEVICE "/dev/ttyAMA0"

#define DEBUG_IMSHOW(name, frame) imshow(name, frame);

#define ROOM1_CAM 0

//#define DEBUG_IMSHOW(name,frame)



using namespace std::chrono_literals;
using namespace std;
using namespace cv;

/*
std::string camTitle(int num){
    std::string title("Video");
    title += std::to_string(num);
    return title;
}
*/

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
	lccv::PiCamera cameras[1];
    int index = 0;
    int resolution_width = 1640;
    int resolution_height = 922; //1232
    for (auto& cam : cameras){
        cam.options->camera = index++;
        
        // https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes
        // width x height
        // 640x480 Aspect ratio with fps 40 to 90
        // 1640x922  Aspect ratio 16:9 which is a bit distorted fps 1 to 40
        // 	1640x1232 Aspect ratio 4:3 (same as the 640x480 resolution) fps 1 to 40
        // 3280x2464 Apsect ratio 4:3 but fps is <= 15
       
        cam.options->video_width= resolution_width;
        cam.options->video_height = resolution_height;
        
        
        cam.options->framerate=30;
        cam.options->verbose=true;
        cam.startVideo();
        //auto title = camTitle(cam.options->camera);
        //std::cout << title << " Started:"<< std::endl;
        //cv::namedWindow(title,cv::WINDOW_NORMAL);
    }






	Mat img;
	int input;
	do{
		
		if (!cameras[ROOM1_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM1_CAM].stopVideo();
			break;
		}
		flip(img, img, -1);
		
		
		Mat og_img = img.clone();
		
		resize(img, img, Size(resolution_width/2,resolution_height/2));
		imshow("img", img);
		
		//imshow("OG img", og_img);	
		
		// add this stuff to line trace i think...
		input = waitKey(1);
		
	}while(input != 'q');
	
	
	cameras[ROOM1_CAM].stopVideo();
	destroyAllWindows();
	
	
	
}
