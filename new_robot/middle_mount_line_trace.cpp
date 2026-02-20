// Line trace with pi camera
#define TESTING 1

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

#include <cmath> // Required for M_PI constant and math functions
#include <iomanip> // Optional: for setting output precision



#define BAUDRATE B115200                                                      
#define MODEMDEVICE "/dev/ttyAMA0"

#define DEBUG_IMSHOW(name, frame) imshow(name, frame);
//#define DEBUG_IMSHOW(name,frame)

#define ROOM1_CAM 0
#define ROOM2_CAM 1





using namespace std::chrono_literals;
using namespace std;
using namespace cv;

int resolution_width = 1640;
int resolution_height = 1232;



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
	
	setprecision(4); // set precision for float calculations
	
	
	
	lccv::PiCamera cameras[1];
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

	unsigned char rx_buffer[256];
	int rx_length = 0;

	printf("Waiting for pico to start...\n");
	
	if(!TESTING){
		while (rx_length <=0) 				 								//remove the while to make this non-blocking
			rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
	}
	// data recieved, meaning the pico is ready to go
	rx_buffer[rx_length] = '\0';
	printf("Start data recieved: %s\n", rx_buffer);
	
	

	// OLD PID stuff
	/*
	int target_speed = 30, motor_change = 0, error = 0;//, total_error = 0, previous_error = 0;
	int targetX = 160;
	// kp of .30 and .35 works well
	//float kp = 0.33;//, ki = 0, kd = 0;
	float kp = 1.0;//, ki = 0, kd = 0;
	float panic_kp = 2.5;
	const float base_kp = kp;
	int panic_range = 5;
	*/
	
	
	char tx_buffer[256];
	int stop_motors = 1;
	int FL = 0, FR = 0, BL = 0, BR = 0;
	Mat img;
	int input;
	
	
	/*  ----------------------------------------------- NEW PID STUFF -----------------------------------------------  */
	// idk what method to use for line trace
	float top_bot_slope = 0;
	float mid_bot_slope = 0;
	//int adjust = 0;
	float kp = 1.4;
	float base_kp = kp;
	float delta = 0;
	int target_speed = 30;
	float error = 0, bot_error = 0;
	do{
		
		if (!cameras[ROOM1_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM1_CAM].stopVideo();
			break;
		}
		// resize image that contains full FOV but will be small for fast compute later
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
		

		
				
		
		// a clone for drawing/displaying later
		Mat og_img = img.clone();
		
		// grayscale, blur, and threshold the line
		Mat img_line, line_only;
		cvtColor(img, img_line, COLOR_BGR2GRAY);
		medianBlur(img_line, img_line, 3);
		threshold(img_line, img_line, 120, 255, THRESH_BINARY_INV);
		
		
		
		// get the contours of the whole line
		vector<vector<Point>> contours_line;
		findContours(img_line, contours_line, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		
		if(contours_line.size() > 0){
			
		
			// find largest contour
			// max element returns pointer so dereference here
			vector<Point> line = *max_element(contours_line.begin(), contours_line.end(), contour_compare);
			
			// draw the largest contour (the line)
			drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
			
			
			// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
			drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
			
			
			
			
			// now to get aditional data from the line contour
			// rect contrustor being used is (point(topLeftx, topLefty), point(botRightx, botRighty))
			// OR
			// x, y, width, height
			//int wide_rect_width = img.cols;
			//int wide_rect_height = img_line.rows/4;
			//int tall_rect_width = img.cols/5;
			//int tall_rect_height = img_line.rows;
			
			
			int wide_rect_scalar = 4;
			int tall_rect_scalar = 10;
			// top slice
			Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
			Mat top_img = line_only(top_rect);
			rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
			
			// bottom slice
			Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
			Mat bot_img = line_only(bot_rect);
			rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
			
			
			// middle slice
			Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
			Mat mid_img = line_only(mid_rect);
			rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
			
			// left slice
			Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat left_img = line_only(left_rect);
			rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
			
			// right slice
			Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat right_img = line_only(right_rect);
			rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
			
			
			// slop from bot line to top line or bot to mid...
			int topx = 0, topy = 0, midx = 0, midy = 0, botx = 0, boty = 0;// MAKE THIS a single array with macros for indexing or somthing????
			Moments m;
			
			vector<vector<Point>> top_contour;
			findContours(top_img, top_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(top_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(top_contour[0]);
				// get center x and center y
				topx = m.m10 / m.m00; // col
				topy = m.m01 / m.m00; // row
			}
			
			vector<vector<Point>> mid_contour;
			findContours(mid_img, mid_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(mid_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(mid_contour[0]);
				// get center x and center y
				midx = m.m10 / m.m00; // col
				midy = m.m01 / m.m00; // row
			}
			
			vector<vector<Point>> bot_contour;
			findContours(bot_img, bot_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(bot_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(bot_contour[0]);
				// get center x and center y
				botx = m.m10 / m.m00; // col
				boty = m.m01 / m.m00; // row
			}
			
			// fix how the centroids are off due to slicing
			// top if fine where it is
			// bot is off in the y direction. its shifted, have to add to it based on the amount that was modified for slicing
			boty += ((wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar));
			// mid is off in the y direction. its shifted, fix like bot
			midy += (line_only.rows/wide_rect_scalar);
			
			topy += 0; // just getting rid of a dumb warning
			
			// print data for debug
			//cout << "TOP CENTROID (" << topx << ", " << topy << ")" << endl;
			//cout << "MID CENTROID (" << midx << ", " << midy << ")" << endl;
			//cout << "BOT CENTROID (" << botx << ", " << boty << ")" << endl;
			
			// set default values for slopes in case they are not there
			top_bot_slope = mid_bot_slope = 10000;
			kp = base_kp;
			
			// check if the bot contour is in the center or not
			if(botx > 0){
				bot_error = botx - (line_only.cols/2);
			}
			
			
			// chck if there was a centroid and then calc the slope 
			if(topx > 0 && botx > 0) {  // greater than 0 means it has a value other than the 0 that it was set to 
				// find the slope from top to bot
				top_bot_slope = topx - botx;
				//top_bot_slope = atanf(((float)boty-topy) / (botx-topx)) * (180.0/M_PI);
				//if(top_bot_slope > 0) top_bot_slope -= 90;
				//else if(top_bot_slope < 0) top_bot_slope += 90;
				
				cout<< "top_bot_slope: " << top_bot_slope << endl;
				
				// SET KP lower because there is a lot of line to be had
				kp = base_kp * 1.3;
				error = top_bot_slope - 0; // target right now is 0
			}
			// check mid and bot
			else if(midx > 0 && botx > 0) {  // greater than 0 means it has a value other than the 0 that it was set to 
				// find the slope from top to bot
				mid_bot_slope = midx - botx;
				//mid_bot_slope = atanf(((float)boty-midy) / (botx-midx)) * (180.0/M_PI);
				//if(mid_bot_slope > 0) mid_bot_slope -= 90;
				//else if(mid_bot_slope < 0) mid_bot_slope += 90;
				cout<< "mid_bot_slope: " << mid_bot_slope << endl;
				
				// SET KP slightly higher since the line is almost gone(ish)
				kp = base_kp * 2;
				error = mid_bot_slope - 0; // target right now is 0
			}
			
			else{
				error = 0;
				cout << "NO SLOPE DETECTED" << endl;
			}
			
			
			// calculate change based on calulcated error and kp
			delta = ((error+bot_error) * kp);
			
			cout << "BOT ERROR: " << bot_error << endl;
			cout << "ERROR: " << error << endl;
			cout << "DELTA: " << delta << endl;
			
			FL = target_speed + delta;
			BL = target_speed + delta;
			FR = target_speed - delta;
			BR = target_speed - delta;
			
			/*
			if(FL < 0 && FL > -20){
				FL += -30;
			}
			if(BL < 0 && BL > -20){
				BL += -30;
			}
			if(FR < 0 && FR > -20){
				FR += -30;
			}
			if(BR < 0 && BR > -20){
				BR += -30;
			}
			*/
			
			cout << "MOTORS ->  " << FL << " | " << FR << endl;
			
			
			
			
			// ----------------------------------------------------  OLD METHOD  ---------------------------------------------------- //
			/*
			// bounding rect gets geometric center
			// moments is the center of mass
			Moments m = moments(line);
			
			// get center x and center y
			int cx = m.m10 / m.m00; // col
			int cy = m.m01 / m.m00; // row
			
			//printf("\nMoments: %d, %d\n\n", cx, cy);
			
			// draw the new center over img
			circle(og_img, Point(img.cols/2, img.rows/2), 10, Scalar(0, 255, 0), FILLED); 	// center of the image itself
			circle(og_img, Point(cx, cy), 5, Scalar(255,0,0), FILLED);			// center of the contour
			
			//target
			targetX = img.cols/2;
			
			// when cy gets above 200, it means the line is about to go out of sight of the camera view
			if(cy > (panic_range-1) * (img.cols/panic_range)){
				kp = panic_kp;
			}
			else{
				kp = base_kp;
			}
			
			// calculate error
			error = cx - targetX;
			
			
			//total_error += error;
			//if(abs(error) < 40){
			//	total_error = 0;
			//}


			motor_change = (int)(error*kp);// + (int)(total_error*ki) + (int)((error - previous_error) * kd);
			
			
			FL = target_speed + motor_change;
			BL = target_speed + motor_change;
			FR = target_speed - motor_change;
			BR = target_speed - motor_change;
			*/
			// ----------------------------------------------------  OLD METHOD  ---------------------------------------------------- //
			
			
		}
		
		if(!line_only.empty())
			imshow("line_only", line_only);
			
		//imshow("TOP", top_img);
		//imshow("BOT", bot_img);
		//imshow("LEFT", left_img);
		imshow("OG img", og_img);
		
		
		
		// add this stuff to line trace i think...
		input = waitKey(1);
		
		if(input == ' '){
			stop_motors = 1;
		}
		else if(input == 'g'){
			stop_motors = 0;
		}
		/*
		else if(input == 82){ // up arrow key
			FL = 40;
			BL = 40;
			FR = 40;
			BR = 40;
		}
		else if(input == 84){ // down arrow key
			FL = -40;
			BL = -40;
			FR = -40;
			BR = -40;
		}
		else if(input == 81){ // left arrow key
			FL = -40;
			BL = -40;
			FR = 40;
			BR = 40;
		}
		else if(input == 83){ // right arrow key
			FL = 40;
			BL = 40;
			FR = -40;
			BR = -40;
		}
		else if(!stop_motors){
			
			// line trace function
			
		}
		*/
		
		
		// if stop motors is true, send all 0s
		else if(stop_motors){
			FL = BL = FR = BR = 0;
		}
		
		
		cout << endl << "DONE WITH ITERATION!!" << endl << endl << endl << endl;
		// send data to arduino
		sprintf(tx_buffer, "[%d,%d,%d,%d]", FL, FR, BL, BR);
		int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
		
	}while(input != 'q');
	
	
	FL = BL = FR = BR = 0;
	sprintf(tx_buffer, "[%d,%d,%d,%d]", FL, FR, BL, BR);
	int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	
	
	cameras[ROOM1_CAM].stopVideo();
	destroyAllWindows();
	
	
	
}
