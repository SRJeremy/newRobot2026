
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
#define TESTING 0

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
lccv::PiCamera cameras[1];



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




// return 1:left 2:right 3:uturn
char green_square(Mat &img, Mat &line){
	// img is just regular image, need to process for hsv and inrange
	
	// low and high range for inrange
	Scalar low_range(70, 75, 40), high_range(97, 255, 188);
	
	
	// process the image directly
	Mat debug = img.clone();
	cvtColor(img, img, COLOR_BGR2HSV);
	inRange(img, low_range, high_range, img);
	
	DEBUG_IMSHOW("green thresh", img);
	
	// find contours
	vector<vector<Point>> contours;
	findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	
	// get rid of the green square from the line threshold..... !!!!!!!!!
	drawContours(line, contours, -1, 0, -1);
	
	cout << "???????? GREEN CONTOURS FOUND   " << contours.size() << endl;
	// if no green contours return -1 right away 
	if(contours.size() == 0){
		return 255;
	}
	
	// if green has already been handled and there are contours, ignore them and just return 0
	if(Green_Already == 1){
		return 0;
	}
	
	// sort by contour area
	//printf("\nSorted %zu contours\n", contours.size());
	sort(contours.begin(), contours.end(), contour_compare);
	//int left_green_count = 0, right_green_count = 0;
	
	vector<char> green_detections;
	
	int valid_contours = 0;
	for (auto& cnt : contours) {
		//cout << "CONTOUR AREA GREEN SQUARE -> " << contourArea(cnt) << endl;
		
		// area is 12,000 when fully on screen, around 8000 partially on and 3000 when its pretty small...
		if(contourArea(cnt) < 3000) {
			continue;
		}
		
		
			
		Rect br = boundingRect(cnt);
		Point center(br.x + (br.width / 2), br.y + (br.height / 2));
		
		if(center.y < img.rows/3 && contours.size() <= 1){
			continue;
		}
		
		valid_contours++;
		
		// Points and values for checking green squares
		Point check_above, check_below, check_left, check_right;
		//int left_val = -1, right_val = -1, above_val = -1, below_val = -1;
		
		// SINGLE byte (4 bits needed) to represent L, R, T, B (left right top bottom) line locations around the green square
		char green_status = 0;
		
		// check the top to see if its false green first
		// CHECK ABOVE GREEN
		if(center.y - (int)(br.height/2) - 10 > 0) {
			check_above = Point(center.x, center.y - (int)(br.height/2) - 10);
			circle(debug, check_above, 3, Scalar(255, 255, 0), FILLED);			// CYAN
			if(line.at<unsigned char>(check_above)){
				green_status |= (1 << GREEN_TOP);
			}
		}
		// CHECK BELOW GREEN
		if(center.y + (int)(br.height/2) + 10 < img.rows){
			check_below = Point(center.x, center.y + (int)(br.height/2) + 10);
			circle(debug, check_below, 3, Scalar(255, 0, 255), FILLED);			// PURPLE
			if(line.at<unsigned char>(check_below)){
				green_status |= (1 << GREEN_BOTTOM);
			}
		}
		// CHECK LEFT OF GREEN
		if(center.x - (int)(br.width/2) - 10 > 0) {
			check_left = Point(center.x - (int)(br.width/2) - 10, center.y);
			circle(debug, check_left, 3, Scalar(255, 0, 0), FILLED);			// BLUE
			if(line.at<unsigned char>(check_left)){
				green_status |= (1 << GREEN_LEFT);
			}
		}
		// CHECK RIGHT OF GREEN
		if(center.x + (int)(br.width/2) + 10 < img.cols){
			check_right = Point(center.x + (int)(br.width/2) + 10, center.y);
			circle(debug, check_right, 3, Scalar(0, 255, 0), FILLED);			// GREEN
			if(line.at<unsigned char>(check_right)){
				green_status |= (1 << GREEN_RIGHT);
			}
		}
		
		green_detections.push_back(green_status);
		
		cout << "GREEN CONTOUR STATUS CODE: " << bitset<4>(green_status) << endl;
		
			
		
	}
	
	// check green_detections and all status codes that have been detected
	char green_turn = 0;
	vector<char> turn_codes;
	for(auto& g : green_detections){
		switch(g){
			case 0b1010:
				cout << "RIGHT GREEN" << endl;
				green_turn |= 1;
				break;
			case 0b0110:
				cout << "LEFT GREEN" << endl;
				green_turn |= (1 << 1);
				break;
				
			// POSSIBLE double green scenarios....
			case 0b1000:
				cout << "RIGHT GREEN not all the way on screen yet" << endl;
				green_turn |= (1 << 3);
				break;
			case 0b0100:
				cout << "LEFT GREEN not all the way on screen yet" << endl;
				green_turn |= (1 << 2);
				break;
			default:
				cout << "false green..... or somthing" << endl;
		}
		//turn_codes.push_back(green_turn);
	}
	
	
	cout << "green TURN CODE " << bitset<2>(green_turn) << endl;
	
	DEBUG_IMSHOW("DEBUG GREEN", debug);
	
	if(valid_contours == 0){
		return 255;
	}
	
	return green_turn;
}






void handle_gap(){
	int gap_centered = 0; // flag to indicate if the gap is centered
	
	
	// this is going to be a lot of work..... need to backup and straighten out on the gaps....
	Mat img;
	
	
	int error = 0;
	int target_speed = -40; // line trace backwards to line up with the gap
	float kp = 1.5;
	float base_kp = kp;
	int delta;
	do{
		
		if (!cameras[ROOM1_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM1_CAM].stopVideo();
			break;
		}
		// resize image that contains full FOV but will be small for fast compute later
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
		imshow("GAP IMG", img);
		input = waitKey(1);
		
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
			//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
			
			
			// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
			drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
			
			int wide_rect_scalar = 4;
			int tall_rect_scalar = 10;
			// top slice
			Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
			Mat top_img = line_only(top_rect);
			//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
			
			// bottom slice
			Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
			Mat bot_img = line_only(bot_rect);
			//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
			
			
			// middle slice
			Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
			Mat mid_img = line_only(mid_rect);
			//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
			
			// left slice
			Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat left_img = line_only(left_rect);
			//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
			
			// right slice
			Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat right_img = line_only(right_rect);
			//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
			
			
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
			
			
			
			
			// CHECK IF IT SEEMS TO BE A GAP OR A VERY SHARP TURN?????
			
			
			
			
			
			
			
			if(topx > 0){
				error = botx - (line_only.cols/2);
				kp = base_kp + .5;
				gap_centered = 1;
				break;//??
			}
			else if(midx > 0){
				error = botx - (line_only.cols/2);
				kp = base_kp + .3;
			}
			else if(botx > 0){
				error = botx - (line_only.cols/2);
				kp = base_kp;
			}
			
			// minimize error if its so close to the line
			//if(abs(error) < 20){
			//	error = 0;
			//}
			
			// if line is centered and on the last part of the line, its all good
			//if(topx <= 0 && midx <= 0 && botx > 0 && error == 0){
			//if(error == 0){
			//	gap_centered = 1;
			//}
			
			
			delta = (error * kp);
			delta *= -1;
			FL = target_speed + delta;
			BL = target_speed + delta;
			FR = target_speed - delta;
			BR = target_speed - delta;
			
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
			
			
			
		
		}
		else{
			// LOST THE LINE, need to back up and find it again??
			/*
			FL = -5; // FL contains the distance in CM when command encoder move is used
			FR = BL = BR = 0;
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));	
			// wait for arduino side to send back the ALL DONE command
			if(!TESTING && !stop_motors){
				rx_length = 0;
				while (rx_length <=0) {				 								//remove the while to make this non-blocking
					rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
				}
				
				rx_buffer[rx_length] = '\0';
				printf("Continue command: %s\n", rx_buffer);
			}
			*/
			gap_centered = 2; // this is kindof an error code.....
		}
		
		
	} while(!gap_centered);
	
	//FL = FR = BL = BR = 0;
	//sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
	//write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
	
	// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	FL = FR = BL = BR = 0;
	sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
	write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
	cameras[ROOM1_CAM].getVideoFrame(img, 1000);
	resize(img, img, Size(640/2, 480/2));
	flip(img, img, -1); // flip so facing right way
	
	imshow("DEBUG42", img);
	input = waitKey(0);
	// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	
	
	// make sure the line is straight
	int line_angle = 1000;
	do{
		cameras[ROOM1_CAM].getVideoFrame(img, 1000);
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
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
				//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
				
				
				// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
				drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
				
				int wide_rect_scalar = 4;
				int tall_rect_scalar = 10;
				// top slice
				Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
				Mat top_img = line_only(top_rect);
				//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
				
				// bottom slice
				Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
				Mat bot_img = line_only(bot_rect);
				//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
				
				
				// middle slice
				Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
				Mat mid_img = line_only(mid_rect);
				//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
				
				// left slice
				Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
				Mat left_img = line_only(left_rect);
				//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
				
				// right slice
				Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
				Mat right_img = line_only(right_rect);
				//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
				
				
				// slop from bot line to top line or bot to mid...
				int topx = 0, topy = 0, midx = 0, midy = 0, botx = 0, boty = 0;// MAKE THIS a single array with macros for indexing or somthing????
				int leftx = 0, lefty = 0, rightx = 0, righty = 0;
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
				
				
				
				
				// CHECK IF IT SEEMS TO BE A GAP OR A VERY SHARP TURN?????
				vector<vector<Point>> left_contour;
				findContours(left_img, left_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				if(left_contour.size() > 0){
					// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
					m = moments(left_contour[0]);
					// get center x and center y
					leftx = m.m10 / m.m00; // col
					lefty = m.m01 / m.m00; // row
				}
				
				vector<vector<Point>> right_contour;
				findContours(right_img, right_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
				if(right_contour.size() > 0){
					// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
					m = moments(right_contour[0]);
					// get center x and center y
					rightx = m.m10 / m.m00; // col
					righty = m.m01 / m.m00; // row
				}
				
				
				
				if(rightx > 0){
					// very sharp right turn
					
					// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
					FL = FR = BL = BR = 0;
					sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
					write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
					cameras[ROOM1_CAM].getVideoFrame(img, 1000);
					resize(img, img, Size(640/2, 480/2));
					flip(img, img, -1); // flip so facing right way
					
					imshow("SR", img);
					input = waitKey(0);
					// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
					
					do{
						cameras[ROOM1_CAM].getVideoFrame(img, 1000);
						resize(img, img, Size(640/2, 480/2));
						flip(img, img, -1); // flip so facing right way
						
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
							//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
							
							
							// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
							line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
							drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
							
							int wide_rect_scalar = 4;
							int tall_rect_scalar = 10;
							// top slice
							Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
							Mat top_img = line_only(top_rect);
							//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
							
							// bottom slice
							Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
							Mat bot_img = line_only(bot_rect);
							//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
							
							
							// middle slice
							Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
							Mat mid_img = line_only(mid_rect);
							//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
							
							// left slice
							Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
							Mat left_img = line_only(left_rect);
							//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
							
							// right slice
							Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
							Mat right_img = line_only(right_rect);
							//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
							
							
							// slop from bot line to top line or bot to mid...
							int topx = 0, topy = 0, midx = 0, midy = 0, botx = 0, boty = 0;// MAKE THIS a single array with macros for indexing or somthing????
							int leftx = 0, lefty = 0, rightx = 0, righty = 0;
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
						
						
							FL = 40;
							BL = 40;
							FR = -40;
							BR = -40;
							
							
							if(abs(midx - (line_only.cols/2)) < 10) {
								break;
							}
						
							sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
							write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
						} // end line contour size > 1 if statement
						
						
						
					}while(1);
					
					
					
					
					
					
					// IF BOTTOM CONTOUR WIDTH IS REALLY BIG.... NEED TO MOVE FORWARD!!!
					
					
					
					
										
					// NOT SURE WHAT TO DO ABOUT THIS RIGHT NOW
					return;
					
				}
				else if(leftx > 0){
					// very sharp left turn
					
					// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
					FL = FR = BL = BR = 0;
					sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
					write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
					cameras[ROOM1_CAM].getVideoFrame(img, 1000);
					resize(img, img, Size(640/2, 480/2));
					flip(img, img, -1); // flip so facing right way
					
					imshow("SL", img);
					input = waitKey(0);
					// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
					
					
					// NOT SURE WHAT TO DO ABOUT THIS RIGHT NOW
					do{
						cameras[ROOM1_CAM].getVideoFrame(img, 1000);
						resize(img, img, Size(640/2, 480/2));
						flip(img, img, -1); // flip so facing right way
						
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
							//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
							
							
							// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
							line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
							drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
							
							int wide_rect_scalar = 4;
							int tall_rect_scalar = 10;
							// top slice
							Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
							Mat top_img = line_only(top_rect);
							//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
							
							// bottom slice
							Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
							Mat bot_img = line_only(bot_rect);
							//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
							
							
							// middle slice
							Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
							Mat mid_img = line_only(mid_rect);
							//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
							
							// left slice
							Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
							Mat left_img = line_only(left_rect);
							//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
							
							// right slice
							Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
							Mat right_img = line_only(right_rect);
							//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
							
							
							// slop from bot line to top line or bot to mid...
							int topx = 0, topy = 0, midx = 0, midy = 0, botx = 0, boty = 0;// MAKE THIS a single array with macros for indexing or somthing????
							int leftx = 0, lefty = 0, rightx = 0, righty = 0;
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
						
						
							FL = -40;
							BL = -40;
							FR = 40;
							BR = 40;
							
							
							if(abs(midx - (line_only.cols/2)) < 10) {
								break;
							}
						
							sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
							write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
						} // end line contour size > 1 if statement
						
						
						
					}while(1);
					
					
					
					
					// IF BOTTOM CONTOUR WIDTH IS REALLY BIG.... NEED TO MOVE FORWARD!!!
					
					
					return;
				}
				
				
				
				
				
				
				if(topx > 0 && botx > 0){
					line_angle = atanf(((float)boty-topy) / (botx-topx)) * (180.0/M_PI);
					if(line_angle > 0) line_angle -= 90;
					else if(line_angle < 0) line_angle += 90;
				}
				else if(midx > 0 && botx > 0){
					line_angle = atanf(((float)boty-midy) / (botx-midx)) * (180.0/M_PI);
					if(line_angle > 0) line_angle -= 90;
					else if(line_angle < 0) line_angle += 90;
				}

				cout << "LINE ANGLE: " << line_angle << endl;

				
				if(line_angle > 0){
					FL = 40;
					BL = 40;
					FR = -40;
					BR = -40;
				}
				else{
					FL = -40;
					BL = -40;
					FR = 40;
					BR = 40;
				}
				
				if(abs(line_angle) < 3){
					break;
				}
				
				sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
				write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
			}
			else{
				// just keep driving forward
				FL = target_speed;
				BL = target_speed;
				FR = target_speed;
				BR = target_speed;
				
				sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
				write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
			}
		}while(abs(line_angle) > 3);
		
		
		FL = FR = BL = BR = 0;
		sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
		write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
		
		// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
		FL = FR = BL = BR = 0;
		sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
		write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
		cameras[ROOM1_CAM].getVideoFrame(img, 1000);
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
		imshow("DEBUG42", img);
		input = waitKey(0);
		// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	
	
		this_thread::sleep_for(50ms);
		
		
		// JUST DRIVE FORWARD WHILE THERE IS A LINE... THEN DRIVE WHILE THERE IS NO LINE...
		target_speed = 40;
		FL = target_speed;
		BL = target_speed;
		FR = target_speed;
		BR = target_speed;
				
		sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
		write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		
		int num_contours = 1;
		do{
			cameras[ROOM1_CAM].getVideoFrame(img, 1000);
			resize(img, img, Size(640/2, 480/2));
			flip(img, img, -1); // flip so facing right way
			
			// grayscale, blur, and threshold the line
			Mat img_line, line_only;
			cvtColor(img, img_line, COLOR_BGR2GRAY);
			medianBlur(img_line, img_line, 3);
			threshold(img_line, img_line, 120, 255, THRESH_BINARY_INV);
			
			// get the contours of the whole line
			vector<vector<Point>> contours_line;
			findContours(img_line, contours_line, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			num_contours = contours_line.size();
		}while(num_contours > 0);
		
		target_speed = 40;
		FL = target_speed;
		BL = target_speed;
		FR = target_speed;
		BR = target_speed;
				
		sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
		write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
	
		do{
			cameras[ROOM1_CAM].getVideoFrame(img, 1000);
			resize(img, img, Size(640/2, 480/2));
			flip(img, img, -1); // flip so facing right way
			
			// grayscale, blur, and threshold the line
			Mat img_line, line_only;
			cvtColor(img, img_line, COLOR_BGR2GRAY);
			medianBlur(img_line, img_line, 3);
			threshold(img_line, img_line, 120, 255, THRESH_BINARY_INV);
			
			// get the contours of the whole line
			vector<vector<Point>> contours_line;
			findContours(img_line, contours_line, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			num_contours = contours_line.size();
		}while(num_contours <= 0);
		
		
		// LINE HAS BEEN FOUND !!!!!
	
	
	// need to drive forward looking for the next line... OR DETERMINE IF THIS IS ACTUALLY JUST A SHARP TURN!!!!
	
	// flag to indicate new line found or not
	/*
	int found_line = 0;
	do{
		if (!cameras[ROOM1_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM1_CAM].stopVideo();
			break;
		}
		// resize image that contains full FOV but will be small for fast compute later
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
		imshow("CROSS GAP IMG", img);
		input = waitKey(1);
		
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
			//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
			
			
			// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
			drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
			
			int wide_rect_scalar = 4;
			int tall_rect_scalar = 10;
			// top slice
			Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
			Mat top_img = line_only(top_rect);
			//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
			
			// bottom slice
			Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
			Mat bot_img = line_only(bot_rect);
			//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
			
			
			// middle slice
			Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
			Mat mid_img = line_only(mid_rect);
			//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
			
			// left slice
			Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat left_img = line_only(left_rect);
			//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
			
			// right slice
			Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat right_img = line_only(right_rect);
			//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
			
			
			// slop from bot line to top line or bot to mid...
			int topx = 0, topy = 0, midx = 0, midy = 0, botx = 0, boty = 0;// MAKE THIS a single array with macros for indexing or somthing????
			int leftx = 0, lefty = 0, rightx = 0, righty = 0;
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
			
			
			
			
			// CHECK IF IT SEEMS TO BE A GAP OR A VERY SHARP TURN?????
			vector<vector<Point>> left_contour;
			findContours(left_img, left_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(left_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(left_contour[0]);
				// get center x and center y
				leftx = m.m10 / m.m00; // col
				lefty = m.m01 / m.m00; // row
			}
			
			vector<vector<Point>> right_contour;
			findContours(right_img, right_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(right_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(right_contour[0]);
				// get center x and center y
				rightx = m.m10 / m.m00; // col
				righty = m.m01 / m.m00; // row
			}
			
			if(rightx > 0){
				// very sharp right turn
				
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
				FL = FR = BL = BR = 0;
				sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
				write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
				cameras[ROOM1_CAM].getVideoFrame(img, 1000);
				resize(img, img, Size(640/2, 480/2));
				flip(img, img, -1); // flip so facing right way
				
				imshow("SR", img);
				input = waitKey(0);
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
				
			}
			else if(leftx > 0){
				// very sharp left turn
				
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
				FL = FR = BL = BR = 0;
				sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
				write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
				cameras[ROOM1_CAM].getVideoFrame(img, 1000);
				resize(img, img, Size(640/2, 480/2));
				flip(img, img, -1); // flip so facing right way
				
				imshow("SL", img);
				input = waitKey(0);
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
			}
			
			
			
			
			
			
			if(topx > 0){
				error = topx - (line_only.cols/2);
				kp = 2.0;
			}
			else if(midx > 0){
				error = midx - (line_only.cols/2);
				kp = 1.5;
			}
			else if(botx > 0){
				error = botx - (line_only.cols/2);
				kp = base_kp;
			}
			
			// check if the line has officially been fine
			if(midx > 0){
				found_line = 1;
			}
			
			delta = (error * kp);
			FL = target_speed + delta;
			BL = target_speed + delta;
			FR = target_speed - delta;
			BR = target_speed - delta;
			
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		}
		else{
			// just keep driving forward
			FL = target_speed;
			BL = target_speed;
			FR = target_speed;
			BR = target_speed;
			
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		}
			
			
			
		
	}while(!found_line);
	
	*/
}






/*
void handle_gap(){
	int gap_centered = 0; // flag to indicate if the gap is centered.
	
	// backup FIRST, then move forward and center the line...
	FL = -5; // FL contains the distance in CM when command encoder move is used
	FR = BL = BR = 0;
	sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
	write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));	
	// wait for arduino side to send back the ALL DONE command
	if(!TESTING && !stop_motors){
		rx_length = 0;
		while (rx_length <=0) {				 								//remove the while to make this non-blocking
			rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		}
		
		rx_buffer[rx_length] = '\0';
		printf("Continue command: %s\n", rx_buffer);
	}
	
	
	// this is going to be a lot of work..... need to backup and straighten out on the gaps....
	Mat img;
	
	// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	FL = FR = BL = BR = 0;
	sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
	write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
	cameras[ROOM1_CAM].getVideoFrame(img, 1000);
	resize(img, img, Size(640/2, 480/2));
	flip(img, img, -1); // flip so facing right way
	
	imshow("DEBUG", img);
	input = waitKey(0);
	// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	
	
	
	
	int error = 0;
	int target_speed = 30;
	float kp = 0.5;
	float base_kp = kp;
	int delta;
	do{
		
		if (!cameras[ROOM1_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM1_CAM].stopVideo();
			break;
		}
		// resize image that contains full FOV but will be small for fast compute later
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
		imshow("GAP IMG", img);
		input = waitKey(1);
		
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
			//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
			
			
			// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
			drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
			
			int wide_rect_scalar = 4;
			int tall_rect_scalar = 10;
			// top slice
			Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
			Mat top_img = line_only(top_rect);
			//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
			
			// bottom slice
			Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
			Mat bot_img = line_only(bot_rect);
			//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
			
			
			// middle slice
			Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
			Mat mid_img = line_only(mid_rect);
			//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
			
			// left slice
			Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat left_img = line_only(left_rect);
			//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
			
			// right slice
			Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat right_img = line_only(right_rect);
			//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
			
			
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
			
			
			
			
			// CHECK IF IT SEEMS TO BE A GAP OR A VERY SHARP TURN?????
			
			
			
			
			
			
			
			if(topx > 0){
				error = topx - (line_only.cols/2);
				kp = 1.0;
			}
			else if(midx > 0){
				error = midx - (line_only.cols/2);
				kp = 0.8;
			}
			else if(botx > 0){
				error = botx - (line_only.cols/2);
				kp = base_kp;
			}
			
			// minimize error if its so close to the line
			if(abs(error) < 20){
				error = 0;
			}
			
			// if line is centered and on the last part of the line, its all good
			//if(topx <= 0 && midx <= 0 && botx > 0 && error == 0){
			if(error == 0){
				gap_centered = 1;
			}
			
			
			delta = (error * kp);
			FL = target_speed + delta;
			BL = target_speed + delta;
			FR = target_speed - delta;
			BR = target_speed - delta;
			
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
			
			
			
		
		}
		else{
			// LOST THE LINE, need to back up and find it again??
			FL = -5; // FL contains the distance in CM when command encoder move is used
			FR = BL = BR = 0;
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));	
			// wait for arduino side to send back the ALL DONE command
			if(!TESTING && !stop_motors){
				rx_length = 0;
				while (rx_length <=0) {				 								//remove the while to make this non-blocking
					rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
				}
				
				rx_buffer[rx_length] = '\0';
				printf("Continue command: %s\n", rx_buffer);
			}
		}
		
		
	} while(!gap_centered);
	
	
	// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	FL = FR = BL = BR = 0;
	sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
	write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
	cameras[ROOM1_CAM].getVideoFrame(img, 1000);
	resize(img, img, Size(640/2, 480/2));
	flip(img, img, -1); // flip so facing right way
	
	imshow("DEBUG", img);
	input = waitKey(0);
	// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
	
	
	// need to drive forward looking for the next line... OR DETERMINE IF THIS IS ACTUALLY JUST A SHARP TURN!!!!
	
	// flag to indicate new line found or not
	int found_line = 0;
	do{
		if (!cameras[ROOM1_CAM].getVideoFrame(img, 1000)) {
			cameras[ROOM1_CAM].stopVideo();
			break;
		}
		// resize image that contains full FOV but will be small for fast compute later
		resize(img, img, Size(640/2, 480/2));
		flip(img, img, -1); // flip so facing right way
		
		imshow("CROSS GAP IMG", img);
		input = waitKey(1);
		
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
			//drawContours(og_img, vector<vector<Point>>(1, line), -1, Scalar(0, 127, 0), 2);
			
			
			// SHOULD REDRAW LINE ON BLANK IMAGE TO ENSURE NOTHING ELSE SHOWS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			line_only = Mat::zeros(img_line.rows, img_line.cols, CV_8UC1);
			drawContours(line_only, vector<vector<Point>>(1, line), -1, 255, -1);
			
			int wide_rect_scalar = 4;
			int tall_rect_scalar = 10;
			// top slice
			Rect top_rect(0,0, line_only.cols, line_only.rows/wide_rect_scalar);
			Mat top_img = line_only(top_rect);
			//rectangle(og_img, top_rect, Scalar(127, 0, 0), 1);
			
			// bottom slice
			Rect bot_rect(0, (wide_rect_scalar-1) * (line_only.rows/wide_rect_scalar), line_only.cols, line_only.rows/wide_rect_scalar);
			Mat bot_img = line_only(bot_rect);
			//rectangle(og_img, bot_rect, Scalar(0, 0, 127), 1);
			
			
			// middle slice
			Rect mid_rect(0, line_only.rows/wide_rect_scalar, line_only.cols, (wide_rect_scalar-2) * (line_only.rows/wide_rect_scalar));
			Mat mid_img = line_only(mid_rect);
			//rectangle(og_img, mid_rect, Scalar(0, 255, 0), 1);
			
			// left slice
			Rect left_rect(0, 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat left_img = line_only(left_rect);
			//rectangle(og_img, left_rect, Scalar(127, 0, 127), 1);
			
			// right slice
			Rect right_rect((tall_rect_scalar-1) * (line_only.cols/tall_rect_scalar), 0, line_only.cols/tall_rect_scalar, line_only.rows);
			Mat right_img = line_only(right_rect);
			//rectangle(og_img, right_rect, Scalar(127, 127, 0), 1);
			
			
			// slop from bot line to top line or bot to mid...
			int topx = 0, topy = 0, midx = 0, midy = 0, botx = 0, boty = 0;// MAKE THIS a single array with macros for indexing or somthing????
			int leftx = 0, lefty = 0, rightx = 0, righty = 0;
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
			
			
			
			
			// CHECK IF IT SEEMS TO BE A GAP OR A VERY SHARP TURN?????
			vector<vector<Point>> left_contour;
			findContours(left_img, left_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(left_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(left_contour[0]);
				// get center x and center y
				leftx = m.m10 / m.m00; // col
				lefty = m.m01 / m.m00; // row
			}
			
			vector<vector<Point>> right_contour;
			findContours(right_img, right_contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			if(right_contour.size() > 0){
				// find centroid (center of mass)  OR MAYBE bounding box (geometric center)????
				m = moments(right_contour[0]);
				// get center x and center y
				rightx = m.m10 / m.m00; // col
				righty = m.m01 / m.m00; // row
			}
			
			if(rightx > 0){
				// very sharp right turn
				
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
				FL = FR = BL = BR = 0;
				sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
				write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
				cameras[ROOM1_CAM].getVideoFrame(img, 1000);
				resize(img, img, Size(640/2, 480/2));
				flip(img, img, -1); // flip so facing right way
				
				imshow("SR", img);
				input = waitKey(0);
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
				
			}
			else if(leftx > 0){
				// very sharp left turn
				
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
				FL = FR = BL = BR = 0;
				sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_ENCODER_MOVE, FL, FR, BL, BR);
				write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
				cameras[ROOM1_CAM].getVideoFrame(img, 1000);
				resize(img, img, Size(640/2, 480/2));
				flip(img, img, -1); // flip so facing right way
				
				imshow("SL", img);
				input = waitKey(0);
				// DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP DEBUG STOP
			}
			
			
			
			
			
			
			if(topx > 0){
				error = topx - (line_only.cols/2);
				kp = 2.0;
			}
			else if(midx > 0){
				error = midx - (line_only.cols/2);
				kp = 1.5;
			}
			else if(botx > 0){
				error = botx - (line_only.cols/2);
				kp = base_kp;
			}
			
			// check if the line has officially been fine
			if(midx > 0){
				found_line = 1;
			}
			
			delta = (error * kp);
			FL = target_speed + delta;
			BL = target_speed + delta;
			FR = target_speed - delta;
			BR = target_speed - delta;
			
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		}
		else{
			// just keep driving forward
			FL = target_speed;
			BL = target_speed;
			FR = target_speed;
			BR = target_speed;
			
			sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
			write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		}
			
			
			
		
	}while(!found_line);
	
	
}
*/





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
	
	

	Mat img;
	
	
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
		
		imshow("img", img);
		input = waitKey(1);

		
				
		
		// a clone for drawing/displaying later AND for green square
		Mat img_g = img.clone();
		Mat og_img = img.clone();
		
		// grayscale, blur, and threshold the line
		Mat img_line, line_only;
		cvtColor(img, img_line, COLOR_BGR2GRAY);
		medianBlur(img_line, img_line, 3);
		threshold(img_line, img_line, 120, 255, THRESH_BINARY_INV);
		
		
		// first do green square
		char green_ret = green_square(img_g, img_line);
		cout << "GREEN ALREADY STATUS -------- " << Green_Already << endl;
		cout << "GREEN RET -- " << (int)green_ret << endl;
		if(Green_Already == 0){
			switch (green_ret){
					
				case 0b10:
					// left green
					if(!stop_motors){
						sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LEFT_GREEN, 0, 0, 0, 0);
						write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
					}
					Green_Already = 1;
					
					// wait for arduino side to send back the ALL DONE command
					if(!TESTING && !stop_motors){
						rx_length = 0;
						while (rx_length <=0) 				 								//remove the while to make this non-blocking
							rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
					
						rx_buffer[rx_length] = '\0';
						printf("Continue command: %s\n", rx_buffer);
					}
					
					if(DEBUG_GREEN_TURN){
						// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG GREEN IMAGE CHECK AFTER THE TURN
						cameras[ROOM1_CAM].getVideoFrame(img, 1000);
						resize(img, img, Size(640/2, 480/2));
						flip(img, img, -1); // flip so facing right way
						imshow("AFTER L GREEN", img);
						waitKey(0);
					}
					
					continue; // ? Go back to the top and get a new image??
					//break;
					
				case 0b01:
					// right green
					if(!stop_motors){
						sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_RIGHT_GREEN, 0, 0, 0, 0);
						write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
					}
					Green_Already = 1;
					
					
					// wait for arduino side to send back the ALL DONE command
					if(!TESTING && !stop_motors){
						rx_length = 0;
						while (rx_length <=0) 				 								//remove the while to make this non-blocking
							rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
					
						rx_buffer[rx_length] = '\0';
						printf("Continue command: %s\n", rx_buffer);
					}
					
					// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG GREEN IMAGE CHECK AFTER THE TURN
					if(DEBUG_GREEN_TURN){
						cameras[ROOM1_CAM].getVideoFrame(img, 1000);
						resize(img, img, Size(640/2, 480/2));
						flip(img, img, -1); // flip so facing right way
						imshow("AFTER R GREEN", img);
						waitKey(0);
					}
					
					continue; // ? Go back to the top and get a new image??
					//break;
					
				case 0b11:
				case 0b0110:
				case 0b1001:
					// double green
					if(!stop_motors){
						sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_DOUBLE_GREEN, 0, 0, 0, 0);
						write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
					}
					Green_Already = 1;
					
					
					// wait for arduino side to send back the ALL DONE command
					if(!TESTING && !stop_motors){
						rx_length = 0;
						while (rx_length <=0) 				 								//remove the while to make this non-blocking
							rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
					
						rx_buffer[rx_length] = '\0';
						printf("Continue command: %s\n", rx_buffer);
					}
				
					// DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG GREEN IMAGE CHECK AFTER THE TURN
					if(DEBUG_GREEN_TURN){
						cameras[ROOM1_CAM].getVideoFrame(img, 1000);
						resize(img, img, Size(640/2, 480/2));
						flip(img, img, -1); // flip so facing right way
						imshow("AFTER D GREEN", img);
						waitKey(0);
					}
					
					
					continue; // ? Go back to the top and get a new image??
					//break;
					
				default:
					// no green
					cout << "NO green detected----------" << endl;
					//Green_Already = 0;
				
			}
		}
		else if(green_ret == 255){
			Green_Already = 0;
		}
		
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
			
			
			
			
			// GAP DETECTION AND HANDLING !!!!!
			if( topx <= 0 && midx <= 0){
				//stop_motors = 1; // REMOVE LATER... when done debugging this
				//sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_DEBUG_STOP, 0, 0, 0, 0);
				//write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));
				
				
				// function to handle gap or crazy line???? how to do this???
				cout << "GAP\nGAP\nGAP\nGAP\nGAP\n" << endl;
				handle_gap();
				cout << "done\ndone\ndone\ndone\ndone\n" << endl;
				
				
				continue; // ??? go back to beginning?
			}
			
			
			
			
			
			
			// set default values for slopes in case they are not there
			top_bot_slope = mid_bot_slope = 10000;
			kp = base_kp;
			
			// check if the bot contour is in the center or not
			if(botx > 0){
				bot_error = botx - (line_only.cols/2);
			}
			
			
			// chck if there was a centroid and then calc the slope
			
			if(Green_Already && topx > 0){
				kp = 0.9;
				error = topx - (line_only.cols/2);
			}
			else if(Green_Already && midx > 0){
				kp = 0.9;
				error = midx - (line_only.cols/2);
			}
			else if(!Green_Already && topx > 0 && botx > 0) {  // greater than 0 means it has a value other than the 0 that it was set to 
				// find the slope from top to bot
				top_bot_slope = topx - botx;
				//top_bot_slope = atanf(((float)boty-topy) / (botx-topx)) * (180.0/M_PI);
				//if(top_bot_slope > 0) top_bot_slope -= 90;
				//else if(top_bot_slope < 0) top_bot_slope += 90;
				
				cout<< "top_bot_slope: " << top_bot_slope << endl;
				
				// SET KP lower because there is a lot of line to be had
				kp = base_kp * 1.1;//1.3;
				error = top_bot_slope - 0; // target right now is 0
			}
			// check mid and bot
			else if(!Green_Already && midx > 0 && botx > 0) {  // greater than 0 means it has a value other than the 0 that it was set to 
				// find the slope from top to bot
				mid_bot_slope = midx - botx;
				//mid_bot_slope = atanf(((float)boty-midy) / (botx-midx)) * (180.0/M_PI);
				//if(mid_bot_slope > 0) mid_bot_slope -= 90;
				//else if(mid_bot_slope < 0) mid_bot_slope += 90;
				cout<< "mid_bot_slope: " << mid_bot_slope << endl;
				
				// SET KP slightly higher since the line is almost gone(ish)
				kp = base_kp * 1.2;//2;
				error = mid_bot_slope - 0; // target right now is 0
			}
			
			else{
				error = 0;
				cout << "NO SLOPE DETECTED" << endl;
			}
			
			
			// calculate change based on calulcated error and kp
			if(Green_Already){
				// green handled and bot is still on intersection, use higher trace section 
				delta = (error * kp);
			}
			else {
				delta = ((error+bot_error) * kp);
			}
			
			cout << "BOT ERROR: " << bot_error << endl;
			cout << "ERROR: " << error << endl;
			cout << "DELTA: " << delta << endl;
			
			// SPECIAL CASE for non-green intersections
			if(!Green_Already && topx > 0){
				if(abs(topx - (line_only.cols/2)) < 20){
					delta = 0; // go forward since the line is basically straight ahead at the furthest point
				}
			}
			
			
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
		//input = waitKey(1);
		
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
		sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", COMMAND_LINE_TRACE, FL, FR, BL, BR);
		int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
		
	}while(input != 'q');
	
	
	FL = BL = FR = BR = 0;
	sprintf(tx_buffer, "[%d][%d,%d,%d,%d]", 0, FL, FR, BL, BR);
	int count = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
	if (count < 0)
	{
		printf("UART TX error\n");
	}
	
	
	
	cameras[ROOM1_CAM].stopVideo();
	destroyAllWindows();
	
	
	
}
