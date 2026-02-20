// g++ -Wall -o "%e" "%f"    -I/usr/include/libcamera -isystem -std=gnu++17 -MD -MT  -L/usr/local/lib    -I/usr/include/opencv4/opencv -I/usr/include/opencv4 -I/usr/local/include/opencv4  -l:libopencv_gapi.so.4.8.0 -l:libopencv_stitching.so.4.8.0 -l:libopencv_aruco.so.4.8.0  -l:libopencv_bgsegm.so.4.8.0 -l:libopencv_bioinspired.so.4.8.0 -l:libopencv_ccalib.so.4.8.0 -l:libopencv_dnn_objdetect.so.4.8.0  -l:libopencv_dnn_superres.so.4.8.0 -l:libopencv_dpm.so.4.8.0 -l:libopencv_face.so.4.8.0  -l:libopencv_freetype.so.4.8.0 -l:libopencv_fuzzy.so.4.8.0 -l:libopencv_hdf.so.4.8.0 -l:libopencv_hfs.so.4.8.0  -l:libopencv_img_hash.so.4.8.0 -l:libopencv_intensity_transform.so.4.8.0 -l:libopencv_line_descriptor.so.4.8.0  -l:libopencv_mcc.so.4.8.0 -l:libopencv_quality.so.4.8.0 -l:libopencv_rapid.so.4.8.0 -l:libopencv_reg.so.4.8.0  -l:libopencv_rgbd.so.4.8.0 -l:libopencv_saliency.so.4.8.0 -l:libopencv_stereo.so.4.8.0 -l:libopencv_structured_light.so.4.8.0  -l:libopencv_superres.so.4.8.0 -l:libopencv_surface_matching.so.4.8.0 -l:libopencv_tracking.so.4.8.0 -l:libopencv_videostab.so.4.8.0  -l:libopencv_wechat_qrcode.so.4.8.0 -l:libopencv_xfeatures2d.so.4.8.0 -l:libopencv_xobjdetect.so.4.8.0 -l:libopencv_xphoto.so.4.8.0  -l:libopencv_shape.so.4.8.0 -l:libopencv_highgui.so.4.8.0 -l:libopencv_datasets.so.4.8.0 -l:libopencv_plot.so.4.8.0  -l:libopencv_text.so.4.8.0 -l:libopencv_ml.so.4.8.0 -l:libopencv_phase_unwrapping.so.4.8.0 -l:libopencv_optflow.so.4.8.0  -l:libopencv_ximgproc.so.4.8.0 -l:libopencv_video.so.4.8.0 -l:libopencv_videoio.so.4.8.0 -l:libopencv_imgcodecs.so.4.8.0  -l:libopencv_objdetect.so.4.8.0 -l:libopencv_calib3d.so.4.8.0 -l:libopencv_dnn.so.4.8.0 -l:libopencv_features2d.so.4.8.0  -l:libopencv_flann.so.4.8.0 -l:libopencv_photo.so.4.8.0 -l:libopencv_imgproc.so.4.8.0 -l:libopencv_core.so.4.8.0    -l:liblccv.so

#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

using namespace cv;
const int max_value_H = 360 / 2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void*)
{
	low_H = min(high_H - 1, low_H);
	setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void*)
{
	high_H = max(high_H, low_H + 1);
	setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void*)
{
	low_S = min(high_S - 1, low_S);
	setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void*)
{
	high_S = max(high_S, low_S + 1);
	setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void*)
{
	low_V = min(high_V - 1, low_V);
	setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void*)
{
	high_V = max(high_V, low_V + 1);
	setTrackbarPos("High V", window_detection_name, high_V);
}
int main(int argc, char* argv[])
{
	std::cout << "Sample program for LCCV video capture" << std::endl;
	std::cout << "Press ESC to stop." << std::endl;
	cv::Mat cam_0_im;
	lccv::PiCamera cam_0;
	cam_0.options->camera = 0;
	cam_0.options->video_width = 1640;//640*2;
	cam_0.options->video_height = 1232;//480*2;
	cam_0.options->framerate = 30;
	cam_0.options->verbose = true;
	cv::namedWindow(window_detection_name, cv::WINDOW_NORMAL);
	cv::namedWindow(window_capture_name, cv::WINDOW_NORMAL);
	cam_0.startVideo();
	std::cout << "Camera 0 Started" << std::endl;

	namedWindow(window_capture_name);
	namedWindow(window_detection_name);
	// Trackbars to set thresholds for HSV values
	createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
	createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
	createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
	createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
	createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
	createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
	Mat frame, frame_HSV, frame_threshold;
	while (true) {
		if (!cam_0.getVideoFrame(cam_0_im, 99999999)) {
			std::cout << "Timeout error " << std::endl;
		}
		else {
			resize(cam_0_im, cam_0_im, Size(1640/4, 1232/4));
			cv::imshow(window_capture_name, cam_0_im);
			// Convert from BGR to HSV colorspace
			cvtColor(cam_0_im, frame_HSV, COLOR_BGR2HSV);
			// Detect the object based on HSV Range Values
			inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
			imshow(window_detection_name, frame_threshold);
			char key = (char)cv::waitKey(1);
			if (key == 'q' || key == 27)
			{
				break;
			}
		}
	}

	cam_0.stopVideo();
	std::cout << "\n\n\ninRange(frame_HSV, Scalar(";
	std::cout << low_H << "," << low_S << "," << low_V;

	std::cout << "), Scalar(";
	std::cout << high_H << "," << high_S << "," << high_V;

	std::cout << "), frame_threshold);\n\n\n" << std::endl;

	destroyAllWindows();
	return 0;
}
