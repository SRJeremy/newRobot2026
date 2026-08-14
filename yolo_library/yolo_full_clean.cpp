
#include <lccv.hpp>



class Yolo_Class{
private:
	const std::string onnx_path;
	const std::vector<std::string> class_list;//{"alive_victim", "az", "dead_victim", "dz"};
	const float SCORE_THRESHOLD;
	const float NMS_THRESHOLD;
	cv::Size2f image_shape;
	const bool is_cuda;
	cv::dnn::Net net;
	

public:	

	Yolo_Class(const std::string file_path, const std::vector<std::string> class_names, const float confidence_threshold = 0.5, const float nms_boxes_threshold = 0.5, const bool using_cuda = false) : 
		onnx_path(file_path),
		class_list(class_names),
		SCORE_THRESHOLD(confidence_threshold),
		NMS_THRESHOLD(nms_boxes_threshold),
		is_cuda(using_cuda)
	{
		image_shape = cv::Size2f(320,320);	
		
		net = cv::dnn::readNet(file_path);
	
		if(is_cuda){
			net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
			net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
		}
		else{
			net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
			net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		}
	
	};
	
	struct Classification_Results{
		std::string class_name;
		float confidence;
		cv::Rect bounding_box;
	
	
		Classification_Results(std::string cn,  float conf, cv::Rect r){
			class_name = cn;
			confidence = conf;
			bounding_box = r;
		}
	};
	
	std::vector<Classification_Results> results;
	
	
	void process_image(cv::Mat &img){
		// clear previous results
		results.clear();
		
		// does necessary scaling and stuff for NN usage
		cv::Mat blob;
		cv::dnn::blobFromImage(img, blob, 1.0/255, image_shape, cv::Scalar(), true, false);
		net.setInput(blob);
				
		std::vector<cv::Mat> outputs;
		net.forward(outputs, net.getUnconnectedOutLayersNames());
			
		int rows = outputs[0].size[1];
		int dimensions = outputs[0].size[2];
		// swap for yolov8 
		rows = outputs[0].size[2];
		dimensions = outputs[0].size[1];
			
		// changing around dimensions (it changed for no reason in yolov8 so we need to switch it back)
		outputs[0] = outputs[0].reshape(1, dimensions);
		cv::transpose(outputs[0], outputs[0]);
		
		float *data = (float *)outputs[0].data;
			
		std::vector<int> class_ids;
		std::vector<float> confidences;
		std::vector<cv::Rect> boxes;
		
		for(int i = 0; i < rows; i++){
			float *classes_scores = data+4;
			
			cv::Mat scores(1, class_list.size(), CV_32FC1, classes_scores);
			cv::Point class_id;
			double maxClassScore;
					
			minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);
					
			if(maxClassScore > SCORE_THRESHOLD){
				confidences.push_back(maxClassScore);
				class_ids.push_back(class_id.x);
				
				float x = data[0];
				float y = data[1];
				float w = data[2];
				float h = data[3];
				int left = x * img.cols - w * img.cols / 2;
				int top = y * img.rows - h * img.rows / 2;
				int width = int(w * img.cols);
				int height = int(h * img.rows);
				
				boxes.push_back(cv::Rect(left, top, width, height));
			}// end confidence if statement check
				
				
			data += dimensions;	
		}// end main for loop
		
		// all done, time to clean up
		std::vector<int> nms_result;
		cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
		
		for(unsigned long i = 0; i < nms_result.size(); i++){
			// add finalized nms results to the class results vector
			results.push_back(Classification_Results(class_list[class_ids[nms_result[i]]], confidences[nms_result[i]], boxes[nms_result[i]]));
		}
	}
	
	
	void draw_results(cv::Mat &img){
		for(auto& r : results){
			cv::rectangle(img, r.bounding_box, cv::Scalar(0,255,0), 1);
			cv::putText(img, std::to_string( int(r.confidence * 100)) + "% " + r.class_name, cv::Point(r.bounding_box.x, r.bounding_box.y), 1, 2, cv::Scalar(0,255,0), 1);
		}
	}
	
};






using namespace cv;
using namespace std;



Mat get_image(lccv::PiCamera &c, int width = 320, int height = 240){
	Mat img;
	if(!c.getVideoFrame(img,1000)){
		cout<<"cam Timeout error " << endl;           
    }
    
    resize(img, img, Size(width, height));
    return img;
}




int main(){
	vector<String> class_list = {"alive_victim", "az", "dead_victim", "dz"};
	String onnx_path = "/home/pi/Downloads/jeremy1.onnx";
	
	
	Yolo_Class yo(onnx_path, class_list, 0.7, 0.7);
	
	
	lccv::PiCamera cam; 
	cam.options->camera = 0;
    cam.options->video_width=1640;
    cam.options->video_height=1232;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cam.startVideo();
    this_thread::sleep_for(50ms);
    
    
	
	while(1){
		Mat img = get_image(cam);
		flip(img, img, -1);
		
		yo.process_image(img);//, res);
		
		if(yo.results.empty()){
			cout << "NOTHING FOUND IN FRAME" << endl;
		}
		else{
			// sort by name so it is easier to read output
			sort(yo.results.begin(), yo.results.end(), [](const Yolo_Class::Classification_Results& a, const Yolo_Class::Classification_Results& b) { return a.class_name < b.class_name; });
			yo.draw_results(img);//, res);
			
			
			// can use vector functions to see if alive victims are found
			
			
			cout << "\n\nItems Found:" << endl;
			for(auto& r : yo.results){
				cout << r.class_name << " found with confidence " << (int)(r.confidence * 100) << "%" << "   Located at (" << r.bounding_box.x << "," << r.bounding_box.y << ")" << endl;
			}
		}
		
		
		imshow("img", img);
		
		
		char input = waitKey(1);
		if(input == 'q'){
			break;
		}
	}
	
	destroyAllWindows();
	cam.stopVideo();
	
	
	
	
	
	return 0;
}
