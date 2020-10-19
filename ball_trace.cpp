
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/lambda/lambda.hpp>
#include <X11/keysym.h>
#include <iostream>
#include <string>

const int max_value_H = 255;
const int max_value = 255;
const cv::String window_capture_name = "Video Capture";
const cv::String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

//slider bars used to tune the color thresholds
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = cv::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = cv::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = cv::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = cv::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = cv::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = cv::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char** argv){
		 
	cv::namedWindow(window_detection_name,CV_WINDOW_NORMAL);

	// Trackbars to set thresholds for HSV values
	cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
	cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
	cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
	cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
	cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
	cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
	
	//cv::VideoCapture capture(0);
	cv::VideoCapture cap(-1);
	cv::Mat robo_eyes;
	cv::Mat hsv_eyes;
	cv::Mat blur_img;
	cv::Mat mask;
	cv::Mat threshold[5];
	cv::Mat frame_threshold;
	cv::Mat slider;
	
	while(1){
		
		/* create image and blur */
		cap >> robo_eyes;
		cv::cvtColor(robo_eyes, hsv_eyes, CV_BGR2Lab);
		//cv::cvtColor(robo_eyes, hsv_eyes, CV_BGR2HSV);
		cv::blur(hsv_eyes, blur_img, cv::Size(3,3));
		cv::GaussianBlur(hsv_eyes, blur_img, cv::Size(5,5),1);
	
		/* set color threshold */
		mask = blur_img.clone();
		cv::split(mask,threshold);
		//cv::Scalar t_low(70*2.55,-35+128,60+128);
		//cv::Scalar t_high(100*2.55,0+128,108+128);
		//cv::inRange(blur_img,t_low,t_high,mask);
		cv::inRange(blur_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), mask);
	
		/* remove noise */
		cv::Mat morph_op = mask.clone(); 
		cv::Mat erosion = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
		erode(morph_op, morph_op, erosion); // causes dark zones to get bigger (min size pixels) 
		cv::Mat dilation = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(25,25)); 
		dilate(morph_op, morph_op, dilation); // causes left-over white to expand (max size pixels)

		/* define circular geometry in image */ 
		cv::Mat morphed = morph_op.clone();
		std::vector<cv::Point3f> circles;
		cv::HoughCircles(morphed, circles, CV_HOUGH_GRADIENT,5, morphed.rows/8, 200, 100, 0, 150);
	
		/* draw a point at center and a circle around the tennisball */ 
		for(int i=0; i < circles.size(); i++){
			cv::Vec3f c = circles[i];
			cv::Point center = cv::Point(c[0],c[1]);
			int radius = c[2];
			cv::circle(robo_eyes, center, 1, cv::Scalar(0,100,0),3, cv::LINE_AA);
			cv::circle(robo_eyes, center, radius, cv::Scalar(255,0,0),3, cv::LINE_AA);
			// std::cout<<circles<<std::endl;
		}
		
    		cv::imshow(window_detection_name, mask);
		
		/* creates, positions, and displays window */
		cv::namedWindow("eyes", CV_WINDOW_NORMAL);
		cv::resizeWindow("eyes",740,500);
		cv::moveWindow("eyes",600,510);
		cv::imshow("eyes", morph_op);

		cv::namedWindow("eyes2", CV_WINDOW_NORMAL);
		cv::resizeWindow("eyes2",740,500);
		cv::moveWindow("eyes2",1300,510);
		cv::imshow("eyes2", robo_eyes);

		cv::namedWindow("eyes3", CV_WINDOW_NORMAL);
		cv::resizeWindow("eyes3",740,500);
		cv::moveWindow("eyes3",600,30);
		cv::imshow("eyes3", blur_img);

		cv::namedWindow("eyes4", CV_WINDOW_NORMAL);
		cv::resizeWindow("eyes4",740,500);
		cv::moveWindow("eyes4",1300,30);
		cv::imshow("eyes4", mask);

		cv::waitKey(1);	
	}	
}





	

