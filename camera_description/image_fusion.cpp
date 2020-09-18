#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <math.h>  
//#include <opencv2/sfm/fundamental.hpp>
#include <iostream>
#include <string>
#include "template_matching.h"
//	#include <opencv2/sfm/fundamental.hpp>

//#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d/nonfree.hpp"

#define PI 3.14159265;

using namespace cv;
using namespace std;
 
int in_progress(){
	Mat r = imread("iron/rgb/rgb0.jpg");
	Mat loc = imread("iron/rgb/rgb0.jpg", IMREAD_GRAYSCALE);
	Mat t = imread("iron/thermal_perspective/thermal0.jpg");
	Mat t_color = imread("iron/thermal_perspective/thermal0.jpg", IMREAD_COLOR); 



	double minVal; 
	double maxVal; 
	Point minLoc; 
	Point maxLoc;

	minMaxLoc( loc, &minVal, &maxVal, &minLoc, &maxLoc);

	cout << "min val: " << minVal << endl;
	cout << "max val: " << maxVal << endl;

	Mat r1, t1;

	Mat M = (Mat_<double>(2,3) << 1, 0, 41, 0, 1, -17);
	warpAffine(t, t, M, t.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(100));
	warpAffine(t_color, t_color, M, t.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(94));

	for(int i=0; i<t.rows; i++){
		for(int j=0; j<t.cols; j++){
			cv::Vec3b color = t_color.at<Vec3b>(Point(j,i));
			cv::Vec3b color_t = t.at<Vec3b>(Point(j,i));
			// std::cout << t.at<Vec3b>(i,j) <<endl;

        	// if(color_t[0]>150){
        	// 	color[2] = color_t[0];
        	// }else{
        	// 	color[2] = 0;
        	// }

        	color[2] = color_t[0];
        	color[1] = 0;
        	color[0] = maxVal-color_t[0];
        	t_color.at<Vec3b>(Point(j,i)) = color;
		}
	}
	// imshow("Win2", t_color);

	// imshow("Win1", r);
	// imshow("Win2", t);
	// waitKey(0);

	cvtColor(r, r1, cv::COLOR_RGB2HLS);
	cvtColor(t_color, t1, cv::COLOR_RGB2HLS);


	Mat fusione = t1.clone();
	Mat hue = t1.clone(), saturation = t1.clone(), intensity = t1.clone();

	for(int i = 0; i <r.rows; i++ ){
		for(int j = 0; j < r.cols; j++){
			cv::Vec3b color = fusione.at<Vec3b>(Point(j,i));
			cv::Vec3b intensity = r1.at<Vec3b>(Point(j,i));

        	color[1] = intensity[1];
        	// color[2] = 255;
        	fusione.at<Vec3b>(Point(j,i)) = color;

		}
	}


	for(int i = 0; i <r.rows; i++ ){
		for(int j = 0; j < r.cols; j++){
			cv::Vec3b color = fusione.at<Vec3b>(Point(j,i));
			cv::Vec3b changed_color = hue.at<Vec3b>(Point(j,i));
			

        	changed_color[0] = color[0];
        	changed_color[1] = 0;
        	changed_color[2] = 0;
 
        	hue.at<Vec3b>(Point(j,i)) = changed_color;


        	changed_color[0] = 0;
        	changed_color[1] = color[1];
        	changed_color[2] = 0;
 
        	intensity.at<Vec3b>(Point(j,i)) = changed_color;

        	changed_color[0] = 0;
        	changed_color[1] = 0;
        	changed_color[2] = color[2];
 
        	saturation.at<Vec3b>(Point(j,i)) = changed_color;


		}
	}


	cvtColor(fusione, fusione, cv::COLOR_HLS2RGB);
	// cvtColor(t1, t1, cv::COLOR_HLS2RGB);

	// imshow("Win1", fusione);
	// imshow("intensity", intensity);
	// imshow("saturation", saturation);
	// imshow("hue", hue);
	// imshow("Win2", t1);
	// waitKey(0);
	imwrite("iron/fusion/AAsaturation.jpg", saturation);
	imwrite("iron/fusion/AAintensity.jpg", intensity);
	imwrite("iron/fusion/AAhue.jpg", hue);

	return 0;
}

int thermal_translation(){
	for(int i = 0; i<3000; i++){
		Mat thermal = imread("iron/thermal_perspective/thermal" + to_string(i) + ".jpg");

		if(!thermal.empty()){
			Mat M = (Mat_<double>(2,3) << 1, 0, 41, 0, 1, -17);
			warpAffine(thermal, thermal, M, thermal.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(94, 94, 94));

			imwrite("iron/thermal_translation/thermal" + to_string(i) + ".jpg", thermal);
		}
	}

}

int test2(){
	for(int i = 0; i<3000; i++){
		Mat img = imread("iron/rgb/rgb" + to_string(i) + ".jpg");
		Mat thermal = imread("iron/thermal_perspective/thermal" + to_string(i) + ".jpg");


		if(!img.empty()){
			Mat M = (Mat_<double>(2,3) << 1, 0, 41, 0, 1, -17);
			warpAffine(thermal, thermal, M, thermal.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(94, 94, 94));
			for(int i=0; i<thermal.rows; i++){
				for(int j=0; j<thermal.cols; j++){
					cv::Vec3b color = thermal.at<Vec3b>(Point(j,i));

        			color[1] = 0;
        			color[0] = 255-color[2];
        			thermal.at<Vec3b>(Point(j,i)) = color;
				}
			}
			// imshow("Win4", thermal);
			// waitKey(0);
			cvtColor(img, img, cv::COLOR_RGB2HLS);
			cvtColor(thermal, thermal, cv::COLOR_RGB2HLS);

			Mat fusione = thermal.clone();


			for(int i = 0; i <img.rows; i++ ){
				for(int j = 0; j < img.cols; j++){
					cv::Vec3b color = thermal.at<Vec3b>(Point(j,i));
					cv::Vec3b intensity = img.at<Vec3b>(Point(j,i));

        			color[1] = intensity[1];
		        	fusione.at<Vec3b>(Point(j,i)) = color;

				}
			}

			cvtColor(fusione, fusione, cv::COLOR_HLS2RGB);
			imwrite("iron/fusion/img" + to_string(i) + ".jpg", fusione);

		}
	}
	return 0;
}

int main(){
	thermal_translation();
	// in_progress();
	// test2();
}