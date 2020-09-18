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

int main(){
	Mat m = imread("iron/thermal/thermal" + to_string(0) + ".jpg", IMREAD_GRAYSCALE);
	// Mat m = imread("sfm/sfm" + to_string(1) + ".png", IMREAD_GRAYSCALE);

	double minVal; 
	double maxVal; 
	Point minLoc; 
	Point maxLoc;

	minMaxLoc( m, &minVal, &maxVal, &minLoc, &maxLoc, noArray() );

	cout << "min val: " << minVal << endl;
	cout << "max val: " << maxVal << endl;
	
	int rows = 600;
	int cols = 300;
	double step = double(maxVal - minVal)/rows;
	double value = minVal;
	Mat pok = Mat::zeros(rows, cols, CV_8UC3);
	// Mat pok = imread("iron/thermal/thermal" + to_string(0) + ".jpg", IMREAD_COLOR);
	resize(pok, pok, Size(cols, rows));


	for(int j = rows-1; j > -1; j--){
		for(int i = cols-1; i > -1; i--){
			// std::cout<<pok.at<Scalar>(j,i)<<endl;
			// pok.at<Scalar>(j,i) = Scalar(value, 0, 255.0-value);
			        // get a pixel
        	cv::Vec3b color = pok.at<Vec3b>(Point(i,j));

      
        	color[2] = value;
        	color[1] = 0;
        	color[0] = maxVal-value;

        	
        	// set a pixel back to the image
        	pok.at<Vec3b>(Point(i,j)) = color;
		}
		value += step;
		std::cout<<value<<" "<<maxVal-value<<endl;

	}

	imshow("M", pok);
	waitKey(0);

	return 0;
}