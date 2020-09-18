#ifndef CameraCalibration_h
#define CameraCalibration_h


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


class CameraCalibration{
	SimpleBlobDetector::Params paramsDark;
	
	bool rgbCameraCalib;
	std::vector<KeyPoint> keypoints, keypoints_rotated; 
   	Mat descriptors;
   	Mat im_with_keypoints;

	Mat sceneIm;
	Mat templateIm;

	bool use_mask = false;
	// uncomment and make  a code pretty
	// Mat rgbTemplate = imread( "./images/rgb_template.jpg", IMREAD_GRAYSCALE ); 
	// Mat rgbTemplate = imread( "./images/thermal_template.jpg", IMREAD_GRAYSCALE ); 
	std::vector<Point2d> cornerPoints;
	// 0-right, 1-left, 2-normal 
	int orientation = 2;
	std::vector<Point3f> obj_keypoints_base; 
	
	int x_size = 640;
	int y_size = 480;


	// camera parameters
	Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
public:
	
	CameraCalibration(Mat image, Mat temp, bool rgbCamera ): sceneIm(image), templateIm(temp), rgbCameraCalib(rgbCamera){

		paramInit();

		// cornerPoints = MatchingMethod(sceneIm, templateIm, 1);
		double a = MatchingMethod(sceneIm, templateIm, 1);
			
		if( rgbCameraCalib ) {
			// to set as previous, delete function below
			// findOrientation();
			sceneIm = rgbTransform();
		}
		// std::cout <<cornerPoints << std::endl;

	}
	void paramInit();
	void findOrientation();

	double MatchingMethod( Mat img, Mat templ, int match_method );

	Mat rgbTransform();

	std::vector<Point3f> getObjectKeypointsBase(){ return obj_keypoints_base;}

	std::vector<Point2d> getCornerPoints(){ return cornerPoints; }

	void setKeypointsRotated(std::vector<KeyPoint> keypoints_new){ keypoints_rotated = keypoints_new; }
	int startDetection();

	std::vector<Point2f> getKeypoints(); 
	std::vector<Point2f> getKeypointsRotated();

	int markKeypoints();
	// u buducnosti staviti static da se ne racuna za svaki objekat instanciran
	void formObjectKeypoints();
	void saveImageWithFeatures( int i );
	std::vector< KeyPoint > sortKeypoints();
	void angleRecovery();

};
#endif

