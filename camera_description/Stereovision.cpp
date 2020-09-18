#include "Stereovision.h"


using namespace cv;
using namespace std;



void Stereovision::findExtrinsics(){

	std::vector<Point2f>  points1, points2;

	Mat rgbImS = imread("./images/rgb/rgb2220.jpg", IMREAD_GRAYSCALE);
	Mat rgbImTem = imread("./images/rgb_template.jpg", IMREAD_GRAYSCALE);
	Mat thermalImS = imread("./images/thermal/thermal2220.jpg", IMREAD_GRAYSCALE);
	Mat thermalImTem = imread("./images/thermal_template.jpg", IMREAD_GRAYSCALE);


	// matrices obtained a result of calibration provess
	// in CameraCalubration class
	K1 = (Mat_<double>(3,3) << 16400.97886798693, 0, 319.9437944177691, 0, 13489.79476835391, 247.0318823419875, 0, 0, 1);
	K2 = (Mat_<double>(3,3) << 3239.882928345035, 0, 79.92079576300202,  0, 1724.549842148334, 59.14363600039724, 0, 0, 1);
	
	distCoeffsT = (Mat_<double>(5,1) << -15.25680038932625, 6436.890502310375, -0.1680064249769486, -0.01820159851484165, 9.930480745053901);
	distCoeffsR = (Mat_<double>(5,1) << 101.5447649891926, -3.723673906867621, -0.5460573349598647, -0.3277112411754122, -0.00234260442385406);

	Mat rgbIm, thermalIm;
	undistort(rgbImS, rgbIm, K1, distCoeffsR);
	undistort(thermalImS, thermalIm, K2, distCoeffsT);

	// imshow ("Win", rgbIm);
	// imshow ("WinWin", thermalIm);
	// waitKey(0);

	cv::resize(thermalIm, thermalIm, cv::Size(640, 480));
	std::vector<KeyPoint> keypoints_1, keypoints_2; 
	CameraCalibration objR = CameraCalibration(rgbIm, rgbImTem, true);
	CameraCalibration objT = CameraCalibration(thermalIm, thermalImTem, false);

	objR.startDetection();
	objT.startDetection();
	// std::cout << "tthermal size" << thermalIm.size();

	objR.markKeypoints();
	objT.markKeypoints();
	
	points1 = objR.getKeypoints();
	points2 = objT.getKeypoints();

	F = findFundamentalMat(points1,points2, FM_8POINT);
	H = findHomography(points2, points1);

	// std::cout << F << endl;

	Mat image_out;
	warpPerspective(thermalIm, image_out, H, rgbIm.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
	// cv::resize(image_out, image_out, thermalIm.size());
	// imshow ("Win", image_out);
	// imshow ("Win2", rgbIm);
	// imshow ("Win3", thermalIm);
	// waitKey(0);

	Mat R1, R2, t1;


	decomposeEssentialMat(E, R1, R2, t1);

	std::cout << "R1 = "<< R1 << endl;
	std::cout << "R2 = "<< R2 << endl;
	std::cout << "t = "<< t1 << endl;

	int inliners = recoverPose(E, points1, points2, K2, R, t);
	std::cout << "recover pose method"<<endl;
	std::cout << "R = "<< R << endl;
	std::cout << "t = "<< t << endl;
	std::cout << inliners << std::endl;

	homographyMapping(rgbIm, thermalIm);
	
	stereoRectify(rgbImS, distCoeffsR, thermalImS, distCoeffsT, rgbImS.size(), R,  t, R1,  R2, P1, P2, Q);

	// imagesTransformHomography();
}



