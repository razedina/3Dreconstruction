#include "CameraCalibration.h"
#include "Stereovision.h"

// example of rgb camera calibration
void mainRgb(){

	int count_features_24 = 0;

	Mat templateIm = imread( "./images/rgb_template.jpg", IMREAD_GRAYSCALE ); 
	Mat image;


	std::vector<std::vector<Point2f> > rgb_keypoints;
	std::vector<std::vector<Point3f> > obj_keypoints;


	for ( int i = 20; i < 5000; i = i + 20 ){
		// image = imread( "./images/rgb/rgb" + to_string(i) + ".jpg", IMREAD_GRAYSCALE );
		image = imread( "./images/rgb/rgb" + to_string(i) + ".jpg", IMREAD_GRAYSCALE );

		if( !image.empty() ){

			CameraCalibration obj = CameraCalibration( image, templateIm, true );

			if( obj.startDetection() == 24 ){
				count_features_24 = count_features_24 + 1;
				//obj.saveImageWithFeatures(i);
				// obj.markKeypoints();
				rgb_keypoints.push_back( obj.getKeypoints() );
				obj_keypoints.push_back( obj.getObjectKeypointsBase() );
			} 
		}
	}
	std::cout << "features equals 24: " << count_features_24 << std::endl;

}
// applying perspective view
void homographyMapping(Mat rgbIm, Mat thermalIm){
	Mat H = (Mat_<double>(3,3) << 2.395875967764361, 0.2902429975583964, -568.2112999373937,
 			0.1504416870250624, 3.183444559204016, -763.3067174042286,
 			-0.0002973742552781666, 0.001556840392379392, 1);
	
	std::cout << H <<std::endl;
	
	Mat R = (Mat_<double>(3,3) << 0.1897081707241889, -0.1433946859958071, -0.9713129124996981,
 								0.1095288830131587, 0.9861936684239152, -0.1241993242593158,
 								0, 0, 1);


	Mat t = (Mat_<double>(3,4) << 1, 0, 0, 0.1425110668074954,
								  0, 1, 0, -0.01449030276750513,
								  0, 0, 1,  0.9896871358985604);

	Mat t_new = (Mat_<double>(2,3) << 1, 0, 0.1425110668074954,
								  0, 1, -0.01449030276750513);
	

	Mat R_t = (Mat_<double>(3,4) << 0.1897081707241889, -0.1433946859958071, -0.9713129124996981, 0.1425110668074954,
 								0.1095288830131587, 0.9861936684239152, -0.1241993242593158, -0.01449030276750513,
 								0, 0, 1, 0.9896871358985604);

	Mat R_t_new = (Mat_<double>(2,3) << 0.1897081707241889, -0.1433946859958071, 0.1425110668074954,
 								0.1095288830131587, 0.9861936684239152, -0.01449030276750513,
 								0, 1, 0.9896871358985604);

	Mat image_out;
	warpPerspective(rgbIm, image_out, R, rgbIm.size());
	warpAffine(image_out, image_out, t_new, rgbIm.size());
	// warpAffine(rgbIm, image_out, R_t_new, rgbIm.size());
	cv::resize(image_out, image_out, thermalIm.size());
	imshow ("Win", image_out);
	imshow ("Win2", rgbIm);
	imshow ("Win3", thermalIm);
	waitKey(0);

}
// transforming set of data
void imagesTransformHomography(){
	Mat H = (Mat_<double>(3,3) << 2.395875967764361, 0.2902429975583964, -568.2112999373937,
 								0.1504416870250624, 3.183444559204016, -763.3067174042286,
 								-0.0002973742552781666, 0.001556840392379392, 1);
	
	for(int i = 0; i < 3000; i++){
		// std::cout << "images_all/rgb/rgb" + to_string(i) + ".jpg";
		Mat image_rgb = imread("iron_rgb/rgb" + to_string(i) + ".jpg");
		// imshow ("Win", image_rgb);
		// waitKey(0);
		Mat image_out;
		if (!image_rgb.empty()){
			warpPerspective(image_rgb, image_out, H, image_rgb.size());
			cv::resize(image_out, image_out, Size(160, 120));

			imwrite("iron_perspective/rgb" + to_string(i) + ".jpg", image_out);

		}

	}

}
void getBorderValue(Mat &image, int &value){
	std::cout<<image<<endl;
}
// transforming set of data
void imagesTransformHomographyThermal(){
	Mat H = (Mat_<double>(3,3) << 0.4936007813637016, 0.02790331194862168, 212.1679694137484,
 								 -0.04242976559838325, 0.4917722709959861, 237.9055140185938,
 								 -2.18703445156973e-05, -2.370225204856451e-05, 1);

	
	for(int i = 0; i < 3000; i++){
		// std::cout << "images_all/rgb/rgb" + to_string(i) + ".jpg";
		Mat image_thermal = imread("iron/thermal/thermal" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
		// imshow ("Win", image_rgb);
		// waitKey(0);
		Mat image_out;
		if (!image_thermal.empty()){
			cv::resize(image_thermal, image_thermal, cv::Size(640, 480));
			int value = 0;
			// getBorderValue(image_thermal, value);
			warpPerspective(image_thermal, image_out, H, image_thermal.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(94));
			imwrite("iron/thermal_perspective/thermal" + to_string(i) + ".jpg", image_out);

		}

	}

}

// finding an angle of board rotation, same function as in CameraCalibration class
float rotationByAngle(std::vector<Point2f> kpoints, Mat sceneIm, Point2f &origin, bool left){
	float pi = 3.14159265;
	std::vector<Point2f> kpoints_new;
	int numPnts = kpoints.size();
	std::vector<int> x, y;
	// for(int i = 0; i < kpoints.size(); i++){
	// 	kpoints_new.append(Point2f(kpoints.x,kpoints.y))
	// }

	for(int i = 0; i < numPnts; i++){
		x.push_back( kpoints[i].x );
		y.push_back( kpoints[i].y );
	}

	auto min_a = std::min_element( x.begin(), x.end() );
	auto mini_a = std::distance( x.begin(), min_a);

	
	// origin = Point2f(0, sceneIm.rows);
	*min_a = 10000;
	auto min_b = std::min_element( x.begin(), x.end() );
	auto mini_b = std::distance( x.begin(), min_b);

	*min_b = 10000;
	auto min_c = std::min_element( x.begin(), x.end() );
	auto mini_c = std::distance( x.begin(), min_c);

	// get correct point for angle recovery
	// || (left && (kpoints[mini_b].y < kpoints[mini_c].y))
	if ((kpoints[mini_b].y > kpoints[mini_c].y && !left) ){
		mini_b = mini_c;
	}


	// std::cout << kpoints[mini_b].x <<" "<< kpoints[mini_a].x << " " << y[mini_a] - y[mini_b] << std::endl;
	// first case
	float angle;

	float D = cv::norm(kpoints[mini_a] - kpoints[mini_b]);
	
	if (left){
		angle = atan2((kpoints[mini_b].x - kpoints[mini_a].x),(y[mini_b] - y[mini_a]) )*180/PI;
		origin = Point2f(kpoints[mini_a].x + 4 * D * sin(angle*pi/180), kpoints[mini_a].y + 4 * D * cos(angle*pi/180));
		origin = Point2f(origin.x + 4.8 * D * cos(angle*pi/180), origin.y - 4.8 * D * sin(angle*pi/180));
		// vratiti na 4.8, bilo 3
		// std::cout << origin.x << " " << origin.y << endl;
		if(origin.x>640){
			origin.x = 640;
		}else if(origin.y > 480){
			origin.y = 480;
		}
	}else{
		angle = atan2(( kpoints[mini_b].x  - kpoints[mini_a].x),( y[mini_a] - y[mini_b]) )*180/PI;
		origin = Point2f(kpoints[mini_a].x - 25, kpoints[mini_a].y + 40);
	}


	std::vector<KeyPoint> v,w,z;
	v.push_back(KeyPoint(kpoints[mini_a],1.f));
	// v.push_back(KeyPoint(kpoints[mini_b],1.f));
	z.push_back(KeyPoint(kpoints[mini_b],1.f));
	w.push_back(KeyPoint(origin,1.f));

	drawKeypoints( sceneIm, v, sceneIm, Scalar(0,0,255), DrawMatchesFlags::DEFAULT );
	drawKeypoints( sceneIm, z, sceneIm, Scalar(0,255,0), DrawMatchesFlags::DEFAULT );
	drawKeypoints( sceneIm, w, sceneIm, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );

	imshow("v", sceneIm);
	waitKey(0);


	return angle;
}


//  same function as in CameraCalibration class
void angleRecovery(Mat img){
	// Mat img = imread("./images_all/rgb/rgb2676.jpg", IMREAD_GRAYSCALE);
	// Mat templateIm = imread("./images_all/rgb_template_angle.jpg", IMREAD_GRAYSCALE);
	// Mat img = imread("./images_all/rgb/rgb2937.jpg", IMREAD_GRAYSCALE);
	// Mat img = imread("./images_all/rgb/rgb2961.jpg", IMREAD_GRAYSCALE);
	Mat templateImL = imread("./images_with_rotation/rgb_template_angle_left.png", IMREAD_GRAYSCALE);
	Mat templateImR = imread("./images_with_rotation/rgb_template_right.png", IMREAD_GRAYSCALE);
	// imshow("win", templateIm);
	// waitKey(0);

	CameraCalibration obj = CameraCalibration( img, templateImL, true );
	bool left=true;
	// if(obj.MatchingMethod(img,templateImL,0)<obj.MatchingMethod(img,templateImR,0)){
	// 	obj = CameraCalibration( img, templateImL, true );
	// 	std::cout << "Left" << endl;
	// 	left = true;
	// }else{
	// 	obj = CameraCalibration( img, templateImR, true );
	// 	std::cout << "Right" << endl;
	// 	left = false;
	// }

	obj.startDetection();

	std::vector<Point2f> kpoints = obj.getKeypoints();
	// int key = obj.markKeypoints(); 
	Point2f origin;
	float angle = rotationByAngle(kpoints, img, origin, left);
	std::cout << "Ugao " << angle << endl;

	if (angle > 40) return;

	std::vector<Point2f> rotated;

	// angle = -18.85;

	for (int i = 0; i < kpoints.size(); i++){
		float d = cv::norm(kpoints[i] - origin);
		float D = 2 * d * sin(angle/180 / 2 * 3.14);
		// rotated.push_back(Point2f(kpoints[i].x + D*cos(angle/2), kpoints[i].y + D*sin(angle/2) ));
		// rotated.push_back(Point2f(kpoints[i].x + D*sin(angle/2), kpoints[i].y + D*cos(angle/2) )); // radi za right
		rotated.push_back(Point2f(kpoints[i].x - D*sin(angle/2*3.14/180), kpoints[i].y - D*cos(angle/2*3.14/180) )); //radi za left
		// std::cout << D << std::endl;

	}

	std::vector<KeyPoint> v, w;
	for (int i = 0; i < kpoints.size(); i++){
		v.push_back(KeyPoint(rotated[i],1.f));
		w.push_back(KeyPoint(kpoints[i],1.f));
	}




	drawKeypoints( img, v, img, Scalar(0,0,255), DrawMatchesFlags::DEFAULT );
	drawKeypoints( img, w, img, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

	imshow("w", img);
	waitKey(0);

	obj.setKeypointsRotated(v);
	int key = obj.markKeypoints(); 

}

// an attempt to perform mathematical mapping between images 
// with relationship matrices
void transform(){
	Mat F = (Mat_<double>(3,3) << -1.594903223437326e-06, -2.510991848803512e-05, 0.00681944831050292,
 								 2.484533520186375e-05, -1.556427127520775e-06, -0.003280194890764738,
 								 -0.001289862230226359, -0.003084143895097567, 1);

	Mat rgbImS = imread("./images/rgb/rgb2220.jpg", IMREAD_GRAYSCALE);
	Mat rgbImTem = imread("./images/rgb_template.jpg", IMREAD_GRAYSCALE);
	Mat thermalImS = imread("./images/thermal/thermal2220.jpg", IMREAD_GRAYSCALE);
	Mat thermalImTem = imread("./images/thermal_template.jpg", IMREAD_GRAYSCALE);

	Mat K1 = (Mat_<double>(3,3) << 16400.97886798693, 0, 319.9437944177691, 0, 13489.79476835391, 247.0318823419875, 0, 0, 1);
	Mat K2 = (Mat_<double>(3,3) << 3239.882928345035, 0, 79.92079576300202,  0, 1724.549842148334, 59.14363600039724, 0, 0, 1);
	Mat distCoeffsT = (Mat_<double>(5,1) << -15.25680038932625, 6436.890502310375, -0.1680064249769486, -0.01820159851484165, 9.930480745053901);
	Mat distCoeffsR = (Mat_<double>(5,1) << 101.5447649891926, -3.723673906867621, -0.5460573349598647, -0.3277112411754122, -0.00234260442385406);

	Mat rgbIm, thermalIm;
	undistort(rgbImS, rgbIm, K1, distCoeffsR);
	undistort(thermalImS, thermalIm, K2, distCoeffsT);
	cv::resize(thermalIm, thermalIm, cv::Size(640, 480));

	int x = 50;
	int y = 50;
	int count = 0;
	int total = 0;
	for(int i=0; i < 640; i++){
		for(int j=0; j<480; j++){
			int a = -1.594903223437326e-06*i + j*2.484533520186375e-05 -0.001289862230226359 ;
			int b = -2.510991848803512e-05*i - j*1.556427127520775e-06  -0.003084143895097567;
			int c = 0.00681944831050292*i -0.003280194890764738*j -1;

			if(a*x + b*y + c < 0.01e-20){
				count += 1;
				
			}
			total += 1;
				
		}
	}
	std::cout << "Line " << count << " "<< 640*480 << " "<< total << endl;
}

int main(int argc, char *argv[]){

	// startCalibration("./images_all/rgb/rgb", "./images/rgb_template.jpg", true);
	// startCalibration("./images/thermal/thermal", "./images/thermal_template.jpg", false);
	// findExtrinsics();
	// homographyMapping(imread("./images/rgb/rgb2220.jpg", IMREAD_GRAYSCALE), imread("./images/thermal/thermal2220.jpg", IMREAD_GRAYSCALE));
	for (int i=0; i<5000;i++){
		Mat image = imread("./images_with_rotation/rgb_left/rgb" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
		if(!image.empty()){
			angleRecovery(image);
		}
	}
	// imagesTransformHomography();
	// findExtrinsics();
	// transform();
	// imagesTransformHomographyThermal();
	
	return 0;
}