#include "CameraCalibration.h"

using namespace cv;
using namespace std;


void CameraCalibration::paramInit(){
		SimpleBlobDetector::Params params;
		
		params.filterByColor = true;
	  	params.blobColor = 255;


	  	// Change thresholds
	  	params.minThreshold = 30;
	  	params.maxThreshold = 250;

	  	// Filter by Area.
	  	params.filterByArea = false;
	  	//params.minArea = 1500;

	  	// Filter by Circularity
	  	params.filterByCircularity = true;
	  	params.minCircularity = 0.02;

	  	// Filter by Convexity
	  	params.filterByConvexity = true;
	  	params.minConvexity = 0.4;

	  	// Filter by Inertia
	  	params.filterByInertia = true;
	  	params.minInertiaRatio = 0.01;
		/*
	  	paramsLight = params;
	  	paramsLight.blobColor = 0;
		*/
	  	paramsDark = params;
	  	paramsDark.blobColor = 255;
	}

void CameraCalibration::findOrientation(){
		Mat temR = imread( "./images_all/rgb_template_angle.jpg", IMREAD_GRAYSCALE);
		Mat temL = imread( "./images_all/rgb_template_angle_left.png", IMREAD_GRAYSCALE);
		Mat temN = imread( "./images/rgb_template.jpg", IMREAD_GRAYSCALE );

		std::vector<double> minValues;
		std::vector<std::vector<Point2d>> cornerPointsAll;
		minValues.push_back(MatchingMethod(sceneIm, temR, 0));
		cornerPointsAll.push_back(cornerPoints);
		
		minValues.push_back(MatchingMethod(sceneIm, temL, 0));
		cornerPointsAll.push_back(cornerPoints);
		
		minValues.push_back(MatchingMethod(sceneIm, temN, 0));
		cornerPointsAll.push_back(cornerPoints);
		// std::cout <<minValues[0]<<" "<<minValues[1]<<" " <<minValues[2]<<" "<<endl;
		// find correct orientation
		auto min_a = std::min_element( minValues.begin(), minValues.end() );
		orientation = std::distance( minValues.begin(), min_a);
		cornerPoints = cornerPointsAll[orientation];
		std::cout<<orientation<<endl;
	}

double CameraCalibration::MatchingMethod( Mat img, Mat templ, int match_method ){
  		Mat mask; Mat result;

  		int result_cols =  img.cols - templ.cols + 1;
  		int result_rows = img.rows - templ.rows + 1;
  		// std::cout << img.cols << " "<< img.rows << " "<< templ.cols << " " << templ.rows << std::endl;
  		result.create( result_rows, result_cols, CV_32FC1 );
  		
  		bool method_accepts_mask = (TM_SQDIFF == match_method || match_method == TM_CCORR_NORMED);



  		if (use_mask && method_accepts_mask)
    		{ matchTemplate( img, templ, result, match_method, mask); }
  		else
    		{ matchTemplate( img, templ, result, match_method); }


  		normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  		double minVal; double maxVal; Point minLoc; Point maxLoc;
  		Point matchLoc;
  		minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  		if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    		{ matchLoc = minLoc; }
  		else
    	{ matchLoc = maxLoc; }

  		// std::cout << "Left up corner " << matchLoc <<std::endl;
  		// std::vector<Point2d> cornerPoints;
  		cornerPoints.push_back( matchLoc );
  		cornerPoints.push_back( Point( matchLoc.x + templ.cols, matchLoc.y ) );
  		cornerPoints.push_back( Point( matchLoc.x, matchLoc.y + templ.rows) );
  		cornerPoints.push_back( Point ( matchLoc.x + templ.cols , matchLoc.y + templ.rows ) );
  		return minVal;
		// std::cout <<cornerPoints << std::endl;
  		// return cornerPoints;
	}

Mat CameraCalibration::rgbTransform(){
		int x = 640;
		int y = 480;
		Mat baseIm;
		sceneIm.copyTo( baseIm );
		//image[0:cornerPoints[0].x, 0:y] = [0, 0, 0];
		int xStart = min(cornerPoints[0].x, cornerPoints[2].x);
		int yStart = min(cornerPoints[0].y, cornerPoints[1].y);
		int xEnd = max(cornerPoints[1].x, cornerPoints[3].x);
		int yEnd = max(cornerPoints[2].y, cornerPoints[3].y);

		// std::cout << xStart << "   "<< xEnd << "   " << yStart << "   "<< yEnd << "   " << std::endl;
		// mask formation
		Mat mask( y, x, CV_64FC1 );
		for ( int i = xStart; i < xEnd; i++ ){
			for ( int j = yStart; j < yEnd; j++){
				mask.at<double>(j,i) = (255,255,255);
			}
		}


		// image formation based on mask free areas
		for (int i = 0; i < x; i++){
			for(int j = 0; j < y; j++){
				if(mask.at<double>(j,i) != 255){
					baseIm.at<uchar>(j,i) = (0);				
				}
			}

		}

		// part from image transform setup
		// imshow("Novi", baseIm);
		// waitKey(0);	
		threshold(baseIm, baseIm, 220, 255, THRESH_BINARY);
		// imshow("Window 1", image);
		// imshow("Novi", baseIm);
		// waitKey(0);	
		return baseIm;
	}

int CameraCalibration::startDetection(){
		Ptr< SimpleBlobDetector > detector = SimpleBlobDetector::create(paramsDark);
		detector->detect( sceneIm, keypoints );
		// Default value for rotated scene
		keypoints_rotated = keypoints;
    	return keypoints.size();
	}

std::vector<Point2f> CameraCalibration::getKeypoints(){
		std::vector<Point2f> points;
		for( int i = 0; i < keypoints.size(); i++ ){
			points.push_back( Point2f( keypoints[i].pt ) );
		}
		return points;
	}



std::vector<Point2f> CameraCalibration::getKeypointsRotated(){
		std::vector<Point2f> points;
		for( int i = 0; i < keypoints_rotated.size(); i++ ){
			points.push_back( Point2f( keypoints_rotated[i].pt ) );
		}
		return points;
	}


int CameraCalibration::markKeypoints(){
		// if( !rgbCameraCalib )
		// std::cout << orientation << endl;
		// orientation is fixed if findAngle function is not called
		if ( orientation != 2 ) this->angleRecovery(); 
		keypoints = sortKeypoints();

		drawKeypoints( sceneIm, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		

		for ( int i = 0; i < keypoints.size(); i++ ){
			putText( im_with_keypoints, to_string( i + 1 ), keypoints[i].pt, cv::FONT_HERSHEY_DUPLEX, 0.6, Scalar( 0, 0, 255 ), 1 );
		}
		
		imshow("Labeled keypoints", im_with_keypoints);
		// SET ZETO TO GET CONTROL
		auto key = waitKey(0); 
		return key;
		
	}	
void CameraCalibration::formObjectKeypoints(){
		for ( int i = 7; i > -1; i-- ){
			if( i % 2 ){
				obj_keypoints_base.push_back( Point3f( 1.5, i, 0 ) );
				obj_keypoints_base.push_back( Point3f( 2.5, i, 0 ) );
				obj_keypoints_base.push_back( Point3f( 3.5, i, 0 ) );
			}else{
				obj_keypoints_base.push_back( Point3f( 1, i, 0 ) );
				obj_keypoints_base.push_back( Point3f( 2, i, 0 ) );
				obj_keypoints_base.push_back( Point3f( 3, i, 0 ) );
			}

		}
	}
void CameraCalibration::saveImageWithFeatures( int i ){
		drawKeypoints( sceneIm, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		// std::cout << "Kejpointovi -----------------"<<keypoints_1.size() << std::endl;
		//imshow("Window 1", im_with_keypoints);
		//waitKey(0);
		if( rgbCameraCalib ) imwrite( "./analysis/rgb/rgb" + to_string(i) + ".jpg", im_with_keypoints );
		else imwrite( "./analysis/thermal/thermal" + to_string(i) + ".jpg", im_with_keypoints );
	}


std::vector< KeyPoint > CameraCalibration::sortKeypoints(){
		int numPnts = 24;
		int numPntsRow = 3;
		std::vector< int > x, y;
		std::vector< int > x_pom, indices;
		std::vector< KeyPoint > keypoints_base;

		std::cout<<"Sorting Keypoints" << endl;
		for(int i = 0; i < numPnts; i++){
			x.push_back( keypoints_rotated[i].pt.x );
			y.push_back( keypoints_rotated[i].pt.y );
		}

		for( int i = 0; i < 8; i++ ){
			x_pom.clear();
			indices.clear();
			for( int j = 0; j < numPntsRow; j++ ){
				auto max = std::max_element( y.begin(), y.end() );
				auto maxi = std::distance( y.begin(), max);
				// provjeriti da li moze samo max - y.begin()
				indices.push_back( maxi );
				x_pom.push_back( x[maxi] ); 
				*max = -10000;

			} 
			for ( int j = 0; j < numPntsRow; j++ ){
				auto min = std::min_element( x_pom.begin(), x_pom.end() );
				auto mini = std::distance( x_pom.begin(), min);
				*min = 100000;
				keypoints_base.push_back(keypoints[ indices[ mini ] ] );
			}
			
		} 

		return keypoints_base;
	}

void CameraCalibration::angleRecovery(){

		std::vector<Point2f> kpoints = this->getKeypoints();
		
		Point2f origin;


		float pi = 3.14159265;
		std::vector<Point2f> kpoints_new;
		int numPnts = kpoints.size();
		std::vector<int> x, y;


		for(int i = 0; i < numPnts; i++){
			x.push_back( kpoints[i].x );
			y.push_back( kpoints[i].y );
		}

		auto min_a = std::min_element( x.begin(), x.end() );
		auto mini_a = std::distance( x.begin(), min_a);

		*min_a = 10000;
		auto min_b = std::min_element( x.begin(), x.end() );
		auto mini_b = std::distance( x.begin(), min_b);

		*min_b = 10000;
		auto min_c = std::min_element( x.begin(), x.end() );
		auto mini_c = std::distance( x.begin(), min_c);

		// get correct point for angle recovery
		if ((kpoints[mini_b].y > kpoints[mini_c].y && orientation == 0) || (orientation == 1 && (kpoints[mini_b].y < kpoints[mini_c].y))){
			mini_b = mini_c;
		}

		float angle;	
		float D = cv::norm(kpoints[mini_a] - kpoints[mini_b]);
		// origin and angle calculations
		if (orientation == 1){
			angle = atan2((kpoints[mini_b].x - kpoints[mini_a].x),(y[mini_b] - y[mini_a]) )*180/PI;
			origin = Point2f(kpoints[mini_a].x + 4 * D * sin(angle*pi/180), kpoints[mini_a].y + 4 * D * cos(angle*pi/180));
			origin = Point2f(origin.x + 3 * D * cos(angle*pi/180), origin.y - 3 * D * sin(angle*pi/180));
		}else{
			angle = atan2(( kpoints[mini_b].x  - kpoints[mini_a].x),( - y[mini_a] + y[mini_b]) );
			origin = Point2f(kpoints[mini_a].x - 15, kpoints[mini_a].y + 20);
		}

		// vector of rotated points
		std::vector<Point2f> rotated;

		for (int i = 0; i < kpoints.size(); i++){
			float d = cv::norm(kpoints[i] - origin);
			float D = 2 * d * sin(angle/180 / 2 * 3.14);
			// rotated.push_back(Point2f(kpoints[i].x + D*cos(angle/2), kpoints[i].y + D*sin(angle/2) ));
			if(orientation == 0){ 
				rotated.push_back(Point2f(kpoints[i].x - D*sin(angle/2), kpoints[i].y - D*cos(angle/2) ));
			} else {
				rotated.push_back(Point2f(kpoints[i].x + D*sin(angle/2*3.14/180), kpoints[i].y - D*cos(angle/2*3.14/180) ));
			}

		}

		std::cout<<"angle "<< angle <<endl;
		std::vector<KeyPoint> v,w;
		v.push_back(KeyPoint(kpoints[mini_a],1.f));
		// v.push_back(KeyPoint(kpoints[mini_b],1.f));
		v.push_back(KeyPoint(kpoints[mini_b],1.f));
		w.push_back(KeyPoint(origin,1.f));

		drawKeypoints( sceneIm, v, sceneIm, Scalar(0,0,255), DrawMatchesFlags::DEFAULT );
		drawKeypoints( sceneIm, w, sceneIm, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );



		std::vector<KeyPoint> v2;
		for (int i = 0; i < kpoints.size(); i++){
			v2.push_back(KeyPoint(rotated[i],1.f));
		}




		// finally obtaining rotated keypoints that will be used for sorting
		this->setKeypointsRotated(v2);

	}

#endif 