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
#include <opencv2/tracking.hpp>

//  #include <opencv2/sfm/fundamental.hpp>

//#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d/nonfree.hpp"

#define PI 3.14159265;

using namespace cv;
using namespace std;

bool use_mask = false;
	
void matchingMethod( Mat img, Mat templ, int match_method ){
  		Mat mask; Mat result;
  		
  		int result_cols =  img.cols - templ.cols + 1;
  		int result_rows = img.rows - templ.rows + 1;
  		// std::cout << img.cols << " "<< img.rows << " "<< templ.cols << " " << templ.rows << std::endl;
  		result.create( result_rows, result_cols, CV_32FC1 );
  		


  		normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
  		double minVal; double maxVal; Point minLoc; Point maxLoc;
  		Point matchLoc;
  		minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
  		if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED )
    		{ matchLoc = minLoc;
        std::cout << "here"; }
  		else
    	{ matchLoc = maxLoc; }

    std::cout << minVal << " Match loc " << matchLoc <<endl;
    imshow("W", result);
    waitKey(0);

		// std::cout <<cornerPoints << std::endl;
  		// return cornerPoints;
	}


    double matchingMethod2( Mat img, Mat templ, int match_method ){
      Mat mask; Mat result;
      // if(templ.rows != 211 && templ.cols !=209){
      //   resize(templ, templ, Size(209, 211));
      //   std::cout << "resizing" << endl;
      // }
      // std::cout<<"Beginning"<<endl;
      // imshow( "W1", img );
      // imshow( "W2", templ );
      // // imwrite("./images_with_rotation/rgb_template_up.jpg", templ);
      // waitKey(0);
      int result_cols =  img.cols - templ.cols + 1;
      int result_rows = img.rows - templ.rows + 1;
      // std::cout << img.cols << " "<< img.rows << " "<< templ.cols << " " << templ.rows << std::endl;
      result.create( result_rows, result_cols, CV_32FC1 );
      
      bool method_accepts_mask = (TM_SQDIFF == match_method || match_method == TM_CCORR_NORMED);

      Mat img_display;
      img.copyTo( img_display );

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
        
      // std::cout << minVal << " Match loc " << matchLoc <<endl;

      // imshow( "Window ", img_display );
      // waitKey(0);

      Rect roi(matchLoc,  Point(matchLoc.x + templ.cols,  matchLoc.y + templ.rows));
      Mat crop;
      // std::cout << roi.x <<" "<< roi.y <<" " <<roi.width + roi.x <<" "<<img.cols<< " "<< roi.height + roi.y << " "<<img.rows<< endl;
      if(roi.x >= 0 && roi.y >= 0 && roi.width + roi.x < img.cols && roi.height + roi.y < img.rows){
        crop = img(roi);
      }else{
        return -1;
      }
      
      Mat diff;
      img.copyTo(diff);
      absdiff(templ,crop,diff);
      // std::cout << "My criteria" << sum(diff)<<endl;

      // std::cout << "Point " << Point(templ.rows, templ.cols )<<endl;
      rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
      rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

      // imshow( "W1", crop );
      // imshow( "W2", templ );
      // // imwrite("./images_with_rotation/rgb_template_up.jpg", templ);
      // waitKey(0);

    // std::cout <<cornerPoints << std::endl;
      // return cornerPoints;
      return sum(diff).val[0];
  }

  int main(){
    Mat img = imread("./images_all/rgb/rgb3579.jpg", IMREAD_GRAYSCALE);
    Mat template1 = imread("./images_with_rotation/rgb_template.jpg", IMREAD_GRAYSCALE);
    Mat template2 = imread("./images_with_rotation/rgb_template_right.png", IMREAD_GRAYSCALE);
    Mat template3 = imread("./images_with_rotation/rgb_template_right_2.png", IMREAD_GRAYSCALE);
    Mat template4 = imread("./images_with_rotation/rgb_template_angle_left.png", IMREAD_GRAYSCALE);
    Mat template5 = imread("./images_with_rotation/rgb_template_left_2.png", IMREAD_GRAYSCALE);
    Mat template6 = imread("./images_with_rotation/rgb_template_left_3.png", IMREAD_GRAYSCALE);
    // Mat template3 = imread("./images/rgb_template.jpg", IMREAD_GRAYSCALE);

    // imshow("W", img);
    // waitKey(0);
    // imshow("W", template1);
    // waitKey(0);
    // imshow("W", template3);
    // waitKey(0);


    // matchingMethod2(img, template1, 0);
    // matchingMethod2(img, template2, 0);
    // matchingMethod2(img, template3, 0);

    std::vector<double> v;
    std::vector<string> ori;

    ori.push_back("Normal");
    ori.push_back("Right");
    ori.push_back("Right");
    ori.push_back("Left");
    ori.push_back("Left");
    ori.push_back("Left");
    
    double a;
    int left=0;
    int total=0;
    for (int i=0; i<5000;i++){
      Mat image = imread("./images_with_rotation/rgb_up/rgb" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
      // Mat image = imread("./images/rgb/rgb" + to_string(i) + ".jpg", IMREAD_GRAYSCALE);
      v.clear();
      if(!image.empty()){
        std::cout << "Image " + to_string(i) << endl;
        a  = matchingMethod2(image, template1, 3);
        v.push_back(a);

        a  = matchingMethod2(image, template2, 3);
        v.push_back(a);

        a  = matchingMethod2(image, template3, 3);
        v.push_back(a);


        a  = matchingMethod2(image, template4, 3);
        v.push_back(a);

        a  = matchingMethod2(image, template5, 3);
        v.push_back(a);

        a  = matchingMethod2(image, template6, 3);
        v.push_back(a);

        auto min_b = std::min_element( v.begin(), v.end() );
        auto mini_b = std::distance( v.begin(), min_b);
        // std::cout << mini_b << endl;
        if("Normal" == ori[mini_b]){
          left += 1;
        }else{
          std::cout << to_string(i) + " --- Image orientation recognition failed, orientation " + ori[mini_b] << endl;
        }

        total +=1;

    }
    
  }
    std::cout << " --- Accurate " << double(left)/double(total) << endl;
    return 0;
  }