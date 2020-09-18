#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";


using namespace std;

cv::Mat img_1, img_2;
int save = 0;
int count_images = 0;
bool a, b;

class ImageConverter
{
  ros::NodeHandle nh_, nh2_;
  image_transport::ImageTransport it_, it2_;
  image_transport::Subscriber image_sub_, image_sub2_;
  image_transport::Publisher image_pub_;
  std::string topic_rgb = "/optris/usb_cam/image_raw";
  std::string topic_thermal = "/optris/thermal_image_view";


public:
  ImageConverter()
    : it_(nh_), it2_(nh2_)
  {
    // Subscrive to input video feed and publish output video feed
    // usb_cam/image_view radi za obicnu kameru
    // /optris/thermal_image_view
    //"/optris/thermal_image_view"
    // "/usb_cam/image_view"
    // /image_view/output

    // za stare bag filove 
    // /camera/image_raw
    // /camera/thermal_image_view

    // recondings_new /image_view/output /optris/thermal_image_view
    image_sub_ = it_.subscribe(topic_rgb, 1,
      &ImageConverter::imageCb, this);
    image_sub2_ = it2_.subscribe(topic_thermal, 1,
      &ImageConverter::imageCbR, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW2);
  }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		
    	img_1 = (cv_ptr->image).clone();
    	
    	// ROS_INFO("%d %d PRVA %d", (cv_ptr->image).rows, (cv_ptr->image).cols, count_images); 	
    	//if (save == 100) imwrite("/root/catkin_ws/src/test/src/base1.jpg", img_1);
      	//else save++;
      	// std::string location = "/root/catkin_ws/src/test/src/images_all/rgb/rgb" + std::to_string(count_images) +".jpg";
      	// if(count_images % 1 == 0) imwrite(location, img_1);
      	
      	a = true;
      	if (a && b){
      		a = false;
      		b = false;
      	      		
      		std::string locationR = "/root/catkin_ws/src/test/src/two/rgb/rgb" + std::to_string(count_images) +".jpg";
      		std::string locationT = "/root/catkin_ws/src/test/src/two/thermal/thermal" + std::to_string(count_images) +".jpg";
      		
      		if (count_images % 1 == 0){
      			imwrite(locationR, img_1);
				imwrite(locationT, img_2);
      		}

      		count_images++;
      	}

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
 	// if(save > 4) {
 	// 	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //   	cv::waitKey(3);
 	// }

  }
    void imageCbR(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		
    	img_2 = (cv_ptr->image).clone();
    	
    	// ROS_INFO("%d %d DRUGA %d", (cv_ptr->image).rows, (cv_ptr->image).cols, count_images); 	
    	// if (save == 100) imwrite("/root/catkin_ws/src/test/src/base2.jpg", img_2);
    	// std::string location = "/root/catkin_ws/src/test/src/images_all/thermal/thermal" + to_string(count_images) +".jpg"; 
        // if(count_images % 1 == 0) imwrite(location, img_2);
        // count_images += 1;
      
      	b = true;

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  //   // Update GUI Window
 	// if(save > 4) {
 	// 	cv::imshow(OPENCV_WINDOW2, img_2);
  //   	cv::waitKey(3);
 	// }

  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}