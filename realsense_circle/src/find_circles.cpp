#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"
using namespace std;
using namespace cv;
static const string OPENCV_WINDOW = "Image Window";


class ImageConverter{
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_,depth_sub;
     image_transport::Publisher image_pub_;
     Mat gray,edge;
     cv_bridge::CvImagePtr cv_ptr;
   public:
     ImageConverter();
     ~ImageConverter();
    void depthCb(const sensor_msgs::ImageConstPtr& msg);
    void Traffic_Light(Mat& H, Mat& S, Mat& BGR);
   
 };



///////////////// define funtions 

ImageConverter::~ImageConverter()
{
	destroyWindow(OPENCV_WINDOW);
};

ImageConverter::ImageConverter(): it_(nh_)
{
       ROS_INFO("GRAP CAMERA");
       depth_sub = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1 , &ImageConverter::depthCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
};

void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr& msg)
    {

     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
     }
     catch (cv_bridge::Exception& e)
      {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
	cout<<cv_ptr->image.size()<<endl;
	Mat image_d;
	cv_ptr->image.convertTo(image_d, CV_32F);
	float* image_data = (float*) image_d.data;
	Mat dem = Mat::zeros(Size(640, 480), CV_32F);
	float* dem_data = (float*) dem.data;
	for(int i=0;i<dem.rows;i++)
	{
		for(int j=0;j<dem.cols;j++)
		{
			dem_data[i*dem.cols+j] = image_data[i*dem.cols+j]/10;
		}
	}
	cout<<dem_data[640*240 + 280]<<"\t"<<dem_data[640*240 + 300]<<"\t"<<dem_data[640*240 + 320];
	cout<<dem_data[640*240 + 340]<<"\t"<<dem_data[640*240 + 360]<<"\t"<<dem_data[640*240 + 380]<<endl;
	dem.convertTo(dem, CV_8U);
	cvtColor(dem, dem, COLOR_GRAY2BGR);
	circle(dem, Point(280, 240), 2, Scalar(255, 0, 0), 2);
	circle(dem, Point(300, 240), 2, Scalar(100, 100, 0), 2);
	circle(dem, Point(320, 240), 2, Scalar(0, 255, 0), 2);	
	circle(dem, Point(340, 240), 2, Scalar(0, 100, 100), 2);
	circle(dem, Point(360, 240), 2, Scalar(0, 0, 255), 2);
	circle(dem, Point(380, 240), 2, Scalar(255,255, 255), 2);
	imshow("depth",dem);
	waitKey(1);
   }


