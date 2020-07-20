#ifndef POINT_DEPTH
#define POINT_DEPTH
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

using namespace std;
using namespace cv;
class PointCloud{
	ros::NodeHandle nh_;
	ros::Subscriber sub;
	Mat origin;
	public:
		PointCloud();
		~PointCloud();
		void PointDepthCb(const sensor_msgs::PointCloud2& msg);
		float GetDepth(const sensor_msgs::PointCloud2& msg);
};

#endif