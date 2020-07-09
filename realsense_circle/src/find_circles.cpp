#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/TransformStamped.h"
using namespace std;
using namespace cv;
static const string OPENCV_WINDOW = "Image Window";


class ImageConverter{
     ros::NodeHandle nh_;
     ros::Subscriber size_sub,start_op;
     ros::Publisher tf_pub,c_pub;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
     std_msgs::Float32MultiArray center_msg;
     void Hough_Circle();
     void circle_t();
     vector<Point3f> circles;
     Mat gray,edge;
     cv_bridge::CvImagePtr cv_ptr;
     int minDist;
     int Threshold;
     int minRadius;
     int maxRadius;
     float depth_data;
     string  circles_names[3];
     std_msgs::Float32MultiArray output_msg;
     geometry_msgs::TransformStamped tf_msg;
     static bool cmp(const Point3f& p1, const Point3f& p2);
   public:
     int Key;
     void ParameterSet(int min_D,int Th,int min_R, int max_R){
     	minDist = min_D;
	Threshold = Th;
	minRadius = min_R;
	maxRadius = max_R;
     }
     ImageConverter();
     ~ImageConverter();
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthCb(const std_msgs::Float32 &msg);
    void opCb(const std_msgs::Float32 &msg);
    
   
 };



///////////////// define funtions 

ImageConverter::~ImageConverter()
{
	destroyWindow(OPENCV_WINDOW);
};

ImageConverter::ImageConverter(): it_(nh_),minDist(30),Threshold(150),minRadius(9),maxRadius(15),depth_data(30)
{
       ROS_INFO("GRAP CAMERA");
       image_sub_ = it_.subscribe("/camera/color/image_raw", 1 , &ImageConverter::imageCb, this);
       size_sub = nh_.subscribe("depth_value", 1 , &ImageConverter::depthCb , this);
       start_op = nh_.subscribe("camera_op", 1 , &ImageConverter::opCb , this);
       tf_pub = nh_.advertise<geometry_msgs::TransformStamped>("TF_update",100);
       c_pub = nh_.advertise<std_msgs::Float32MultiArray>("cirlce_center",100);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
       circles_names[0] = "circle_1";
       circles_names[1] = "circle_2";
       circles_names[2] = "circle_3";
       tf_msg.header.frame_id = "sensor_d435_color_optical_frame";
       tf_msg.transform.rotation.x=0;
       tf_msg.transform.rotation.y=01;
       tf_msg.transform.rotation.z=0;
       tf_msg.transform.rotation.w=0.001;
};

void ImageConverter::depthCb(const std_msgs::Float32 &msg){
	ROS_INFO("GRAP_DEPTH_DATA");
	depth_data = msg.data;
	ROS_INFO("RECIEVED DATA = %f",depth_data);
};
void ImageConverter::opCb(const std_msgs::Float32 &msg){
	ROS_INFO("OPERATING START ...");
	circle_t();
     Mat copy;
     cvtColor(gray,copy,COLOR_GRAY2BGR);
     for(int i{};i<circles.size();i++){
     	circle(copy, Point(circles[i].x,circles[i].y),circles[i].z,Scalar(255,0,0),2);
     }
     imshow("1",copy);
     ROS_INFO("FIND_NUM_CIRCLES = %d",circles.size());
     if(circles.size()){
     	output_msg.data.clear();
     	center_msg.data.clear();
     	for(int i=0;i<circles.size();i++){
     	center_msg.data.push_back(circles[i].x);
     	center_msg.data.push_back(circles[i].y);
     	cout<<circles[i].x<<"\t"<<circles[i].y<<"\t"<<endl<<endl;
        tf_msg.child_frame_id = circles_names[i];
        tf_msg.transform.translation.x=((circles[i].x - 320)*10/17)/1000;
        tf_msg.transform.translation.y=((circles[i].y - 240)*10/17)/1000;
        tf_msg.transform.translation.z=0.3;
		tf_pub.publish(tf_msg);
		}
		c_pub.publish(center_msg);
	}
}
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
    {

     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
      {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
     circles.clear();
     imshow(OPENCV_WINDOW,cv_ptr->image);
     waitKey(1);
     cvtColor(cv_ptr->image,gray,COLOR_BGR2GRAY);
     
   }

bool ImageConverter::cmp(const Point3f& p1, const Point3f& p2) {
	static float a, b;
	a = pow(p1.x, 2) + pow(p1.y, 2);
	b = pow(p2.x, 2) + pow(p2.y, 2);
	if (a < b)
		return true;
	else if (a == b)
		return p1.x < p2.x;
	else
		return false;
}

void ImageConverter::Hough_Circle() {
	static uchar* edge_output;
	edge_output = edge.data;
	Mat visited = Mat::zeros(edge.size(), edge.type());
	static uchar* visited_output;
	visited_output = visited.data;
	static int H_lenght;
	H_lenght = (edge.rows + 2 * maxRadius) * (edge.cols + 2 * maxRadius) * (maxRadius - minRadius);
	static int* H;
	H = new int [H_lenght] {};
	static int a_idx, b_idx;
	static vector<Point3f> cir_vec;
	static Point3f circlept;
	for (int j{}; j < edge.rows; j++)
		for (int i{}; i < edge.cols; i++) {
			if (edge_output[j * edge.cols + i] > 0) {
				for (int r = minRadius; r < maxRadius; r++)
					for (int t{}; t < 360; t++) {
						a_idx = cvRound(i - r * (cos(t * CV_PI / 180)));
						b_idx = cvRound(j - r * (sin(t * CV_PI / 180)));
						H[(r - minRadius) * (edge.rows + 2 * maxRadius) * (edge.cols + 2 * maxRadius) + (maxRadius + b_idx) * (edge.cols + 2 * maxRadius) + (maxRadius + a_idx)] += 1;
					}
			}
		}
	static Point pt_pre;
	pt_pre = Point(0,0);
	for (int x = maxRadius; x < edge.cols + maxRadius; x++)
		for (int y = maxRadius; y < edge.rows + maxRadius; y++)
			for (int z{}; z < (maxRadius - minRadius); z++) {
				if (H[z * (edge.rows + 2 * maxRadius) * (edge.cols + 2 * maxRadius) + y * (edge.cols + 2 * maxRadius) + x] > Threshold || (x == edge.cols + maxRadius - 1 && y == edge.rows + maxRadius - 1 && z == (maxRadius - minRadius) - 1)) {
					circlept = Point3f(x - maxRadius, y - maxRadius, z + minRadius);
					if (sqrt(pow(pt_pre.x - circlept.x, 2) + pow(pt_pre.y - circlept.y, 2)) <= minDist) {
						cir_vec.push_back(circlept);
						pt_pre = Point(circlept.x, circlept.y);
					}
					else if (sqrt(pow(pt_pre.x - circlept.x, 2) + pow(pt_pre.y - circlept.y, 2)) > minDist) {
						if (!cir_vec.empty()) {
							int pt_x = 0;
							int pt_y = 0;
							int c = 0;
							int Radius = 0;
							for (int k{}; k < cir_vec.size(); k++) {
								pt_x += cir_vec[k].x;
								pt_y += cir_vec[k].y;
								Radius += cir_vec[k].z;
							}
							pt_x = pt_x / cir_vec.size();
							pt_y = pt_y / cir_vec.size();
							Radius = Radius / cir_vec.size();
							for (int j = -minDist; j <= minDist; j++)
								for (int i = -minDist; i <= minDist; i++)
									if (visited_output[(pt_y - j) * visited.cols + (pt_x - i)] > 0) c += 1;
							if (c == 0) {
								visited_output[pt_y * visited.cols + pt_x] = 1;
								circles.push_back(Point3f(pt_x, pt_y, Radius));
							}
							cir_vec.clear();
						}
						cir_vec.push_back(circlept);
						pt_pre = Point(circlept.x, circlept.y);
					}
				}
			}
	delete[] H;
}

void ImageConverter::circle_t() {
	GaussianBlur(gray, gray, Size(5,5), 0);
	Canny(gray, edge, 100, 50);
	imshow("2",edge);
	Hough_Circle();
	sort(circles.begin(), circles.end(), ImageConverter::cmp);
}
