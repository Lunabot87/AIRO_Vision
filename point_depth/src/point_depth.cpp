#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"

using namespace std;
using namespace cv;
typedef sensor_msgs::PointCloud& PointMsg;
class PointCloud{
	ros::NodeHandle nh_;
	ros::Subscriber sub;
	double output[3] = {999, 999, 999};
	vector<vector<Point3d>> point_segments;
	Mat *mean_filter;
	vector<int> Z_Index;
	sensor_msgs::PointCloud point_img;
	int PointCloudMax_Xaxis(sensor_msgs::PointCloud& input);
	int PointCloudMax_Yaxis(sensor_msgs::PointCloud& input);
	void PointCloud2ToPointCloud(const sensor_msgs::PointCloud2& msg,sensor_msgs::PointCloud& point_img,int resize);
	Mat Point2ToMat(const sensor_msgs::PointCloud2& msg);
	void PointToSegment(PointMsg point_img, vector<int>& Z_Index, vector<vector<Point3d>>& point_segments);
	Mat origin;
	vector<Point3d> SelectedPoint;
	void SelectPoint(vector<vector<Point3d>>& point_segments, vector<Point3d>& SelectedPoint);
	Scalar scalar_n[3] = {Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255)};
	void op_zaxis(vector<Point3d>& SelectedPoint, double output[3]);
	void PointZ_MinMax(PointMsg input,float* output);
	void PointX_MinMax(PointMsg input,float* output);
	void PointY_MinMax(PointMsg input,float* output);
	void PointHistogram(PointMsg input, float min_max[2], vector<int>& Z_Index);
	public:
		PointCloud();
		~PointCloud();
		void PointDepthCb(const sensor_msgs::PointCloud2& msg);
		
};

PointCloud::PointCloud()
{
	ROS_INFO("START NODE");
	sub = nh_.subscribe("/camera/depth/color/points", 1 , &PointCloud::PointDepthCb , this);
	mean_filter= new Mat[30];
	for(int i=0;i<30;i++)
			mean_filter[i] = Mat::zeros(Size(640, 480), CV_32F);
}
PointCloud::~PointCloud()
{
	delete[] mean_filter;
}

void PointCloud::PointDepthCb(const sensor_msgs::PointCloud2& msg)
{
	origin = Point2ToMat(msg);
	if(!origin.empty()){
		float min_max[2];
		output[0] = 999;
		output[1] = 999;
		output[2] = 999;
		PointZ_MinMax(point_img,min_max);
		PointHistogram(point_img, min_max, Z_Index);
		PointToSegment(point_img, Z_Index, point_segments);
		while(output[0] == 999 && output[1] == 999 & output[2] == 999){
			SelectPoint(point_segments, SelectedPoint);
			op_zaxis(SelectedPoint, output);
		}
		cout<<output[0]<<", "<<output[1]<<", "<<output[2]<<endl;
		cout<<SelectedPoint[0].x<<", "<<SelectedPoint[0].y<<", "<<SelectedPoint[0].z<<endl;
		cout<<SelectedPoint[1].x<<", "<<SelectedPoint[1].y<<", "<<SelectedPoint[1].z<<endl;
		cout<<SelectedPoint[2].x<<", "<<SelectedPoint[2].y<<", "<<SelectedPoint[2].z<<endl;
		cout<<SelectedPoint[3].x<<", "<<SelectedPoint[3].y<<", "<<SelectedPoint[3].z<<endl;
		cout<<acosf(output[2]) * 180 / CV_PI<<endl;;
		origin.convertTo(origin, CV_8U);
		cvtColor(origin, origin, COLOR_GRAY2BGR);
		for(int i = 0; i < SelectedPoint.size(); i++)
			circle(origin,Point((int)SelectedPoint[i].x + 320, (int)SelectedPoint[i].y + 240), 2, Scalar(255,0,0),2);
		/*for(int i=0;i<point_img.points.size();i++)
			cout<<point_img.points[i].z<<endl;*/
		imshow("origin", origin);
		waitKey(100);
		point_segments.clear();
		Z_Index.clear();
		SelectedPoint.clear();	
	}
}
void PointCloud::SelectPoint(vector<vector<Point3d>>& point_segments, vector<Point3d>& SelectedPoint){
	int Seg_Size = point_segments[0].size();
	int random;
	int swsw = 0;
	int count = 0;
	Point3d FirstSelect;
	Point3d SecondSelect;
	Point3d ThirdSelect;
	Point3d FourthSelect;
	double gap_1, gap_2, gap_3;
	SelectedPoint.clear();
	while(SelectedPoint.size() != 4 && count != 100){
		unsigned int rand_count = (unsigned int)ros::Time::now().nsec;
		srand(rand_count);
		random = rand() % Seg_Size;
		FirstSelect = Point3d(point_segments[0][random]);
		for(int i = 0; i < Seg_Size; i++){
			SecondSelect = point_segments[0][i];
			gap_1 = abs(sqrt((FirstSelect.x-SecondSelect.x)*(FirstSelect.x-SecondSelect.x)+(FirstSelect.y-SecondSelect.y)*(FirstSelect.y-SecondSelect.y))-((double)20.00 * sqrt(2)));
			if(gap_1 <= (double)1.00){
				for(int j = 0; j < Seg_Size; j++){
					ThirdSelect = point_segments[0][j];
					gap_1 = abs(sqrt((FirstSelect.x-ThirdSelect.x)*(FirstSelect.x-ThirdSelect.x)+(FirstSelect.y-ThirdSelect.y)*(FirstSelect.y-ThirdSelect.y))-(double)20.00);
					gap_2 = abs(sqrt((SecondSelect.x-ThirdSelect.x)*(SecondSelect.x-ThirdSelect.x)+(SecondSelect.y-ThirdSelect.y)*(SecondSelect.y-ThirdSelect.y))-(double)20.00);
					if(gap_1 <= (double)1.00 && gap_2 <= (double)1.00){
						for(int k = 0; k < Seg_Size; k++){
							FourthSelect = point_segments[0][k];
							gap_1 = abs(sqrt((FirstSelect.x-FourthSelect.x)*(FirstSelect.x-FourthSelect.x)+(FirstSelect.y-FourthSelect.y)*(FirstSelect.y-FourthSelect.y))-(double)20.00);
							gap_2 = abs(sqrt((SecondSelect.x-FourthSelect.x)*(SecondSelect.x-FourthSelect.x)+(SecondSelect.y-FourthSelect.y)*(SecondSelect.y-FourthSelect.y))-(double)20.00);
							gap_3 = abs(sqrt((ThirdSelect.x-FourthSelect.x)*(ThirdSelect.x-FourthSelect.x)+(ThirdSelect.y-FourthSelect.y)*(ThirdSelect.y-FourthSelect.y))-((double)20.00 * sqrt(2)));
							if(gap_1 <= (double)1.00 && gap_2 <= (double)1.00 && gap_3 <= (double)1.00){
								SelectedPoint.push_back(FirstSelect);
								SelectedPoint.push_back(SecondSelect);
								SelectedPoint.push_back(ThirdSelect);
								SelectedPoint.push_back(FourthSelect);
								swsw = 1;
								break;
							}
						}
					}
					if(swsw == 1)
						break;
				}
			}
			if(swsw == 1)
				break;
		}
		if(SelectedPoint.size() != 4)
			SelectedPoint.clear();
		count++;
	}
}
void PointCloud::op_zaxis(vector<Point3d>& SelectedPoint, double output[3]){
	Point3d cross_vector_1;
	Point3d cross_vector_2;
	Point3d cross_vector_3;
	Point3d cross_vector_4;
	if(SelectedPoint.size() == 4){
		double vector_a[3] ,vector_b[3];
		vector_a[0] = SelectedPoint[0].x - SelectedPoint[2].x;
		vector_a[1] = SelectedPoint[0].y - SelectedPoint[2].y;
		vector_a[2] = SelectedPoint[0].z - SelectedPoint[2].z;
		vector_b[0] = SelectedPoint[1].x - SelectedPoint[2].x;
		vector_b[1] = SelectedPoint[1].y - SelectedPoint[2].y;
		vector_b[2] = SelectedPoint[1].z - SelectedPoint[2].z;
		output[0] = vector_b[1]*vector_a[2] - vector_b[2]*vector_a[1];
		output[1] = vector_b[2]*vector_a[0] - vector_b[0]*vector_a[2];
		output[2] = vector_b[0]*vector_a[1] - vector_b[1]*vector_a[0];
		double norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
		output[0] /= norm;
		output[1] /= norm;
		output[2] /= norm;
		cross_vector_1 = Point3d(output[0],output[1],output[2]);
		vector_a[0] = SelectedPoint[0].x - SelectedPoint[3].x;
		vector_a[1] = SelectedPoint[0].y - SelectedPoint[3].y;
		vector_a[2] = SelectedPoint[0].z - SelectedPoint[3].z;
		vector_b[0] = SelectedPoint[1].x - SelectedPoint[3].x;
		vector_b[1] = SelectedPoint[1].y - SelectedPoint[3].y;
		vector_b[2] = SelectedPoint[1].z - SelectedPoint[3].z;
		output[0] = vector_b[1]*vector_a[2] - vector_b[2]*vector_a[1];
		output[1] = vector_b[2]*vector_a[0] - vector_b[0]*vector_a[2];
		output[2] = vector_b[0]*vector_a[1] - vector_b[1]*vector_a[0];
		norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
		output[0] /= norm;
		output[1] /= norm;
		output[2] /= norm;
		cross_vector_2 = Point3d(output[0],output[1],output[2]);
		vector_a[0] = SelectedPoint[2].x - SelectedPoint[0].x;
		vector_a[1] = SelectedPoint[2].y - SelectedPoint[0].y;
		vector_a[2] = SelectedPoint[2].z - SelectedPoint[0].z;
		vector_b[0] = SelectedPoint[3].x - SelectedPoint[0].x;
		vector_b[1] = SelectedPoint[3].y - SelectedPoint[0].y;
		vector_b[2] = SelectedPoint[3].z - SelectedPoint[0].z;
		output[0] = vector_b[1]*vector_a[2] - vector_b[2]*vector_a[1];
		output[1] = vector_b[2]*vector_a[0] - vector_b[0]*vector_a[2];
		output[2] = vector_b[0]*vector_a[1] - vector_b[1]*vector_a[0];
		norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
		output[0] /= norm;
		output[1] /= norm;
		output[2] /= norm;
		cross_vector_3 = Point3d(output[0],output[1],output[2]);
		vector_a[0] = SelectedPoint[2].x - SelectedPoint[1].x;
		vector_a[1] = SelectedPoint[2].y - SelectedPoint[1].y;
		vector_a[2] = SelectedPoint[2].z - SelectedPoint[1].z;
		vector_b[0] = SelectedPoint[3].x - SelectedPoint[1].x;
		vector_b[1] = SelectedPoint[3].y - SelectedPoint[1].y;
		vector_b[2] = SelectedPoint[3].z - SelectedPoint[1].z;
		output[0] = vector_b[1]*vector_a[2] - vector_b[2]*vector_a[1];
		output[1] = vector_b[2]*vector_a[0] - vector_b[0]*vector_a[2];
		output[2] = vector_b[0]*vector_a[1] - vector_b[1]*vector_a[0];
		norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
		output[0] /= norm;
		output[1] /= norm;
		output[2] /= norm;
		cross_vector_4 = Point3d(output[0],output[1],output[2]);
		if(cross_vector_1.z < 0){
			cross_vector_1.x *= -1;
			cross_vector_1.y *= -1;
			cross_vector_1.z *= -1;
		}
		if(cross_vector_2.z < 0){
			cross_vector_2.x *= -1;
			cross_vector_2.y *= -1;
			cross_vector_2.z *= -1;
		}
		if(cross_vector_3.z < 0){
			cross_vector_3.x *= -1;
			cross_vector_3.y *= -1;
			cross_vector_3.z *= -1;
		}
		if(cross_vector_4.z < 0){
			cross_vector_4.x *= -1;
			cross_vector_4.y *= -1;
			cross_vector_4.z *= -1;
		}
		if(cross_vector_1.x > 0 && cross_vector_2.x > 0 && cross_vector_3.x > 0 && cross_vector_4.x > 0){
			if(cross_vector_1.y > 0 && cross_vector_2.y > 0 && cross_vector_3.y > 0 && cross_vector_4.y > 0){
				output[0] = cross_vector_1.x + cross_vector_2.x + cross_vector_3.x + cross_vector_4.x;
				output[1] = cross_vector_1.y + cross_vector_2.y + cross_vector_3.y + cross_vector_4.y;
				output[2] = cross_vector_1.z + cross_vector_2.z + cross_vector_3.z + cross_vector_4.z;
				norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
				output[0] /= norm;
				output[1] /= norm;
				output[2] /= norm;
			}
			else if(cross_vector_1.y < 0 && cross_vector_2.y < 0 && cross_vector_3.y < 0 && cross_vector_4.y < 0){
				output[0] = cross_vector_1.x + cross_vector_2.x + cross_vector_3.x + cross_vector_4.x;
				output[1] = cross_vector_1.y + cross_vector_2.y + cross_vector_3.y + cross_vector_4.y;
				output[2] = cross_vector_1.z + cross_vector_2.z + cross_vector_3.z + cross_vector_4.z;
				norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
				output[0] /= norm;
				output[1] /= norm;
				output[2] /= norm;
			}
			else{
				output[0] = 999;
				output[1] = 999;
				output[2] = 999;
				//cout << "Not Found Point" << endl;
			}
		}
		else if(cross_vector_1.x < 0 && cross_vector_2.x < 0 && cross_vector_3.x < 0 && cross_vector_4.x < 0){
			if(cross_vector_1.y > 0 && cross_vector_2.y > 0 && cross_vector_3.y > 0 && cross_vector_4.y > 0){
				output[0] = cross_vector_1.x + cross_vector_2.x + cross_vector_3.x + cross_vector_4.x;
				output[1] = cross_vector_1.y + cross_vector_2.y + cross_vector_3.y + cross_vector_4.y;
				output[2] = cross_vector_1.z + cross_vector_2.z + cross_vector_3.z + cross_vector_4.z;
				norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
				output[0] /= norm;
				output[1] /= norm;
				output[2] /= norm;
			}
			else if(cross_vector_1.y < 0 && cross_vector_2.y < 0 && cross_vector_3.y < 0 && cross_vector_4.y < 0){
				output[0] = cross_vector_1.x + cross_vector_2.x + cross_vector_3.x + cross_vector_4.x;
				output[1] = cross_vector_1.y + cross_vector_2.y + cross_vector_3.y + cross_vector_4.y;
				output[2] = cross_vector_1.z + cross_vector_2.z + cross_vector_3.z + cross_vector_4.z;
				norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
				output[0] /= norm;
				output[1] /= norm;
				output[2] /= norm;
			}
			else{
				output[0] = 999;
				output[1] = 999;
				output[2] = 999;
				//cout << "Not Found Point" << endl;
			}
		}
		else{
			output[0] = 999;
			output[1] = 999;
			output[2] = 999;
			//cout << "Not Found Point" << endl;
		}
	}
	else{
		output[0] = 999;
		output[1] = 999;
		output[2] = 999;
		//cout<<"No Selected Point"<<endl;
	}
}
void PointCloud::PointToSegment(PointMsg point_img, vector<int>& Z_Index, vector<vector<Point3d>>& point_segments){ 
	int LOOP_SIZE = point_img.points.size();
	int z_size = Z_Index.size();
	vector<Point3d> point_segment;
	vector<vector<int>> Z_Segments;
	vector<int> Z_Segment;
	for(int i = 0; i < z_size; i++){
		if(i != z_size - 1){
			if(Z_Index[i + 1] - Z_Index[i] <= 1){
				Z_Segment.push_back(Z_Index[i]);
			}
			else if(Z_Index[i + 1] - Z_Index[i] > 1){
				Z_Segment.push_back(Z_Index[i]);
				Z_Segments.push_back(Z_Segment);
				Z_Segment.clear();
			}
		}
		else if(i == z_size - 1){
			Z_Segment.push_back(Z_Index[i]);
			Z_Segments.push_back(Z_Segment);
			Z_Segment.clear();
		}
	}
	int segment_size = Z_Segments.size();
	for(int k = 0; k < segment_size; k++){
		for(int j = 0; j < Z_Segments[k].size(); j++){
			for(int i = 0; i < LOOP_SIZE; i++){
				if(Z_Segments[k][j] == (int)point_img.points[i].z)
					point_segment.push_back(Point3d(point_img.points[i].x, point_img.points[i].y, point_img.points[i].z));
			}
		}
		point_segments.push_back(point_segment);
		point_segment.clear();
	}
	Z_Segments.clear();
}
void PointCloud::PointHistogram(PointMsg input, float min_max[2], vector<int>& Z_Index)
{
	int LOOP_SIZE = input.points.size();
	vector<int> Point_Histogram;
	int hist_size = min_max[1] - min_max[0] + 1; 
	int z_min = min_max[0];
	for(int i = 0; i < hist_size; i++)
		Point_Histogram.push_back(0);
	for(int i = 0; i < LOOP_SIZE; i++)
		Point_Histogram[(int)input.points[i].z - (int)min_max[0]] += 1;
	int critical_value = 1000;
	for(int i = 0; i < hist_size; i++)
		if(Point_Histogram[i] > critical_value)
			Z_Index.push_back(i + z_min);
	Point_Histogram.clear();
}
int PointCloud::PointCloudMax_Xaxis(sensor_msgs::PointCloud& input){
	int LOOP_SIZE = input.points.size();
	int max_x =0;
	for(int i = 0 ; i < LOOP_SIZE; ++i){
		if(input.points[i].x> max_x)
			max_x = input.points[i].x;
	}
	if(max_x>320)
		return 320;
	else
		return max_x;
}
int PointCloud::PointCloudMax_Yaxis(sensor_msgs::PointCloud& input){
	int LOOP_SIZE = input.points.size();
	int max_y =0;
	for(int i = 0 ; i < LOOP_SIZE; ++i){
		if(input.points[i].y> max_y)
			max_y = input.points[i].y;
	}
	if(max_y>240)
		return 240;
	else
		return max_y;
}
void PointCloud::PointCloud2ToPointCloud(const sensor_msgs::PointCloud2& msg,sensor_msgs::PointCloud& point_img,int resize)
{
	sensor_msgs::convertPointCloud2ToPointCloud(msg, point_img);
	for(int i = 0 ; i < point_img.points.size(); ++i){
		point_img.points[i].x *= resize;
		point_img.points[i].y *= resize;
		point_img.points[i].z *= resize;
	}
}

Mat PointCloud::Point2ToMat(const sensor_msgs::PointCloud2& msg){
	PointCloud::PointCloud2ToPointCloud(msg,point_img,1000);
	int width = PointCloudMax_Xaxis(point_img)*2;
	int height = PointCloudMax_Yaxis(point_img)*2;
	Mat temp = Mat::zeros(Size(640, 480), CV_32F);
	float* temp_data = (float*) temp.data;

	int LOOP_SIZE = point_img.points.size();
	
	for(int i =0; i<LOOP_SIZE;++i)
	{

		int x_pos = point_img.points[i].x + 320;
		int y_pos = point_img.points[i].y + 240;
		if(x_pos >= 0 && x_pos<640)
			if(y_pos >= 0 && y_pos<480)
				if(temp_data[y_pos*temp.cols+x_pos] != 0)
					temp_data[y_pos*temp.cols+x_pos] = (temp_data[y_pos*temp.cols+x_pos]+point_img.points[i].z)/2;
				else
					temp_data[y_pos*temp.cols+x_pos] = point_img.points[i].z;
	}
	return temp.clone();
}
void PointCloud::PointX_MinMax(PointMsg input,float* output)
{
	int LOOP_SIZE = input.points.size();
	output[0] = 100000.0;
	output[1] = 0.0;
	for(int i=0;i<LOOP_SIZE;i++)
	{
		if(output[0]>input.points[i].x)
			output[0] = input.points[i].x;
		if(output[1] < input.points[i].x)
			output[1] = input.points[i].x;
	}
}
void PointCloud::PointY_MinMax(PointMsg input,float* output)
{
	int LOOP_SIZE = input.points.size();
	output[0] = 100000.0;
	output[1] = 0.0;
	for(int i=0;i<LOOP_SIZE;i++)
	{
		if(output[0]>input.points[i].y)
			output[0] = input.points[i].y;
		if(output[1] < input.points[i].y)
			output[1] = input.points[i].y;
	}
}
void PointCloud::PointZ_MinMax(PointMsg input,float* output)
{
	int LOOP_SIZE = input.points.size();
	output[0] = 100000.0;
	output[1] = 0.0;
	for(int i=0;i<LOOP_SIZE;i++)
	{
		if(output[0]>input.points[i].z)
			output[0] = input.points[i].z;
		if(output[1] < input.points[i].z)
			output[1] = input.points[i].z;
	}
}