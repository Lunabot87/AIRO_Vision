#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
using namespace std;
using namespace cv;

class ImageConverter{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_,depth_sub;
    image_transport::Publisher image_pub_;
    Mat gray,edge;
    cv_bridge::CvImagePtr cv_ptr;
    bool filter_flag;
    ros::Subscriber start_op;
    Mat origin,filtered,non_filter;
    float min_max[2];
	float histogram[2];
	float test[6];
	Point op_array[3];
	float axis_vector[3];
	bool op_flag;
	int depth_count;
	int filter_size;
	Mat *mean_filter;
   public:
     ImageConverter();
     ~ImageConverter();
    void depthCb(const sensor_msgs::ImageConstPtr& msg);
    void Traffic_Light(Mat& H, Mat& S, Mat& BGR);

	private:
	Mat image_filter(Mat& input_image, int filter_size,Mat* temp_image);
    float find_mean(Mat &input_image,int offset);
    Mat filter_ROI(Mat &input_image, float cut,float cut_err,float offset);
    void find_min_max(Mat &input_image,float min_offset,float max_offset,float output[2]);
    void find_max_histogram(Mat &input_image,float min_max[2],float output[2]);
    Mat blur(Mat &input_image,int blur_size);
    void opCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void op_zaxis(Mat &input_image,Point op_array[3] ,float output[3]);
 };

///////////////// define funtions 

ImageConverter::~ImageConverter()
{
	delete[] mean_filter;	
};

ImageConverter::ImageConverter(): it_(nh_),filter_flag(false),op_flag(false),depth_count(0)
{
       ROS_INFO("GRAP CAMERA");
       depth_sub = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1 , &ImageConverter::depthCb, this);
       image_pub_ = it_.advertise("depth_node", 1);
       start_op = nh_.subscribe("cirlce_center", 1 , &ImageConverter::opCb , this);
	filter_size =30;
	mean_filter= new Mat[filter_size];
	for(int i=0;i<filter_size;i++)
			mean_filter[i] = Mat::zeros(Size(640, 480), CV_32F);
};
void ImageConverter::opCb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	int msg_size = msg->data.size();
	if(msg_size ==0)
		cout<<"Point_error"<<endl;
	else{
		op_array[0]= Point((msg->data.at(msg_size-2)+msg->data.at(0))/2,(msg->data.at(msg_size-1)+msg->data.at(1))/2);
		op_array[1]= Point((msg->data.at(msg_size-2)-msg->data.at(0)),(msg->data.at(msg_size-1)-msg->data.at(1)));
		op_array[2]= Point(-(msg->data.at(msg_size-1)-msg->data.at(1))/3.5,(msg->data.at(msg_size-2)-msg->data.at(0))/3.5);
	}
	
}
void ImageConverter::depthCb(const sensor_msgs::ImageConstPtr& msg){
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
     }
     catch (cv_bridge::Exception& e)
      {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }
	cv_ptr->image.convertTo(origin, CV_32F);
	non_filter = origin.clone();
	filtered = origin.clone();
	filtered = image_filter(filtered,30,mean_filter);
	find_min_max(filtered,0.9,1.1,min_max);
	find_max_histogram(filtered,min_max,histogram);
	filtered = filter_ROI(filtered,histogram[1],histogram[0],2.2);
	filtered = blur(origin,2);
	if(op_flag){
		if(depth_count==30){
			op_flag = false;
			depth_count=0;
			op_zaxis(origin,op_array,axis_vector);
			test[0] = msg->data.at(0);
			test[1] = msg->data.at(1);
			test[2] = msg->data.at(2);
			test[3] = msg->data.at(3);
			test[4] = msg->data.at(4);
			test[5] = msg->data.at(5);
		}
		else
			depth_count++;
	}
	filtered.convertTo(filtered, CV_8U);
	non_filter.convertTo(non_filter, CV_8U);
	cvtColor(filtered, filtered, COLOR_GRAY2BGR);
	cvtColor(non_filter, non_filter, COLOR_GRAY2BGR);
	circle(filtered, Point(test[0], test[1]), 2, Scalar(0, 255, 0), 17);
	circle(filtered, Point(test[2], test[3]), 2, Scalar(0, 255, 0), 17);
	circle(filtered, Point(test[4], test[5]), 2, Scalar(0, 255, 0), 17);
	circle(filtered, op_array[0], 2, Scalar(50, 50, 50), 5);
	circle(filtered, op_array[0]+op_array[1], 2, Scalar(0, 0, 255), 5);
	circle(filtered, op_array[0]-op_array[1], 2, Scalar(0, 0, 255), 5);
	circle(filtered, op_array[0]+op_array[2], 2, Scalar(255, 0, 0), 5);
	circle(filtered, op_array[0]-op_array[2], 2, Scalar(255, 0, 0), 5);
	imshow("depth",filtered);
	imshow("non_filter",non_filter);
	waitKey(1);

   }
void ImageConverter::op_zaxis(Mat &input_image,Point op_array[3] ,float output[3]){
	float vector_a[3] ,vector_b[3];
	vector_a[0] = (sqrt(op_array[1].x*op_array[1].x + op_array[1].y*op_array[1].y))*2;
	vector_a[1] = 0;
	vector_a[2] = ((float*)input_image.data)[(op_array[0].y+op_array[1].y)*input_image.cols+op_array[0].x+op_array[1].x] - ((float*)input_image.data)[(op_array[0].y-op_array[1].y)*input_image.cols+op_array[0].x-op_array[1].x] ;

	vector_b[0] = 0;
	vector_b[1] = (sqrt(op_array[2].x*op_array[2].x + op_array[2].y*op_array[2].y))*2;
	vector_b[2] = ((float*)input_image.data)[(op_array[0].y+op_array[2].y)*input_image.cols+op_array[0].x+op_array[2].x] - ((float*)input_image.data)[(op_array[0].y-op_array[2].y)*input_image.cols+op_array[0].x-op_array[2].x] ;
	output[0] = vector_b[1]*vector_a[2] - vector_b[2]*vector_a[1];
	output[1] = vector_b[2]*vector_a[0] - vector_b[0]*vector_a[2];
	output[2] = vector_b[0]*vector_a[1] - vector_b[1]*vector_a[0];
	float norm = sqrt(output[0]*output[0] + output[1]*output[1] +output[2]*output[2]);
	output[0] /= norm;
	output[1] /= norm;
	output[2] /= norm;
	cout<<"==================================="<<endl<<endl;
	cout<<"-----------depth---------------"<<endl;
	cout<<((float*)input_image.data)[(op_array[0].y+op_array[1].y)*input_image.cols+op_array[0].x+op_array[1].x]<<"\t"<<((float*)input_image.data)[(op_array[0].y-op_array[1].y)*input_image.cols+op_array[0].x-op_array[1].x]<<"\t"
	<<((float*)input_image.data)[(op_array[0].y+op_array[2].y)*input_image.cols+op_array[0].x+op_array[2].x]<<"\t"<<((float*)input_image.data)[(op_array[0].y-op_array[2].y)*input_image.cols+op_array[0].x-op_array[2].x]<<endl<<endl;
	cout<<"-----------vector--------------"<<endl;
	cout<< vector_a[0] << "\t"<< vector_a[1] << "\t"<< vector_a[2] <<endl;
	cout<< vector_b[0] << "\t"<< vector_b[1] << "\t"<< vector_b[2] <<endl<<endl;
	cout<<"-----------result--------------"<<endl<<endl<<endl;
	cout<< output[0] << "\t"<< output[1] << "\t"<< output[2] <<endl;
}
Mat ImageConverter::image_filter(Mat &input_image, int filter_size,Mat* temp_image){
	Mat output_image = Mat::zeros(Size(640, 480), CV_32F);
	int index =0;
	
	temp_image[index] = input_image;
	index = (++index)%filter_size;
	for(int i =0;i<filter_size;i++){
		output_image += temp_image[i];
	}
	output_image /= filter_size;
	return output_image;
}
void ImageConverter::find_min_max(Mat &input_image,float min_offset,float max_offset,float output[2]){

	float mean = find_mean(input_image,0);
	output[0] = output[1]  = mean;
	for(int i=0;i<input_image.rows;i++){
		for(int j =0;j<input_image.cols;j++){
			if(((float*)input_image.data)[i*input_image.cols+j]<output[0] && ((float*)input_image.data)[i*input_image.cols+j] >= mean*min_offset)
				output[0] = ((float*)input_image.data)[i*input_image.cols+j];
			if(((float*)input_image.data)[i*input_image.cols+j]>output[1] && ((float*)input_image.data)[i*input_image.cols+j] <= mean*max_offset)
				output[1] = ((float*)input_image.data)[i*input_image.cols+j];
			if(((float*)input_image.data)[i*input_image.cols+j]<= 80)
				((float*)input_image.data)[i*input_image.cols+j] = 1000;
		}
	}
}
float ImageConverter::find_mean(Mat &input_image,int offset){
	float output = 0;
	int count =0;
	for(int i=offset;i<input_image.rows-offset;i++)
		for(int j =offset;j<input_image.cols-offset;j++){
			if(((float*)input_image.data)[i*input_image.cols+j] >=100){
				output += ((float*)input_image.data)[i*input_image.cols+j];
				count++;
			}
		}
	output /= count;
	return output;
}

void ImageConverter::find_max_histogram(Mat &input_image,float min_max[2],float output[2]){

	float histogram_range = (min_max[1] - min_max[0])/5;
	int histogram[6] {};
	for(int i=0;i<input_image.rows;i++)
	{
		for(int j=0;j<input_image.cols;j++)
		{
			if(((float*)input_image.data)[i*input_image.cols+j]/histogram_range ==0)
				histogram[0]++;
			else if(((float*)input_image.data)[i*input_image.cols+j]/histogram_range ==1)
				histogram[1]++;
			else if(((float*)input_image.data)[i*input_image.cols+j]/histogram_range ==2)
				histogram[2]++;
			else if(((float*)input_image.data)[i*input_image.cols+j]/histogram_range ==3)
				histogram[3]++;
			else if(((float*)input_image.data)[i*input_image.cols+j]/histogram_range ==4)
				histogram[4]++;
			else
				histogram[5]++;
		}
	}
	int max_index =0;
	int max = histogram[0];
	for(int i =0;i<6;i++){
		if(histogram[i]>max){
			max = histogram[i];
			max_index = i;
		}
	}
	output[0] = histogram_range;
	output[1] = histogram_range*max_index + min_max[0];

}
Mat ImageConverter::filter_ROI(Mat &input_image, float cut,float cut_err,float offset){
	Mat temp = Mat::zeros(Size(640, 480), CV_32F);
	for(int i=0;i<input_image.rows;i++)
	{
		for(int j=0;j<input_image.cols;j++)
		{
			if(((float*)input_image.data)[i*input_image.cols+j]>=(cut - cut_err*offset))
				((float*)temp.data)[i*input_image.cols+j] =  255;
			else if(((float*)input_image.data)[i*input_image.cols+j]<= cut/2)
				((float*)temp.data)[i*input_image.cols+j] =  255;
			else
				((float*)temp.data)[i*input_image.cols+j] = 255 - (cut  - cut_err*offset - ((float*)input_image.data)[i*input_image.cols+j])*7.8;
		}
	}
	return temp;
}

Mat ImageConverter::blur(Mat &input_image,int blur_size){
	Mat temp = Mat::zeros(Size(640, 480), CV_32F);
	for(int i=0;i<input_image.rows;i++)
	{
		for(int j=0;j<input_image.cols;j++)
		{
			int blur_i =i - blur_size/2;
			int blur_j = j - blur_size/2;
			int i_max = i + blur_size/2;
			int j_max = j + blur_size/2;
			int mean_size = 0;
			if(blur_i < 0)
				blur_i =0;
			if(blur_j < 0)
				blur_j =0;
			if(i_max>480)
				i_max = 480;
			if(j_max>640)
				j_max = 640;
			for(int i_b = blur_i;i_b <i_max;i_b++)
				for(int j_b = blur_j;j_b <j_max;j_b++){
					((float*)temp.data)[i*input_image.cols+j] += ((float*)input_image.data)[i_b*input_image.cols+j_b];
					mean_size++;
				}
			((float*)temp.data)[i*input_image.cols+j] /= mean_size;
		}
	}
	return temp;
}
