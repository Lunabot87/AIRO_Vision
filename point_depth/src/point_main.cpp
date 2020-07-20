#include "point_depth.cpp"
 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "point_depth_node");
   PointCloud pc;
   ros::spin();
   return 0;
}