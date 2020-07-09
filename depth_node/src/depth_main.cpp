#include "depth_node.cpp"

 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "depth_camera");
   ImageConverter ic;
   ros::spin();
   return 0;
}

