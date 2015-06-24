#include <vehicle_perception/surf_detector.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "surf_detection");
  SurfDetector sd("/phone1/camera/image", argv[1], argv[2]);
  ros::spin();
  return 0;

}


