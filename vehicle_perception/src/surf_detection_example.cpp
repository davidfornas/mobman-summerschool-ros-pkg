#include <vehicle_perception/surf_detector.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "surf_detection_" + std::string(argv[1]));
  SurfDetector sd("/phone1/camera/image", argv[1], argv[1] + std::string("error"), argv[2]);
  sd.setDisplay(false);
  ros::spin();
  return 0;

}

