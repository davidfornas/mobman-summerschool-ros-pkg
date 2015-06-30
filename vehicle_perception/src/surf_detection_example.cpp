#include <vehicle_perception/surf_detector.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "surf_detection");
  ros::NodeHandle nh;

  std::string image_topic("/phone1/camera/image"), debug_image("/debug"), error_topic("/error"), template_file_path("clip.jpg");
  bool display_image;

  nh.getParam("image_topic", image_topic);
  nh.getParam("debug_image", debug_image);
  nh.getParam("error_topic", error_topic);
  nh.getParam("template_file_path", template_file_path);
  nh.param("display_image", display_image, false);

  SurfDetector sd(image_topic, debug_image, error_topic, template_file_path);
  sd.setDisplay(display_image);
  ros::spin();
  return 0;

}

