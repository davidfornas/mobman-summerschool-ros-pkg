#ifndef _SURFDETECTOR_H
#define _SURFDETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class SurfDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  //Previous valid homography, used to compared with current error-wise.
  cv::Mat H_;

public:
  SurfDetector(std::string input_topic, std::string output_topic, std::string template_path) :
      it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe(input_topic, 1, &SurfDetector::imageCb, this, hints);
    image_pub_ = it_.advertise(output_topic, 1);
    //H_ = cv::Mat::zeros(3, 3, );
    cv::namedWindow(OPENCV_WINDOW);
    setTrackingObject(template_path);
  }

  ~SurfDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  cv::Mat getHomography()
  {
    return H_;
  }

  void setTrackingObject(std::string template_path);

private:

  //Object template features
  cv::Mat descriptors_object_;
  std::vector<cv::KeyPoint> keypoints_object_;
  cv::Mat img_object_;

  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  bool isInitialized()
  {
    return H_.size().height != 0;
  }

  void setHomography(cv::Mat H)
  {
    H_ = H;
  }

};

#endif
