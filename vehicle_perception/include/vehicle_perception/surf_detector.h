#ifndef _SURFDETECTOR_H
#define _SURFDETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>

#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class SurfDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // Target error
  ros::Publisher error_x_pub_, error_y_pub_;

  //Previous valid homography, used to compared with current error-wise.
  cv::Mat H_;

  //Show window display
  bool display_;

public:
  SurfDetector(std::string input_topic, std::string output_topic, std::string error_topic, std::string template_path) :
      it_(nh_)
  {
    // Subscribe to input video feed and publish output debug video feed
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe(input_topic, 1, &SurfDetector::imageCb, this, hints);
    image_pub_ = it_.advertise(output_topic, 1);

    //Publish image error
    error_x_pub_ = nh_.advertise<std_msgs::Int32>(error_topic+std::string("_x"), 1);
    error_y_pub_ = nh_.advertise<std_msgs::Int32>(error_topic+std::string("_y"), 1);

    //H_ = cv::Mat::zeros(3, 3, );
    cv::namedWindow(OPENCV_WINDOW);
    setTrackingObject(template_path);

    display_ = false;
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

  void setDisplay( bool display ){ display_ = display; }

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
