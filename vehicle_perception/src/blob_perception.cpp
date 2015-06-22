#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() :
      it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed

    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe("/phone1/camera/image", 1, &ImageConverter::imageCb, this, hints);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat im0, im;
    cv::cvtColor(cv_ptr->image, im0, CV_RGB2GRAY);
    cv::resize(im0, im, cv::Size(640, 480), 0, 0, cv::INTER_LANCZOS4);

    // Set up the detector with default parameters.
    cv::SimpleBlobDetector detector;

    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;
    detector.detect(im, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::Mat im_with_keypoints;
    cv::drawKeypoints(im, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Show blobs
    cv::imshow("keypoints", im_with_keypoints);
    cv::waitKey(0);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;

}

