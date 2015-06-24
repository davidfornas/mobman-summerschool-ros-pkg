#include <vehicle_perception/surf_detector.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace cv;

void SurfDetector::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

  cv::Mat im0, img_scene;
  cv::cvtColor(cv_ptr->image, im0, CV_RGB2GRAY);
  //IMAGE RESIZE
  cv::resize(im0, img_scene, cv::Size(), 0.5, 0.5, cv::INTER_LANCZOS4);

  if ( !img_scene.data)
  {
    ROS_ERROR(" --(!) Error reading scene images ");
    return;
  }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector(minHessian);

  std::vector<KeyPoint> keypoints_scene;

  detector.detect(img_scene, keypoints_scene);

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_scene;

  extractor.compute(img_scene, keypoints_scene, descriptors_scene);

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector<DMatch> matches;
  matcher.match(descriptors_object_, descriptors_scene, matches);

  double max_dist = 0, min_dist = 100, mean_dist = 0;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < descriptors_object_.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist)
      min_dist = dist;
    if (dist > max_dist)
      max_dist = dist;
    mean_dist += dist;
  }

  mean_dist /= descriptors_object_.rows;
  ROS_INFO("-- Max dist : %f | Min dist : %f | Mean dist : %f ", max_dist, min_dist, mean_dist);

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector<DMatch> good_matches;

  for (int i = 0; i < descriptors_object_.rows; i++)
  {
    if (matches[i].distance < 3 * min_dist)
    {
      good_matches.push_back(matches[i]);
    }
  }

  Mat img_matches;
  drawMatches(img_object_, keypoints_object_, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
              Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  //-- Localize the object
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    //-- Get the keypoints from the good matches
    obj.push_back(keypoints_object_[good_matches[i].queryIdx].pt);
    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
  }
  std::cout << "Good matches: " << good_matches.size() << std::endl;

  cv::Mat mask;
  Mat H = findHomography(obj, scene, CV_RANSAC, 3, mask);
  std::cout << "Homography: " << std::endl << H << std::endl;

  //Compute errors
  double rotation_error=0;
  if (isInitialized())
  {
    rotation_error= cv::norm(H(cv::Rect_<int>(0, 0, 2, 2)), H_(cv::Rect_<int>(0, 0, 2, 2)));
    std::cout << "Homography rotation error: " << rotation_error << std::endl;
    //Translation error is mostly irrelevant
    std::cout << "Homography translation error: "
        << cv::norm(H(cv::Rect_<int>(2, 0, 1, 2)), H_(cv::Rect_<int>(2, 0, 1, 2))) << std::endl;
  }

  //Count inliers to check for success
  int num_inliers = 0;
  for (int i = 0; i < good_matches.size(); i++)
    if ((unsigned int)mask.at<uchar>(i))
      num_inliers++;
  double inliers_ratio = num_inliers * 100.0 / good_matches.size();
  std::cout << "Number of inliers: " << num_inliers << "Percentage: " << inliers_ratio << std::endl;

  //If the homography is considered valid, store it
  if((inliers_ratio>40 && rotation_error < 1.5) || (inliers_ratio>20 && rotation_error < 0.8) || (inliers_ratio>10 && rotation_error < 0.3)){
    ROS_INFO("VALID H");
    setHomography(H);
  }else ROS_ERROR("H IS NOT GOOD ENOUGH");



  // store match or keypoints or points somewhere where you can access them later

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0, 0);
  obj_corners[1] = cvPoint(img_object_.cols, 0);
  obj_corners[2] = cvPoint(img_object_.cols, img_object_.rows);
  obj_corners[3] = cvPoint(0, img_object_.rows);
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform(obj_corners, scene_corners, H);

  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line(img_matches, scene_corners[0] + Point2f(img_object_.cols, 0), scene_corners[1] + Point2f(img_object_.cols, 0),
       Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[1] + Point2f(img_object_.cols, 0), scene_corners[2] + Point2f(img_object_.cols, 0),
       Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[2] + Point2f(img_object_.cols, 0), scene_corners[3] + Point2f(img_object_.cols, 0),
       Scalar(0, 255, 0), 4);
  line(img_matches, scene_corners[3] + Point2f(img_object_.cols, 0), scene_corners[0] + Point2f(img_object_.cols, 0),
       Scalar(0, 255, 0), 4);

  //-- Show detected matches
  imshow("Good Matches & Object detection", img_matches);
  waitKey(0);

  // Output modified video stream
  cv_ptr->image = img_matches;
  image_pub_.publish(cv_ptr->toImageMsg());

}

void SurfDetector::setTrackingObject(string template_path)
{

  img_object_ = imread(template_path, CV_LOAD_IMAGE_GRAYSCALE);
  if (!img_object_.data)
  {
    ROS_ERROR(" --(!) Error reading template image ");
    return;
  }

  int minHessian = 400;
  SurfFeatureDetector detector(minHessian);
  detector.detect(img_object_, keypoints_object_);

  SurfDescriptorExtractor extractor;
  extractor.compute(img_object_, keypoints_object_, descriptors_object_);

}
