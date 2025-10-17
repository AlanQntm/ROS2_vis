#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "armor_vision/types.hpp"

namespace armor_vision {

struct DetectorParams {
  std::string method = "adaptive"; // adaptive | otsu | auto
  int block_size = 21;
  int C = 5;
  int morph_ksize = 3;
  float ratio_min = 2.0f;
  float ratio_max = 3.2f;
  int area_min_px = 800;
  float rectangularity_min = 0.75f;
  int max_candidates = 6;
};

struct TrackerParams {
  float iou_thresh = 0.2f;
  int max_missed = 8;
  float smooth_alpha = 0.25f;
};

class Detector {
 public:
  explicit Detector(const DetectorParams& p = {}): p_(p) {}
  std::vector<Detection> detect(const cv::Mat& bgr);
 private:
  static void sort_corners_tl_tr_br_bl(std::array<cv::Point2f,4>& pts);
  static float aspect_ratio_from_rotated(const cv::RotatedRect& rr);
  static float rectangularity(const std::vector<cv::Point>& contour, const cv::RotatedRect& rr);
  DetectorParams p_;
};

class Tracker {
 public:
  explicit Tracker(const TrackerParams& p = {}): p_(p) {}
  std::vector<Track> update(const std::vector<Detection>& dets);
 private:
  TrackerParams p_;
  std::vector<Track> tracks_;
  int next_id_ = 0;
};

class ArmorVisionNode : public rclcpp::Node {
 public:
  ArmorVisionNode();

 private:
  void declare_and_get_params();
  bool open_input();
  void on_timer();
  void draw_overlay(cv::Mat& img, const std::vector<Track>& tracks, double fps);
  rcl_interfaces::msg::SetParametersResult on_params(const std::vector<rclcpp::Parameter>& params);

  // Params
  int device_id_ = 0;
  std::string video_path_;
  int width_ = 1280;
  int height_ = 720;
  int fps_expect_ = 60;
  std::string camera_info_path_;
  DetectorParams detp_;
  TrackerParams trkp_;
  bool show_fps_ = true;

  // Runtime
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher pub_debug_;
  rclcpp::Publisher<armor_vision::msg::ArmorDetections>::SharedPtr pub_det_; 
  std::shared_ptr<image_transport::ImageTransport> it_;
  Detector detector_;
  Tracker tracker_;
  rclcpp::Time last_stamp_;
  double fps_smooth_ = 0.0;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
};

} // namespace armor_vision
