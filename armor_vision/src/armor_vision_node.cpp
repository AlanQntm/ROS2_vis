#include "armor_vision/armor_vision_node.hpp"

#include <chrono>
#include <algorithm>

#include "armor_vision/msg/armor_detections.hpp"
#include <geometry_msgs/msg/point32.hpp>

using namespace std::chrono_literals;

namespace armor_vision {

ArmorVisionNode::ArmorVisionNode()
: rclcpp::Node("armor_vision")
, detector_(detp_)
, tracker_(trkp_)
{
  declare_and_get_params();

  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  pub_debug_ = it_->advertise("/armor/debug_image", 1);
  pub_det_ = this->create_publisher<armor_vision::msg::ArmorDetections>("/armor/detections", 10);

  if (!open_input()) {
    RCLCPP_ERROR(get_logger(), "Failed to open input (device or video)");
  }

  last_stamp_ = now();
  timer_ = this->create_wall_timer(1ms, std::bind(&ArmorVisionNode::on_timer, this));

  // Enable parameter hot-reload
  cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ArmorVisionNode::on_params, this, std::placeholders::_1));
}

void ArmorVisionNode::declare_and_get_params() {
  // Input
  device_id_ = this->declare_parameter<int>("device_id", 0);
  video_path_ = this->declare_parameter<std::string>("video_path", "");
  width_ = this->declare_parameter<int>("width", 1280);
  height_ = this->declare_parameter<int>("height", 720);
  fps_expect_ = this->declare_parameter<int>("fps", 60);
  camera_info_path_ = this->declare_parameter<std::string>("camera_info_path", "config/camera_info.yaml");

  // Detector params
  detp_.method = this->declare_parameter<std::string>("bin.method", "adaptive");
  detp_.block_size = this->declare_parameter<int>("bin.block_size", 21);
  detp_.C = this->declare_parameter<int>("bin.C", 5);
  detp_.morph_ksize = this->declare_parameter<int>("morph.ksize", 3);
  detp_.ratio_min = this->declare_parameter<double>("ratio.min", 2.0);
  detp_.ratio_max = this->declare_parameter<double>("ratio.max", 3.2);
  detp_.area_min_px = this->declare_parameter<int>("area.min_px", 800);
  detp_.rectangularity_min = this->declare_parameter<double>("rectangularity.min", 0.75);
  detp_.max_candidates = this->declare_parameter<int>("candidate.max", 6);

  // Tracker params
  trkp_.iou_thresh = this->declare_parameter<double>("track.iou_thresh", 0.2);
  trkp_.max_missed = this->declare_parameter<int>("track.max_age", 8);
  trkp_.smooth_alpha = this->declare_parameter<double>("smooth.alpha", 0.25);

  show_fps_ = this->declare_parameter<bool>("show_fps", true);
}

bool ArmorVisionNode::open_input() {
  if (!video_path_.empty()) {
    cap_.open(video_path_);
  } else {
    cap_.open(device_id_, cv::CAP_V4L2);
    if (width_ > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    if (height_ > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    if (fps_expect_ > 0) cap_.set(cv::CAP_PROP_FPS, fps_expect_);
  }
  return cap_.isOpened();
}

void ArmorVisionNode::draw_overlay(cv::Mat& out, const std::vector<Track>& tracks, double fps) {
  for (const auto& t : tracks) {
    cv::rectangle(out, t.bbox_smooth, cv::Scalar(0, 255, 0), 2);
    const auto& cs = t.corners_smooth;
    for (int i = 0; i < 4; ++i) {
      cv::circle(out, cs[i], 2, (i==0?cv::Scalar(0,0,255):cv::Scalar(255,0,0)), -1);
      cv::line(out, cs[i], cs[(i+1)%4], cv::Scalar(255, 200, 0), 1);
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf), "id:%d s:%.2f", t.id, t.score);
    cv::putText(out, buf, t.bbox_smooth.tl() + cv::Point2f(0, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
  }
  if (show_fps_) {
    char f[64];
    std::snprintf(f, sizeof(f), "FPS: %.1f", fps);
    cv::putText(out, f, {10, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 220, 50), 2);
  }
}

void ArmorVisionNode::on_timer() {
  cv::Mat frame;
  if (!cap_.isOpened() || !cap_.read(frame) || frame.empty()) {
    return;
  }

  const rclcpp::Time stamp = now();
  const double dt = (stamp - last_stamp_).seconds();
  last_stamp_ = stamp;
  const double inst_fps = dt > 1e-6 ? (1.0 / dt) : 0.0;
  fps_smooth_ = 0.9 * fps_smooth_ + 0.1 * inst_fps;

  auto dets = detector_.detect(frame);
  auto tracks = tracker_.update(dets);

  // Publish detections
  armor_vision::msg::ArmorDetections msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "camera";
  msg.detections.reserve(tracks.size());
  for (const auto& t : tracks) {
    armor_vision::msg::ArmorDetection d;
    d.header = msg.header;
    d.score = t.score;
    d.track_id = t.id;
    d.bbox = { t.bbox_smooth.x, t.bbox_smooth.y, t.bbox_smooth.width, t.bbox_smooth.height };
    for (int k = 0; k < 4; ++k) {
      geometry_msgs::msg::Point32 p; p.x = t.corners_smooth[k].x; p.y = t.corners_smooth[k].y; p.z = 0.0f;
      d.corners[k] = p;
    }
    msg.detections.push_back(d);
  }
  pub_det_->publish(std::move(msg));

  // Debug image
  cv::Mat debug = frame.clone();
  draw_overlay(debug, tracks, fps_smooth_);
  std_msgs::msg::Header header; header.stamp = stamp; header.frame_id = "camera";
  auto img_msg = cv_bridge::CvImage(header, "bgr8", debug).toImageMsg();
  pub_debug_.publish(img_msg);
}

rcl_interfaces::msg::SetParametersResult ArmorVisionNode::on_params(const std::vector<rclcpp::Parameter>& params) {
  for (const auto& p : params) {
    const std::string& name = p.get_name();
    if (name == "bin.method") detp_.method = p.as_string();
    else if (name == "bin.block_size") detp_.block_size = p.as_int();
    else if (name == "bin.C") detp_.C = p.as_int();
    else if (name == "morph.ksize") detp_.morph_ksize = p.as_int();
    else if (name == "ratio.min") detp_.ratio_min = static_cast<float>(p.as_double());
    else if (name == "ratio.max") detp_.ratio_max = static_cast<float>(p.as_double());
    else if (name == "area.min_px") detp_.area_min_px = p.as_int();
    else if (name == "rectangularity.min") detp_.rectangularity_min = static_cast<float>(p.as_double());
    else if (name == "candidate.max") detp_.max_candidates = p.as_int();
    else if (name == "track.iou_thresh") trkp_.iou_thresh = static_cast<float>(p.as_double());
    else if (name == "track.max_age") trkp_.max_missed = p.as_int();
    else if (name == "smooth.alpha") trkp_.smooth_alpha = static_cast<float>(p.as_double());
    else if (name == "show_fps") show_fps_ = p.as_bool();
    // input params are not hot-updated here (require reopen capture)
  }
  // Recreate processing objects with new params
  detector_ = Detector(detp_);
  tracker_ = Tracker(trkp_);
  rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
}

} // namespace armor_vision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<armor_vision::ArmorVisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
