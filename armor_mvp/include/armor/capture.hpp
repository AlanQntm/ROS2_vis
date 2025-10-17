// Capture wrapper for camera or video file
#pragma once

#include <opencv2/videoio.hpp>
#include <string>

namespace armor {

class Capture {
 public:
  Capture() = default;
  ~Capture() { close(); }

  bool open(int device_id, int width, int height, int fps) {
    close();
    cap_.open(device_id, cv::CAP_V4L2);
    if (!cap_.isOpened()) return false;
    if (width > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    if (height > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    if (fps > 0) cap_.set(cv::CAP_PROP_FPS, fps);
    return true;
  }

  bool open(const std::string& video_path) {
    close();
    cap_.open(video_path);
    return cap_.isOpened();
  }

  bool isOpened() const { return cap_.isOpened(); }

  bool read(cv::Mat& frame) { return cap_.read(frame); }

  void close() {
    if (cap_.isOpened()) cap_.release();
  }

  double fps() const { return cap_.get(cv::CAP_PROP_FPS); }

 private:
  cv::VideoCapture cap_;
};

}  // namespace armor

