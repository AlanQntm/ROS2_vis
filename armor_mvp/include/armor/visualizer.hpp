// Visualization helper for drawing detections/tracks
#pragma once

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "armor/types.hpp"

namespace armor {

struct UiParams {
  bool show_fps = true;
  std::string window = "Armor MVP";
  std::string save_video = ""; // optional path
};

class Visualizer {
 public:
  Visualizer(const UiParams& ui, double init_fps = 60.0)
      : ui_(ui) {
    if (!ui_.window.empty()) {
      try {
        cv::namedWindow(ui_.window, cv::WINDOW_NORMAL);
      } catch (const cv::Exception&) {
        // Headless fallback: disable window drawing
        ui_.window.clear();
      }
    }
  }

  ~Visualizer() {
    if (!ui_.window.empty()) cv::destroyWindow(ui_.window);
    if (writer_.isOpened()) writer_.release();
  }

  void ensure_writer(const cv::Size& size, double fps) {
    if (ui_.save_video.empty() || writer_.isOpened()) return;
    int fourcc = cv::VideoWriter::fourcc('a','v','c','1'); // mp4/h264 if available, fallback next
    if (!writer_.open(ui_.save_video, fourcc, fps > 0 ? fps : 30.0, size)) {
      // fallback to mp4v
      writer_.open(ui_.save_video, cv::VideoWriter::fourcc('m','p','4','v'), fps > 0 ? fps : 30.0, size);
    }
  }

  cv::Mat draw(const cv::Mat& frame, const FrameResult& result) {
    cv::Mat out;
    frame.copyTo(out);
    for (const auto& t : result.tracks) {
      // bbox
      cv::rectangle(out, t.bbox_smooth, cv::Scalar(0, 255, 0), 2);
      // corners TL-TR-BR-BL with TL highlighted
      const auto& cs = t.corners_smooth;
      for (int i = 0; i < 4; ++i) {
        cv::circle(out, cs[i], 2, (i==0?cv::Scalar(0,0,255):cv::Scalar(255,0,0)), -1);
        cv::line(out, cs[i], cs[(i+1)%4], cv::Scalar(255, 200, 0), 1);
      }
      char buf[128];
      std::snprintf(buf, sizeof(buf), "id:%d s:%.2f", t.id, t.score);
      cv::putText(out, buf, t.bbox_smooth.tl() + cv::Point2f(0, -5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
    }
    if (ui_.show_fps) {
      char f[64];
      std::snprintf(f, sizeof(f), "FPS: %.1f", result.fps);
      cv::putText(out, f, {10, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 220, 50), 2);
    }
    if (!ui_.window.empty()) {
      cv::imshow(ui_.window, out);
    }
    if (writer_.isOpened()) writer_.write(out);
    return out;
  }

 private:
  UiParams ui_;
  cv::VideoWriter writer_;
};

}  // namespace armor
