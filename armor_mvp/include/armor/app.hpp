// Application: config loading, main loop
#pragma once

#include <string>
#include <optional>
#include <opencv2/core.hpp>

#include "armor/capture.hpp"
#include "armor/detector.hpp"
#include "armor/tracker.hpp"
#include "armor/visualizer.hpp"

namespace armor {

struct InputParams {
  int device_id = 0;
  std::string video = "";
  int width = 1280;
  int height = 720;
  int fps = 60;
};

struct AppParams {
  InputParams input;
  DetectorParams det;
  TrackerParams trk;
  UiParams ui;
};

  class App {
 public:
  App() = default;
  bool load_config(const std::string& path);
  void override_video(const std::string& video_path) { params_.input.video = video_path; }
  void set_headless(bool headless) { params_.ui.window = headless ? std::string("") : params_.ui.window; }
  void set_save_video(const std::string& path) { params_.ui.save_video = path; }
  void set_dump_path(const std::string& path) { dump_path_ = path; }
  int run();

 private:
  static int read_int(const cv::FileNode& n, const std::string& k, int d);
  static float read_float(const cv::FileNode& n, const std::string& k, float d);
  static std::string read_string(const cv::FileNode& n, const std::string& k, const std::string& d);
  static bool read_bool(const cv::FileNode& n, const std::string& k, bool d);

  AppParams params_;
  std::string dump_path_;
};

}  // namespace armor
