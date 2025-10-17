// App implementation
#include "armor/app.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace armor {

int App::read_int(const cv::FileNode& n, const std::string& k, int d) {
  if (n.empty()) return d;
  const cv::FileNode v = n[k];
  if (v.empty()) return d;
  int out = d; v >> out; return out;
}

float App::read_float(const cv::FileNode& n, const std::string& k, float d) {
  if (n.empty()) return d;
  const cv::FileNode v = n[k];
  if (v.empty()) return d;
  float out = d; v >> out; return out;
}

std::string App::read_string(const cv::FileNode& n, const std::string& k, const std::string& d) {
  if (n.empty()) return d;
  const cv::FileNode v = n[k];
  if (v.empty()) return d;
  std::string out = d; v >> out; return out;
}

bool App::read_bool(const cv::FileNode& n, const std::string& k, bool d) {
  if (n.empty()) return d;
  const cv::FileNode v = n[k];
  if (v.empty()) return d;
  int out = d ? 1 : 0; v >> out; return out != 0;
}

bool App::load_config(const std::string& path) {
  cv::FileStorage fs;
  try {
    fs.open(path, cv::FileStorage::READ);
  } catch (const cv::Exception& e) {
    std::cerr << "[App] Failed to open config (fallback to defaults): " << path << "\n" << e.what() << std::endl;
    return true;
  }
  if (!fs.isOpened()) {
    std::cerr << "[App] Config not found or invalid, using defaults: " << path << std::endl;
    return true;
  }

  cv::FileNode n_input = fs["input"];
  params_.input.device_id = read_int(n_input, "device_id", params_.input.device_id);
  params_.input.video = read_string(n_input, "video", params_.input.video);
  params_.input.width = read_int(n_input, "width", params_.input.width);
  params_.input.height = read_int(n_input, "height", params_.input.height);
  params_.input.fps = read_int(n_input, "fps", params_.input.fps);

  cv::FileNode n_bin = fs["bin"];
  params_.det.method = read_string(n_bin, "method", params_.det.method);
  params_.det.block_size = read_int(n_bin, "block_size", params_.det.block_size);
  params_.det.C = read_int(n_bin, "C", params_.det.C);

  cv::FileNode n_morph = fs["morph"];
  params_.det.morph_ksize = read_int(n_morph, "ksize", params_.det.morph_ksize);

  cv::FileNode n_filter = fs["filter"];
  params_.det.ratio_min = read_float(n_filter, "ratio_min", params_.det.ratio_min);
  params_.det.ratio_max = read_float(n_filter, "ratio_max", params_.det.ratio_max);
  params_.det.area_min_px = read_int(n_filter, "area_min_px", params_.det.area_min_px);
  params_.det.rectangularity_min = read_float(n_filter, "rectangularity_min", params_.det.rectangularity_min);

  cv::FileNode n_detector = fs["detector"];
  params_.det.max_candidates = read_int(n_detector, "max_candidates", params_.det.max_candidates);

  cv::FileNode n_tracker = fs["tracker"];
  params_.trk.iou_thresh = read_float(n_tracker, "iou_thresh", params_.trk.iou_thresh);
  params_.trk.max_missed = read_int(n_tracker, "max_missed", params_.trk.max_missed);
  params_.trk.smooth_alpha = read_float(n_tracker, "smooth_alpha", params_.trk.smooth_alpha);

  cv::FileNode n_ui = fs["ui"];
  params_.ui.show_fps = read_bool(n_ui, "show_fps", params_.ui.show_fps);
  params_.ui.window = read_string(n_ui, "window", params_.ui.window);
  params_.ui.save_video = read_string(n_ui, "save_video", params_.ui.save_video);

  return true;
}

int App::run() {
  Detector detector(params_.det);
  Tracker tracker(params_.trk);
  Visualizer viz(params_.ui);
  Capture cap;

  bool ok = false;
  if (!params_.input.video.empty()) {
    ok = cap.open(params_.input.video);
  } else {
    ok = cap.open(params_.input.device_id, params_.input.width, params_.input.height, params_.input.fps);
  }
  if (!ok) {
    std::cerr << "[App] Failed to open input." << std::endl;
    return 1;
  }

  cv::Mat frame;
  if (!cap.read(frame) || frame.empty()) {
    std::cerr << "[App] Failed to read first frame." << std::endl;
    return 2;
  }

  viz.ensure_writer(frame.size(), cap.fps());

  using clock = std::chrono::steady_clock;
  auto t_prev = clock::now();
  double fps = 0.0;
  int frame_idx = 0;

  std::ofstream dump;
  if (!dump_path_.empty()) {
    dump.open(dump_path_, std::ios::out | std::ios::trunc);
    if (!dump.is_open()) {
      std::cerr << "[App] Cannot open dump file: " << dump_path_ << std::endl;
    }
  }

  while (true) {
    if (!cap.read(frame) || frame.empty()) break;

    auto t0 = clock::now();
    std::vector<Detection> dets = detector.detect(frame);
    std::vector<Track> tracks = tracker.update(dets);
    auto t1 = clock::now();
    double dt = std::chrono::duration<double>(t1 - t_prev).count();
    t_prev = t1;
    // Smooth FPS
    double inst = dt > 1e-6 ? (1.0 / dt) : 0.0;
    fps = 0.9 * fps + 0.1 * inst;

    FrameResult fr{tracks, fps};
    viz.draw(frame, fr);

    if (dump.is_open()) {
      dump << std::fixed << std::setprecision(3);
      dump << "{\"frame\":" << frame_idx
           << ",\"fps\":" << fps
           << ",\"tracks\":[";
      for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& t = tracks[i];
        dump << "{\"id\":" << t.id
             << ",\"score\":" << t.score
             << ",\"bbox\":[" << t.bbox_smooth.x << "," << t.bbox_smooth.y << "," << t.bbox_smooth.width << "," << t.bbox_smooth.height << "]";
        dump << ",\"corners\":[";
        for (int k = 0; k < 4; ++k) {
          dump << "[" << t.corners_smooth[k].x << "," << t.corners_smooth[k].y << "]";
          if (k != 3) dump << ",";
        }
        dump << "]}";
        if (i + 1 != tracks.size()) dump << ",";
      }
      dump << "]}\n";
    }

    ++frame_idx;

    int key = cv::waitKey(1);
    if (key == 27 || key == 'q' || key == 'Q') break;
  }

  return 0;
}

} // namespace armor
