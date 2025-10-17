// Minimal types for armor MVP
#pragma once

#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace armor {

struct Detection {
  cv::Rect2f bbox;                          // x, y, w, h
  std::array<cv::Point2f, 4> corners;       // TL, TR, BR, BL
  float score = 0.f;                         // 0..1
};

struct Track {
  int id = -1;
  cv::Rect2f bbox_smooth;
  std::array<cv::Point2f, 4> corners_smooth{};
  float score = 0.f;
  int age = 0;
  int missed = 0;
};

struct FrameResult {
  std::vector<Track> tracks;
  double fps = 0.0;
};

inline float iou(const cv::Rect2f& a, const cv::Rect2f& b) {
  const float x1 = std::max(a.x, b.x);
  const float y1 = std::max(a.y, b.y);
  const float x2 = std::min(a.x + a.width, b.x + b.width);
  const float y2 = std::min(a.y + a.height, b.y + b.height);
  const float w = std::max(0.0f, x2 - x1);
  const float h = std::max(0.0f, y2 - y1);
  const float inter = w * h;
  const float uni = a.area() + b.area() - inter + 1e-6f;
  return inter / uni;
}

} // namespace armor

