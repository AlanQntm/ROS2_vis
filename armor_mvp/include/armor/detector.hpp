// Traditional vision detector producing quadrilateral candidates
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>
#include <array>

#include "armor/types.hpp"

namespace armor {

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

class Detector {
 public:
  explicit Detector(const DetectorParams& p = {}) : p_(p) {}

  std::vector<Detection> detect(const cv::Mat& bgr);

 private:
  static void sort_corners_tl_tr_br_bl(std::array<cv::Point2f,4>& pts);
  static float aspect_ratio_from_rotated(const cv::RotatedRect& rr);
  static float rectangularity(const std::vector<cv::Point>& contour, const cv::RotatedRect& rr);

  DetectorParams p_;
};

}  // namespace armor

