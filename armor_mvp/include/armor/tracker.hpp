// Simple IOU-based multi-object tracker with EMA smoothing
#pragma once

#include <vector>
#include <limits>
#include <algorithm>
#include <opencv2/core.hpp>

#include "armor/types.hpp"

namespace armor {

struct TrackerParams {
  float iou_thresh = 0.2f;
  int max_missed = 8;
  float smooth_alpha = 0.25f; // EMA alpha
};

class Tracker {
 public:
  explicit Tracker(const TrackerParams& p = {}) : p_(p) {}

  std::vector<Track> update(const std::vector<Detection>& dets);

 private:
  TrackerParams p_;
  std::vector<Track> tracks_;
  int next_id_ = 0;
};

}  // namespace armor

