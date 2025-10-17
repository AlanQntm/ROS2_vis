#include "armor_vision/armor_vision_node.hpp"

namespace armor_vision {

std::vector<Track> Tracker::update(const std::vector<Detection>& dets) {
  const int N = static_cast<int>(tracks_.size());
  const int M = static_cast<int>(dets.size());
  std::vector<std::vector<float>> iou_mat(N, std::vector<float>(M, 0.f));
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < M; ++j) {
      iou_mat[i][j] = iou(tracks_[i].bbox_smooth, dets[j].bbox);
    }
  }

  std::vector<int> det_assigned(M, -1), trk_assigned(N, -1);
  while (true) {
    float best = p_.iou_thresh;
    int bi = -1, bj = -1;
    for (int i = 0; i < N; ++i) {
      if (trk_assigned[i] != -1) continue;
      for (int j = 0; j < M; ++j) {
        if (det_assigned[j] != -1) continue;
        if (iou_mat[i][j] > best) { best = iou_mat[i][j]; bi = i; bj = j; }
      }
    }
    if (bi == -1) break;
    trk_assigned[bi] = bj;
    det_assigned[bj] = bi;
  }

  for (int i = 0; i < N; ++i) {
    if (trk_assigned[i] != -1) {
      const Detection& d = dets[trk_assigned[i]];
      auto& t = tracks_[i];
      t.age++; t.missed = 0; t.score = d.score;
      float a = p_.smooth_alpha;
      t.bbox_smooth.x = a * d.bbox.x + (1 - a) * t.bbox_smooth.x;
      t.bbox_smooth.y = a * d.bbox.y + (1 - a) * t.bbox_smooth.y;
      t.bbox_smooth.width = a * d.bbox.width + (1 - a) * t.bbox_smooth.width;
      t.bbox_smooth.height = a * d.bbox.height + (1 - a) * t.bbox_smooth.height;
      for (int k = 0; k < 4; ++k) {
        t.corners_smooth[k].x = a * d.corners[k].x + (1 - a) * t.corners_smooth[k].x;
        t.corners_smooth[k].y = a * d.corners[k].y + (1 - a) * t.corners_smooth[k].y;
      }
    } else {
      tracks_[i].age++; tracks_[i].missed++;
    }
  }

  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& t){ return t.missed > p_.max_missed; }), tracks_.end());

  for (int j = 0; j < M; ++j) {
    if (det_assigned[j] == -1) {
      Track t; t.id = next_id_++; t.bbox_smooth = dets[j].bbox; t.corners_smooth = dets[j].corners; t.score = dets[j].score; t.age = 1; t.missed = 0;
      tracks_.push_back(t);
    }
  }
  return tracks_;
}

} // namespace armor_vision

