// Detector implementation
#include "armor/detector.hpp"

#include <cmath>

namespace armor {

static inline int make_odd(int v) { return (v % 2 == 0) ? v + 1 : v; }

float Detector::aspect_ratio_from_rotated(const cv::RotatedRect& rr) {
  float w = rr.size.width;
  float h = rr.size.height;
  if (w < 1 || h < 1) return 0.0f;
  float r = (w > h) ? (w / h) : (h / w);
  return r;
}

float Detector::rectangularity(const std::vector<cv::Point>& contour, const cv::RotatedRect& rr) {
  double area_c = cv::contourArea(contour);
  double area_r = rr.size.area();
  if (area_r <= 0.0) return 0.0f;
  return static_cast<float>(area_c / area_r);
}

void Detector::sort_corners_tl_tr_br_bl(std::array<cv::Point2f,4>& pts) {
  // Separate top vs bottom by y
  std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b){ return a.y < b.y; });
  std::array<cv::Point2f,2> top = {pts[0], pts[1]};
  std::array<cv::Point2f,2> bottom = {pts[2], pts[3]};
  if (top[0].x > top[1].x) std::swap(top[0], top[1]);
  if (bottom[0].x > bottom[1].x) std::swap(bottom[0], bottom[1]);
  pts = { top[0], top[1], bottom[1], bottom[0] }; // TL, TR, BR, BL
}

std::vector<Detection> Detector::detect(const cv::Mat& bgr) {
  std::vector<Detection> results;
  if (bgr.empty()) return results;

  cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat bin;

  std::string m = p_.method;
  for (auto& c : m) c = static_cast<char>(std::tolower(c));

  if (m == "auto") {
    cv::Scalar meanv = cv::mean(gray);
    m = (meanv[0] < 80.0) ? std::string("adaptive") : std::string("otsu");
  }

  if (m == "adaptive") {
    int bs = std::max(3, make_odd(p_.block_size));
    cv::adaptiveThreshold(gray, bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, bs, p_.C);
  } else {
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  }

  if (p_.morph_ksize > 1) {
    int k = std::max(1, p_.morph_ksize);
    if (k % 2 == 0) k += 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {k, k});
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  struct Cand { Detection d; float score; };
  std::vector<Cand> cands;
  cands.reserve(contours.size());

  constexpr float target_ratio = 2.41f;

  for (const auto& c : contours) {
    double area = cv::contourArea(c);
    if (area < p_.area_min_px) continue;
    cv::RotatedRect rr = cv::minAreaRect(c);
    float recty = rectangularity(c, rr);
    if (recty < p_.rectangularity_min) continue;
    float ratio = aspect_ratio_from_rotated(rr);
    if (ratio < p_.ratio_min || ratio > p_.ratio_max) continue;

    // Use rotated rect vertices as corners
    cv::Point2f pts4[4];
    rr.points(pts4);
    std::array<cv::Point2f,4> corners = {pts4[0], pts4[1], pts4[2], pts4[3]};
    sort_corners_tl_tr_br_bl(corners);

    // Basic score: rectangularity * ratio closeness (0..1)
    float ratio_score = std::exp(-std::abs(ratio - target_ratio)); // in (0,1]
    float score = std::clamp(recty * ratio_score, 0.0f, 1.0f);

    Detection d;
    d.bbox = rr.boundingRect2f();
    d.corners = corners;
    d.score = score;
    cands.push_back({d, score});
  }

  // Sort by score and keep top-K
  std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b){ return a.score > b.score; });
  int keep = std::min((int)cands.size(), std::max(1, p_.max_candidates));
  results.reserve(keep);
  for (int i = 0; i < keep; ++i) results.push_back(cands[i].d);

  return results;
}

} // namespace armor

