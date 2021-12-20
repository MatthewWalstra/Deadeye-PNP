#pragma once
#include <nlohmann/json.hpp>
#include <opencv2/core/types.hpp>

#include "link/target_data.h"

using json = nlohmann::json;

namespace custom {
struct CustomTargetData : public deadeye::TargetData {
  cv::RotatedRect rect;
  cv::Point center;
  cv::Point2f rect_corners[4];
  cv::Mat tvec;
  cv::Mat rvec;
  std::vector<cv::Point2d> pnp_corners;
  CustomTargetData(std::string id, int sn, bool valid, cv::RotatedRect rect,
                   cv::Point center, cv::Mat tvec, cv::Mat rvec,
                   std::vector<cv::Point2d> pnp_corners);

  void DrawMarkers(cv::Mat& preview) const override;
  std::string Dump() const override;
  std::string ToString() const override;
};
}  // namespace custom
