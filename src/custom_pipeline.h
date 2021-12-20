#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "pipeline/abstract_pipeline.h"

namespace custom {

class CustomPipeline : public deadeye::AbstractPipeline {
 public:
  CustomPipeline(int inum, std::string name);

  void Configure(const deadeye::CaptureConfig& config) override;

  std::unique_ptr<deadeye::TargetData> ProcessContours(
      deadeye::Contours const& contours) final;

 protected:
  std::string ToString() const final;

 private:
  const std::vector<cv::Point3d> hex_model{
      cv::Point3d(-0.498475, 0.4318, 0.0),
      cv::Point3d(-0.2492375, 0.0, 0.0),
      cv::Point3d(0.2492375, 0.0, 0.0),
      cv::Point3d(0.498475, 0.4318, 0.0),
    //   cv::Point3d(0.439816, 0.4318, 0.0),
      cv::Point3d(0.2199081073, 0.0508, 0.0),
      cv::Point3d(-0.2199081073, 0.0508, 0.0)};

  double focal = 640.0 * (7.5/1e3 /7.14e-3);
  const cv::Mat cam_mtx = (cv::Mat_<double>(3, 3) << focal, 0.0, 320.0, 0.0,
                           focal, 180.0, 0.0, 0.0, 1.0);
  const cv::Mat dist_coeffs =
      (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
  cv::Point2f center2f_;
  std::string capture_type_{"unknown"};
};

}  // namespace custom
