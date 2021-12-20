#include "custom_pipeline.h"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>

#include "custom_target_data.h"

using namespace deadeye;
using namespace custom;

CustomPipeline::CustomPipeline(int inum, std::string name)
    : AbstractPipeline{inum, name} {
  spdlog::set_level(spdlog::level::debug);
}

void CustomPipeline::Configure(const CaptureConfig& config) {
  center2f_ = static_cast<cv::Point2f>(config.Size() / 2);
  capture_type_ = config.PipelineType();
}

// Target is center of contour bounding box.
std::unique_ptr<TargetData> CustomPipeline::ProcessContours(
    Contours const& contours) {
  if (contours.empty())
    return std::make_unique<CustomTargetData>(
        id_, 0, false, cv::RotatedRect(), center2f_,
        cv::Mat::zeros(cv::Size(3, 1), CV_64F),
        cv::Mat::zeros(cv::Size(3, 1), CV_64F),
        std::vector<cv::Point2d>{8, cv::Point2d()});
  auto contour = contours[0];
  cv::RotatedRect rect = cv::minAreaRect(contour);
  std::vector<cv::Point2d> corners;
  cv::Mat rvec;  // axis angle form
  cv::Mat tvec;
  double a = 0.01 * cv::arcLength(contour, true);
  cv::approxPolyDP(contour, corners, a, true);
  
  // Remove inside top points (usually inaccurate)
  corners.erase(std::next(corners.begin(), 4));
  corners.erase(std::next(corners.begin(), 6));
  
  cv::solvePnP(hex_model, corners, cam_mtx, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE );

  cv::Mat r;
  cv::Rodrigues(rvec, r);
  r = r.t();
  tvec = -r* tvec;
  spdlog::debug(
      "cam_mtx[{},{}] = [({}, {}, {}), ({}, {}, {}), ({}, {}, {})], "
      "dist_coeffs[{},{}] = [{}, {}, {}, {}, {}], "
      "corners[{}] = [({},{}), ({},{}), ({},{}), ({},{}), ({},{}), ({},{})], tvec[{}, {}] = [{}, {}, {}], rvec[{}, {}] = [{},{},{}]",
      cam_mtx.rows, cam_mtx.cols, cam_mtx.at<double>(0, 0), cam_mtx.at<double>(0, 1),
      cam_mtx.at<double>(0, 2), cam_mtx.at<double>(1, 0),
      cam_mtx.at<double>(1, 1), cam_mtx.at<double>(1, 2),
      cam_mtx.at<double>(2, 0), cam_mtx.at<double>(2, 1),
      cam_mtx.at<double>(2, 2), dist_coeffs.rows, dist_coeffs.cols, dist_coeffs.at<double>(0),
      dist_coeffs.at<double>(1), dist_coeffs.at<double>(2),
      dist_coeffs.at<double>(3), dist_coeffs.at<double>(4), corners.size(),
      corners.at(0).x, corners.at(0).y, corners.at(1).x, corners.at(1).y,
      corners.at(2).x, corners.at(2).y, corners.at(3).x, corners.at(3).y,
      corners.at(4).x, corners.at(4).y, corners.at(5).x, corners.at(5).y,
      tvec.rows, tvec.cols, tvec.at<double>(0), tvec.at<double>(1),
      tvec.at<double>(2), rvec.rows, rvec.cols, rvec.at<double>(0),
      rvec.at<double>(1), rvec.at<double>(2));
  return std::make_unique<CustomTargetData>(id_, 0, true, rect, center2f_, tvec, rvec, corners);
}

std::string CustomPipeline::ToString() const {
  return fmt::format("CustomPipeline<{}, {}>", id_, capture_type_);
}
