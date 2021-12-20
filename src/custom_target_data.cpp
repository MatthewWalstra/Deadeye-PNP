#include "custom_target_data.h"

#include <fmt/core.h>

#include <opencv2/imgproc.hpp>

using namespace deadeye;
using namespace custom;
using json = nlohmann::json;

CustomTargetData::CustomTargetData(std::string id, int sn, bool valid, cv::RotatedRect rect, cv::Point center, cv::Mat tvec, cv::Mat rvec, std::vector<cv::Point2d> pnp_corners)
    : TargetData{id, sn, valid}, rect(rect), center(center), tvec(tvec), rvec(rvec), pnp_corners(pnp_corners) {
      rect.points(rect_corners);
    }

void CustomTargetData::DrawMarkers(cv::Mat& preview) const {
  cv::Point center{preview.cols / 2, preview.rows / 2};
  cv::Point2f target = rect.center;
  cv::drawMarker(preview, center, cv::Scalar::all(255),
                 cv::MARKER_TILTED_CROSS);
  cv::drawMarker(preview, target, cv::Scalar::all(255));

  for (int i = 0; i < 4; i++) {
    cv::circle(preview, rect_corners[i], 3, cv::Scalar(0, 0, 255), cv::FILLED,
               cv::LINE_8);
    cv::line(preview, rect_corners[i], rect_corners[(i+1) % 4], cv::Scalar(0, 0, 255));
  }
    
  for (int i = 0; i < 6; i++) {
    cv::circle(preview, pnp_corners[i], 3, cv::Scalar(255, 0, 0), cv::FILLED,
               cv::LINE_8);
  }
}

std::string CustomTargetData::Dump() const {
  json j = json{{TargetData::kIdKey, id},
                {TargetData::kSerialKey, serial},
                {TargetData::kValidKey, valid},
                {TargetData::kDataKey,
                  {
                    rect.angle,
                    rect.center.x,
                    rect.center.y,
                    rect.size.height,
                    rect.size.width,
                    rect_corners[0].x,
                    rect_corners[0].y,
                    rect_corners[1].x,
                    rect_corners[1].y,
                    rect_corners[2].x,
                    rect_corners[2].y,
                    rect_corners[3].x,
                    rect_corners[3].y,
                    tvec.at<double>(0),
                    tvec.at<double>(1),
                    tvec.at<double>(2),
                    rvec.at<double>(0),
                    rvec.at<double>(1),
                    rvec.at<double>(2),
                    pnp_corners[0].x,
                    pnp_corners[0].y,
                    pnp_corners[1].x,
                    pnp_corners[1].y,
                    pnp_corners[2].x,
                    pnp_corners[2].y,
                    pnp_corners[3].x,
                    pnp_corners[3].y,
                    pnp_corners[4].x,
                    pnp_corners[4].y,
                    pnp_corners[5].x,
                    pnp_corners[5].y,
                    // pnp_corners[6].x,
                    // pnp_corners[6].y,
                    // pnp_corners[7].x,
                    // pnp_corners[7].y,
                  }}};
  return j.dump();
}

std::string CustomTargetData::ToString() const {
  return fmt::format(
      "id={} sn={} val={} bl=({:.1f},{:.1f}) tl=({:.1f},{:.1f}) "
      "tr=({:.1f},{:.1f}) br=({:.1f},{:.1f}) "
      "ctr=({:.1f},{:.1f}) w={:.1f} h={:.1f} a={:.1f} "
      "tvec=({:.3f},{:.3f},{:.3f}) rvec=({:.3f},{:.3f},{:.3f}) "
      "otl=({:.3f},{:.3f}) obl=({:.3f},{:.3f}), obr=({:.3f},{:.3f}) "
      "otr=({:.3f},{:.3f}) ibr=({:.3f},{:.3f}) "
      "ibl=({:.3f},{:.3f})",
      id, serial, valid, rect_corners[0].x, rect_corners[0].y,
      rect_corners[1].x, rect_corners[1].y, rect_corners[2].x,
      rect_corners[2].y, rect_corners[3].x, rect_corners[3].y, center.x,
      center.y, rect.size.width, rect.size.height, rect.angle,
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
      rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
      pnp_corners[0].x, pnp_corners[0].y, pnp_corners[1].x, pnp_corners[1].y,
      pnp_corners[2].x, pnp_corners[2].y, pnp_corners[3].x, pnp_corners[3].y,
      pnp_corners[4].x, pnp_corners[4].y, pnp_corners[5].x, pnp_corners[5].y);
}
