#include "custom_pipeline.h"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>

#include "custom_target_data.h"

using namespace deadeye;
using namespace custom;

CustomPipeline::CustomPipeline(int inum, std::string name)
    : AbstractPipeline{inum, name} {}

void CustomPipeline::Configure(const CaptureConfig& config) {
  capture_type_ = config.PipelineType();
}

// Target is center of contour bounding box.
std::unique_ptr<TargetData> CustomPipeline::ProcessContours(
    Contours const& contours) {
  if (contours.empty())
    return std::make_unique<CustomTargetData>(id_, 0, false,
                                              cv::Rect{0, 0, 0, 0});
  auto contour = contours[0];
  cv::Rect bb = cv::boundingRect(contour);
  return std::make_unique<CustomTargetData>(id_, 0, true, bb);
}

std::string CustomPipeline::ToString() const {
  return fmt::format("CustomPipeline<{}, {}>", id_, capture_type_);
}
