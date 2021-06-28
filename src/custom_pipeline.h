#pragma once
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
  std::string capture_type_{"unknown"};
};

}  // namespace custom
