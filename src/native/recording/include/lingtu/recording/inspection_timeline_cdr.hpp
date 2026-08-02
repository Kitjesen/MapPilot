#pragma once

#include <string>
#include <vector>

#include "lingtu/recording/inspection_timeline.hpp"
#include "lingtu/recording/recording_core.hpp"
#include "lingtu/recording/topic_catalog.hpp"

namespace lingtu::recording {

class InspectionTimelineCapture {
 public:
  void observe(const TopicBinding &binding, const RecordedMessage &message);

  [[nodiscard]] InspectionTimelineReport verify(const std::string &task_id) const;

 private:
  std::vector<InspectionTaskEventFact> events_;
  std::vector<FinalOutputFact> outputs_;
  std::vector<DriverControlFact> driver_states_;
};

}  // namespace lingtu::recording
