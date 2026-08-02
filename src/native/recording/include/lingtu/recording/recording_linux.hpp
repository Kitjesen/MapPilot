#pragma once

#if !defined(__linux__)
#error "LingTu recording process supervision requires Linux"
#endif

#include <memory>

#include "lingtu/recording/recording_manager.hpp"

namespace lingtu::recording {

class PosixRecordingProcessFactory final : public RecordingProcessFactory {
 public:
  std::unique_ptr<RecordingProcess> start(const RecordingProcessSpec &spec) override;
};

}  // namespace lingtu::recording
