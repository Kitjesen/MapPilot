#pragma once

#include "slam.hpp"

namespace lingtu::slam {

// DDS request strings are borrowed. Own the identity until registration finishes.
struct RelocalizationRequestIdentity {
  std::string request_id;
  std::string action;
  std::string engine;
};

struct ManualRelocalizationCompletion {
  RelocalizationRequestIdentity request;
  Status status;
};

class ManualRelocalization {
 public:
  bool pending() const { return request_.has_value(); }

  bool owns(const std::string& request_id) const {
    return request_ && request_->request_id == request_id;
  }

  Status start(ISlamBackend& backend, RelocalizationRequestIdentity request,
               const std::optional<Pose3d>& guess, RelocalizationSearch search) {
    if (pending() || backend.relocalizeAsyncInFlight()) {
      return Status::Error("async_relocalization_in_progress");
    }
    const auto status = backend.startRelocalizeAsync(guess, search);
    if (status.ok) request_ = std::move(request);
    return status;
  }

  std::optional<ManualRelocalizationCompletion> poll(ISlamBackend& backend) {
    if (!request_) return std::nullopt;
    const auto status = backend.pollRelocalizeAsync();
    if (!status) return std::nullopt;
    ManualRelocalizationCompletion completed{std::move(*request_), *status};
    request_.reset();
    return completed;
  }

 private:
  std::optional<RelocalizationRequestIdentity> request_;
};

}  // namespace lingtu::slam
