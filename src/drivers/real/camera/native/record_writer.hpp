#pragma once

#include <ostream>
#include <stdexcept>

#include "camera_record.hpp"

namespace lingtu::drivers::camera::record {

// SDK adapters retain frame ownership until write() returns.
class RecordWriter {
 public:
  explicit RecordWriter(std::ostream& output) : output_(output) {}

  void write(const RecordHeader& header, const void* payload = nullptr) {
    const auto validation = validateRecordHeader(header);
    if (validation != RecordValidation::kValid) {
      throw std::runtime_error(recordValidationReason(validation));
    }
    const auto payload_validation = validateRecordPayloadPointer(header, payload);
    if (payload_validation != RecordValidation::kValid) {
      throw std::runtime_error(recordValidationReason(payload_validation));
    }
    output_.write(reinterpret_cast<const char*>(&header), sizeof(header));
    if (header.payload_size > 0) {
      output_.write(static_cast<const char*>(payload), header.payload_size);
    }
    if (!output_) {
      throw std::runtime_error("camera_record_output_failed");
    }
  }

  void flush() {
    output_.flush();
    if (!output_) {
      throw std::runtime_error("camera_record_output_failed");
    }
  }

 private:
  std::ostream& output_;
};

}  // namespace lingtu::drivers::camera::record
