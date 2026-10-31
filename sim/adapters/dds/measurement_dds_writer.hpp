#pragma once

#include "dds/dds.h"
#include "measurement_protocol.hpp"

namespace lingtu::sim::dds_adapter {

class MeasurementDdsWriter final {
 public:
  explicit MeasurementDdsWriter(int domain);
  MeasurementDdsWriter(const MeasurementDdsWriter &) = delete;
  MeasurementDdsWriter &operator=(const MeasurementDdsWriter &) = delete;
  ~MeasurementDdsWriter();
  // Input must have passed decode_measurement_payload and stream validation.
  void write(const MeasurementRecord &record);

 private:
  dds_entity_t participant_{0}, thermal_{0}, acoustic_{0}, semantic_{0};
};
}  // namespace lingtu::sim::dds_adapter
