#pragma once

#include "dds/dds.h"

#include "gnss_protocol.hpp"

namespace lingtu::sim::dds_adapter {

class GnssDdsWriter final {
 public:
  explicit GnssDdsWriter(int domain);
  GnssDdsWriter(const GnssDdsWriter &) = delete;
  GnssDdsWriter &operator=(const GnssDdsWriter &) = delete;
  ~GnssDdsWriter();
  void write(const GnssRecord &record);

 private:
  dds_entity_t participant_{0};
  dds_entity_t fix_writer_{0};
  dds_entity_t status_writer_{0};
};

}  // namespace lingtu::sim::dds_adapter
