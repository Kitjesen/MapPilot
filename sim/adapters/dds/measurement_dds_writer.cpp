#include "measurement_dds_writer.hpp"

#include "message/generated/topics.hpp"
#include "transport/dds/qos.hpp"
#include "messages.h"

#include <stdexcept>
#include <string>

namespace lingtu::sim::dds_adapter {
namespace {
dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  return static_cast<dds_entity_t>(value);
}
template <typename Sequence, typename T> void sequence(Sequence &out, const std::vector<T> &values) {
  out._maximum = out._length = static_cast<std::uint32_t>(values.size());
  out._buffer = const_cast<T *>(values.data());
  out._release = false;
}
template <typename Message> void identity(Message &message, const MeasurementRecord &r) {
  message.sensor_id = const_cast<char *>(r.sensor_id.c_str());
  message.header.stamp.sec = static_cast<std::int32_t>(r.sim_time_ns / 1000000000ULL);
  message.header.stamp.nanosec = static_cast<std::uint32_t>(r.sim_time_ns % 1000000000ULL);
  message.header.frame_id = const_cast<char *>(r.frame_id.c_str());
  message.identity.product_session_id = const_cast<char *>(r.product_session_id.c_str());
  message.identity.session_id = const_cast<char *>(r.session_id.c_str());
  message.identity.instance_id = const_cast<char *>(r.instance_id.c_str());
  message.identity.model_generation = r.model_generation;
  message.identity.reset_generation = r.reset_generation;
  message.identity.sample_sequence = r.sequence;
  message.identity.sim_time_ns = r.sim_time_ns;
  sequence(message.valid_mask, r.valid_mask);
  message.metadata_json = const_cast<char *>(r.metadata_json.c_str());
}
}  // namespace

MeasurementDdsWriter::MeasurementDdsWriter(int domain) {
  participant_ = checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr),
                         "measurement participant");
  try {
    const auto writer = [this](const dds_topic_descriptor_t *descriptor,
                               const lingtu::message::TopicContract &contract) {
      const auto topic = checked(dds_create_topic(participant_, descriptor, contract.dds_topic.data(),
                                                  nullptr, nullptr), "measurement topic");
      auto qos = lingtu::dds::make_qos(contract.qos_profile);
      return checked(dds_create_writer(participant_, topic, qos.get(), nullptr), "measurement writer");
    };
    thermal_ = writer(&lingtu_dds_ThermalImage_desc, lingtu::message::kSensorThermal);
    acoustic_ = writer(&lingtu_dds_AcousticBlock_desc, lingtu::message::kSensorAcoustic);
    semantic_ = writer(&lingtu_dds_SemanticImage_desc, lingtu::message::kSensorSemantic);
  } catch (...) {
    dds_delete(participant_); participant_ = 0; throw;
  }
}
MeasurementDdsWriter::~MeasurementDdsWriter() {
  if (participant_ > 0) dds_delete(participant_);
}
void MeasurementDdsWriter::write(const MeasurementRecord &r) {
  switch (r.modality) {
    case MeasurementModality::Thermal: {
      lingtu_dds_ThermalImage message{};
      identity(message, r);
      message.width = r.columns; message.height = r.rows;
      sequence(message.temperature_k, r.primary);
      checked(dds_write(thermal_, &message), "dds_write(thermal)");
      break;
    }
    case MeasurementModality::Acoustic: {
      lingtu_dds_AcousticBlock message{};
      identity(message, r);
      message.channels = r.rows; message.samples = r.columns; message.sample_rate_hz = r.sample_rate_hz;
      sequence(message.pressure_pa, r.primary); sequence(message.voltage_v, r.voltage_v);
      checked(dds_write(acoustic_, &message), "dds_write(acoustic)");
      break;
    }
    case MeasurementModality::Semantic: {
      lingtu_dds_SemanticImage message{};
      identity(message, r);
      message.width = r.columns; message.height = r.rows;
      sequence(message.class_ids, r.class_ids); sequence(message.instance_ids, r.instance_ids);
      checked(dds_write(semantic_, &message), "dds_write(semantic)");
      break;
    }
  }
}
}  // namespace lingtu::sim::dds_adapter
