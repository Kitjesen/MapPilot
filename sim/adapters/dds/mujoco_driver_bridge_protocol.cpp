#include "mujoco_driver_bridge_protocol.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

namespace lingtu::sim::driver_bridge {
namespace {

constexpr std::string_view kInternalProducerSentinel = "-";
constexpr std::string_view kActivateName = "LT_DRIVER_ACTIVATE_V2";
constexpr std::string_view kAppliedName = "LT_DRIVER_APPLIED_V2";
constexpr std::string_view kHeartbeatName = "LT_DRIVER_HEARTBEAT_V2";
constexpr std::string_view kDeactivateName = "LT_DRIVER_DEACTIVATE_V2";
constexpr std::string_view kCommandName = "LT_DRIVER_COMMAND_V2";
constexpr std::string_view kHelloName = "LT_DRIVER_HELLO_V2";
constexpr std::string_view kReadyName = "LT_DRIVER_READY_V2";
constexpr std::string_view kFaultName = "LT_DRIVER_FAULT_V2";
constexpr std::string_view kStoppedName = "LT_DRIVER_STOPPED_V2";

std::vector<std::string_view> splitFields(std::string_view line) {
  if (line.empty()) {
    throw ProtocolError("protocol line must not be empty");
  }
  if (line.size() > kMaxProtocolLineBytes) {
    throw ProtocolError("protocol line exceeds fixed byte limit");
  }
  std::vector<std::string_view> fields;
  std::size_t start = 0;
  while (true) {
    const auto tab = line.find('\t', start);
    fields.push_back(
        line.substr(start, tab == std::string_view::npos ? line.size() - start : tab - start));
    if (tab == std::string_view::npos) {
      break;
    }
    start = tab + 1;
  }
  return fields;
}

void requireFieldCount(const std::vector<std::string_view> &fields, std::size_t expected,
                       std::string_view message_name) {
  if (fields.size() != expected) {
    throw ProtocolError(std::string(message_name) + " has missing or extra fields");
  }
}

void requireBootId(std::string_view value, std::string_view field) {
  if (!validBootId(value)) {
    throw ProtocolError(std::string(field) + " must be lowercase 32 hex");
  }
}

std::uint64_t parseUint64(std::string_view value, bool require_positive, std::string_view field) {
  if (value.empty()) {
    throw ProtocolError(std::string(field) + " must not be empty");
  }
  for (const char character : value) {
    if (character < '0' || character > '9') {
      throw ProtocolError(std::string(field) + " must be decimal uint64");
    }
  }
  std::uint64_t parsed = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed, 10);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw ProtocolError(std::string(field) + " overflows uint64");
  }
  if (require_positive && parsed == 0) {
    throw ProtocolError(std::string(field) + " must be greater than zero");
  }
  return parsed;
}

bool validDoubleGrammar(std::string_view value) {
  if (value.empty()) {
    return false;
  }
  std::size_t index = 0;
  if (value[index] == '-') {
    ++index;
  }
  const auto consume_digits = [&]() {
    const auto begin = index;
    while (index < value.size() && value[index] >= '0' && value[index] <= '9') {
      ++index;
    }
    return index > begin;
  };
  if (!consume_digits()) {
    return false;
  }
  if (index < value.size() && value[index] == '.') {
    ++index;
    if (!consume_digits()) {
      return false;
    }
  }
  if (index < value.size() && (value[index] == 'e' || value[index] == 'E')) {
    ++index;
    if (index < value.size() && (value[index] == '+' || value[index] == '-')) {
      ++index;
    }
    if (!consume_digits()) {
      return false;
    }
  }
  return index == value.size();
}

double parseFiniteDouble(std::string_view value, std::string_view field) {
  if (!validDoubleGrammar(value)) {
    throw ProtocolError(std::string(field) + " has invalid double grammar");
  }
  double parsed = 0.0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed,
                                      std::chars_format::general);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size() ||
      !std::isfinite(parsed)) {
    throw ProtocolError(std::string(field) + " must be finite");
  }
  return parsed;
}

bool validCommandKind(BridgeCommandKind kind) noexcept {
  switch (kind) {
    case BridgeCommandKind::ActivationZero:
    case BridgeCommandKind::Nav:
    case BridgeCommandKind::DeactivateZero:
    case BridgeCommandKind::WriterFaultZero:
    case BridgeCommandKind::SafetyZero:
      return true;
  }
  return false;
}

bool validFaultCode(BridgeFaultCode fault) noexcept {
  switch (fault) {
    case BridgeFaultCode::ProtocolViolation:
    case BridgeFaultCode::ControllerEof:
    case BridgeFaultCode::HeartbeatTimeout:
    case BridgeFaultCode::ApplyTimeout:
    case BridgeFaultCode::WriterMissing:
    case BridgeFaultCode::WriterAmbiguous:
    case BridgeFaultCode::CommandSequenceOverflow:
      return true;
    case BridgeFaultCode::None:
      return false;
  }
  return false;
}

BridgeCommandKind parseCommandKind(std::string_view value) {
  if (value == "activation_zero") {
    return BridgeCommandKind::ActivationZero;
  }
  if (value == "nav") {
    return BridgeCommandKind::Nav;
  }
  if (value == "deactivate_zero") {
    return BridgeCommandKind::DeactivateZero;
  }
  if (value == "writer_fault_zero") {
    return BridgeCommandKind::WriterFaultZero;
  }
  if (value == "safety_zero") {
    return BridgeCommandKind::SafetyZero;
  }
  throw ProtocolError("unknown bridge command kind");
}

bool exactZero(double value) noexcept {
  return value == 0.0 && !std::signbit(value);
}

void validateCommandPayload(BridgeCommandKind kind, std::string_view producer_boot_id,
                            std::uint64_t output_sequence, double walk_x, double walk_y,
                            double walk_z) {
  if (!std::isfinite(walk_x) || !std::isfinite(walk_y) || !std::isfinite(walk_z)) {
    throw ProtocolError("command walk must be finite");
  }
  if (kind == BridgeCommandKind::Nav) {
    if (!validProducerToken(producer_boot_id) || producer_boot_id == kInternalProducerSentinel) {
      throw ProtocolError("Nav producer token is invalid");
    }
    if (output_sequence == 0) {
      throw ProtocolError("Nav output sequence must be greater than zero");
    }
    return;
  }
  if (!producer_boot_id.empty() || output_sequence != 0 || !exactZero(walk_x) ||
      !exactZero(walk_y) || !exactZero(walk_z)) {
    throw ProtocolError("internal zero must have empty identity and exact zero walk");
  }
}

std::string producerField(std::string_view producer_boot_id) {
  return producer_boot_id.empty() ? std::string(kInternalProducerSentinel)
                                  : std::string(producer_boot_id);
}

std::string formatDouble(double value) {
  if (!std::isfinite(value)) {
    throw ProtocolError("cannot serialize non-finite double");
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return stream.str();
}

std::string checkedSerializedLine(std::string line) {
  if (line.size() > kMaxProtocolLineBytes) {
    throw ProtocolError("serialized protocol line exceeds fixed byte limit");
  }
  return line;
}

}  // namespace

bool validBootId(std::string_view value) noexcept {
  if (value.size() != 32) {
    return false;
  }
  for (const char character : value) {
    if (!((character >= '0' && character <= '9') || (character >= 'a' && character <= 'f'))) {
      return false;
    }
  }
  return true;
}

bool validProducerToken(std::string_view value) noexcept {
  if (value.empty() || value.size() > kMaxProducerTokenBytes) {
    return false;
  }
  const auto is_alnum = [](char character) {
    return (character >= '0' && character <= '9') || (character >= 'A' && character <= 'Z') ||
           (character >= 'a' && character <= 'z');
  };
  if (!is_alnum(value.front())) {
    return false;
  }
  for (const char character : value) {
    if (!is_alnum(character) && character != '.' && character != '_' && character != ':' &&
        character != '@' && character != '-') {
      return false;
    }
  }
  return true;
}

const char *commandKindName(BridgeCommandKind kind) noexcept {
  switch (kind) {
    case BridgeCommandKind::ActivationZero:
      return "activation_zero";
    case BridgeCommandKind::Nav:
      return "nav";
    case BridgeCommandKind::DeactivateZero:
      return "deactivate_zero";
    case BridgeCommandKind::WriterFaultZero:
      return "writer_fault_zero";
    case BridgeCommandKind::SafetyZero:
      return "safety_zero";
  }
  return "unknown";
}

const char *faultCodeName(BridgeFaultCode fault) noexcept {
  switch (fault) {
    case BridgeFaultCode::None:
      return "none";
    case BridgeFaultCode::ProtocolViolation:
      return "protocol_violation";
    case BridgeFaultCode::ControllerEof:
      return "controller_eof";
    case BridgeFaultCode::HeartbeatTimeout:
      return "heartbeat_timeout";
    case BridgeFaultCode::ApplyTimeout:
      return "apply_timeout";
    case BridgeFaultCode::WriterMissing:
      return "writer_missing";
    case BridgeFaultCode::WriterAmbiguous:
      return "writer_ambiguous";
    case BridgeFaultCode::CommandSequenceOverflow:
      return "command_sequence_overflow";
  }
  return "unknown";
}

ControllerMessage parseControllerLine(std::string_view line) {
  const auto fields = splitFields(line);
  const auto message_name = fields.front();
  if (message_name == kActivateName) {
    requireFieldCount(fields, 4, message_name);
    requireBootId(fields[1], "bridge_boot_id");
    requireBootId(fields[2], "controller_boot_id");
    return ActivateMessage{
        std::string(fields[1]),
        std::string(fields[2]),
        parseUint64(fields[3], true, "control_seq"),
    };
  }
  if (message_name == kHeartbeatName) {
    requireFieldCount(fields, 5, message_name);
    requireBootId(fields[1], "bridge_boot_id");
    requireBootId(fields[2], "controller_boot_id");
    return HeartbeatMessage{
        std::string(fields[1]),
        std::string(fields[2]),
        parseUint64(fields[3], true, "control_seq"),
        parseUint64(fields[4], true, "step_seq"),
    };
  }
  if (message_name == kDeactivateName) {
    requireFieldCount(fields, 4, message_name);
    requireBootId(fields[1], "bridge_boot_id");
    requireBootId(fields[2], "controller_boot_id");
    return DeactivateMessage{
        std::string(fields[1]),
        std::string(fields[2]),
        parseUint64(fields[3], true, "control_seq"),
    };
  }
  if (message_name == kAppliedName) {
    requireFieldCount(fields, 11, message_name);
    requireBootId(fields[1], "bridge_boot_id");
    requireBootId(fields[2], "controller_boot_id");
    const auto kind = parseCommandKind(fields[4]);
    if (kind != BridgeCommandKind::Nav && fields[5] != kInternalProducerSentinel) {
      throw ProtocolError("internal zero producer field must be exactly '-'");
    }
    const std::string producer =
        fields[5] == kInternalProducerSentinel ? "" : std::string(fields[5]);
    const auto output_sequence = parseUint64(fields[6], false, "output_sequence");
    const double walk_x = parseFiniteDouble(fields[7], "walk_x");
    const double walk_y = parseFiniteDouble(fields[8], "walk_y");
    const double walk_z = parseFiniteDouble(fields[9], "walk_z");
    validateCommandPayload(kind, producer, output_sequence, walk_x, walk_y, walk_z);
    return AppliedMessage{
        std::string(fields[1]),
        std::string(fields[2]),
        parseUint64(fields[3], true, "bridge_command_seq"),
        kind,
        producer,
        output_sequence,
        walk_x,
        walk_y,
        walk_z,
        parseUint64(fields[10], true, "applied_step_seq"),
    };
  }
  throw ProtocolError("unknown controller message name");
}

std::string serializeCommand(const BridgeCommand &command) {
  requireBootId(command.bridge_boot_id, "bridge_boot_id");
  requireBootId(command.controller_boot_id, "controller_boot_id");
  if (command.bridge_command_seq == 0) {
    throw ProtocolError("bridge_command_seq must be greater than zero");
  }
  if (!validCommandKind(command.kind)) {
    throw ProtocolError("cannot serialize unknown bridge command kind");
  }
  validateCommandPayload(command.kind, command.producer_boot_id, command.output_sequence,
                         command.walk.x, command.walk.y, command.walk.z);
  return checkedSerializedLine(
      std::string(kCommandName) + "\t" + command.bridge_boot_id + "\t" +
      command.controller_boot_id + "\t" + std::to_string(command.bridge_command_seq) + "\t" +
      commandKindName(command.kind) + "\t" + producerField(command.producer_boot_id) + "\t" +
      std::to_string(command.output_sequence) + "\t" + formatDouble(command.walk.x) + "\t" +
      formatDouble(command.walk.y) + "\t" + formatDouble(command.walk.z));
}

std::string serializeHello(const HelloMessage &hello) {
  requireBootId(hello.bridge_boot_id, "bridge_boot_id");
  return checkedSerializedLine(std::string(kHelloName) + "\t" + hello.bridge_boot_id);
}

std::string serializeReady(const ReadyMessage &ready) {
  requireBootId(ready.bridge_boot_id, "bridge_boot_id");
  requireBootId(ready.controller_boot_id, "controller_boot_id");
  if (ready.accepted_sequence == 0) {
    throw ProtocolError("accepted_sequence must be greater than zero");
  }
  const bool has_output_identity =
      !ready.accepted_producer_boot_id.empty() || ready.accepted_output_sequence != 0;
  if (has_output_identity && (!validProducerToken(ready.accepted_producer_boot_id) ||
                              ready.accepted_output_sequence == 0)) {
    throw ProtocolError("READY output identity is incomplete");
  }
  return checkedSerializedLine(std::string(kReadyName) + "\t" + ready.bridge_boot_id + "\t" +
                               ready.controller_boot_id + "\t" +
                               std::to_string(ready.accepted_sequence) + "\t" +
                               producerField(ready.accepted_producer_boot_id) + "\t" +
                               std::to_string(ready.accepted_output_sequence));
}

std::string serializeFault(const FaultMessage &fault) {
  requireBootId(fault.bridge_boot_id, "bridge_boot_id");
  if (!fault.controller_boot_id.empty()) {
    requireBootId(fault.controller_boot_id, "controller_boot_id");
  }
  if (!validFaultCode(fault.fault)) {
    throw ProtocolError("FAULT requires a non-empty typed fault code");
  }
  return checkedSerializedLine(std::string(kFaultName) + "\t" + fault.bridge_boot_id + "\t" +
                               (fault.controller_boot_id.empty()
                                    ? std::string(kInternalProducerSentinel)
                                    : fault.controller_boot_id) +
                               "\t" + faultCodeName(fault.fault));
}

std::string serializeStopped(const StopMessage &stopped) {
  requireBootId(stopped.bridge_boot_id, "bridge_boot_id");
  requireBootId(stopped.controller_boot_id, "controller_boot_id");
  if (stopped.bridge_command_seq == 0) {
    throw ProtocolError("STOPPED bridge_command_seq must be greater than zero");
  }
  if (stopped.applied_step_seq == 0) {
    throw ProtocolError("STOPPED applied_step_seq must be greater than zero");
  }
  if (stopped.kind != BridgeCommandKind::DeactivateZero) {
    throw ProtocolError("STOPPED kind must be deactivate_zero");
  }
  return checkedSerializedLine(std::string(kStoppedName) + "\t" + stopped.bridge_boot_id + "\t" +
                               stopped.controller_boot_id + "\t" +
                               std::to_string(stopped.bridge_command_seq) + "\t" +
                               std::to_string(stopped.applied_step_seq) + "\tdeactivate_zero");
}

}  // namespace lingtu::sim::driver_bridge
