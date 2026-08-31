#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lingtu/sim/mujoco_runtime.hpp"

namespace {

constexpr std::uint32_t kMaxSampledAdvanceSteps = 4096;

std::uint32_t parse_steps(const std::string &text) {
  std::size_t consumed = 0;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size() || value == 0 ||
      value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("steps must be in the range 1..4294967295");
  }
  return static_cast<std::uint32_t>(value);
}

std::uint64_t parse_generation(const std::string &text) {
  std::size_t consumed = 0;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size()) {
    throw std::invalid_argument("model generation must be an unsigned integer");
  }
  return value;
}

std::uint64_t parse_unsigned(const std::string &text, const char *field) {
  if (text.empty() || text.front() == '-' || text.front() == '+') {
    throw std::invalid_argument(std::string(field) +
                                " must be an unsigned integer");
  }
  std::size_t consumed = 0;
  const auto value = std::stoull(text, &consumed);
  if (consumed != text.size()) {
    throw std::invalid_argument(std::string(field) +
                                " must be an unsigned integer");
  }
  return value;
}

std::uint32_t parse_uint32(const std::string &text, const char *field) {
  const auto value = parse_unsigned(text, field);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(std::string(field) +
                                " must be in the range 0..4294967295");
  }
  return static_cast<std::uint32_t>(value);
}

std::uint8_t parse_uint8(const std::string &text, const char *field) {
  const auto value = parse_unsigned(text, field);
  if (value > std::numeric_limits<std::uint8_t>::max()) {
    throw std::invalid_argument(std::string(field) +
                                " must be in the range 0..255");
  }
  return static_cast<std::uint8_t>(value);
}

std::size_t parse_count(const std::string &text, const char *field,
                        std::size_t maximum) {
  const auto value = parse_unsigned(text, field);
  if (value == 0 || value > maximum) {
    throw std::invalid_argument(std::string(field) + " must be in the range 1.." +
                                std::to_string(maximum));
  }
  return static_cast<std::size_t>(value);
}

bool parse_boolean_flag(const std::string &text, const char *field) {
  if (text == "0") {
    return false;
  }
  if (text == "1") {
    return true;
  }
  throw std::invalid_argument(std::string(field) + " must be 0 or 1");
}

double parse_finite(const std::string &text, const char *field) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(std::string(field) + " must be finite numeric data");
  }
  return value;
}

int parse_integrator(const std::string &text) {
  if (text == "euler") return mjINT_EULER;
  if (text == "rk4") return mjINT_RK4;
  if (text == "implicit") return mjINT_IMPLICIT;
  if (text == "implicitfast") return mjINT_IMPLICITFAST;
  throw std::invalid_argument("global policy integrator is unsupported");
}

int parse_solver(const std::string &text) {
  if (text == "pgs") return mjSOL_PGS;
  if (text == "cg") return mjSOL_CG;
  if (text == "newton") return mjSOL_NEWTON;
  throw std::invalid_argument("global policy solver is unsupported");
}

bool valid_id(const std::string &value) {
  if (value.empty() || value.size() > 63) return false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char character = value[index];
    const bool alphanumeric =
        (character >= 'A' && character <= 'Z') ||
        (character >= 'a' && character <= 'z') ||
        (character >= '0' && character <= '9');
    if (!alphanumeric &&
        (index == 0 || (character != '_' && character != '.' && character != '-'))) return false;
  }
  return true;
}

std::string json_string(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (character < 0x20) {
          constexpr char hex[] = "0123456789abcdef";
          escaped += "\\u00";
          escaped.push_back(hex[(character >> 4) & 0x0f]);
          escaped.push_back(hex[character & 0x0f]);
        } else {
          escaped.push_back(static_cast<char>(character));
        }
    }
  }
  escaped.push_back('"');
  return escaped;
}

template <std::size_t Size>
void write_array(const std::array<double, Size> &values) {
  std::cout << '[';
  for (std::size_t index = 0; index < Size; ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << values[index];
  }
  std::cout << ']';
}

void write_array(const std::vector<double> &values) {
  std::cout << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << values[index];
  }
  std::cout << ']';
}

void write_state(const char *event, const lingtu::sim::SimulationSnapshot &snapshot) {
  std::cout << std::setprecision(17)
            << "{\"event\":" << json_string(event)
            << ",\"session_id\":" << json_string(snapshot.session_id)
            << ",\"model_generation\":" << snapshot.model_generation
            << ",\"reset_generation\":" << snapshot.reset_generation
            << ",\"sequence\":" << snapshot.sequence
            << ",\"physics_step\":" << snapshot.physics_step
            << ",\"sim_time_ns\":" << snapshot.sim_time_ns
            << ",\"body_count\":" << snapshot.bodies.size()
            << ",\"joint_count\":" << snapshot.joints.size()
            << ",\"actuator_count\":" << snapshot.actuators.size()
            << ",\"sensor_count\":" << snapshot.sensors.size()
            << "}\n" << std::flush;
}

void write_snapshot(const lingtu::sim::SimulationSnapshot &snapshot,
                    bool terminate_line = true,
                    bool realtime_projection = false) {
  std::cout << std::setprecision(realtime_projection ? 9 : 17)
            << "{\"event\":\"snapshot\""
            << ",\"session_id\":" << json_string(snapshot.session_id)
            << ",\"model_generation\":" << snapshot.model_generation
            << ",\"reset_generation\":" << snapshot.reset_generation
            << ",\"sequence\":" << snapshot.sequence
            << ",\"physics_step\":" << snapshot.physics_step
            << ",\"sim_time_ns\":" << snapshot.sim_time_ns
            << ",\"bodies\":[";
  for (std::size_t index = 0; index < snapshot.bodies.size(); ++index) {
    const auto &body = snapshot.bodies[index];
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << '{';
    if (!realtime_projection) {
      std::cout << "\"body_id\":" << body.body_id
                << ",\"name\":" << json_string(body.name) << ',';
    }
    std::cout << "\"stable_id\":" << json_string(body.stable_id)
              << ",\"instance_id\":" << json_string(body.instance_id)
              << ",\"frame_id\":" << json_string(body.frame_id)
              << ",\"position_m\":";
    write_array(body.position_m);
    std::cout << ",\"quaternion_wxyz\":";
    write_array(body.quaternion_wxyz);
    std::cout << ",\"linear_velocity_mps\":";
    write_array(body.linear_velocity_mps);
    std::cout << ",\"angular_velocity_rps\":";
    write_array(body.angular_velocity_rps);
    std::cout << '}';
  }
  std::cout << "],\"joints\":[";
  for (std::size_t index = 0; index < snapshot.joints.size(); ++index) {
    const auto &joint = snapshot.joints[index];
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << '{';
    if (!realtime_projection) {
      std::cout << "\"joint_id\":" << joint.joint_id
                << ",\"name\":" << json_string(joint.name)
                << ",\"stable_id\":" << json_string(joint.stable_id)
                << ",\"instance_id\":" << json_string(joint.instance_id)
                << ",\"frame_id\":" << json_string(joint.frame_id)
                << ',';
    } else {
      std::cout << "\"stable_id\":" << json_string(joint.stable_id) << ',';
    }
    std::cout << "\"position_rad\":";
    write_array(joint.position_rad);
    std::cout << ",\"velocity_rps\":";
    write_array(joint.velocity_rps);
    std::cout << '}';
  }
  std::cout << "],\"actuators\":[";
  for (std::size_t index = 0;
       !realtime_projection && index < snapshot.actuators.size(); ++index) {
    const auto &actuator = snapshot.actuators[index];
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << "{\"actuator_id\":" << actuator.actuator_id
              << ",\"name\":" << json_string(actuator.name)
              << ",\"stable_id\":" << json_string(actuator.stable_id)
              << ",\"instance_id\":" << json_string(actuator.instance_id)
              << ",\"channel_id\":" << json_string(actuator.channel_id)
              << ",\"control\":" << actuator.control << '}';
  }
  std::cout << "],\"sensors\":[";
  for (std::size_t index = 0; index < snapshot.sensors.size(); ++index) {
    const auto &sensor = snapshot.sensors[index];
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << '{';
    if (!realtime_projection) {
      std::cout << "\"sensor_id\":" << sensor.sensor_id
                << ",\"name\":" << json_string(sensor.name)
                << ",\"stable_id\":" << json_string(sensor.stable_id)
                << ",\"instance_id\":" << json_string(sensor.instance_id)
                << ",\"frame_id\":" << json_string(sensor.frame_id) << ',';
    }
    std::cout << "\"sensor_type\":" << json_string(sensor.sensor_type)
              << ",\"source_stable_id\":"
              << json_string(sensor.source_stable_id)
              << ",\"values\":";
    write_array(sensor.values);
    std::cout << '}';
  }
  std::cout << "]}";
  if (terminate_line) {
    std::cout << '\n' << std::flush;
  }
}

void write_snapshot_batch(
    const std::vector<lingtu::sim::SimulationSnapshot> &snapshots,
    bool realtime_projection = false) {
  std::cout << "{\"event\":\"snapshot_batch\",\"snapshots\":[";
  for (std::size_t index = 0; index < snapshots.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    write_snapshot(snapshots[index], false, realtime_projection);
  }
  std::cout << "]}\n" << std::flush;
}

void write_actuator_snapshot_batch(
    const std::string &source_id, std::uint64_t sequence,
    lingtu::sim::ActuatorCommandResult result,
    const std::vector<lingtu::sim::SimulationSnapshot> &snapshots) {
  std::cout << "{\"event\":\"actuator_snapshot_batch\""
            << ",\"source_id\":" << json_string(source_id)
            << ",\"sequence\":" << sequence
            << ",\"result\":"
            << json_string(std::string(
                   lingtu::sim::actuator_command_result_name(result)))
            << ",\"snapshots\":[";
  for (std::size_t index = 0; index < snapshots.size(); ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    write_snapshot(snapshots[index], false, true);
  }
  std::cout << "]}\n" << std::flush;
}

void write_actuator_bound(const lingtu::sim::ActuatorBindingSpec &binding) {
  std::cout << "{\"event\":\"actuator_bound\""
            << ",\"source_id\":" << json_string(binding.source_id)
            << ",\"instance_id\":" << json_string(binding.instance_id)
            << ",\"command_type\":" << json_string(binding.command_type)
            << ",\"channel_count\":" << binding.channels.size()
            << "}\n" << std::flush;
}

void write_actuator_result(const std::string &source_id,
                           std::uint64_t sequence,
                           lingtu::sim::ActuatorCommandResult result) {
  std::cout << "{\"event\":\"actuator_command\""
            << ",\"source_id\":" << json_string(source_id)
            << ",\"sequence\":" << sequence
            << ",\"result\":"
            << json_string(std::string(lingtu::sim::actuator_command_result_name(result)))
            << "}\n" << std::flush;
}

struct ParsedActuatorCommand {
  std::string source_id;
  lingtu::sim::CommandEnvelope envelope;
};

ParsedActuatorCommand parse_actuator_command(std::istringstream &command,
                                             const char *action) {
  std::string source_id;
  std::string instance_id;
  std::string command_type;
  std::string session_id;
  std::string model_generation_text;
  std::string reset_generation_text;
  std::string sequence_text;
  std::string apply_time_text;
  std::string safe_stop_text;
  std::string count_text;
  if (!(command >> source_id >> instance_id >> command_type >> session_id >>
        model_generation_text >> reset_generation_text >> sequence_text >>
        apply_time_text >> safe_stop_text >> count_text)) {
    throw std::invalid_argument(
        std::string(action) +
        " requires SOURCE INSTANCE TYPE SESSION_ID MODEL_GEN RESET_GEN "
        "SEQUENCE APPLY_TIME_NS SAFE_STOP COUNT VALUE...");
  }

  lingtu::sim::CommandEnvelope envelope;
  if (!valid_id(session_id) ||
      !envelope.session_id.assign(session_id) ||
      !envelope.source_id.assign(source_id) ||
      !envelope.instance_id.assign(instance_id) ||
      !envelope.type.assign(command_type)) {
    throw std::invalid_argument(std::string(action) +
                                " identifier exceeds protocol capacity");
  }
  envelope.generation.model_generation =
      parse_unsigned(model_generation_text, "model generation");
  envelope.generation.reset_generation =
      parse_unsigned(reset_generation_text, "reset generation");
  envelope.sequence = parse_unsigned(sequence_text, "command sequence");
  envelope.apply_time_ns =
      parse_unsigned(apply_time_text, "command apply time");

  lingtu::sim::ActuatorCommandPayload payload;
  payload.safe_stop =
      parse_boolean_flag(safe_stop_text, "safe-stop flag") ? 1 : 0;
  const std::size_t value_count =
      parse_count(count_text, "actuator value count",
                  lingtu::sim::kMaxActuatorValues);
  payload.value_count = static_cast<std::uint32_t>(value_count);
  for (std::size_t index = 0; index < value_count; ++index) {
    std::string value;
    if (!(command >> value)) {
      throw std::invalid_argument(std::string(action) +
                                  " value count does not match its payload");
    }
    payload.values[index] = parse_finite(value, "actuator value");
  }
  std::string extra;
  if (command >> extra) {
    throw std::invalid_argument(std::string(action) + " contains extra fields");
  }
  if (!envelope.set_payload(payload)) {
    throw std::logic_error("actuator payload exceeds command capacity");
  }
  return {std::move(source_id), std::move(envelope)};
}

std::vector<lingtu::sim::SimulationSnapshot> advance_sampled(
    lingtu::sim::MujocoRuntime &runtime, std::uint32_t steps,
    std::uint32_t sample_stride) {
  if (steps > kMaxSampledAdvanceSteps) {
    throw std::invalid_argument(
        "advance-sampled step count must be in the range 1..4096");
  }
  if (sample_stride > kMaxSampledAdvanceSteps) {
    throw std::invalid_argument(
        "advance-sampled sample stride must be in the range 1..4096");
  }
  std::vector<lingtu::sim::SimulationSnapshot> snapshots;
  snapshots.reserve(
      static_cast<std::size_t>((steps + sample_stride - 1U) / sample_stride) +
      1U);
  std::uint32_t remaining = steps;
  while (remaining > 0) {
    const auto current_step = runtime.snapshot().physics_step;
    const auto remainder =
        static_cast<std::uint32_t>(current_step % sample_stride);
    const auto until_stride =
        remainder == 0U ? sample_stride : sample_stride - remainder;
    const auto chunk = std::min(remaining, until_stride);
    snapshots.push_back(runtime.advance(chunk));
    remaining -= chunk;
  }
  return snapshots;
}

void write_kinematic_result(
    const lingtu::sim::KinematicPoseBatch &batch,
    lingtu::sim::KinematicPoseResult result) {
  std::cout << "{\"event\":\"kinematic_poses\""
            << ",\"session_id\":" << json_string(batch.session_id)
            << ",\"model_generation\":" << batch.model_generation
            << ",\"reset_generation\":" << batch.reset_generation
            << ",\"sequence\":" << batch.sequence
            << ",\"sim_time_ns\":" << batch.sim_time_ns
            << ",\"entity_count\":" << batch.entities.size()
            << ",\"result\":"
            << json_string(std::string(
                   lingtu::sim::kinematic_pose_result_name(result)))
            << "}\n" << std::flush;
}

void write_raycast(const std::string &sensor_frame_id,
                   const lingtu::sim::MujocoRaycastFrame &frame) {
  std::cout << std::setprecision(17)
            << "{\"event\":\"raycast\""
            << ",\"sensor_frame_id\":"
            << json_string(sensor_frame_id)
            << ",\"session_id\":" << json_string(frame.session_id)
            << ",\"model_generation\":" << frame.model_generation
            << ",\"reset_generation\":" << frame.reset_generation
            << ",\"sequence\":" << frame.sequence
            << ",\"sim_time_ns\":" << frame.sim_time_ns
            << ",\"hit_count\":" << frame.hits.size()
            << ",\"hits\":[";
  for (std::size_t index = 0; index < frame.hits.size(); ++index) {
    const auto &hit = frame.hits[index];
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << "{\"xyz_sensor\":["
              << hit.xyz_sensor[0] << ',' << hit.xyz_sensor[1] << ','
              << hit.xyz_sensor[2] << "]"
              << ",\"origin_world_m\":["
              << hit.origin_world_m[0] << ',' << hit.origin_world_m[1] << ','
              << hit.origin_world_m[2] << "]"
              << ",\"direction_world\":["
              << hit.direction_world[0] << ',' << hit.direction_world[1] << ','
              << hit.direction_world[2] << "]"
              << ",\"position_world_m\":["
              << hit.position_world_m[0] << ',' << hit.position_world_m[1] << ','
              << hit.position_world_m[2] << "]"
              << ",\"distance_m\":" << hit.distance_m
              << ",\"body_stable_id\":" << json_string(hit.body_stable_id)
              << ",\"entity_id\":" << json_string(hit.entity_id)
              << ",\"offset_time_ns\":" << hit.offset_time_ns
              << ",\"reflectivity\":" << static_cast<unsigned int>(hit.reflectivity)
              << ",\"tag\":" << static_cast<unsigned int>(hit.tag)
              << ",\"line\":" << static_cast<unsigned int>(hit.line)
              << '}';
  }
  std::cout << "]}\n" << std::flush;
}

void write_error(const std::string &message) {
  std::cout << "{\"event\":\"error\",\"message\":" << json_string(message)
            << "}\n" << std::flush;
}

lingtu::sim::PhysicsScenePlan parse_session_plan(int argc, char **argv) {
  if (argc < 14) {
    throw std::invalid_argument("session mode requires SESSION_ID GENERATION WORLD and at least one robot");
  }

  lingtu::sim::PhysicsScenePlan plan;
  plan.session_id = argv[2];
  if (!valid_id(plan.session_id)) {
    throw std::invalid_argument("session id is invalid");
  }
  plan.model_generation = parse_generation(argv[3]);
  plan.world_model_path = argv[4];
  if (std::string(argv[5]) != "--global-policy") {
    throw std::invalid_argument(
        "session mode requires --global-policy TIMESTEP INTEGRATOR SOLVER ITERATIONS GX GY GZ");
  }
  plan.global_policy.timestep_seconds = parse_finite(argv[6], "global policy timestep");
  if (plan.global_policy.timestep_seconds <= 0.0) {
    throw std::invalid_argument("global policy timestep must be positive");
  }
  plan.global_policy.integrator = parse_integrator(argv[7]);
  plan.global_policy.solver = parse_solver(argv[8]);
  plan.global_policy.iterations = static_cast<int>(
      parse_count(argv[9], "global policy iterations",
                  static_cast<std::size_t>(std::numeric_limits<int>::max())));
  for (std::size_t axis = 0; axis < plan.global_policy.gravity_mps2.size(); ++axis) {
    plan.global_policy.gravity_mps2[axis] =
        parse_finite(argv[10 + static_cast<int>(axis)], "global policy gravity");
  }

  int index = 13;
  while (index < argc) {
    const std::string argument = argv[index];
    if (argument == "--robot") {
      if (index + 10 >= argc) {
        throw std::invalid_argument(
            "each robot requires --robot ID MODEL ATTACH_ROOT PX PY PZ QW QX QY QZ");
      }
      lingtu::sim::PhysicsRobotSpec robot;
      robot.instance_id = argv[index + 1];
      robot.model_path = argv[index + 2];
      robot.attach_root = argv[index + 3];
      for (std::size_t axis = 0; axis < robot.spawn.position_m.size(); ++axis) {
        robot.spawn.position_m[axis] =
            parse_finite(argv[index + 4 + static_cast<int>(axis)],
                         "spawn position");
      }
      for (std::size_t axis = 0;
           axis < robot.spawn.quaternion_wxyz.size(); ++axis) {
        robot.spawn.quaternion_wxyz[axis] =
            parse_finite(argv[index + 7 + static_cast<int>(axis)],
                         "spawn quaternion");
      }
      index += 11;
      if (index < argc &&
          std::string(argv[index]) == "--initial-keyframe") {
        if (index + 1 >= argc || std::string(argv[index + 1]).empty()) {
          throw std::invalid_argument(
              "--initial-keyframe requires a non-empty name");
        }
        robot.initial_keyframe = argv[index + 1];
        index += 2;
      }
      plan.robots.push_back(std::move(robot));
      continue;
    }
    if (argument == "--payload") {
      if (index + 21 >= argc) {
        throw std::invalid_argument(
            "each payload requires --payload ID NAMESPACE ROBOT_ID PACKAGE_ID PACKAGE_VERSION PACKAGE_KIND PACKAGE_MANIFEST PARENT_FRAME PARENT_BODY PX PY PZ QW QX QY QZ MODEL.xml ATTACH_ROOT AUTHORITY COLLISION FRAME_COUNT [FRAME_NAME FRAME_ROLE FRAME_PARENT...]");
      }
      lingtu::sim::PhysicsPayloadSpec payload;
      payload.instance_id = argv[index + 1];
      payload.namespace_name = argv[index + 2];
      payload.robot_instance_id = argv[index + 3];
      payload.package.id = argv[index + 4];
      payload.package.version = argv[index + 5];
      payload.package.kind = argv[index + 6];
      payload.package.manifest = argv[index + 7];
      payload.parent_frame = argv[index + 8];
      payload.parent_body = argv[index + 9];
      for (std::size_t axis = 0;
           axis < payload.mount_transform.position_m.size(); ++axis) {
        payload.mount_transform.position_m[axis] = parse_finite(
            argv[index + 10 + static_cast<int>(axis)],
            "payload mount position");
      }
      for (std::size_t axis = 0;
           axis < payload.mount_transform.quaternion_wxyz.size(); ++axis) {
        payload.mount_transform.quaternion_wxyz[axis] = parse_finite(
            argv[index + 13 + static_cast<int>(axis)],
            "payload mount quaternion");
      }
      payload.model.mjcf_path = argv[index + 17];
      payload.model.attach_root = argv[index + 18];
      payload.authority = argv[index + 19];
      payload.collision_representation = argv[index + 20];
      const std::size_t frame_count =
          parse_count(argv[index + 21], "payload frame count", 10000);
      index += 22;
      if (frame_count > static_cast<std::size_t>((argc - index) / 3)) {
        throw std::invalid_argument(
            "payload frame count does not match its frame fields");
      }
      payload.frames.reserve(frame_count);
      for (std::size_t frame_index = 0; frame_index < frame_count;
           ++frame_index) {
        lingtu::sim::PhysicsPayloadFrameSpec frame;
        frame.name = argv[index];
        frame.role = argv[index + 1];
        frame.parent_frame =
            std::string(argv[index + 2]) == "-" ? "" : argv[index + 2];
        payload.frames.push_back(std::move(frame));
        index += 3;
      }
      plan.payloads.push_back(std::move(payload));
      continue;
    }
    if (argument == "--kinematic-entity") {
      if (index + 10 >= argc) {
        throw std::invalid_argument(
            "each kinematic entity requires --kinematic-entity ID MODEL ATTACH_ROOT PX PY PZ QW QX QY QZ");
      }
      lingtu::sim::PhysicsKinematicEntitySpec entity;
      entity.entity_id = argv[index + 1];
      entity.model_path = argv[index + 2];
      entity.attach_root = argv[index + 3];
      for (std::size_t axis = 0;
           axis < entity.initial_transform.position_m.size(); ++axis) {
        entity.initial_transform.position_m[axis] =
            parse_finite(argv[index + 4 + static_cast<int>(axis)],
                         "kinematic initial position");
      }
      for (std::size_t axis = 0;
           axis < entity.initial_transform.quaternion_wxyz.size(); ++axis) {
        entity.initial_transform.quaternion_wxyz[axis] =
            parse_finite(argv[index + 7 + static_cast<int>(axis)],
                         "kinematic initial quaternion");
      }
      plan.kinematic_entities.push_back(std::move(entity));
      index += 11;
      continue;
    }
    throw std::invalid_argument("unsupported session attachment argument");
  }
  if (plan.robots.empty()) {
    throw std::invalid_argument("session mode requires at least one robot");
  }
  return plan;
}

int run_session(lingtu::sim::PhysicsScenePlan plan) {
  auto runtime = lingtu::sim::MujocoRuntime::from_plan(std::move(plan));
  runtime.set_paused(true);
  write_state("ready", runtime.snapshot());

  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      std::istringstream command(line);
      std::string action;
      command >> action;
      if (action == "start") {
        runtime.set_paused(false);
        write_state("running", runtime.snapshot());
      } else if (action == "pause") {
        runtime.set_paused(true);
        write_state("paused", runtime.snapshot());
      } else if (action == "advance") {
        std::string value;
        std::string extra;
        if (!(command >> value) || (command >> extra)) {
          throw std::invalid_argument("advance requires exactly one step count");
        }
        if (runtime.paused()) {
          throw std::runtime_error("cannot advance while paused");
        }
        write_snapshot(runtime.advance(parse_steps(value)));
      } else if (action == "advance-sampled" ||
                 action == "advance-sampled-realtime") {
        std::string value;
        std::string stride_value;
        std::string extra;
        if (!(command >> value)) {
          throw std::invalid_argument(
              "advance-sampled requires a step count and optional sample stride");
        }
        const bool has_stride = static_cast<bool>(command >> stride_value);
        if (command >> extra) {
          throw std::invalid_argument(
              "advance-sampled requires a step count and optional sample stride");
        }
        if (runtime.paused()) {
          throw std::runtime_error("cannot advance while paused");
        }
        const auto steps = parse_steps(value);
        const auto sample_stride = has_stride ? parse_steps(stride_value) : 1U;
        const auto snapshots = advance_sampled(runtime, steps, sample_stride);
        write_snapshot_batch(
            snapshots, action == "advance-sampled-realtime");
      } else if (action == "reset") {
        write_snapshot(runtime.reset());
      } else if (action == "snapshot") {
        write_snapshot(runtime.snapshot());
      } else if (action == "raycast") {
        std::string sensor_frame_id;
        std::string session_id;
        std::string model_generation_text;
        std::string reset_generation_text;
        std::string sequence_text;
        std::string sim_time_text;
        std::string range_min_text;
        std::string range_max_text;
        std::string reflectivity_text;
        std::string line_text;
        std::string count_text;
        if (!(command >> sensor_frame_id >> session_id >>
              model_generation_text >> reset_generation_text >> sequence_text >>
              sim_time_text >> range_min_text >> range_max_text >> reflectivity_text >>
              line_text >> count_text)) {
          throw std::invalid_argument(
              "raycast requires FRAME SESSION_ID MODEL_GENERATION RESET_GENERATION SEQUENCE SIM_TIME_NS RANGE_MIN RANGE_MAX "
              "REFLECTIVITY LINE COUNT DX DY DZ OFFSET_NS...");
        }
        if (!valid_id(session_id)) {
          throw std::invalid_argument("raycast session id is invalid");
        }
        lingtu::sim::MujocoRaycastRequest request;
        request.sensor_frame_id = sensor_frame_id;
        request.session_id = session_id;
        request.model_generation =
            parse_unsigned(model_generation_text, "raycast model generation");
        request.reset_generation =
            parse_uint32(reset_generation_text, "raycast reset generation");
        request.sequence = parse_unsigned(sequence_text, "raycast sequence");
        request.sim_time_ns = parse_unsigned(sim_time_text, "raycast sim time");
        request.range_min_m = parse_finite(range_min_text, "raycast range min");
        request.range_max_m = parse_finite(range_max_text, "raycast range max");
        request.reflectivity_proxy =
            parse_uint8(reflectivity_text, "raycast reflectivity");
        request.unknown_line = parse_uint8(line_text, "raycast line");
        const std::size_t ray_count =
            parse_count(count_text, "raycast ray count", 200000);
        request.rays.reserve(ray_count);
        for (std::size_t index = 0; index < ray_count; ++index) {
          lingtu::sim::MujocoRayDirection ray;
          std::string dx;
          std::string dy;
          std::string dz;
          std::string offset;
          if (!(command >> dx >> dy >> dz >> offset)) {
            throw std::invalid_argument(
                "raycast ray count does not match its payload");
          }
          ray.direction_sensor = {
              parse_finite(dx, "raycast direction"),
              parse_finite(dy, "raycast direction"),
              parse_finite(dz, "raycast direction"),
          };
          ray.offset_time_ns = parse_uint32(offset, "raycast offset time");
          request.rays.push_back(ray);
        }
        std::string extra;
        if (command >> extra) {
          throw std::invalid_argument("raycast contains extra fields");
        }
        write_raycast(sensor_frame_id, runtime.raycast(request));
      } else if (action == "bind-actuators") {
        std::string source_id;
        std::string instance_id;
        std::string command_type;
        std::string timeout_text;
        std::string count_text;
        if (!(command >> source_id >> instance_id >> command_type >> timeout_text >>
              count_text)) {
          throw std::invalid_argument(
              "bind-actuators requires SOURCE INSTANCE TYPE TIMEOUT_NS COUNT CHANNEL...");
        }
        lingtu::sim::ActuatorBindingSpec binding;
        binding.source_id = source_id;
        binding.instance_id = instance_id;
        binding.command_type = command_type;
        binding.stale_timeout_ns =
            parse_unsigned(timeout_text, "stale timeout");
        const std::size_t channel_count =
            parse_count(count_text, "actuator channel count",
                        lingtu::sim::kMaxActuatorValues);
        binding.channels.reserve(channel_count);
        for (std::size_t index = 0; index < channel_count; ++index) {
          std::string channel;
          if (!(command >> channel)) {
            throw std::invalid_argument(
                "bind-actuators channel count does not match its payload");
          }
          binding.channels.push_back(std::move(channel));
        }
        std::string extra;
        if (command >> extra) {
          throw std::invalid_argument("bind-actuators contains extra fields");
        }
        runtime.bind_actuators(binding);
        write_actuator_bound(binding);
      } else if (action == "actuate-advance-sampled-realtime") {
        std::string steps_text;
        std::string stride_text;
        if (!(command >> steps_text >> stride_text)) {
          throw std::invalid_argument(
              "actuate-advance-sampled-realtime requires STEPS STRIDE and an "
              "actuator command");
        }
        const auto steps = parse_steps(steps_text);
        const auto sample_stride = parse_steps(stride_text);
        auto parsed = parse_actuator_command(
            command, "actuate-advance-sampled-realtime");
        const auto result = runtime.apply_command(parsed.envelope);
        std::vector<lingtu::sim::SimulationSnapshot> snapshots;
        if (result == lingtu::sim::ActuatorCommandResult::applied) {
          snapshots = advance_sampled(runtime, steps, sample_stride);
        }
        write_actuator_snapshot_batch(parsed.source_id, parsed.envelope.sequence,
                                      result, snapshots);
      } else if (action == "actuate") {
        auto parsed = parse_actuator_command(command, "actuate");
        write_actuator_result(parsed.source_id, parsed.envelope.sequence,
                              runtime.apply_command(parsed.envelope));
      } else if (action == "kinematic-poses") {
        std::string session_id;
        std::string model_generation_text;
        std::string reset_generation_text;
        std::string sequence_text;
        std::string sim_time_text;
        std::string count_text;
        if (!(command >> session_id >> model_generation_text >>
              reset_generation_text >> sequence_text >> sim_time_text >>
              count_text)) {
          throw std::invalid_argument(
              "kinematic-poses requires SESSION_ID MODEL_GEN RESET_GEN SEQUENCE SIM_TIME_NS COUNT ID BODY_STABLE_ID PX PY PZ QW QX QY QZ...");
        }
        if (!valid_id(session_id)) {
          throw std::invalid_argument("kinematic-poses session id is invalid");
        }
        lingtu::sim::KinematicPoseBatch batch;
        batch.session_id = session_id;
        batch.model_generation =
            parse_unsigned(model_generation_text, "model generation");
        batch.reset_generation =
            parse_uint32(reset_generation_text, "reset generation");
        batch.sequence = parse_unsigned(sequence_text, "scenario sequence");
        batch.sim_time_ns =
            parse_unsigned(sim_time_text, "scenario simulation time");
        const std::size_t entity_count =
            parse_count(count_text, "kinematic entity count", 10000);
        batch.entities.reserve(entity_count);
        for (std::size_t entity_index = 0; entity_index < entity_count;
             ++entity_index) {
          lingtu::sim::KinematicEntityPose pose;
          std::string px;
          std::string py;
          std::string pz;
          std::string qw;
          std::string qx;
          std::string qy;
          std::string qz;
          if (!(command >> pose.entity_id >> pose.body_stable_id >> px >> py >>
                pz >> qw >> qx >> qy >> qz)) {
            throw std::invalid_argument(
                "kinematic entity count does not match its payload");
          }
          pose.position_m = {
              parse_finite(px, "kinematic position"),
              parse_finite(py, "kinematic position"),
              parse_finite(pz, "kinematic position"),
          };
          pose.quaternion_wxyz = {
              parse_finite(qw, "kinematic quaternion"),
              parse_finite(qx, "kinematic quaternion"),
              parse_finite(qy, "kinematic quaternion"),
              parse_finite(qz, "kinematic quaternion"),
          };
          batch.entities.push_back(std::move(pose));
        }
        std::string extra;
        if (command >> extra) {
          throw std::invalid_argument(
              "kinematic-poses contains extra fields");
        }
        write_kinematic_result(batch, runtime.apply_kinematic_poses(batch));
      } else if (action == "stop") {
        write_state("stopped", runtime.snapshot());
        return EXIT_SUCCESS;
      } else {
        throw std::invalid_argument("unsupported command");
      }
    } catch (const std::exception &error) {
      write_error(error.what());
    }
  }
  return EXIT_SUCCESS;
}

int run_legacy(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    throw std::invalid_argument("legacy mode requires MODEL.xml [STEPS]");
  }
  const std::uint32_t steps = argc == 3 ? parse_steps(argv[2]) : 1;
  lingtu::sim::MujocoRuntime runtime(lingtu::sim::MujocoRuntimeConfig{argv[1]});
  const auto &snapshot = runtime.advance(steps);

  std::cout << "{\n"
            << "  \"physics_step\": " << snapshot.physics_step << ",\n"
            << "  \"sequence\": " << snapshot.sequence << ",\n"
            << "  \"sim_time_ns\": " << snapshot.sim_time_ns << ",\n"
            << "  \"reset_generation\": " << snapshot.reset_generation << ",\n"
            << "  \"timestep_seconds\": " << runtime.timestep_seconds() << ",\n"
            << "  \"body_count\": " << snapshot.bodies.size() << "\n"
            << "}\n";
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage:\n"
              << "  lingtu_mujoco_headless MODEL.xml [STEPS]\n"
              << "  lingtu_mujoco_headless --session SESSION_ID GENERATION WORLD.xml "
                 "--global-policy TIMESTEP INTEGRATOR SOLVER ITERATIONS GX GY GZ "
                 "--robot ID MODEL.xml ATTACH_ROOT PX PY PZ QW QX QY QZ "
                 "[--initial-keyframe NAME] "
                 "[--payload ID NAMESPACE ROBOT_ID PACKAGE_ID PACKAGE_VERSION PACKAGE_KIND "
                 "PACKAGE_MANIFEST PARENT_FRAME PARENT_BODY PX PY PZ "
                 "QW QX QY QZ MODEL.xml ATTACH_ROOT AUTHORITY COLLISION "
                 "FRAME_COUNT FRAME_NAME FRAME_ROLE FRAME_PARENT...] [...]\n";
    return EXIT_FAILURE;
  }

  try {
    return std::string(argv[1]) == "--session"
               ? run_session(parse_session_plan(argc, argv))
               : run_legacy(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << "lingtu_mujoco_headless: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
