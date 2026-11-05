#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <afunix.h>
#else
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

#include "lingtu/maps/io/json.hpp"
#include "mapd/query.hpp"
#include "lingtu/maps/mapd/protocol.hpp"
#include "mapd/server.hpp"
#include "lingtu/maps/service.hpp"

namespace {

using lingtu::maps::MapsServiceConfig;
using lingtu::maps::MapsServiceCore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::mapd::query::MapQueryCore;
using lingtu::maps::mapd::query::Opcode;
using lingtu::maps::mapd::query::QueryServer;
using lingtu::maps::mapd::query::QueryServerConfig;
using lingtu::maps::mapd::query::QuerySocket;
using lingtu::maps::mapd::query::kInvalidQuerySocket;
using lingtu::maps::mapd::query::Status;

struct ClientResponse {
  Status status{Status::kError};
  std::uint16_t flags{0U};
  std::string json;
  int fd{-1};
};

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root =
      std::filesystem::temp_directory_path() / ("lingtu_mapd_uds_test_" + std::to_string(stamp));
  std::filesystem::create_directories(root);
  return root;
}

void Write(const std::filesystem::path &path, const std::string &value) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << value;
  assert(output.good());
}

void CreateMap(const std::filesystem::path &root, const std::string &id,
               const std::string &frame = "map") {
  const auto map = root / id;
  Write(map / "map.pcd", "pcd payload");
  Write(map / "occupancy.npz", "occupancy payload");
  Write(map / "metadata.json", std::string{"{\"frame_id\":\"" + frame + "\",\"artifacts\":{"} +
                                   "\"map_pcd\":{\"path\":\"map.pcd\"},"
                                   "\"occupancy_grid\":{\"path\":\"occupancy.npz\"}}}");
}

void PutU16(std::vector<unsigned char> &data, std::uint16_t value) {
  data.push_back(static_cast<unsigned char>((value >> 8U) & 0xffU));
  data.push_back(static_cast<unsigned char>(value & 0xffU));
}

void PutU32(std::vector<unsigned char> &data, std::uint32_t value) {
  data.push_back(static_cast<unsigned char>((value >> 24U) & 0xffU));
  data.push_back(static_cast<unsigned char>((value >> 16U) & 0xffU));
  data.push_back(static_cast<unsigned char>((value >> 8U) & 0xffU));
  data.push_back(static_cast<unsigned char>(value & 0xffU));
}

std::uint16_t ReadU16(const unsigned char *data) {
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[0]) << 8U) |
                                    static_cast<std::uint16_t>(data[1]));
}

std::uint32_t ReadU32(const unsigned char *data) {
  return (static_cast<std::uint32_t>(data[0]) << 24U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[2]) << 8U) | static_cast<std::uint32_t>(data[3]);
}

std::vector<unsigned char> RequestBytes(Opcode opcode, const std::string &json = "{}",
                                        const std::string &request_id = "test-request") {
  std::vector<unsigned char> out;
  out.insert(out.end(), {'L', 'T', 'M', 'P'});
  out.push_back(2U);
  out.push_back(static_cast<unsigned char>(opcode));
  PutU16(out, 0U);
  PutU32(out, 2000U);
  PutU16(out, static_cast<std::uint16_t>(request_id.size()));
  PutU32(out, static_cast<std::uint32_t>(json.size()));
  out.insert(out.end(), request_id.begin(), request_id.end());
  out.insert(out.end(), json.begin(), json.end());
  return out;
}

void CloseSocket(QuerySocket fd) {
#if defined(_WIN32)
  ::closesocket(fd);
#else
  ::close(fd);
#endif
}

bool SendAll(QuerySocket fd, const unsigned char *data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
#if defined(_WIN32)
    const int sent = ::send(fd, reinterpret_cast<const char*>(data + offset),
                           static_cast<int>(size - offset), 0);
#else
    const ssize_t sent = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
#endif
    if (sent < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

bool RecvAll(QuerySocket fd, char *data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
#if defined(_WIN32)
    const int got = ::recv(fd, data + offset, static_cast<int>(size - offset), 0);
#else
    const ssize_t got = ::recv(fd, data + offset, size - offset, 0);
#endif
    if (got <= 0) {
      if (got < 0 && errno == EINTR) {
        continue;
      }
      return false;
    }
    offset += static_cast<std::size_t>(got);
  }
  return true;
}

QuerySocket Connect(const std::filesystem::path &socket_path) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
#if defined(_WIN32)
    const QuerySocket fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
#else
    const QuerySocket fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
#endif
    assert(fd != kInvalidQuerySocket);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path.u8string().c_str(), sizeof(address.sun_path) - 1U);
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0) {
      return fd;
    }
    CloseSocket(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(false && "failed to connect to mapd UDS test server");
  return kInvalidQuerySocket;
}

ClientResponse Query(const std::filesystem::path &socket_path, Opcode opcode,
                     const std::string &json = "{}") {
  const QuerySocket fd = Connect(socket_path);
  const auto request = RequestBytes(opcode, json);
  assert(SendAll(fd, request.data(), request.size()));

  unsigned char header[16]{};
#if defined(_WIN32)
  assert(RecvAll(fd, reinterpret_cast<char*>(header), sizeof(header)));
#else
  struct iovec iov{};
  iov.iov_base = header;
  iov.iov_len = sizeof(header);
  alignas(struct cmsghdr) unsigned char control[CMSG_SPACE(sizeof(int))]{};
  struct msghdr message{};
  message.msg_iov = &iov;
  message.msg_iovlen = 1;
  message.msg_control = control;
  message.msg_controllen = sizeof(control);
  const ssize_t header_size = ::recvmsg(fd, &message, 0);
  assert(header_size > 0);
  assert(header_size <= static_cast<ssize_t>(sizeof(header)));
  if (header_size < static_cast<ssize_t>(sizeof(header))) {
    assert(RecvAll(fd, reinterpret_cast<char *>(header) + header_size,
                   sizeof(header) - static_cast<std::size_t>(header_size)));
  }
#endif
  assert(std::equal(header, header + 4, reinterpret_cast<const unsigned char *>("LTMR")));
  assert(header[4] == 2U);

  ClientResponse response;
  response.status = static_cast<Status>(header[5]);
  response.flags = ReadU16(header + 6);
  const std::uint32_t json_len = ReadU32(header + 8);
  assert(ReadU32(header + 12) == 0U);
#if !defined(_WIN32)
  for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&message); cmsg != nullptr;
       cmsg = CMSG_NXTHDR(&message, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_RIGHTS) {
      std::memcpy(&response.fd, CMSG_DATA(cmsg), sizeof(int));
    }
  }
#endif
  response.json.resize(json_len);
  assert(RecvAll(fd, response.json.data(), response.json.size()));
  CloseSocket(fd);
  return response;
}

QuerySocket OpenIdleClient(const std::filesystem::path &socket_path) {
  return Connect(socket_path);
}

#if !defined(_WIN32)
std::string ReadFd(int fd) {
  char buffer[64]{};
  const ssize_t got = ::read(fd, buffer, sizeof(buffer));
  assert(got >= 0);
  return std::string(buffer, static_cast<std::size_t>(got));
}
#endif

}  // namespace

int main() {
  const auto root = TempRoot();
  const auto socket_path = root / "mapd.sock";
  const auto maps_root = root / "maps";
  CreateMap(maps_root, "map_a");
  CreateMap(maps_root, "map_odom", "odom");
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{maps_root}});
  assert(service.Store().SetActiveMap("map_a", false).ok);

  lingtu::maps::SaveMapRequest capture_request;
  capture_request.request_id = "source-job";
  capture_request.map_id = "captured-map";
  capture_request.auto_process = false;
  const auto begun = service.BeginSaveMapJson(capture_request);
  assert(lingtu::maps::JsonObjectBoolAtPath(begun, {"accepted"}) == true);
  const auto capture_dir = lingtu::maps::JsonObjectStringAtPath(begun, {"status", "capture_dir"});
  assert(capture_dir.has_value());
  Write(std::filesystem::path(*capture_dir) / "map.pcd", "original pcd payload");
  lingtu::maps::MapSnapshot snapshot;
  snapshot.snapshot_id = "source-snapshot";
  snapshot.source_dir = *capture_dir;
  snapshot.frame_id = "map";
  snapshot.slam_boot_id = "slam-source-test";
  snapshot.product_session_id = "source-session";
  snapshot.reset_epoch = 1;
  snapshot.first_sequence = 1;
  snapshot.last_sequence = 1;
  snapshot.observation_sequence = 1;
  snapshot.source_point_count = 1;
  snapshot.slam_healthy = true;
  const auto captured = service.ProvideSaveMapSnapshotJson("source-job", snapshot);
  assert(lingtu::maps::JsonObjectBoolAtPath(captured, {"accepted"}) == true);
  assert(lingtu::maps::JsonObjectStringAtPath(captured, {"status", "state"}) == "CAPTURED");

  MapQueryCore query(service);
  QueryServer server(query, QueryServerConfig{socket_path, 1024U * 1024U, 50U});
  server.Start();

  const auto source = Query(socket_path, Opcode::kOpenArtifact, R"({"job_id":"source-job"})");
#if defined(_WIN32)
  assert(source.status == Status::kError);
  assert(source.json.find("unsupported_platform") != std::string::npos);
#else
  assert(source.status == Status::kOk);
  assert(source.fd >= 0);
  assert(ReadFd(source.fd) == "original pcd payload");
  ::close(source.fd);
  assert(lingtu::maps::JsonObjectStringAtPath(source.json, {"job_id"}) == "source-job");
  assert(source.json.find(*capture_dir) == std::string::npos);
  assert(source.json.find("content_epoch") == std::string::npos);
  const auto missing_source = Query(socket_path, Opcode::kOpenArtifact, R"({"job_id":"unknown"})");
  assert(missing_source.status == Status::kError);
  assert(missing_source.json.find("saved_source_missing") != std::string::npos);
#endif
  const auto ambiguous_source = Query(socket_path, Opcode::kOpenArtifact,
      R"({"job_id":"source-job","map_id":"map_a","capability":"source_pointcloud"})");
  assert(ambiguous_source.status == Status::kError);
  assert(ambiguous_source.json.find("bad_request") != std::string::npos);

  const QuerySocket idle = OpenIdleClient(socket_path);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto ping = Query(socket_path, Opcode::kPing);
  assert(ping.status == Status::kOk);
  assert(ping.fd < 0);
  assert(ping.json.find("\"action\":\"ping\"") != std::string::npos);
  CloseSocket(idle);

#if defined(_WIN32)
  const auto unsupported = Query(socket_path, Opcode::kOpenArtifact,
                                R"({"map_id":"map_a","capability":"source_pointcloud"})");
  assert(unsupported.status == Status::kError);
  assert(unsupported.flags == 0U);
  assert(unsupported.json.find("\"reason_code\":\"unsupported_platform\"") != std::string::npos);
#else
  auto opened = Query(socket_path, Opcode::kOpenArtifact,
                      R"({"map_id":"map_a","capability":"path_planning_2d"})");
  assert(opened.status == Status::kOk);
  assert((opened.flags & 1U) == 1U);
  assert(opened.fd >= 0);
  assert(ReadFd(opened.fd) == "occupancy payload");
  assert(opened.json.find("\"size_bytes\":17") != std::string::npos);
  ::close(opened.fd);

  auto odom = Query(socket_path, Opcode::kOpenArtifact,
                    R"({"map_id":"map_odom","capability":"source_pointcloud"})");
  assert(odom.status == Status::kOk);
  assert(odom.fd >= 0);
  assert(odom.json.find("\"frame_id\":\"odom\"") != std::string::npos);
  assert(odom.json.find("\"version_id\"") == std::string::npos);
  assert(odom.json.find("\"map_dir\"") == std::string::npos);
  assert(odom.json.find("\"uri\"") == std::string::npos);
  assert(odom.json.find("\"hash\"") == std::string::npos);
  assert(odom.json.find("sha256") == std::string::npos);
  ::close(odom.fd);

  const auto missing = Query(socket_path, Opcode::kOpenArtifact,
                             R"({"map_id":"map_a","capability":"semantic_query"})");
  assert(missing.status == Status::kError);
  assert(missing.fd < 0);
  assert(missing.json.find("\"reason_code\":\"artifact_not_found\"") != std::string::npos);

  const auto missing_map = Query(socket_path, Opcode::kOpenArtifact,
                                 R"({"map_id":"does_not_exist","capability":"source_pointcloud"})");
  assert(missing_map.status == Status::kError);
  assert(missing_map.json.find("\"reason_code\":\"map_not_found\"") != std::string::npos);

  const auto missing_capability =
      Query(socket_path, Opcode::kOpenArtifact, R"({"map_id":"map_a","capability":"bogus"})");
  assert(missing_capability.status == Status::kError);
  assert(missing_capability.json.find("\"reason_code\":\"missing_capability\"") !=
         std::string::npos);

  const auto invalid_map = Query(socket_path, Opcode::kOpenArtifact,
                                 R"({"map_id":"../bad","capability":"source_pointcloud"})");
  assert(invalid_map.status == Status::kError);
  assert(invalid_map.json.find("\"reason_code\":\"invalid_map_id\"") != std::string::npos);
#endif

  const auto listed = Query(socket_path, Opcode::kService, R"({"action":"list_maps"})");
  if (listed.status != Status::kOk) {
    std::cerr << listed.json << '\n';
  }
  assert(listed.status == Status::kOk);
  assert(listed.json.find("map_a") != std::string::npos);

  const auto created =
      Query(socket_path, Opcode::kService, R"({"action":"create_map","map_id":"created"})");
  assert(created.status == Status::kOk);
  const auto record =
      Query(socket_path, Opcode::kService, R"({"action":"get_record","map_id":"created"})");
  assert(record.status == Status::kOk);
  assert(record.json.find("created") != std::string::npos);
  const auto deleted =
      Query(socket_path, Opcode::kService, R"({"action":"delete_map","map_id":"created"})");
  assert(deleted.status == Status::kOk);

  const auto save_without_coordinator =
      Query(socket_path, Opcode::kService,
            R"({"action":"save_map","request_id":"save-1","map_id":"saved"})");
  assert(save_without_coordinator.status == Status::kError);
  assert(save_without_coordinator.json.find("\"reason_code\":\"internal_error\"") !=
         std::string::npos);
  const auto legacy_begin =
      Query(socket_path, Opcode::kService,
            R"({"action":"begin_save_map","request_id":"save-legacy","map_id":"saved"})");
  assert(legacy_begin.status == Status::kError);
  assert(legacy_begin.json.find("\"reason_code\":\"unknown_action\"") != std::string::npos);

  for (const std::string action : {"set_active_map", "clear_active_map"}) {
    const auto activation = Query(socket_path, Opcode::kService,
                                  "{\"action\":\"" + action + "\",\"map_id\":\"map_a\"}");
    assert(activation.status == Status::kError);
    assert(activation.json.find("\"reason_code\":\"unknown_action\"") != std::string::npos);
  }

  server.Stop();
  server.Stop();
  assert(!std::filesystem::exists(socket_path));
  std::filesystem::remove_all(root);
  return 0;
}
