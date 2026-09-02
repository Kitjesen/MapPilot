#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "lingtu/maps/json.hpp"
#include "lingtu/maps/mapd/query_core.hpp"
#include "lingtu/maps/mapd/query_protocol.hpp"
#include "lingtu/maps/mapd/query_server.hpp"
#include "lingtu/maps/service.hpp"

namespace {

using lingtu::maps::MapsServiceConfig;
using lingtu::maps::MapsServiceCore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::mapd::query::MapQueryCore;
using lingtu::maps::mapd::query::Opcode;
using lingtu::maps::mapd::query::QueryServer;
using lingtu::maps::mapd::query::QueryServerConfig;
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

bool SendAll(int fd, const unsigned char *data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t sent = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
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

bool RecvAll(int fd, char *data, std::size_t size) {
  std::size_t offset = 0U;
  while (offset < size) {
    const ssize_t got = ::recv(fd, data + offset, size - offset, 0);
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

int Connect(const std::filesystem::path &socket_path) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    assert(fd >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1U);
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0) {
      return fd;
    }
    ::close(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  assert(false && "failed to connect to mapd UDS test server");
  return -1;
}

ClientResponse Query(const std::filesystem::path &socket_path, Opcode opcode,
                     const std::string &json = "{}") {
  const int fd = Connect(socket_path);
  const auto request = RequestBytes(opcode, json);
  assert(SendAll(fd, request.data(), request.size()));

  unsigned char header[16]{};
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
  assert(std::equal(header, header + 4, reinterpret_cast<const unsigned char *>("LTMR")));
  assert(header[4] == 2U);

  ClientResponse response;
  response.status = static_cast<Status>(header[5]);
  response.flags = ReadU16(header + 6);
  const std::uint32_t json_len = ReadU32(header + 8);
  assert(ReadU32(header + 12) == 0U);
  for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&message); cmsg != nullptr;
       cmsg = CMSG_NXTHDR(&message, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_RIGHTS) {
      std::memcpy(&response.fd, CMSG_DATA(cmsg), sizeof(int));
    }
  }
  response.json.resize(json_len);
  assert(RecvAll(fd, response.json.data(), response.json.size()));
  ::close(fd);
  return response;
}

int OpenIdleClient(const std::filesystem::path &socket_path) {
  return Connect(socket_path);
}

std::string ReadFd(int fd) {
  char buffer[64]{};
  const ssize_t got = ::read(fd, buffer, sizeof(buffer));
  assert(got >= 0);
  return std::string(buffer, static_cast<std::size_t>(got));
}

}  // namespace

int main() {
  const auto root = TempRoot();
  const auto socket_path = root / "mapd.sock";
  CreateMap(root, "map_a");
  CreateMap(root, "map_odom", "odom");
  MapsServiceCore service(MapsServiceConfig{MapStoreConfig{root}});
  assert(service.Store().SetActiveMap("map_a", false).ok);

  MapQueryCore query(service);
  QueryServer server(query, QueryServerConfig{socket_path, 1024U * 1024U, 50U});
  server.Start();

  const int idle = OpenIdleClient(socket_path);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto ping = Query(socket_path, Opcode::kPing);
  assert(ping.status == Status::kOk);
  assert(ping.fd < 0);
  assert(ping.json.find("\"action\":\"ping\"") != std::string::npos);
  ::close(idle);

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

  const auto listed = Query(socket_path, Opcode::kService, R"({"action":"list_maps"})");
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
  std::filesystem::remove_all(root);
  return 0;
}
