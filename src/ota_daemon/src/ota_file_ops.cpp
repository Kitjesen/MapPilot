// ota_file_ops.cpp — 文件传输 RPC 实现 (Upload/List/Delete/DownloadFromUrl)
// 从 ota_service.cpp 拆分

#include "ota_service.hpp"
#include "utils.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>

namespace ota {

// =====================================================================
// RPC: UploadFile
// =====================================================================
grpc::Status OtaServiceImpl::UploadFile(
    grpc::ServerContext *ctx,
    grpc::ServerReader<robot::v1::UploadFileChunk> *reader,
    robot::v1::UploadFileResponse *response) {

  robot::v1::UploadFileChunk chunk;
  std::string remote_path;
  std::string filename;
  uint64_t total_size = 0;
  std::string expected_sha256;
  uint64_t resume_from = 0;
  std::ofstream output;
  uint64_t bytes_written = 0;

  while (reader->Read(&chunk)) {
    // First chunk: extract metadata
    if (chunk.has_metadata() && remote_path.empty()) {
      const auto &meta = chunk.metadata();
      remote_path = meta.remote_path();
      filename = meta.filename();
      total_size = meta.total_size();
      expected_sha256 = meta.sha256();
      resume_from = meta.resume_from_offset();

      if (remote_path.empty()) {
        response->set_success(false);
        response->set_message("remote_path is required");
        return grpc::Status::OK;
      }
      if (!IsPathAllowed(remote_path)) {
        response->set_success(false);
        response->set_message("Path not in allowed directories: " + remote_path);
        return grpc::Status::OK;
      }
      if (remote_path.find("..") != std::string::npos) {
        response->set_success(false);
        response->set_message("Path traversal not allowed");
        return grpc::Status::OK;
      }

      // Ensure parent dir
      auto dir = remote_path.substr(0, remote_path.rfind('/'));
      std::filesystem::create_directories(dir);

      auto mode = (resume_from > 0) ? (std::ios::binary | std::ios::app)
                                     : (std::ios::binary | std::ios::trunc);
      output.open(remote_path, mode);
      if (!output.is_open()) {
        response->set_success(false);
        response->set_message("Cannot open file for writing: " + remote_path);
        return grpc::Status::OK;
      }

      OtaLogInfo("UploadFile: %s size=%lu resume=%lu", remote_path.c_str(),
               static_cast<unsigned long>(total_size), static_cast<unsigned long>(resume_from));
    }

    // Write data
    if (!chunk.data().empty() && output.is_open()) {
      output.write(chunk.data().data(), chunk.data().size());
      bytes_written += chunk.data().size();
    }
  }

  if (output.is_open()) output.close();

  // SHA256 check
  std::string actual_sha256;
  if (!remote_path.empty()) {
    actual_sha256 = ComputeSHA256(remote_path);
  }

  if (!expected_sha256.empty() && actual_sha256 != expected_sha256) {
    response->set_success(false);
    response->set_message("SHA256 mismatch: expected=" + expected_sha256 +
                          " actual=" + actual_sha256);
    return grpc::Status::OK;
  }

  response->set_success(true);
  response->set_remote_path(remote_path);
  response->set_bytes_received(bytes_written);
  response->set_sha256(actual_sha256);
  response->set_message("Upload complete");
  response->set_resumed_from(resume_from);
  OtaLogInfo("UploadFile complete: %s (%lu bytes)", remote_path.c_str(),
           static_cast<unsigned long>(bytes_written));
  return grpc::Status::OK;
}

// =====================================================================
// RPC: ListRemoteFiles
// =====================================================================
grpc::Status OtaServiceImpl::ListRemoteFiles(
    grpc::ServerContext *, const robot::v1::ListRemoteFilesRequest *request,
    robot::v1::ListRemoteFilesResponse *response) {

  std::string dir = request->directory();
  if (dir.empty() || dir.find("..") != std::string::npos) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_INVALID_REQUEST);
    return grpc::Status::OK;
  }
  if (!IsPathAllowed(dir)) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_FORBIDDEN);
    return grpc::Status::OK;
  }

  try {
    uint64_t total = 0;
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
      if (!entry.is_regular_file()) continue;
      auto *info = response->add_files();
      info->set_path(entry.path().string());
      info->set_filename(entry.path().filename().string());
      info->set_size(entry.file_size());
      total += entry.file_size();

      // Use stat() for file time — avoids C++20 file_clock requirement
      struct stat st;
      stat(entry.path().c_str(), &st);
      auto t = st.st_mtime;
      char tbuf[32];
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%SZ", gmtime(&t));
      info->set_modified_time(tbuf);
    }
    response->set_total_size(total);
    response->set_free_space(GetDiskFreeBytes(dir));
  } catch (const std::exception &e) {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_INTERNAL_ERROR);
    response->mutable_base()->set_error_message(e.what());
  }
  return grpc::Status::OK;
}

// =====================================================================
// RPC: DeleteRemoteFile
// =====================================================================
grpc::Status OtaServiceImpl::DeleteRemoteFile(
    grpc::ServerContext *, const robot::v1::DeleteRemoteFileRequest *request,
    robot::v1::DeleteRemoteFileResponse *response) {

  const std::string &path = request->remote_path();
  if (path.empty() || path.find("..") != std::string::npos || !IsPathAllowed(path)) {
    response->set_success(false);
    response->set_message("Invalid or disallowed path");
    return grpc::Status::OK;
  }

  try {
    if (std::filesystem::remove(path)) {
      response->set_success(true);
      response->set_message("Deleted: " + path);
    } else {
      response->set_success(false);
      response->set_message("File not found: " + path);
    }
  } catch (const std::exception &e) {
    response->set_success(false);
    response->set_message(std::string("Delete failed: ") + e.what());
  }
  return grpc::Status::OK;
}

// =====================================================================
// RPC: DownloadFromUrl
// =====================================================================
grpc::Status OtaServiceImpl::DownloadFromUrl(
    grpc::ServerContext *ctx, const robot::v1::DownloadFromUrlRequest *request,
    grpc::ServerWriter<robot::v1::OtaProgress> *writer) {

  const std::string &url = request->url();
  const std::string &staging_path = request->staging_path();

  if (url.empty() || staging_path.empty()) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("url and staging_path are required");
    writer->Write(p);
    return grpc::Status::OK;
  }
  if (staging_path.find("..") != std::string::npos || !IsPathAllowed(staging_path)) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("Path not allowed: " + staging_path);
    writer->Write(p);
    return grpc::Status::OK;
  }

  auto dir = staging_path.substr(0, staging_path.rfind('/'));
  std::filesystem::create_directories(dir);

  OtaLogInfo("DownloadFromUrl: %s -> %s", url.c_str(), staging_path.c_str());

  // Start progress
  {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_PENDING);
    p.set_progress_percent(0.0f);
    p.set_bytes_total(request->expected_size());
    p.set_message("Starting download...");
    writer->Write(p);
  }

  // Build curl command
  std::string cmd = "curl -fSL --connect-timeout 15 --max-time 3600";
  for (const auto &[key, val] : request->headers()) {
    cmd += " -H '" + key + ": " + val + "'";
  }
  cmd += " -o '" + staging_path + "'";
  cmd += " -w '\\n__CURL_DONE__ %{http_code} %{size_download}'";
  cmd += " '" + url + "' 2>&1";

  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("Failed to execute curl");
    writer->Write(p);
    return grpc::Status::OK;
  }

  char line[1024];
  auto last_progress = std::chrono::steady_clock::now();
  uint64_t expected = request->expected_size();

  while (fgets(line, sizeof(line), pipe) && !ctx->IsCancelled()) {
    if (std::string(line).find("__CURL_DONE__") != std::string::npos) break;

    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_progress).count();
    if (elapsed_ms >= 500 && expected > 0 && FileExists(staging_path)) {
      std::ifstream probe(staging_path, std::ios::binary | std::ios::ate);
      uint64_t cur = static_cast<uint64_t>(probe.tellg());

      robot::v1::OtaProgress p;
      p.set_status(robot::v1::OTA_UPDATE_STATUS_INSTALLING);
      p.set_progress_percent(std::min(static_cast<float>(cur) / expected * 100.0f, 99.9f));
      p.set_bytes_completed(cur);
      p.set_bytes_total(expected);
      p.set_message("Downloading...");
      writer->Write(p);
      last_progress = now;
    }
  }

  int ret = pclose(pipe);

  if (ctx->IsCancelled()) {
    std::filesystem::remove(staging_path);
    return grpc::Status(grpc::CANCELLED, "Download cancelled");
  }

  if (ret != 0) {
    robot::v1::OtaProgress p;
    p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
    p.set_message("curl failed with exit code " + std::to_string(WEXITSTATUS(ret)));
    writer->Write(p);
    return grpc::Status::OK;
  }

  // SHA256 verification
  if (!request->expected_sha256().empty()) {
    robot::v1::OtaProgress v;
    v.set_status(robot::v1::OTA_UPDATE_STATUS_VERIFYING);
    v.set_progress_percent(100.0f);
    v.set_message("Verifying SHA256...");
    writer->Write(v);

    std::string actual = ComputeSHA256(staging_path);
    if (actual != request->expected_sha256()) {
      std::filesystem::remove(staging_path);
      robot::v1::OtaProgress p;
      p.set_status(robot::v1::OTA_UPDATE_STATUS_FAILED);
      p.set_message("SHA256 mismatch");
      writer->Write(p);
      return grpc::Status::OK;
    }
  }

  uint64_t final_size = 0;
  if (FileExists(staging_path)) {
    std::ifstream f(staging_path, std::ios::binary | std::ios::ate);
    final_size = static_cast<uint64_t>(f.tellg());
  }

  OtaLogInfo("DownloadFromUrl complete: %s (%lu bytes)", staging_path.c_str(),
           static_cast<unsigned long>(final_size));

  robot::v1::OtaProgress done;
  done.set_status(robot::v1::OTA_UPDATE_STATUS_SUCCESS);
  done.set_progress_percent(100.0f);
  done.set_bytes_completed(final_size);
  done.set_bytes_total(final_size);
  done.set_message("Download complete: " + staging_path);
  writer->Write(done);
  return grpc::Status::OK;
}

} // namespace ota
