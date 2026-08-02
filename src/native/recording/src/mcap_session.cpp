#include "lingtu/recording/mcap_session.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "mcap/writer.hpp"

#if defined(_WIN32)
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace lingtu::recording {
namespace {

class DurableFileWriter final : public mcap::IWritable {
 public:
  explicit DurableFileWriter(const std::filesystem::path &path) {
#if defined(_WIN32)
    file_ = _wfopen(path.c_str(), L"wb");
#else
    file_ = std::fopen(path.c_str(), "wb");
#endif
    if (file_ == nullptr) {
      throw std::runtime_error("failed to open MCAP temporary file: " + path.string() + ": " +
                               std::strerror(errno));
    }
  }

  ~DurableFileWriter() override { close_noexcept(); }

  void handleWrite(const std::byte *data, std::uint64_t size) override {
    if (file_ == nullptr || std::fwrite(data, 1, static_cast<std::size_t>(size), file_) != size) {
      throw std::runtime_error("failed to write MCAP data");
    }
    size_ += size;
  }

  void end() override {
    if (file_ == nullptr) {
      return;
    }
    durable_flush();
    if (std::fclose(file_) != 0) {
      file_ = nullptr;
      throw std::runtime_error("failed to close MCAP data file");
    }
    file_ = nullptr;
  }

  void flush() override { durable_flush(); }

  std::uint64_t size() const override { return size_; }

  void close_noexcept() noexcept {
    if (file_ != nullptr) {
      std::fflush(file_);
      std::fclose(file_);
      file_ = nullptr;
    }
  }

 private:
  void durable_flush() {
    if (file_ == nullptr) {
      return;
    }
    if (std::fflush(file_) != 0) {
      throw std::runtime_error("failed to flush MCAP data");
    }
#if defined(_WIN32)
    if (_commit(_fileno(file_)) != 0) {
      throw std::runtime_error("failed to commit MCAP data to disk");
    }
#else
    if (::fdatasync(fileno(file_)) != 0) {
      throw std::runtime_error("failed to sync MCAP data to disk");
    }
#endif
  }

  std::FILE *file_{nullptr};
  std::uint64_t size_{0};
};

void sync_parent_directory(const std::filesystem::path &path) {
#if defined(_WIN32)
  static_cast<void>(path);
#else
  const auto parent = path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path();
  const int fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("failed to open MCAP output directory for sync: " +
                             std::string(std::strerror(errno)));
  }
  const int result = ::fsync(fd);
  const int sync_error = errno;
  ::close(fd);
  if (result != 0) {
    throw std::runtime_error("failed to sync MCAP output directory: " +
                             std::string(std::strerror(sync_error)));
  }
#endif
}

}  // namespace

class McapSessionWriter::Impl {
 public:
  Impl(std::filesystem::path output, std::string idl, std::vector<ChannelDefinition> definitions,
       std::uint64_t chunk_size)
      : final_path(std::move(output)),
        temporary_path(final_path.string() + ".tmp"),
        idl_schema(std::move(idl)) {
    if (idl_schema.empty()) {
      throw std::invalid_argument("IDL schema must not be empty");
    }
    if (definitions.empty()) {
      throw std::invalid_argument("at least one recording channel is required");
    }
    if (chunk_size == 0) {
      throw std::invalid_argument("MCAP chunk size must be positive");
    }
    std::unordered_set<std::string> unique_topics;
    for (const auto &definition : definitions) {
      if (definition.canonical_topic.empty() || definition.wire_topic.empty() ||
          definition.idl_type.empty()) {
        throw std::invalid_argument("recording channel fields must not be empty");
      }
      if (!unique_topics.insert(definition.wire_topic).second) {
        throw std::invalid_argument("duplicate recording channel: " + definition.wire_topic);
      }
    }

    const auto prepared_path = prepare_path();
    sink = std::make_unique<DurableFileWriter>(prepared_path);

    mcap::McapWriterOptions options(kMcapProfile);
    options.library = "lingtu-native-recorder/0.1";
    options.compression = mcap::Compression::None;
    options.chunkSize = chunk_size;
    options.noChunkCRC = false;
    options.enableDataCRC = true;
    writer.open(*sink, options);

    std::unordered_map<std::string, mcap::SchemaId> schemas;
    for (const auto &definition : definitions) {
      auto schema_it = schemas.find(definition.idl_type);
      if (schema_it == schemas.end()) {
        mcap::Schema schema(definition.idl_type, "omgidl", idl_schema);
        writer.addSchema(schema);
        schema_it = schemas.emplace(definition.idl_type, schema.id).first;
      }
      mcap::KeyValueMap metadata{
          {"lingtu.canonical_topic", definition.canonical_topic},
          {"lingtu.idl_type", definition.idl_type},
          {"lingtu.qos_profile", definition.qos_profile},
          {"lingtu.format_version", "1"},
      };
      mcap::Channel channel(definition.wire_topic, "cdr", schema_it->second, metadata);
      writer.addChannel(channel);
      channel_ids.emplace(definition.wire_topic, channel.id);
    }

    const auto metadata_status = writer.write(mcap::Metadata{
        "lingtu.session", {{"format", "native-dds-cdr"}, {"profile", kMcapProfile}}});
    if (!metadata_status.ok()) {
      throw std::runtime_error("failed to write MCAP session metadata: " + metadata_status.message);
    }
  }

  ~Impl() {
    if (!committed) {
      writer.terminate();
      if (sink) {
        sink->close_noexcept();
      }
    }
  }

  std::filesystem::path prepare_path() {
    if (final_path.empty()) {
      throw std::invalid_argument("MCAP output path must not be empty");
    }
    if (std::filesystem::exists(final_path) || std::filesystem::exists(temporary_path)) {
      throw std::runtime_error("MCAP output or temporary path already exists: " +
                               final_path.string());
    }
    const auto parent = final_path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    return temporary_path;
  }

  void write(const RecordedMessage &recorded) {
    if (committed) {
      throw std::logic_error("cannot write to a committed MCAP session");
    }
    const auto it = channel_ids.find(recorded.wire_topic);
    if (it == channel_ids.end()) {
      throw std::invalid_argument("message uses an unregistered channel: " + recorded.wire_topic);
    }
    mcap::Message message;
    message.channelId = it->second;
    message.sequence = recorded.sequence;
    message.logTime = recorded.log_time_ns;
    message.publishTime =
        recorded.publish_time_ns == 0 ? recorded.log_time_ns : recorded.publish_time_ns;
    message.data = recorded.payload.data();
    message.dataSize = recorded.payload.size();
    const auto status = writer.write(message);
    if (!status.ok()) {
      throw std::runtime_error("failed to write MCAP message: " + status.message);
    }
  }

  void commit() {
    if (committed) {
      throw std::logic_error("MCAP session was already committed");
    }
    writer.close();
    std::filesystem::rename(temporary_path, final_path);
    sync_parent_directory(final_path);
    committed = true;
  }

  std::filesystem::path final_path;
  std::filesystem::path temporary_path;
  std::unique_ptr<DurableFileWriter> sink;
  std::string idl_schema;
  mcap::McapWriter writer;
  std::unordered_map<std::string, mcap::ChannelId> channel_ids;
  bool committed{false};
};

McapSessionWriter::McapSessionWriter(std::filesystem::path final_path, std::string idl_schema,
                                     std::vector<ChannelDefinition> channels,
                                     std::uint64_t chunk_size_bytes)
    : impl_(std::make_unique<Impl>(std::move(final_path), std::move(idl_schema),
                                   std::move(channels), chunk_size_bytes)) {}

McapSessionWriter::~McapSessionWriter() = default;

void McapSessionWriter::write(const RecordedMessage &message) {
  impl_->write(message);
}

void McapSessionWriter::commit() {
  impl_->commit();
}

const std::filesystem::path &McapSessionWriter::final_path() const noexcept {
  return impl_->final_path;
}

const std::filesystem::path &McapSessionWriter::temporary_path() const noexcept {
  return impl_->temporary_path;
}

std::string read_text_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open text file: " + path.string());
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

}  // namespace lingtu::recording
