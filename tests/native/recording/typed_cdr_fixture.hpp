#pragma once
#include "lingtu/recording/topic_catalog.hpp"
#if __has_include("dds/cdr/dds_cdrstream.h")
#include "dds/cdr/dds_cdrstream.h"
#define LINGTU_CDR_STREAM_ALLOCATOR 1
#else
#include "dds/ddsi/ddsi_cdrstream.h"
#endif
#include <stdexcept>
namespace lingtu::recording::test {
inline void serialization_require(bool condition, const char *message) { if(!condition)throw std::runtime_error(message); }
inline std::vector<std::byte> serialize(const TopicBinding &binding, const void *sample) {
  dds_ostreamLE_t stream;
#ifdef LINGTU_CDR_STREAM_ALLOCATOR
  const dds_cdrstream_allocator allocator{dds_alloc, dds_realloc, dds_free};
  dds_cdrstream_desc descriptor{};
  dds_cdrstream_desc_from_topic_desc(&descriptor, binding.descriptor);
  dds_ostreamLE_init(&stream, &allocator, 256, 1);
  serialization_require(dds_stream_write_sampleLE(&stream, &allocator, sample, &descriptor), "fixture serialization failed");
#else
  dds_ostreamLE_init(&stream, 256, 1);
  serialization_require(dds_stream_writeLE(&stream, static_cast<const char *>(sample), binding.descriptor->m_ops) != nullptr, "fixture serialization failed");
#endif
  std::vector<std::byte> bytes{std::byte{0}, std::byte{1}, std::byte{0}, std::byte{0}};
  const auto *begin = reinterpret_cast<const std::byte *>(stream.x.m_buffer);
  bytes.insert(bytes.end(), begin, begin + stream.x.m_index);
#ifdef LINGTU_CDR_STREAM_ALLOCATOR
  dds_ostreamLE_fini(&stream, &allocator);
  dds_cdrstream_desc_fini(&descriptor, &allocator);
#else
  dds_ostreamLE_fini(&stream);
#endif
  return bytes;
}
}
