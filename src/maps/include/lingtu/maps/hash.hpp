#pragma once

#include <filesystem>
#include <string>

namespace lingtu::maps {

std::string Sha256File(const std::filesystem::path& path);
std::string Sha256FileDescriptor(int fd);
std::string Sha256Text(const std::string& value);

}  // namespace lingtu::maps
