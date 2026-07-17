#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::map_cleaning {

std::string trim(std::string value);

std::vector<std::string> splitWords(const std::string &line);

std::string jsonEscape(const std::string &value);

std::string genericString(const std::filesystem::path &path);

}  // namespace lingtu::map_cleaning
