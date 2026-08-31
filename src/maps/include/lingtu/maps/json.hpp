#pragma once

#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::maps {

bool IsValidJsonObject(std::string_view input) noexcept;
bool JsonObjectHasPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<bool> JsonObjectBoolAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<double> JsonObjectNumberAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<std::string> JsonObjectStringAtPath(
    std::string_view input,
    std::initializer_list<std::string_view> path) noexcept;
std::optional<std::vector<std::string>> JsonObjectPathList(
    std::string_view input,
    std::string_view key) noexcept;

}  // namespace lingtu::maps
