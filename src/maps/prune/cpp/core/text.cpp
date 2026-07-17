#include "core/text.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace lingtu::map_cleaning {

std::string trim(std::string value) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::vector<std::string> splitWords(const std::string &line) {
  std::istringstream in(line);
  std::vector<std::string> out;
  std::string item;
  while (in >> item) {
    out.push_back(item);
  }
  return out;
}

std::string jsonEscape(const std::string &value) {
  std::ostringstream out;
  for (char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string genericString(const std::filesystem::path &path) {
  return path.empty() ? std::string() : path.generic_string();
}

}  // namespace lingtu::map_cleaning
