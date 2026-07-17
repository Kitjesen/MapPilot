#include "lingtu/maps/active_slots.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

constexpr const char* kMagic = "LINGTU_ACTIVE_SLOTS 1";

bool SafeValue(const std::string& value) {
  return value.find('\n') == std::string::npos &&
      value.find('\r') == std::string::npos &&
      value.find('=') == std::string::npos;
}

void SyncPath(const std::filesystem::path& path, bool directory) {
#if defined(_WIN32)
  const DWORD flags = directory ? FILE_FLAG_BACKUP_SEMANTICS : FILE_ATTRIBUTE_NORMAL;
  HANDLE handle = CreateFileW(
      path.wstring().c_str(),
      GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      nullptr,
      OPEN_EXISTING,
      flags,
      nullptr);
  if (handle != INVALID_HANDLE_VALUE) {
    FlushFileBuffers(handle);
    CloseHandle(handle);
  }
#else
  const int flags = directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY;
  const int fd = open(path.c_str(), flags);
  if (fd >= 0) {
    fsync(fd);
    close(fd);
  }
#endif
}

void AtomicReplace(const std::filesystem::path& source, const std::filesystem::path& target) {
#if defined(_WIN32)
  if (MoveFileExW(
          source.wstring().c_str(),
          target.wstring().c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    throw std::runtime_error("failed to atomically replace active slots");
  }
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace active slots");
  }
#endif
  SyncPath(target.parent_path(), true);
}

}  // namespace

ActiveSlots::ActiveSlots(std::filesystem::path path, MapIdValidator validator)
    : path_(std::move(path)), validator_(std::move(validator)) {}

bool ActiveSlots::IsValidSlotName(const std::string& slot) {
  if (slot.empty() || slot.size() > 64U) {
    return false;
  }
  if (!std::isalpha(static_cast<unsigned char>(slot.front()))) {
    return false;
  }
  return std::all_of(slot.begin(), slot.end(), [](unsigned char ch) {
    return std::isalnum(ch) || ch == '_' || ch == '-';
  });
}

std::vector<std::string> ActiveSlots::DefaultSlots() {
  return {"navigation", "mapping", "reference"};
}

std::unordered_map<std::string, std::string> ActiveSlots::Load(bool validate_map_ids) const {
  std::unordered_map<std::string, std::string> slots;
  for (const auto& slot : DefaultSlots()) {
    slots.emplace(slot, "");
  }
  std::ifstream file(path_, std::ios::binary);
  if (!file) {
    return slots;
  }
  std::string line;
  if (!std::getline(file, line) || line != kMagic) {
    return slots;
  }
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto sep = line.find('=');
    if (sep == std::string::npos) {
      continue;
    }
    const std::string slot = line.substr(0, sep);
    std::string map_id = line.substr(sep + 1U);
    if (IsValidSlotName(slot) && SafeValue(map_id)) {
      if (validate_map_ids && !map_id.empty() && validator_ && !validator_(map_id)) {
        map_id.clear();
      }
      slots[slot] = map_id;
    }
  }
  return slots;
}

ActiveSlotResult ActiveSlots::Save(
    const std::unordered_map<std::string, std::string>& slots) const {
  try {
    std::filesystem::create_directories(path_.parent_path());
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto temp = path_.parent_path() /
        (path_.filename().string() + ".tmp-" + std::to_string(stamp));
    {
      std::ofstream file(temp, std::ios::binary | std::ios::trunc);
      if (!file) {
        return {false, "failed to open active slots temp file", std::nullopt};
      }
      file << kMagic << '\n';
      std::vector<std::string> names;
      names.reserve(slots.size());
      for (const auto& [slot, _] : slots) {
        names.push_back(slot);
      }
      std::sort(names.begin(), names.end());
      for (const auto& slot : names) {
        const auto map_id = slots.at(slot);
        if (!IsValidSlotName(slot) || !SafeValue(map_id)) {
          return {false, "invalid active slot record", std::nullopt};
        }
        file << slot << '=' << map_id << '\n';
      }
      file.flush();
      if (!file) {
        return {false, "failed to flush active slots", std::nullopt};
      }
    }
    SyncPath(temp, false);
    AtomicReplace(temp, path_);
    return {true, "saved", std::nullopt};
  } catch (const std::exception& exc) {
    return {false, exc.what(), std::nullopt};
  }
}

std::vector<ActiveSlotEntry> ActiveSlots::List() const {
  const auto slots = Load();
  std::vector<ActiveSlotEntry> out;
  out.reserve(slots.size());
  for (const auto& [slot, map_id] : slots) {
    out.push_back(ActiveSlotEntry{slot, map_id});
  }
  std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.slot < rhs.slot;
  });
  return out;
}

std::string ActiveSlots::Get(const std::string& slot) const {
  if (!IsValidSlotName(slot)) {
    return "";
  }
  const auto slots = Load();
  const auto it = slots.find(slot);
  return it == slots.end() ? std::string{} : it->second;
}

ActiveSlotResult ActiveSlots::Set(const std::string& slot, const std::string& map_id) {
  if (!IsValidSlotName(slot)) {
    return {false, "invalid active slot: " + slot, std::nullopt};
  }
  if (map_id.empty() || !SafeValue(map_id) || (validator_ && !validator_(map_id))) {
    return {false, "invalid or unknown map id: " + map_id, std::nullopt};
  }
  auto slots = Load();
  slots[slot] = map_id;
  auto saved = Save(slots);
  if (!saved.ok) {
    return saved;
  }
  return {true, "slot set", ActiveSlotEntry{slot, map_id}};
}

ActiveSlotResult ActiveSlots::Clear(const std::string& slot) {
  if (!IsValidSlotName(slot)) {
    return {false, "invalid active slot: " + slot, std::nullopt};
  }
  auto slots = Load();
  slots[slot] = "";
  auto saved = Save(slots);
  if (!saved.ok) {
    return saved;
  }
  return {true, "slot cleared", ActiveSlotEntry{slot, ""}};
}

ActiveSlotResult ActiveSlots::ReplaceMapId(
    const std::string& old_map_id,
    const std::string& new_map_id) {
  if (old_map_id.empty() || !SafeValue(old_map_id) || !SafeValue(new_map_id)) {
    return {false, "invalid active slot map replacement", std::nullopt};
  }
  auto slots = Load(false);
  bool changed = false;
  for (const auto& [_, map_id] : slots) {
    if (map_id == old_map_id) {
      changed = true;
      break;
    }
  }
  if (!changed) {
    return {true, "no active slots referenced map", std::nullopt};
  }
  if (!new_map_id.empty() && validator_ && !validator_(new_map_id)) {
    return {false, "invalid active slot map replacement", std::nullopt};
  }
  for (auto& [_, map_id] : slots) {
    if (map_id == old_map_id) {
      map_id = new_map_id;
    }
  }
  auto saved = Save(slots);
  if (!saved.ok) {
    return saved;
  }
  return {true, new_map_id.empty() ? "map slots cleared" : "map slots replaced", std::nullopt};
}

ActiveSlotResult ActiveSlots::ClearMapId(const std::string& map_id) {
  return ReplaceMapId(map_id, "");
}

}  // namespace lingtu::maps
