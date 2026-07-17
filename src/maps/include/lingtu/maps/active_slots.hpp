#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace lingtu::maps {

struct ActiveSlotEntry {
  std::string slot;
  std::string map_id;
};

struct ActiveSlotResult {
  bool ok{false};
  std::string message;
  std::optional<ActiveSlotEntry> entry;
};

class ActiveSlots {
 public:
  using MapIdValidator = std::function<bool(const std::string&)>;

  ActiveSlots(std::filesystem::path path, MapIdValidator validator = {});

  static bool IsValidSlotName(const std::string& slot);
  static std::vector<std::string> DefaultSlots();

  std::vector<ActiveSlotEntry> List() const;
  std::string Get(const std::string& slot) const;
  ActiveSlotResult Set(const std::string& slot, const std::string& map_id);
  ActiveSlotResult Clear(const std::string& slot);
  ActiveSlotResult ReplaceMapId(
      const std::string& old_map_id,
      const std::string& new_map_id);
  ActiveSlotResult ClearMapId(const std::string& map_id);

 private:
  std::unordered_map<std::string, std::string> Load(bool validate_map_ids = true) const;
  ActiveSlotResult Save(const std::unordered_map<std::string, std::string>& slots) const;

  std::filesystem::path path_;
  MapIdValidator validator_;
};

}  // namespace lingtu::maps
