#include "core/save.hpp"

#include <system_error>
#include <utility>

#include "core/io.hpp"

namespace fs = std::filesystem;

namespace lingtu::map_cleaning {
namespace {

SaveResult saveFail(std::string reason, std::string message) {
  SaveResult result;
  result.success = false;
  result.reason_code = std::move(reason);
  result.message = std::move(message);
  return result;
}

}  // namespace

SaveResult writeCleanedMap(const SaveOptions &options, const std::vector<PointXYZI> &kept,
                           const std::vector<PointXYZI> &removed) {
  writePcd(options.clean_pcd, kept);
  writePcd(options.removed_pcd, removed);

  if (options.apply_to_map) {
    std::error_code ec;
    if (options.overwrite && fs::exists(options.backup_pcd)) {
      fs::remove(options.backup_pcd, ec);
      if (ec) {
        return saveFail("backup_remove_failed",
                        "failed to remove existing backup: " + options.backup_pcd.string() + ": " +
                            ec.message());
      }
    }
    ec.clear();
    fs::copy_file(options.map_pcd, options.backup_pcd, fs::copy_options::none, ec);
    if (ec) {
      return saveFail("backup_failed", "failed to backup map.pcd to " +
                                           options.backup_pcd.string() + ": " + ec.message());
    }

    if (options.overwrite && fs::exists(options.tmp_map_pcd)) {
      fs::remove(options.tmp_map_pcd, ec);
      if (ec) {
        return saveFail("tmp_remove_failed",
                        "failed to remove stale temp map: " + options.tmp_map_pcd.string() + ": " +
                            ec.message());
      }
    }
    ec.clear();
    fs::copy_file(options.clean_pcd, options.tmp_map_pcd, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      return saveFail("tmp_write_failed", "failed to stage cleaned map: " +
                                              options.tmp_map_pcd.string() + ": " + ec.message());
    }

    ec.clear();
    fs::rename(options.tmp_map_pcd, options.map_pcd, ec);
    if (ec && fs::exists(options.map_pcd)) {
      std::error_code remove_ec;
      fs::remove(options.map_pcd, remove_ec);
      if (!remove_ec) {
        ec.clear();
        fs::rename(options.tmp_map_pcd, options.map_pcd, ec);
      }
    }
    if (ec) {
      std::error_code restore_ec;
      if (!fs::exists(options.map_pcd)) {
        fs::copy_file(options.backup_pcd, options.map_pcd, fs::copy_options::overwrite_existing,
                      restore_ec);
      }
      std::error_code cleanup_ec;
      fs::remove(options.tmp_map_pcd, cleanup_ec);
      return saveFail("replace_failed", "failed to replace map.pcd with cleaned map; backup is " +
                                            options.backup_pcd.string() + ": " + ec.message());
    }
  }

  SaveResult result;
  result.success = true;
  result.reason_code = "saved";
  result.message = "cleaned map outputs written";
  result.backup_pcd = options.apply_to_map ? options.backup_pcd : fs::path();
  return result;
}

}  // namespace lingtu::map_cleaning
