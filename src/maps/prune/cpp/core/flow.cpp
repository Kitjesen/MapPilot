#include "core/flow.hpp"

#include <sstream>

namespace lingtu::map_cleaning {
namespace {

std::string escapeJson(const std::string &value) {
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

}  // namespace

std::vector<StageSpec> productFlow() {
  return {
      {
          "load",
          "ready",
          "map.pcd + patches/*.pcd + poses.txt + calibrated scan origin",
          "source map points + posed scan frames",
          "prune_v2",
          "Reads LingTu saved-map artifacts and matches patches to poses.",
      },
      {
          "label",
          "partial",
          "posed scan frames",
          "ground marks + future instance marks",
          "prune_v2",
          "Ground is currently a simple local-z threshold; instance labels are not product-ready.",
      },
      {
          "submap",
          "partial",
          "posed scan frames",
          "voxel evidence grid",
          "prune_v2",
          "Current implementation accumulates one global evidence grid; explicit submap windows "
          "come next.",
      },
      {
          "evidence",
          "ready",
          "voxelized posed scans",
          "hits + ground_hits + frame_count",
          "prune_v2",
          "Counts later observed free rays once per frame; measured endpoints win within a frame.",
      },
      {
          "protect",
          "partial",
          "voxel evidence",
          "protected static voxels",
          "prune_v2",
          "Keeps unconfirmed, low-height and locally supported planar points. Missing scans and "
          "occlusion are not free evidence; full terrain segmentation is not implemented.",
      },
      {
          "score",
          "partial",
          "protected voxels + instance marks",
          "moving-object instance scores",
          "prune_v2",
          "Scores XY instance cells by non-protected candidate ratio; this is report-only for "
          "now.",
      },
      {
          "split",
          "ready",
          "source map points + protected voxels",
          "static points + removed candidate points",
          "prune_v2",
          "Splits map.pcd into kept and removed PCD outputs.",
      },
      {
          "save",
          "ready",
          "static points + removed candidate points",
          "map.clean.pcd + map.removed.pcd + optional map.pcd replacement",
          "prune_v2",
          "Dry-run writes no files. Apply refuses empty output and preserves the first map.pcd.preclean backup.",
      },
  };
}

std::string flowJson() {
  std::ostringstream out;
  const std::vector<StageSpec> flow = productFlow();
  out << "[";
  for (std::size_t i = 0; i < flow.size(); ++i) {
    const StageSpec &stage = flow[i];
    if (i > 0) {
      out << ",";
    }
    out << "\n    {";
    out << "\"id\":\"" << escapeJson(stage.id) << "\",";
    out << "\"state\":\"" << escapeJson(stage.state) << "\",";
    out << "\"input\":\"" << escapeJson(stage.input) << "\",";
    out << "\"output\":\"" << escapeJson(stage.output) << "\",";
    out << "\"owner\":\"" << escapeJson(stage.owner) << "\",";
    out << "\"note\":\"" << escapeJson(stage.note) << "\"";
    out << "}";
  }
  out << "\n  ]";
  return out.str();
}

}  // namespace lingtu::map_cleaning
