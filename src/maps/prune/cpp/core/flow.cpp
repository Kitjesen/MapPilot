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
          "map.pcd + patches/*.pcd + poses.txt",
          "source map points + posed scan frames",
          "prune_v1",
          "Reads LingTu saved-map artifacts and matches patches to poses.",
      },
      {
          "label",
          "partial",
          "posed scan frames",
          "ground marks + future instance marks",
          "prune_v1",
          "Ground is currently a simple local-z threshold; instance labels are not product-ready.",
      },
      {
          "submap",
          "partial",
          "posed scan frames",
          "voxel evidence grid",
          "prune_v1",
          "Current implementation accumulates one global evidence grid; explicit submap windows "
          "come next.",
      },
      {
          "evidence",
          "ready",
          "voxelized posed scans",
          "hits + ground_hits + frame_count",
          "prune_v1",
          "Temporal pseudo occupancy is implemented with conservative voxel support.",
      },
      {
          "protect",
          "partial",
          "voxel evidence",
          "protected static voxels",
          "prune_v1",
          "Protects ground-like, multi-frame, and high-hit voxels; stronger terrain protection "
          "comes next.",
      },
      {
          "score",
          "partial",
          "protected voxels + instance marks",
          "moving-object instance scores",
          "prune_v1",
          "Scores XY instance cells by non-protected candidate ratio; this is report-only for "
          "now.",
      },
      {
          "split",
          "ready",
          "source map points + protected voxels",
          "static points + removed candidate points",
          "prune_v1",
          "Splits map.pcd into kept and removed PCD outputs.",
      },
      {
          "save",
          "ready",
          "static points + removed candidate points",
          "map.clean.pcd + map.removed.pcd + optional map.pcd replacement",
          "prune_v1",
          "Apply mode backs up map.pcd to map.pcd.preclean before replacing it.",
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
