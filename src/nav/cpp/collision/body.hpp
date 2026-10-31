#pragma once

#include <array>
#include <string>
#include <vector>

namespace lingtu::nav::collision {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class Shape { Box, Sphere, Cylinder, Capsule };

// Rotation maps the primitive's local axes to the body frame, row-major.
// Cylinder and capsule length lies along local Z. Legacy models remain boxes.
struct Part {
  std::string link;
  Vec3 center{};
  std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  Vec3 half_size{};
  Shape shape{Shape::Box};
  double radius_m{0.0};
  double half_length_m{0.0};
  bool support_contact{false};
};

struct Body {
  std::vector<Part> parts;
  double margin_m{0.02};
  // World XY square clearance; does not enlarge legs toward their support plane.
  double horizontal_margin_m{0.0};
};

struct Pose {
  Vec3 position{};
  double yaw{0.0};
};

struct Aabb {
  Vec3 center{};
  Vec3 half_size{};
};

// A value snapshot for repeated cell queries at one part pose and margin.
// Rebuild it when the part, pose or margin changes; it owns no model state.
struct PreparedPart {
  Vec3 center{};
  std::array<Vec3, 3> axes{};
  std::array<double, 3> half_size{};
  Shape shape{Shape::Box};
  double radius_m{0.0};
  bool support_contact{false};
  Aabb bounds{};
  Aabb primitive_bounds{};
  Aabb unpadded_bounds{};
  double horizontal_margin_m{0.0};
};

[[nodiscard]] PreparedPart preparePart(const Part &part, const Pose &pose,
                                       double margin = 0.0, double horizontal_margin = 0.0);

// Models supplied to these queries have finite transforms, orthonormal
// rotations, nonnegative half sizes and nonnegative margins. Model loading
// checks this once; the navigation hot path does not revalidate each part.
// Expanding all three local half sizes contains the Euclidean margin around
// the original part. It can be more conservative near box corners.
// The independent horizontal margin expands world X/Y only, even for tilted parts.
[[nodiscard]] Aabb partBounds(const Part &part, const Pose &pose, double margin = 0.0,
                               double horizontal_margin = 0.0);
[[nodiscard]] Aabb bounds(const Body &body, const Pose &pose);

// Tight axis-aligned bounds of the source primitive without a safety margin.
[[nodiscard]] Aabb primitiveBounds(const Part &part, const Pose &pose);
[[nodiscard]] double primitiveLowestZ(const Part &part, const Pose &pose);

// Contact parts may touch material entirely below their unpadded bottom.
// This geometric contact rule does not establish a foothold or require ground observations.
[[nodiscard]] bool allowsGroundContact(const PreparedPart &part, const Aabb &occupied);

// Sphere/AABB is exact. Cylinder queries use the round primitive's projection
// on the box axes and two radial axes; an undecided overlap stays occupied.
// Other shapes retain 15-axis OBB/AABB SAT. Boundary contact is occupied.
[[nodiscard]] bool intersects(const Part &part, const Pose &pose, const Aabb &box,
                              double margin = 0.0, double horizontal_margin = 0.0);
[[nodiscard]] bool intersects(const PreparedPart &part, const Aabb &box);

[[nodiscard]] double shortestYawDelta(double from, double to);

// Upper bound on the total travel of every expanded-part point during linear
// translation and shortest-yaw interpolation. Joint poses stay fixed over this
// segment; a new articulated state must provide updated Part transforms.
[[nodiscard]] double sweepBound(const Body &body, const Pose &from, const Pose &to);
// Prepare once for repeated sweeps of the same fixed Body and margin.
[[nodiscard]] double sweepRadius(const Body &body);
[[nodiscard]] double sweepBound(double radius, const Pose &from, const Pose &to);

}  // namespace lingtu::nav::collision
