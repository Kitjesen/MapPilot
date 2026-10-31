#include "collision/body.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace lingtu::nav::collision {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kContactToleranceM = 1e-12;

Vec3 add(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 scale(const Vec3 &v, double factor) {
  return {v.x * factor, v.y * factor, v.z * factor};
}

double dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

Vec3 yawRotate(const Vec3 &point, double cosine, double sine) {
  return {cosine * point.x - sine * point.y, sine * point.x + cosine * point.y,
          point.z};
}

struct WorldPart {
  Vec3 center;
  std::array<Vec3, 3> axes;
  std::array<double, 3> half_size;
};

std::array<double, 3> paddedHalfSize(const Part &part, double margin) {
  if (part.shape == Shape::Sphere)
    return {part.radius_m + margin, part.radius_m + margin, part.radius_m + margin};
  if (part.shape == Shape::Cylinder || part.shape == Shape::Capsule)
    return {part.radius_m + margin, part.radius_m + margin,
            part.half_length_m + margin +
                (part.shape == Shape::Capsule ? part.radius_m : 0.0)};
  return {part.half_size.x + margin, part.half_size.y + margin, part.half_size.z + margin};
}

WorldPart transform(const Part &part, const Pose &pose, double margin) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  WorldPart result;
  result.center = add(pose.position, yawRotate(part.center, cosine, sine));
  for (std::size_t axis = 0; axis < result.axes.size(); ++axis) {
    result.axes[axis] = yawRotate(
        {part.rotation[axis], part.rotation[3 + axis], part.rotation[6 + axis]},
        cosine, sine);
  }
  result.half_size = paddedHalfSize(part, margin);
  if (part.shape == Shape::Sphere) {
    result.axes = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  } else if (part.shape == Shape::Cylinder || part.shape == Shape::Capsule) {
    const auto axis = result.axes[2];
    // Rotation about a round primitive's own axis does not change its shape.
    // Align one radial axis with projected gravity so the enclosing box's
    // vertical extent equals the actual cylinder's vertical extent.
    auto radial = cross(axis, cross({0.0, 0.0, 1.0}, axis));
    const double radial_norm = std::sqrt(dot(radial, radial));
    if (radial_norm > 1e-12) {
      radial = scale(radial, 1.0 / radial_norm);
    } else {
      radial = cross({0.0, 1.0, 0.0}, axis);
      radial = scale(radial, 1.0 / std::sqrt(dot(radial, radial)));
    }
    result.axes = {{radial, cross(axis, radial), axis}};
  }
  return result;
}

Aabb worldBounds(const WorldPart &part) {
  Vec3 extent{};
  for (std::size_t i = 0; i < part.axes.size(); ++i) {
    extent.x += std::abs(part.axes[i].x) * part.half_size[i];
    extent.y += std::abs(part.axes[i].y) * part.half_size[i];
    extent.z += std::abs(part.axes[i].z) * part.half_size[i];
  }
  return {part.center, extent};
}

Aabb horizontalExpansion(Aabb box, double margin) {
  box.half_size.x += margin;
  box.half_size.y += margin;
  return box;
}

template <class Geometry>
bool separates(const Geometry &part, const Aabb &box, const Vec3 &difference,
               const Vec3 &axis, Shape shape, double primitive_radius) {
  const double squared_norm = dot(axis, axis);
  // Parallel edge pairs have no additional separating axis. Skipping a
  // numerically vanishing cross axis is conservative rather than permissive.
  if (squared_norm <= 1e-30) return false;
  double radius = std::abs(axis.x) * box.half_size.x +
                  std::abs(axis.y) * box.half_size.y +
                  std::abs(axis.z) * box.half_size.z;
  if (shape == Shape::Cylinder) {
    const auto perpendicular = cross(axis, part.axes[2]);
    radius += part.half_size[2] * std::abs(dot(axis, part.axes[2])) +
              primitive_radius * std::sqrt(dot(perpendicular, perpendicular));
  } else {
    for (std::size_t i = 0; i < part.axes.size(); ++i)
      radius += part.half_size[i] * std::abs(dot(axis, part.axes[i]));
  }
  return std::abs(dot(difference, axis)) >
         radius + kContactToleranceM * std::sqrt(squared_norm);
}

Aabb sourceBounds(const Part &part, const WorldPart &world) {
  if (part.shape == Shape::Box) return worldBounds(world);
  if (part.shape == Shape::Sphere) {
    return {world.center, {part.radius_m, part.radius_m, part.radius_m}};
  }
  const auto &axis = world.axes[2];
  const auto extent = [&](double component, double perpendicular_norm) {
    const double radial = part.shape == Shape::Capsule
                              ? part.radius_m
                              : part.radius_m * perpendicular_norm;
    return std::abs(component) * part.half_length_m + radial;
  };
  return {world.center, {extent(axis.x, std::hypot(axis.y, axis.z)),
                         extent(axis.y, std::hypot(axis.x, axis.z)),
                         extent(axis.z, std::hypot(axis.x, axis.y))}};
}

template <class Geometry>
bool geometryIntersects(const Geometry &world, Shape shape, double sphereRadius,
                        const Aabb &box) {
  if (shape == Shape::Sphere) {
    const auto offset = subtract(world.center, box.center);
    const double x = std::max(0.0, std::abs(offset.x) - box.half_size.x);
    const double y = std::max(0.0, std::abs(offset.y) - box.half_size.y);
    const double z = std::max(0.0, std::abs(offset.z) - box.half_size.z);
    const double radius = sphereRadius + kContactToleranceM;
    return x * x + y * y + z * z <= radius * radius;
  }
  const Vec3 difference = subtract(box.center, world.center);
  const std::array<Vec3, 3> box_axes{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0},
                                      {0.0, 0.0, 1.0}}};
  for (const auto &axis : world.axes) {
    if (separates(world, box, difference, axis, shape, sphereRadius)) return false;
  }
  for (const auto &axis : box_axes) {
    if (separates(world, box, difference, axis, shape, sphereRadius)) return false;
  }
  for (const auto &part_axis : world.axes) {
    for (const auto &box_axis : box_axes) {
      if (separates(world, box, difference, cross(part_axis, box_axis), shape, sphereRadius)) return false;
    }
  }
  if (shape == Shape::Cylinder) {
    const Vec3 nearest{
        std::clamp(world.center.x, box.center.x-box.half_size.x, box.center.x+box.half_size.x)-world.center.x,
        std::clamp(world.center.y, box.center.y-box.half_size.y, box.center.y+box.half_size.y)-world.center.y,
        std::clamp(world.center.z, box.center.z-box.half_size.z, box.center.z+box.half_size.z)-world.center.z};
    // These radial axes can separate the round surface from an OBB corner.
    // A finite set of axes can retain false positives, never authorize overlap.
    for (const auto &candidate : {difference, nearest}) {
      const auto radial = subtract(candidate, scale(world.axes[2], dot(candidate, world.axes[2])));
      if (separates(world, box, difference, radial, shape, sphereRadius)) return false;
    }
  }
  return true;
}

}  // namespace

PreparedPart preparePart(const Part &part, const Pose &pose, double margin,
                         double horizontal_margin) {
  const auto world = transform(part, pose, margin);
  auto unpadded = world;
  unpadded.half_size = paddedHalfSize(part, 0.0);
  return {world.center, world.axes, world.half_size, part.shape, part.radius_m + margin,
          part.support_contact, horizontalExpansion(worldBounds(world), horizontal_margin),
          sourceBounds(part, unpadded), worldBounds(unpadded), horizontal_margin};
}

Aabb partBounds(const Part &part, const Pose &pose, double margin, double horizontal_margin) {
  return horizontalExpansion(worldBounds(transform(part, pose, margin)), horizontal_margin);
}

Aabb primitiveBounds(const Part &part, const Pose &pose) {
  return sourceBounds(part, transform(part, pose, 0.0));
}

double primitiveLowestZ(const Part &part, const Pose &pose) {
  const auto box = primitiveBounds(part, pose);
  return box.center.z - box.half_size.z;
}

bool allowsGroundContact(const PreparedPart &part, const Aabb &occupied) {
  return part.support_contact && occupied.center.z + occupied.half_size.z <=
      part.primitive_bounds.center.z - part.primitive_bounds.half_size.z + kContactToleranceM;
}

Aabb bounds(const Body &body, const Pose &pose) {
  if (body.parts.empty()) return {pose.position, {}};
  const double infinity = std::numeric_limits<double>::infinity();
  Vec3 low{infinity, infinity, infinity};
  Vec3 high{-infinity, -infinity, -infinity};
  for (const auto &part : body.parts) {
    const auto box = partBounds(part, pose, body.margin_m, body.horizontal_margin_m);
    low.x = std::min(low.x, box.center.x - box.half_size.x);
    low.y = std::min(low.y, box.center.y - box.half_size.y);
    low.z = std::min(low.z, box.center.z - box.half_size.z);
    high.x = std::max(high.x, box.center.x + box.half_size.x);
    high.y = std::max(high.y, box.center.y + box.half_size.y);
    high.z = std::max(high.z, box.center.z + box.half_size.z);
  }
  return {scale(add(low, high), 0.5), scale(subtract(high, low), 0.5)};
}

bool intersects(const Part &part, const Pose &pose, const Aabb &box, double margin,
                 double horizontal_margin) {
  return geometryIntersects(transform(part, pose, margin), part.shape,
                             part.radius_m + margin, horizontalExpansion(box, horizontal_margin));
}

bool intersects(const PreparedPart &part, const Aabb &box) {
  return geometryIntersects(part, part.shape, part.radius_m,
                             horizontalExpansion(box, part.horizontal_margin_m));
}

double shortestYawDelta(double from, double to) {
  return std::remainder(to - from, 2.0 * kPi);
}

double sweepRadius(const Body &body) {
  double radius = 0.0;
  const Pose body_frame{};
  for (const auto &part : body.parts) {
    const auto world = transform(part, body_frame, body.margin_m);
    for (int x : {-1, 1}) {
      for (int y : {-1, 1}) {
        for (int z : {-1, 1}) {
          auto corner = world.center;
          corner = add(corner, scale(world.axes[0], x * world.half_size[0]));
          corner = add(corner, scale(world.axes[1], y * world.half_size[1]));
          corner = add(corner, scale(world.axes[2], z * world.half_size[2]));
          radius = std::max(radius, std::hypot(corner.x, corner.y));
        }
      }
    }
  }
  return radius + std::sqrt(2.0) * body.horizontal_margin_m;
}

double sweepBound(const Body &body, const Pose &from, const Pose &to) {
  return sweepBound(sweepRadius(body), from, to);
}

double sweepBound(double radius, const Pose &from, const Pose &to) {
  const auto translation = subtract(to.position, from.position);
  return std::sqrt(dot(translation, translation)) +
         radius * std::abs(shortestYawDelta(from.yaw, to.yaw));
}

}  // namespace lingtu::nav::collision
