#include "collision/body.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

namespace c = lingtu::nav::collision;
constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
    throw std::runtime_error(message);
  }
}

bool close(double a, double b) { return std::abs(a - b) < 1e-10; }

c::Part wheel(double spin = 0.0, double tilt = 0.0) {
  c::Part part;
  part.link = "wheel";
  // Local Z is the axle. A radial-frame spin must not change the wheel.
  const double co = std::cos(spin), si = std::sin(spin);
  const double ct = std::cos(tilt), st = std::sin(tilt);
  part.rotation = {co, -si, 0.0, st * si, st * co, ct,
                   -ct * si, -ct * co, st};
  part.shape = c::Shape::Cylinder;
  part.radius_m = 0.10;
  part.half_length_m = 0.025;
  part.half_size = {0.10, 0.10, 0.025};
  part.center.z = 0.10;
  part.support_contact = true;
  return part;
}

void testRoundPrimitiveBounds() {
  const c::Pose pose{{1.0, 2.0, 3.0}, 0.63};
  const auto reference = c::partBounds(wheel(), pose);
  for (double spin : {0.0, 0.25, kPi / 4.0, kPi / 2.0, 2.1}) {
    const auto part = wheel(spin);
    const auto box = c::partBounds(part, pose);
    require(close(box.half_size.x, reference.half_size.x) &&
                close(box.half_size.y, reference.half_size.y) &&
                close(box.half_size.z, reference.half_size.z),
            "wheel spin must not change its collision envelope");
    require(close(c::primitiveLowestZ(part, pose), 3.0),
            "horizontal wheel axle must put the primitive bottom at radius");
    require(close(box.center.z - box.half_size.z, 3.0),
            "canonical wheel box must have the primitive's exact lowest Z");
  }
  auto tilted = wheel(1.1, 0.3);
  const double vertical_radius = std::sin(0.3) * 0.025 + std::cos(0.3) * 0.1;
  require(close(c::primitiveLowestZ(tilted, {}), 0.1 - vertical_radius),
          "tilted cylinder bottom must include axial and radial components");
  const auto near_vertical = wheel(0.7, kPi / 2.0 - 1e-8);
  const auto canonical = c::partBounds(near_vertical, {});
  require(close(canonical.center.z - canonical.half_size.z,
                c::primitiveLowestZ(near_vertical, {})),
          "nearly vertical axes must retain an orthogonal canonical frame");
  auto sphere = tilted;
  sphere.shape = c::Shape::Sphere;
  require(close(c::primitiveLowestZ(sphere, {}), 0.0),
          "sphere bottom must be invariant under primitive rotation");
  auto capsule = tilted;
  capsule.shape = c::Shape::Capsule;
  require(close(c::primitiveLowestZ(capsule, {}),
                -std::sin(0.3) * capsule.half_length_m),
          "capsule bounds must contain the axial segment and spherical cap");
}

void testGeometricGroundContact() {
  const c::Aabb floor{{0,0,-.025},{.025,.025,.025}};
  auto part=wheel(kPi/4);
  require(c::intersects(part,{},floor,.02), "margin must still reach the floor");
  require(c::allowsGroundContact(c::preparePart(part,{},.02),floor),
          "designated contact may touch its geometric bottom");
  require(c::allowsGroundContact(c::preparePart(part,{{0,0,.03},0},.05),floor),
          "raised feet do not need a ground-gap proof");
  require(!c::allowsGroundContact(c::preparePart(part,{{0,0,-.02},0},.02),floor),
          "material penetrating the part is not a bottom contact");
  const c::Aabb side{{0,.045,.1},{.025,.01,.1}};
  const c::Aabb riser{{.08,0,.025},{.025,.025,.025}};
  const auto prepared=c::preparePart(part,{},.02);
  require(!c::allowsGroundContact(prepared,side), "side wall must remain collision");
  require(!c::allowsGroundContact(prepared,riser), "riser must remain collision");
  part.support_contact=false;
  require(!c::allowsGroundContact(c::preparePart(part,{},.02),floor),
          "ordinary body parts have no ground-contact exception");
}

void testCylinderSeparationKeepsTheRoundSurfaceAndMargin() {
  for (double spin : {0.0, .7, kPi / 4}) {
    const auto part = wheel(spin);
    const c::Aabb empty_corner{{.085, 0, .015}, {.005, .01, .005}};
    require(!c::intersects(part, {}, empty_corner),
            "the wheel's enclosing-box corner must not become its round surface");
    const c::Aabb rim{{.1, 0, .1}, {}};
    require(c::intersects(part, {}, rim), "contact with the actual circular rim remains occupied");
    const c::Aabb side_obstacle{{.06, 0, .14}, {.01, .01, .01}};
    require(c::intersects(part, {}, side_obstacle) &&
                !c::allowsGroundContact(c::preparePart(part, {}), side_obstacle),
            "an observed obstacle at14cm must collide with the wheel, not become ground contact");
    const c::Aabb outside_margin{{.15, 0, .1}, {.005, .005, .005}};
    require(!c::intersects(part, {}, outside_margin) &&
                c::intersects(part, {}, outside_margin, 0, .05),
            "round separation must retain the configured world-horizontal margin");
    const c::Aabb past_cap{{0, .04, .1}, {.005, .005, .005}};
    require(!c::intersects(part, {}, past_cap), "finite axle length must retain its end-cap separation");
  }
}

void testRecordedTiltedWheelDoesNotOccupyItsEnclosingBoxCorner() {
  auto part = wheel();
  part.radius_m = .093;
  part.half_length_m = .025585;
  part.center = {.32309387027230946, -.26872413802439127, -.31197902705932373};
  part.rotation = {.5450374689693668, .8384115473072654, .0004845217277037725,
                   .07258035360814873, -.04660753594403127, -.9962729695536977,
                   -.8352641795800797, .5430412644864696, -.08625505999854997};
  const c::Pose pose{{41.96826543548765, 7.278253446772424, .44487599314978515},
                     1.582432552501033};
  const c::Aabb ground_hit{{42.124895095216885, 7.430489538432105, .022677098097513698},
                           {.02191734413147728, .019303799201430216, .01853464529109869}};
  require(!c::intersects(part, pose, ground_hit, 0, .08),
          "recorded R13 tilted wheel must separate from the ground bound with its original8cm margin");
  const auto world = c::preparePart(part, pose);
  const c::Aabb rim{{world.center.x + part.radius_m * world.axes[0].x,
                     world.center.y + part.radius_m * world.axes[0].y,
                     world.center.z + part.radius_m * world.axes[0].z}, {}};
  require(c::intersects(part, pose, rim), "the same tilted wheel must retain actual round-rim contact");
}

}  // namespace

int main() {
  testRoundPrimitiveBounds();
  testGeometricGroundContact();
  testCylinderSeparationKeepsTheRoundSurfaceAndMargin();
  testRecordedTiltedWheelDoesNotOccupyItsEnclosingBoxCorner();
  std::cout << "body contact passed: spin, primitive Z, ground contact, wall, riser\n";
}
