#include "collision/body.hpp"
#include "collision/body_io.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

using lingtu::nav::collision::Aabb;
using lingtu::nav::collision::Body;
using lingtu::nav::collision::Part;
using lingtu::nav::collision::Pose;
using lingtu::nav::collision::Vec3;
using lingtu::nav::collision::bounds;
using lingtu::nav::collision::intersects;
using lingtu::nav::collision::partBounds;
using lingtu::nav::collision::shortestYawDelta;
using lingtu::nav::collision::sweepBound;

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

bool close(double a, double b) { return std::abs(a - b) < 1e-10; }

std::array<double, 9> rotationZ(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0};
}

bool aabbIntersects(const Aabb &a, const Aabb &b) {
  return std::abs(a.center.x - b.center.x) <= a.half_size.x + b.half_size.x &&
         std::abs(a.center.y - b.center.y) <= a.half_size.y + b.half_size.y &&
         std::abs(a.center.z - b.center.z) <= a.half_size.z + b.half_size.z;
}

void testHeadingAndIntermediateTurn() {
  const Part torso{"torso", {}, rotationZ(0.0), {0.35, 0.164, 0.12}};
  const Aabb wall{{0.0, 0.35, 0.0}, {2.0, 0.05, 1.0}};
  require(!intersects(torso, {{}, 0.0}, wall), "aligned body should fit 0.60 m corridor");
  require(!intersects(torso, {{}, kPi}, wall), "opposite aligned heading should fit");
  require(intersects(torso, {{}, kPi / 2.0}, wall), "turning arc must hit corridor wall");
  const Aabb narrow_wall{{0.0, 0.19, 0.0}, {2.0, 0.05, 1.0}};
  require(intersects(torso, {{}, 0.0}, narrow_wall), "0.28 m opening should reject this stance");
}

void testExactObbAfterBroadBounds() {
  const Part diagonal{"diagonal", {}, rotationZ(kPi / 4.0), {0.5, 0.04, 0.05}};
  const Aabb corner{{0.32, -0.32, 0.0}, {0.02, 0.02, 0.05}};
  require(aabbIntersects(partBounds(diagonal, {}), corner), "fixture must overlap broad bounds");
  require(!intersects(diagonal, {}, corner), "separated OBB must survive overlapping broad bounds");
  require(intersects(diagonal, {}, corner, 0.45), "expanding margin must not shrink the part");
}

void testCrossAxesSeparateTiltedPart() {
  const Part tilted{
      "tilted_leg", {},
      {0.5750168603707414, -0.5075353547305227, 0.6416879880346352,
       0.5920595303917607, 0.7994431637697289, 0.10176512356467891,
       -0.5646424733950354, 0.3214008270064177, 0.7601844418546908},
      {0.35, 0.05, 0.10}};
  const Aabb obstacle{{-0.33660552536937116, -0.14003545830642983, 0.35000896209019405},
                       {0.09, 0.12, 0.08}};
  // The six face axes overlap by at least .0197 m; an edge-cross axis
  // separates by .0516 m. Checking faces alone would incorrectly reject it.
  require(!intersects(tilted, {}, obstacle), "tilted parts require all edge-cross axes");
}

void testWorldTransformAndBodyBounds() {
  const Part offset{"payload", {0.2, 0.1, 0.4}, rotationZ(kPi / 4.0), {0.2, 0.05, 0.1}};
  const Pose pose{{2.0, 3.0, 4.0}, kPi / 2.0};
  const auto box = partBounds(offset, pose);
  require(close(box.center.x, 1.9) && close(box.center.y, 3.2) && close(box.center.z, 4.4),
          "part center must be rotated and translated in the correct order");
  require(close(box.half_size.x, 0.25 / std::sqrt(2.0)) &&
              close(box.half_size.y, 0.25 / std::sqrt(2.0)) && close(box.half_size.z, 0.1),
          "world bounds must include composed part and body rotation");
  const Body body{{offset}, 0.02};
  const auto expanded = bounds(body, pose);
  require(close(expanded.center.x, box.center.x) &&
              expanded.half_size.x > box.half_size.x &&
              close(expanded.half_size.z, 0.12),
          "body bounds must include the configured margin");
  const auto empty = bounds({}, pose);
  require(close(empty.center.x, pose.position.x) && close(empty.half_size.x, 0.0),
          "empty bounds should remain a degenerate box at the supplied pose");
}

void testMarginAndBoundaryContact() {
  const Part box{"box", {}, rotationZ(0.0), {0.2, 0.1, 0.1}};
  const Aabb gap{{0.0, 0.16, 0.0}, {0.5, 0.05, 0.5}};
  require(!intersects(box, {}, gap), "one centimetre gap should remain free without margin");
  require(intersects(box, {}, gap, 0.02), "two centimetre margin should reject the close wall");
  const Aabb contact{{0.0, 0.15, 0.0}, {0.5, 0.05, 0.5}};
  require(intersects(box, {}, contact), "boundary contact must count as occupied");
  const Aabb above{{0.0, 0.0, 0.3}, {0.1, 0.1, 0.05}};
  require(!intersects(box, {}, above), "3D vertical separation must remain free");
  require(intersects(box, {{0.0, 0.0, 0.2}, 0.0}, above), "body height changes must affect collision");
}

void testHorizontalMarginKeepsTiltedVerticalExtent() {
  namespace c = lingtu::nav::collision;
  const double angle = .7;
  const Part leg{"leg", {}, {std::cos(angle), 0, std::sin(angle), 0, 1, 0,
                              -std::sin(angle), 0, std::cos(angle)}, {.3, .04, .08}};
  Body body{{leg}, 0.0};
  const auto original = c::bounds(body, {});
  body.horizontal_margin_m = .05;
  const auto expanded = c::bounds(body, {});
  require(close(expanded.half_size.x, original.half_size.x + .05) &&
              close(expanded.half_size.y, original.half_size.y + .05) &&
              close(expanded.half_size.z, original.half_size.z),
          "horizontal clearance must not push a tilted leg toward the floor");
  const Aabb wall{{0, .10, 0}, {.5, .02, .5}};
  require(!c::intersects(leg, {}, wall) && c::intersects(leg, {}, wall, 0, .05),
          "configured horizontal clearance must reject a nearby side wall");
  const Aabb floor{{0, 0, -original.half_size.z - .015}, {1, 1, .005}};
  require(!c::intersects(leg, {}, floor, 0, .05) && c::intersects(leg, {}, floor, .05),
          "horizontal padding must retain the gap that isotropic padding closes");
  const auto prepared = c::preparePart(leg, {}, 0, .05);
  require(c::intersects(prepared, wall) && !c::intersects(prepared, floor),
          "prepared queries must use the same horizontal-only clearance");
  require(c::sweepRadius(body) >= c::sweepRadius(Body{{leg}, 0}) + .05,
          "sweep coverage must include the horizontal clearance");
}

void testGarageOriginalLeaf() {
  const Part base{"base", {}, rotationZ(0.0), {0.1881, 0.04675, 0.057}};
  const Pose pose{{3.675405740737915, -0.20373751223087311, -1.1807905435562134},
                  -0.090659887607880671};
  const Aabb leaf{{3.45, -0.15, -1.25}, {0.05, 0.05, 0.05}};
  require(intersects(base, pose, leaf), "original Garage leaf must still intersect URDF base");
  auto away = leaf;
  away.center.x -= 0.04;
  require(!intersects(base, pose, away), "moving the leaf away must establish separation");
}

void testShortestRotationSweepBound() {
  const Body body{{Part{"offset", {0.2, 0.0, 0.0}, rotationZ(0.0), {0.3, 0.1, 0.1}}},
                  0.02};
  const Pose from{{0.0, 0.0, 0.0}, 179.0 * kPi / 180.0};
  const Pose to{{0.7, 0.0, 0.0}, -179.0 * kPi / 180.0};
  require(close(shortestYawDelta(from.yaw, to.yaw), 2.0 * kPi / 180.0),
          "rotation across pi must use the shortest angular difference");
  const double bound = sweepBound(body, from, to);
  require(bound >= 0.7 && bound < 0.73, "sweep must bound translation and short rotation");
  require(close(sweepBound(body, from, from), 0.0), "stationary body must have zero sweep bound");
  const Pose rotated{from.position, from.yaw + kPi / 2.0};
  require(sweepBound(body, from, rotated) > 0.8,
          "offset parts and margin must participate in the rotation radius");
}

void testPreparedRadiusRetainsBodyAndMargin() {
  namespace c = lingtu::nav::collision;
  const Body body{{Part{"offset",{.2,0,0},rotationZ(0),{.3,.1,.1}}},.02};
  const double radius=c::sweepRadius(body);
  require(close(radius,std::hypot(.52,.12)),
          "prepared radius must include offset and original body margin");
  require(close(c::sweepBound(radius,{{},0},{{.7,0,0},.5}),.7+radius*.5),
          "prepared sweep bound must retain translation plus rotational travel");
}

void testPreparedPartMatchesSingleQueries() {
  namespace c = lingtu::nav::collision;
  const std::array<std::array<double,9>,3> rotations{{rotationZ(0),rotationZ(.7),
      {0.5750168603707414,-0.5075353547305227,0.6416879880346352,
       0.5920595303917607,0.7994431637697289,0.10176512356467891,
       -0.5646424733950354,0.3214008270064177,0.7601844418546908}}};
  const auto sameBounds=[](const Aabb &a,const Aabb &b) {
    return close(a.center.x,b.center.x) && close(a.center.y,b.center.y) &&
           close(a.center.z,b.center.z) && close(a.half_size.x,b.half_size.x) &&
           close(a.half_size.y,b.half_size.y) && close(a.half_size.z,b.half_size.z);
  };
  for (const auto shape : {c::Shape::Box,c::Shape::Sphere,c::Shape::Cylinder,c::Shape::Capsule})
    for (const auto &rotation : rotations)
      for (const double yaw : {-kPi,-.7,0.,1.7,kPi})
        for (const double margin : {0.,1e-12,.02,.1}) {
          Part part{"prepared",{.12,-.08,.2},rotation,{.31,.07,.13}};
          part.shape=shape;
          part.radius_m=.08;
          part.half_length_m=.12;
          part.support_contact=true;
          const Pose pose{{2.,-3.,.7},yaw};
          const auto prepared=c::preparePart(part,pose,margin);
          require(sameBounds(prepared.bounds,c::partBounds(part,pose,margin)),
                  "prepared bounds must match the expanded single query");
          require(sameBounds(prepared.unpadded_bounds,c::partBounds(part,pose,0)),
                  "prepared support footprint must retain the unpadded conservative box");
          require(sameBounds(prepared.primitive_bounds,c::primitiveBounds(part,pose)),
                  "prepared primitive bounds must retain source shape and zero margin");
          for (int x=-4;x<=4;++x) for (int y=-4;y<=4;++y) for (int z=-3;z<=3;++z) {
            const Aabb cell{{prepared.center.x+x*.09,prepared.center.y+y*.09,
                              prepared.center.z+z*.09},{.035,.025,.04}};
            require(c::intersects(prepared,cell)==c::intersects(part,pose,cell,margin),
                    "prepared collision must match every original shape/pose/margin query");
          }
          for (const double offset : {-2e-12,-.5e-12,0.,.5e-12,2e-12}) {
            const Aabb touching{{prepared.bounds.center.x+prepared.bounds.half_size.x+.025+offset,
                                  prepared.center.y,prepared.center.z},{.025,.025,.025}};
            require(c::intersects(prepared,touching)==c::intersects(part,pose,touching,margin),
                    "prepared collision must retain the original contact tolerance");
          }
          const auto primitive=prepared.primitive_bounds;
          const double bottom=primitive.center.z-primitive.half_size.z;
          const Aabb floor{{primitive.center.x,primitive.center.y,bottom-.025},{.025,.025,.025}};
          require(c::allowsGroundContact(prepared,floor),
                  "contact primitive must allow material below its true bottom");
        }
  const Part part{"snapshot",{},rotationZ(0),{.2,.1,.1}};
  const auto prepared=c::preparePart(part,{},.02);
  require(!c::allowsGroundContact(prepared,{{0,0,-.125},{.025,.025,.025}}),
          "prepared non-contact parts must not gain floor exemptions");
}

}  // namespace

int main(int argc, char **argv) {
  testHeadingAndIntermediateTurn();
  testExactObbAfterBroadBounds();
  testCrossAxesSeparateTiltedPart();
  testWorldTransformAndBodyBounds();
  testMarginAndBoundaryContact();
  testHorizontalMarginKeepsTiltedVerticalExtent();
  testGarageOriginalLeaf();
  testShortestRotationSweepBound();
  testPreparedRadiusRetainsBodyAndMargin();
  testPreparedPartMatchesSingleQueries();
  if (argc==2) {
    std::ifstream input(argv[1]);
    const auto body=lingtu::nav::collision::readBody(input);
    const double radius=lingtu::nav::collision::sweepRadius(*body);
    volatile double sink=0;
    for(int repeat=0;repeat<6;++repeat) for(int order=0;order<2;++order) {
      const bool prepared=(repeat+order)%2!=0;
      const auto started=std::chrono::steady_clock::now();
      double sum=0;
      for(int i=0;i<20000;++i) {
        const Pose from{{},.01*(i%37)},to{{.1,0,0},.02*(i%41)};
        sum+=prepared ? sweepBound(radius,from,to) : sweepBound(*body,from,to);
      }
      sink=sum;
      std::cout << "radius_cost," << repeat << ',' << prepared << ','
                << std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()
                << ',' << sum << '\n';
    }
    (void)sink;
  }
  std::cout << "body geometry passed: heading, turn arc, 15-axis OBB, transforms, margin, Garage, sweep\n";
}
