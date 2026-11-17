#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

#include "planner_fixture.hpp"
#include "planning/local/planner.hpp"
#include "planning/local/scan/grid.hpp"

using namespace nav_kernel;

static LocalCollisionMapView observedFreeFixture() {
  LocalCollisionMapView view;
  view.sizeX = 4;
  view.sizeY = 2;
  view.sizeZ = 2;
  view.resolution = .25;
  view.aabbMin = {-.5, -.25, 0};
  view.aabbMax = {.5, .25, .5};
  view.inflatedStorage = std::make_shared<const std::vector<std::uint8_t>>(2, 255U);
  view.inflatedBits = view.inflatedStorage->data();
  view.inflatedBytes = view.inflatedStorage->size();
  view.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(2, 0U);
  view.knownFreeStorage = std::make_shared<const std::vector<std::uint8_t>>(2, 255U);
  return view;
}

TEST(LocalCollisionMapView, AlignedCellFaceRetainsItsPublishedIndex) {
  std::vector<std::uint8_t> bits(3, 0);
  bits[14 / 8] = static_cast<std::uint8_t>(1U << (14 % 8));
  LocalCollisionMapView view;
  view.inflatedBits = bits.data();
  view.inflatedBytes = bits.size();
  view.sizeX = 20;
  view.sizeY = view.sizeZ = 1;
  view.resolution = .1;
  view.aabbMin = {-1, 0, 0};
  view.aabbMax = {1, .1, .1};
  ASSERT_TRUE(view.valid());
  // Subtracting -1 first rounds this cell face below index 14 in binary64.
  EXPECT_TRUE(view.occupied({.4, .05, .05}));
  EXPECT_TRUE(view.occupied(view.planningCellCenter(14)));
  EXPECT_FALSE(view.occupied({.35, .05, .05}));
}

TEST(LocalCollisionMapView, NegativeCellFaceMatchesMapdPublishedBitmap) {
  std::vector<std::uint8_t> bits(50, 0);
  bits[187 / 8] = static_cast<std::uint8_t>(1U << (187 % 8));
  LocalPlanRequest request;
  auto &view = request.environment.collision;
  view.inflatedBits = bits.data();
  view.inflatedBytes = bits.size();
  view.sizeX = 400;
  view.sizeY = view.sizeZ = 1;
  view.resolution = .05;
  view.aabbMin = {-10, 0, 0};
  view.aabbMax = {10, .05, .05};
  const Vec3 point{-12 * view.resolution, .025, .025};
  ASSERT_TRUE(view.valid());
  // Mapd floors the world coordinate before subtracting its integer origin.
  ASSERT_EQ(std::floor(point.x / view.resolution) -
                std::llround(view.aabbMin.x / view.resolution), 187);
  EXPECT_TRUE(view.occupied(point));
  LocalPlannerParams params;
  params.scan.voxelResolution = view.resolution;
  params.scan.cylinderOffset = 0;
  const nav_kernel::local::scan::Grid grid(params, request);
  ASSERT_TRUE(grid.valid()) << grid.reason();
  EXPECT_EQ(grid.inflatedOccupancy(point, 0), 1);
  EXPECT_EQ(grid.inflatedOccupancy({-.575, .025, .025}, 0), 0);
}

TEST(LocalCollisionMapView, ObservedFreeBoxUsesMeasuredVolumeInsteadOfInflation) {
  auto view = observedFreeFixture();
  ASSERT_TRUE(view.valid());
  EXPECT_TRUE(view.observedFreeBox(view.aabbMin, view.aabbMax));
  view.gridFromPlanningTranslation = {10, 20, 30};
  view.gridFromPlanningYaw = .5;
  EXPECT_TRUE(view.observedFreeBox(view.aabbMin, view.aabbMax));
}

TEST(LocalCollisionMapView, ObservedFreeBoxRejectsUnknownEdgesAndMeasuredOccupation) {
  auto view = observedFreeFixture();
  view.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(std::initializer_list<std::uint8_t>{4U, 0U});
  EXPECT_FALSE(view.observedFreeBox({-.25, -.25, 0}, {.25, 0, .25}));
  EXPECT_TRUE(view.observedFreeBox({0, -.25, 0}, {.25, 0, .25}));
  view.measuredOccupiedStorage = view.knownFreeStorage;
  EXPECT_FALSE(view.observedFreeBox({0, -.25, 0}, {.25, 0, .25}));
}

TEST(LocalCollisionMapView, ObservedFreeBoxIncludesAllCellsWithPositiveVolume) {
  auto view = observedFreeFixture();
  view.knownFreeStorage =
      std::make_shared<const std::vector<std::uint8_t>>(std::initializer_list<std::uint8_t>{2U, 0U});
  EXPECT_TRUE(view.observedFreeBox({-.25, -.25, 0}, {0, 0, .25}));
  EXPECT_TRUE(view.observedFreeBox({-.2, -.2, .01}, {-.05, -.01, .24}));
  EXPECT_FALSE(view.observedFreeBox({-.25, -.25, 0}, {.000001, 0, .25}));

  view.sizeY = view.sizeZ = 1;
  view.resolution = .1;
  view.aabbMin = {-.2, 0, 0};
  view.aabbMax = {.2, .1, .1};
  view.inflatedStorage = std::make_shared<const std::vector<std::uint8_t>>(1, 255U);
  view.inflatedBits = view.inflatedStorage->data();
  view.inflatedBytes = 1;
  view.measuredOccupiedStorage = std::make_shared<const std::vector<std::uint8_t>>(1, 0U);
  view.knownFreeStorage = std::make_shared<const std::vector<std::uint8_t>>(1, 4U);
  ASSERT_TRUE(view.valid());
  EXPECT_TRUE(view.observedFreeBox({0, .01, .01}, {.1, .09, .09}));
  EXPECT_FALSE(view.observedFreeBox({0, .01, .01}, {.100001, .09, .09}));
}

TEST(LocalCollisionMapView, ObservedFreeBoxMatchesPublishedCellsInShiftedFiveCentimeterGrid) {
  LocalCollisionMapView view;
  view.sizeX=view.sizeY=200;
  view.sizeZ=100;
  view.resolution=.05;
  view.aabbMin={-3.5,-4.85,-1.6};
  view.aabbMax={6.5,5.15,3.4};
  const std::size_t bytes=(view.cellCount()+7U)/8U;
  view.inflatedStorage=std::make_shared<const std::vector<std::uint8_t>>(bytes,0U);
  view.inflatedBits=view.inflatedStorage->data();
  view.inflatedBytes=bytes;
  view.measuredOccupiedStorage=std::make_shared<const std::vector<std::uint8_t>>(bytes,0U);
  auto free=std::make_shared<std::vector<std::uint8_t>>(bytes,0U);
  for (int z=40;z<=41;++z) for (int y=99;y<=100;++y) for (int x=106;x<=107;++x) {
    const std::size_t linear=(static_cast<std::size_t>(z)*view.sizeY+y)*view.sizeX+x;
    (*free)[linear/8U]|=static_cast<std::uint8_t>(1U<<(linear%8U));
  }
  view.knownFreeStorage=free;
  ASSERT_TRUE(view.valid());
  EXPECT_TRUE(view.observedFreeBox({1.8,.1,.4},{1.9,.2,.5}));
  EXPECT_FALSE(view.observedFreeBox({1.8,.1,.4},{1.900001,.2,.5}));
}

TEST(LocalCollisionMapView, ObservedFreeBoxPreservesFractionalOriginAndRealBoundaryCrossing) {
  for (double origin : {-.025,.025}) {
    auto view=observedFreeFixture();
    view.sizeX=4;
    view.sizeY=view.sizeZ=1;
    view.resolution=.1;
    view.aabbMin={origin,0,0};
    view.aabbMax={origin+.4,.1,.1};
    view.inflatedStorage=std::make_shared<const std::vector<std::uint8_t>>(1,0U);
    view.inflatedBits=view.inflatedStorage->data();
    view.inflatedBytes=1;
    view.measuredOccupiedStorage=std::make_shared<const std::vector<std::uint8_t>>(1,0U);
    view.knownFreeStorage=std::make_shared<const std::vector<std::uint8_t>>(1,1U);
    ASSERT_TRUE(view.valid());
    EXPECT_TRUE(view.observedFreeBox({origin,.01,.01},{origin+.1,.09,.09}));
    EXPECT_FALSE(view.observedFreeBox({origin,.01,.01},{origin+.100001,.09,.09}));
  }
}

TEST(LocalCollisionMapView, ObservedFreeBoxRequiresCoverageAndRawEvidence) {
  auto view = observedFreeFixture();
  EXPECT_FALSE(view.observedFreeBox({-.500001, -.25, 0}, {.5, .25, .5}));
  EXPECT_FALSE(view.observedFreeBox({-.5, -.25, 0}, {.500001, .25, .5}));
  EXPECT_FALSE(view.observedFreeBox({0, 0, 0}, {0, .1, .1}));
  EXPECT_FALSE(view.observedFreeBox({0, 0, 0}, {std::numeric_limits<double>::infinity(), .1, .1}));
  view.knownFreeStorage.reset();
  EXPECT_FALSE(view.observedFreeBox(view.aabbMin, view.aabbMax));
  view = observedFreeFixture();
  view.measuredOccupiedStorage.reset();
  EXPECT_FALSE(view.observedFreeBox(view.aabbMin, view.aabbMax));
  view = observedFreeFixture();
  view.knownFreeStorage = std::make_shared<const std::vector<std::uint8_t>>(1, 255U);
  EXPECT_FALSE(view.observedFreeBox(view.aabbMin, view.aabbMax));
}

TEST(LocalCollisionMapView, ObservedClearBoxDistinguishesPartialFreeUnknownAndOccupied) {
  auto view=observedFreeFixture();
  const Vec3 low{0,-.25,0},high{.5,.25,.5};
  view.knownFreeStorage=std::make_shared<const std::vector<std::uint8_t>>(
      std::initializer_list<std::uint8_t>{0xccU,0x0cU});
  EXPECT_TRUE(view.observedClearBox(low,high));
  EXPECT_FALSE(view.observedFreeBox(low,high));
  view.knownFreeStorage=std::make_shared<const std::vector<std::uint8_t>>(2,0U);
  EXPECT_FALSE(view.observedClearBox(low,high));
  EXPECT_FALSE(view.observedFreeBox(low,high));
  view.knownFreeStorage=std::make_shared<const std::vector<std::uint8_t>>(
      std::initializer_list<std::uint8_t>{0xccU,0x0cU});
  view.measuredOccupiedStorage=std::make_shared<const std::vector<std::uint8_t>>(
      std::initializer_list<std::uint8_t>{0U,0x40U});
  EXPECT_FALSE(view.observedClearBox(low,high));
  EXPECT_FALSE(view.observedFreeBox(low,high));
}

TEST(LocalCollisionMapView, ObservedClearBoxKeepsCoverageAndPositiveVolumeBoundaries) {
  auto view=observedFreeFixture();
  view.sizeY=view.sizeZ=1;
  view.resolution=.1;
  view.aabbMin={-.2,0,0};
  view.aabbMax={.2,.1,.1};
  view.inflatedStorage=std::make_shared<const std::vector<std::uint8_t>>(1,0U);
  view.inflatedBits=view.inflatedStorage->data();
  view.inflatedBytes=1;
  view.measuredOccupiedStorage=std::make_shared<const std::vector<std::uint8_t>>(1,8U);
  view.knownFreeStorage=std::make_shared<const std::vector<std::uint8_t>>(1,4U);
  ASSERT_TRUE(view.valid());
  EXPECT_TRUE(view.observedClearBox({0,.01,.01},{.1,.09,.09}));
  EXPECT_TRUE(view.observedClearBox({.01,.01,.01},{.09,.09,.09}));
  EXPECT_FALSE(view.observedClearBox({0,.01,.01},{.100001,.09,.09}));
  EXPECT_FALSE(view.observedClearBox({-.200001,.01,.01},{.1,.09,.09}));
  view.knownFreeStorage.reset();
  EXPECT_FALSE(view.observedClearBox({0,.01,.01},{.1,.09,.09}));
}
