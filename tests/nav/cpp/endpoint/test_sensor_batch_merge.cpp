#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "input/samples.hpp"

namespace {

using lingtu::nav::endpoint::ClearSource;
using lingtu::nav::endpoint::InputSample;
using lingtu::nav::endpoint::LocalCollisionMap;
using lingtu::nav::endpoint::OdometrySample;
using lingtu::nav::endpoint::PlannerClearSample;
using lingtu::nav::endpoint::PointCloudSample;
using lingtu::nav::endpoint::SensorBatch;
using lingtu::nav::endpoint::TransformSample;
using lingtu::nav::endpoint::mergeSensorBatch;

void require(bool ok, const std::string &message) {
  if (!ok) throw std::runtime_error(message);
}

InputSample<OdometrySample> odometry(double stamp_s) {
  InputSample<OdometrySample> sample;
  sample.value.header.stamp_s = stamp_s;
  return sample;
}

InputSample<PointCloudSample> cloud(double stamp_s) {
  InputSample<PointCloudSample> sample;
  sample.value.header.stamp_s = stamp_s;
  return sample;
}

SensorBatch odometryOnly(double receive_s, double stamp_s) {
  SensorBatch batch;
  batch.receive_steady_s = receive_s;
  batch.receive_wall_s = receive_s + 1000.0;
  batch.simulation_time_s = stamp_s;
  batch.odometry.push_back(odometry(stamp_s));
  return batch;
}

void lowRateInputsSurviveOdometryOnlyBatches() {
  SensorBatch pending = odometryOnly(1.000, 10.000);
  InputSample<LocalCollisionMap> collision;
  collision.value.stamp_s = 9.95;
  collision.value.generation = 7;
  pending.local_collision = collision;
  pending.obstacles = cloud(9.96);

  // Ten 1 ms drains of 200 Hz odometry arrive before the next control tick.
  for (int index = 1; index <= 10; ++index) {
    mergeSensorBatch(pending, odometryOnly(1.000 + 0.001 * index, 10.000 + 0.005 * index));
  }

  require(pending.local_collision.has_value() && pending.local_collision->value.generation == 7,
          "an odometry-only batch dropped the pending local collision layer");
  require(pending.obstacles.has_value() && pending.obstacles->value.header.stamp_s == 9.96,
          "an odometry-only batch dropped the pending registered cloud");
  require(pending.odometry.size() == 11U, "odometry samples were not all retained");
  for (std::size_t index = 1; index < pending.odometry.size(); ++index) {
    require(pending.odometry[index - 1].value.header.stamp_s <
                pending.odometry[index].value.header.stamp_s,
            "merged odometry is out of arrival order");
  }
  require(pending.receive_steady_s == 1.010 && pending.receive_wall_s == 1001.010,
          "merged batch did not take the newest receive time");
  require(pending.simulation_time_s && *pending.simulation_time_s == 10.050,
          "merged batch did not take the newest simulation clock");
}

void newerInputsReplaceOlderOnesAndListsAppend() {
  SensorBatch pending;
  pending.obstacles = cloud(1.0);
  pending.localization_health = std::string("TRACKING old");
  pending.transforms.push_back(InputSample<TransformSample>{});
  pending.clears.push_back(PlannerClearSample{ClearSource::Map, true});

  SensorBatch next;
  next.obstacles = cloud(2.0);
  next.transforms.push_back(InputSample<TransformSample>{});
  next.clears.push_back(PlannerClearSample{ClearSource::Cloud, true});
  mergeSensorBatch(pending, std::move(next));

  require(pending.obstacles && pending.obstacles->value.header.stamp_s == 2.0,
          "a newer registered cloud did not replace the older one");
  require(pending.localization_health && *pending.localization_health == "TRACKING old",
          "an absent optional in the newer batch cleared the pending value");
  require(pending.transforms.size() == 2U, "transforms were not appended");
  require(pending.clears.size() == 2U && pending.clears[0].source == ClearSource::Map &&
              pending.clears[1].source == ClearSource::Cloud,
          "clears were not appended in arrival order");
}

}  // namespace

int main() {
  try {
    lowRateInputsSurviveOdometryOnlyBatches();
    newerInputsReplaceOlderOnesAndListsAppend();
  } catch (const std::exception &error) {
    std::cerr << "test_sensor_batch_merge failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "test_sensor_batch_merge passed\n";
  return 0;
}
