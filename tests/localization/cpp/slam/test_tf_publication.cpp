// Compile the real converter and publication boundary without running SLAM.
#define main lingtu_slam_runtime_entry
#include "cyclone_runtime.cpp"
#undef main

namespace {

void testRequire(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

void requireNear(double actual, double expected, const char* reason) {
  testRequire(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, reason);
}

void checkReceivedTransform(const lingtu_dds_TFMessage& message,
                            const Transform3d& expected, double stamp_s) {
  testRequire(message.transforms._length == 1, "TF must contain exactly one transform");
  testRequire(message.transforms._buffer != nullptr, "TF transform storage is missing");
  const auto& received = message.transforms._buffer[0];
  testRequire(received.header.frame_id != nullptr &&
                  expected.frame_id == received.header.frame_id,
              "TF parent frame changed during publication");
  testRequire(received.child_frame_id != nullptr &&
                  expected.child_frame_id == received.child_frame_id,
              "TF child frame changed during publication");
  requireNear(stampSeconds(received.header.stamp), stamp_s, "TF source stamp changed");
  requireNear(received.transform.translation.x, expected.pose.x, "TF translation x changed");
  requireNear(received.transform.translation.y, expected.pose.y, "TF translation y changed");
  requireNear(received.transform.translation.z, expected.pose.z, "TF translation z changed");
  requireNear(received.transform.rotation.x, expected.pose.qx, "TF quaternion x changed");
  requireNear(received.transform.rotation.y, expected.pose.qy, "TF quaternion y changed");
  requireNear(received.transform.rotation.z, expected.pose.qz, "TF quaternion z changed");
  requireNear(received.transform.rotation.w, expected.pose.qw, "TF quaternion w changed");
}

void testReturnedTfPublishesOwnedValues() {
  // Loopback-only and a separate domain keep this test outside robot sessions.
  constexpr dds_domainid_t domain_id = 230;
  const auto domain = checked(dds_create_domain(domain_id,
      "<CycloneDDS><Domain id='230'><General>"
      "<Interfaces><NetworkInterface address='127.0.0.1'/></Interfaces>"
      "<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>"),
      "test dds_create_domain");
  const auto participant = checked(dds_create_participant(domain_id, nullptr, nullptr),
                                   "test dds_create_participant");
  const auto topic = checked(dds_create_topic(participant, &lingtu_dds_TFMessage_desc,
      lingtu::message::kTf.dds_topic.data(), nullptr, nullptr), "test dds_create_topic");
  const auto qos = make_qos(QosProfile::TfDynamic);
  const auto reader = checked(dds_create_reader(participant, topic, qos.get(), nullptr),
                              "test dds_create_reader");
  const auto cloud_topic = checked(dds_create_topic(participant, &lingtu_dds_PointCloud2_desc,
      lingtu::message::kSlamCumulativeMapCloud.dds_topic.data(), nullptr, nullptr),
      "global map topic");
  const auto cloud_qos = make_qos(QosProfile::LidarPointcloud);
  const auto cloud_reader = checked(dds_create_reader(participant, cloud_topic, cloud_qos.get(), nullptr),
                                    "global map reader");
  {
    DdsRuntime publisher(static_cast<int>(domain_id));
    const auto matched_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    bool matched = false;
    do {
      dds_subscription_matched_status_t status{};
      checked(dds_get_subscription_matched_status(reader, &status), "test subscription status");
      matched = status.current_count > 0;
      if (!matched) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (!matched && std::chrono::steady_clock::now() < matched_deadline);
    testRequire(matched, "production TF writer did not match the test reader");

    Transform3d transform;
    transform.pose = {61.018063, 16.497034, 0.456980, 0.0, 0.0,
                      std::sqrt(0.5), std::sqrt(0.5)};
    for (int sequence = 0; sequence < 2; ++sequence) {
      const double stamp_s = 100.0 + sequence;
      if (sequence != 0) {
        transform.pose.x = -8.0;
        transform.pose.y = 5.0;
        transform.pose.z = 1.2;
        transform.pose.qz = -std::sqrt(0.5);
      }
      auto message = toDdsTfMessage(transform, stamp_s);
      publisher.writeTf(message);
      testRequire(message.msg.transforms._buffer == message.transforms.data(),
                  "production writeTf must bind the returned owner's storage");

      const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      bool received = false;
      do {
        void* samples[1]{};
        dds_sample_info_t info[1]{};
        const auto count = checked(dds_take(reader, samples, info, 1, 1), "test dds_take");
        if (count > 0) {
          if (info[0].valid_data) {
            const auto& sample = *static_cast<lingtu_dds_TFMessage*>(samples[0]);
            checkReceivedTransform(sample, transform, stamp_s);
            received = true;
          }
          checked(dds_return_loan(reader, samples, count), "test dds_return_loan");
        }
        if (!received) std::this_thread::sleep_for(std::chrono::milliseconds(5));
      } while (!received && std::chrono::steady_clock::now() < receive_deadline);
      testRequire(received, "production TF publication did not reach the reader");
      std::printf("TF publication %d: map<-odom xyz=(%.6f,%.6f,%.6f), yaw=%s90deg PASS\n",
                  sequence + 1, transform.pose.x, transform.pose.y, transform.pose.z,
                  sequence == 0 ? "+" : "-");
    }
    Cloud cloud;
    cloud.frame_id = "map";
    cloud.stamp_s = 1234;
    PointXYZIT point;
    point.x = 2; point.y = -3; point.z = .4F;
    cloud.points.push_back(point);
    auto message = toDdsCloud(cloud);
    message.bindStorage();
    bool received = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      publisher.writeGlobalMap(message.msg);
      void* samples[1]{};
      dds_sample_info_t info[1]{};
      const auto count = checked(dds_take(cloud_reader, samples, info, 1, 1), "global map take");
      if (count > 0) {
        if (info[0].valid_data) {
          const auto& sample = *static_cast<lingtu_dds_PointCloud2*>(samples[0]);
          testRequire(sample.width == 1 && std::string(sample.header.frame_id) == "map",
                      "global map DDS payload/frame changed");
          requireNear(stampSeconds(sample.header.stamp), 1234, "global map stamp changed");
          received = true;
        }
        checked(dds_return_loan(cloud_reader, samples, count), "global map return loan");
      }
      if (!received) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (!received && std::chrono::steady_clock::now() < deadline);
    testRequire(received, "global mapping snapshot did not reach native DDS");
    const auto geometry_topic=checked(dds_create_topic(participant,&lingtu_dds_SlamMapGeometrySnapshot_desc,
        lingtu::message::kSlamMapGeometrySnapshot.dds_topic.data(),nullptr,nullptr),"geometry topic");
    const auto geometry_qos=make_qos(QosProfile::SystemStatus);
    // Publish before discovery: transient metadata must reach a late reader.
    lingtu::slam::MapGeometrySnapshot geometry{101,12,345,"map","/tmp/frozen-geometry",15};
    geometry.captured_stamp_ns=123456789000LL;
    geometry.map_body={100,2,3,0,0,std::sqrt(.5),std::sqrt(.5)};
    geometry.sensor_origin_z_m=3.2;
    publisher.writeGeometrySnapshot(geometry);
    const auto geometry_reader=checked(dds_create_reader(participant,geometry_topic,geometry_qos.get(),nullptr),
                                      "geometry reader");
    const auto geometry_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    bool geometry_received=false;
    do {
      void* samples[1]{};
      dds_sample_info_t infos[1]{};
      const auto count=checked(dds_take(geometry_reader,samples,infos,1,1),"geometry take");
      if (count>0) {
        if (infos[0].valid_data) {
          const auto& sample=*static_cast<lingtu_dds_SlamMapGeometrySnapshot*>(samples[0]);
          testRequire(sample.reset_epoch==101 && sample.graph_revision==12 && sample.captured_sequence==345 &&
              sample.keyframe_count==15 && sample.captured_stamp_ns==123456789000LL,
              "production geometry identity/source stamp changed");
          testRequire(std::string(sample.frozen_dir)==geometry.frozen_dir && std::string(sample.frame_id)=="map",
              "production geometry strings changed");
          requireNear(sample.map_body.position.x,100,"geometry anchor x changed");
          requireNear(sample.map_body.orientation.z,std::sqrt(.5),"geometry anchor rotation changed");
          requireNear(sample.sensor_origin_z_m,3.2,"geometry physical origin height changed");
          geometry_received=true;
        }
        checked(dds_return_loan(geometry_reader,samples,count),"geometry return loan");
      }
      if (!geometry_received) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (!geometry_received && std::chrono::steady_clock::now()<geometry_deadline);
    testRequire(geometry_received,"production geometry snapshot did not reach late reader");
    const auto ack_topic=checked(dds_create_topic(participant,&lingtu_dds_SlamMapGeometryApplied_desc,
        lingtu::message::kSlamMapGeometryApplied.dds_topic.data(),nullptr,nullptr),"geometry ack topic");
    const auto ack_qos=make_qos(QosProfile::CommandAck);
    const auto ack_writer=checked(dds_create_writer(participant,ack_topic,ack_qos.get(),nullptr),"geometry ack writer");
    lingtu_dds_SlamMapGeometryApplied ack{};
    ack.reset_epoch=101;
    ack.graph_revision=12;
    ack.success=true;
    ack.message=const_cast<char*>("applied");
    checked(dds_write(ack_writer,&ack),"geometry ack publish");
    bool ack_received=false;
    const auto ack_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    do {
      publisher.drainGeometryApplied([&](const lingtu_dds_SlamMapGeometryApplied& sample) {
        testRequire(sample.reset_epoch==101 && sample.graph_revision==12 && sample.success &&
            std::string(sample.message)=="applied","production geometry ack changed");
        ack_received=true;
      });
      if (!ack_received) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (!ack_received && std::chrono::steady_clock::now()<ack_deadline);
    testRequire(ack_received,"production geometry applied reader did not receive receipt");
  }
  checked(dds_delete(participant), "test dds_delete participant");
  checked(dds_delete(domain), "test dds_delete domain");
}

}  // namespace

int main() {
  try {
    testReturnedTfPublishesOwnedValues();
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "TF publication regression failed: %s\n", error.what());
    return 1;
  }
}
