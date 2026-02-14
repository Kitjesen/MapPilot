#include <gtest/gtest.h>
#include <cstring>
#include <filesystem>
#include "remote_monitoring/core/flight_recorder.hpp"

using remote_monitoring::core::FlightRecorder;
using remote_monitoring::core::FlightSnapshot;
using remote_monitoring::core::FlightDumpHeader;

namespace fs = std::filesystem;

class FlightRecorderTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() / "fdr_test";
    fs::create_directories(test_dir_);
    recorder_ = std::make_unique<FlightRecorder>(
        30,  // 30 frames capacity (3 seconds @ 10Hz)
        test_dir_.string(),
        5    // 5 frames post-trigger
    );
  }

  void TearDown() override {
    recorder_.reset();
    fs::remove_all(test_dir_);
  }

  FlightSnapshot MakeSnap(double ts, float x = 0, float y = 0) {
    FlightSnapshot snap{};
    snap.timestamp_sec = ts;
    snap.x = x;
    snap.y = y;
    snap.tf_ok = 1;
    return snap;
  }

  fs::path test_dir_;
  std::unique_ptr<FlightRecorder> recorder_;
};

TEST_F(FlightRecorderTest, SnapshotSize) {
  EXPECT_EQ(sizeof(FlightSnapshot), 64u);
}

TEST_F(FlightRecorderTest, DumpHeaderSize) {
  EXPECT_EQ(sizeof(FlightDumpHeader), 256u);
}

TEST_F(FlightRecorderTest, RecordIncrementsCounter) {
  EXPECT_EQ(recorder_->TotalFramesRecorded(), 0u);
  recorder_->RecordSnapshot(MakeSnap(1.0));
  EXPECT_EQ(recorder_->TotalFramesRecorded(), 1u);
  recorder_->RecordSnapshot(MakeSnap(2.0));
  EXPECT_EQ(recorder_->TotalFramesRecorded(), 2u);
}

TEST_F(FlightRecorderTest, ImmediateDumpCreatesFile) {
  // Create a recorder with 0 post-trigger frames for immediate dump
  auto imm = std::make_unique<FlightRecorder>(10, test_dir_.string(), 0);

  for (int i = 0; i < 10; ++i) {
    imm->RecordSnapshot(MakeSnap(static_cast<double>(i)));
  }

  std::string path = imm->TriggerDump("test_reason");
  EXPECT_FALSE(path.empty());
  EXPECT_TRUE(fs::exists(path));

  // Verify file size: 256 header + 10 * 64 snapshots = 896 bytes
  EXPECT_EQ(fs::file_size(path), 256u + 10u * 64u);
}

TEST_F(FlightRecorderTest, DumpHeaderMagic) {
  auto imm = std::make_unique<FlightRecorder>(5, test_dir_.string(), 0);
  for (int i = 0; i < 5; ++i) {
    imm->RecordSnapshot(MakeSnap(static_cast<double>(i)));
  }

  std::string path = imm->TriggerDump("magic_test");
  ASSERT_TRUE(fs::exists(path));

  std::ifstream ifs(path, std::ios::binary);
  FlightDumpHeader hdr{};
  ifs.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));

  EXPECT_EQ(std::strncmp(hdr.magic, "FLTREC01", 8), 0);
  EXPECT_EQ(hdr.version, 1u);
  EXPECT_EQ(hdr.snapshot_count, 5u);
  EXPECT_TRUE(std::string(hdr.trigger_reason).find("magic_test") != std::string::npos);
}

TEST_F(FlightRecorderTest, CooldownPreventsDoubleDump) {
  auto imm = std::make_unique<FlightRecorder>(5, test_dir_.string(), 0);
  for (int i = 0; i < 5; ++i) {
    imm->RecordSnapshot(MakeSnap(static_cast<double>(i)));
  }

  std::string first = imm->TriggerDump("first");
  EXPECT_FALSE(first.empty());

  // Second trigger within 10s cooldown should fail
  std::string second = imm->TriggerDump("second");
  EXPECT_TRUE(second.empty());
}

TEST_F(FlightRecorderTest, ListDumpsReturnsFiles) {
  auto imm = std::make_unique<FlightRecorder>(5, test_dir_.string(), 0);
  for (int i = 0; i < 5; ++i) {
    imm->RecordSnapshot(MakeSnap(static_cast<double>(i)));
  }
  imm->TriggerDump("list_test");

  auto dumps = imm->ListDumps();
  EXPECT_EQ(dumps.size(), 1u);
  EXPECT_EQ(dumps[0].snapshot_count, 5u);
  EXPECT_TRUE(dumps[0].reason.find("list_test") != std::string::npos);
}

TEST_F(FlightRecorderTest, DeleteDumpRemovesFile) {
  auto imm = std::make_unique<FlightRecorder>(5, test_dir_.string(), 0);
  for (int i = 0; i < 5; ++i) {
    imm->RecordSnapshot(MakeSnap(static_cast<double>(i)));
  }
  imm->TriggerDump("delete_test");

  auto dumps = imm->ListDumps();
  ASSERT_EQ(dumps.size(), 1u);

  EXPECT_TRUE(imm->DeleteDump(dumps[0].filename));
  EXPECT_EQ(imm->ListDumps().size(), 0u);
}

TEST_F(FlightRecorderTest, DeleteRejectsPathTraversal) {
  EXPECT_FALSE(recorder_->DeleteDump("../../../etc/passwd"));
  EXPECT_FALSE(recorder_->DeleteDump("..\\secret.fdr"));
  EXPECT_FALSE(recorder_->DeleteDump("sub/dir/file.fdr"));
}

TEST_F(FlightRecorderTest, RingBufferWrapsCorrectly) {
  // Capacity is 30, write 50 frames
  for (int i = 0; i < 50; ++i) {
    recorder_->RecordSnapshot(MakeSnap(static_cast<double>(i),
                                       static_cast<float>(i), 0));
  }
  EXPECT_EQ(recorder_->TotalFramesRecorded(), 50u);
}
