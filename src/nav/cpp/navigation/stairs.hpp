#pragma once

#include <optional>
#include <string>
#include <vector>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::navigation {

// Surface geometry measured from the saved 3D map. The first and last patches
// are landings; no body-height or vendor-model data is stored in the map.
struct StairTread {
  double begin{0.0};
  double end{0.0};
  double height{0.0};
};

struct StairFlight {
  std::string id;
  nav_kernel::Vec3 origin{};
  nav_kernel::Vec3 axis{1.0, 0.0, 0.0};
  double lateralMin{0.0};
  double lateralMax{0.0};
  std::vector<StairTread> treads;
};

struct StairSegment {
  StairFlight flight;
  bool ascending{true};
  double pathBegin{0.0};
  double pathEnd{0.0};
  double bodyHeight{0.0};
  nav_kernel::Vec3 entry{};
  nav_kernel::Vec3 exit{};
};

// Match both crossing direction and observed tread elevations. A route beside
// or underneath a flight does not acquire its motion-profile requirement.
std::vector<StairSegment> resolveStairSegments(
    const std::vector<nav_kernel::Vec3>& path,
    const std::vector<StairFlight>& flights, double body_height_m,
    double height_tolerance_m = 0.10);

enum class StairProfile { None, Ascent, Descent, Restore };
enum class StairPhase { Ordinary, Approach, Switching, Traversing, Restoring, Waiting };

struct StairProfileFeedback {
  StairProfile requirement{StairProfile::None};
  bool ready{false};
  bool unsupported{false};
  std::string activeProfile;
  std::string reason;
  double stampS{0.0};
};

struct StairTraversalConfig {
  double bodyLength{0.6};
  double bodyWidth{0.4};
  double entryMargin{0.10};
  double maxSpeed{0.15};
  double heightTolerance{0.12};
  double feedbackMaxAge{1.0};
};

struct StairTraversalOutput {
  StairPhase phase{StairPhase::Ordinary};
  StairProfile request{StairProfile::None};
  bool hold{false};
  bool onFlight{false};
  bool constrainRecovery{false};
  double speedLimit{0.0};
  std::optional<nav_kernel::Vec3> approachTarget;
  std::optional<double> bodyYaw;  // Map-frame flight direction, independent of local path tangent.
  std::string flightId;
  std::string reason;
};

struct StairGoalGuide {
  bool arrivalAllowed{true};
  bool hold{false};
  std::optional<nav_kernel::Vec3> target;
};

// A phase within Executor, not a second navigation task. Cancellation/replan
// cannot erase a profile already in use while the robot remains on the flight.
class StairTraversal {
 public:
  explicit StairTraversal(StairTraversalConfig config = {});
  void setSegments(std::vector<StairSegment> segments);
  void cancel();
  [[nodiscard]] bool pending() const;
  StairTraversalOutput step(const nav_kernel::Pose& body,
                            const nav_kernel::Twist& velocity,
                            bool velocity_valid, double now,
                            const StairProfileFeedback& feedback);
  [[nodiscard]] bool localPathAllowed(const std::vector<nav_kernel::Vec3>& path,
                                      double yaw) const;
  [[nodiscard]] StairGoalGuide guideGoal(const nav_kernel::Vec3& goal,
                                         double tolerance, double yaw) const;

 private:
  double extentAlong(double yaw, const StairFlight& flight) const;
  bool confirmed(StairProfile request, double now,
                 const StairProfileFeedback& feedback) const;
  StairTraversalConfig config_;
  std::vector<StairSegment> pending_;
  std::optional<StairSegment> active_;
  StairPhase phase_{StairPhase::Ordinary};
  bool profile_requested_{false};
  bool cancelled_{false};
};

const char* stairPhaseName(StairPhase phase);
const char* stairProfileName(StairProfile profile);

}  // namespace lingtu::nav::navigation
