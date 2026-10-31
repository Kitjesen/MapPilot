#if WITH_DEV_AUTOMATION_TESTS
#include "LingTuSimRuntimeUIStatus.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerMenuExitRetryTest, "LingTuSim.UI.Runtime.PlayerMenu.ExitRetry",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerMenuExitRetryTest::RunTest(const FString&) {
  using namespace LingTuSim;
  using namespace LingTuSim::UI;
  FRuntimeUIStatusSnapshot Status;
  FString Reason;
  TestFalse(TEXT("a missing reply waits within the bounded window"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT("event-1"), 9.9, Status, Reason));
  TestTrue(TEXT("a never-arriving reply releases pending after ten seconds"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT("event-1"), 10.0, Status, Reason));
  TestEqual(TEXT("timeout is visible instead of a permanent wait"), Reason, FString(TEXT("runtime_exit_confirmation_timeout")));
  Status.bFullStatusAvailable = Status.bFullStatusFresh = Status.bFullStatusIdentityCoherent = true;
  Status.FullStatus.EventId = TEXT("event-1");
  Status.FullStatus.Status = EControlAckStatus::Rejected;
  TestTrue(TEXT("an exact current rejection releases pending immediately"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT("event-1"), .1, Status, Reason));
  TestEqual(TEXT("rejection is exposed"), Reason, FString(TEXT("runtime_exit_rejected")));
  TestFalse(TEXT("another request cannot reject the pending exit"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT("event-2"), .1, Status, Reason));
  Status.bFullStatusFresh = false;
  TestFalse(TEXT("stale rejection cannot terminate a fresh request"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT("event-1"), .1, Status, Reason));
  TestTrue(TEXT("stale replies do not wait forever"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT("event-1"), 10.1, Status, Reason));
  TestFalse(TEXT("there is no pending transaction without an event ID"),
      FRuntimeUIExitPolicy::ShouldEndPendingRequest(TEXT(""), 10.1, Status, Reason));
  return true;
}
#endif
