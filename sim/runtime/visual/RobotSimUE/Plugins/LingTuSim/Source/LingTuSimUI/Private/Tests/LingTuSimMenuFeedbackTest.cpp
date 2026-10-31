#if WITH_DEV_AUTOMATION_TESTS
#include "LingTuSimMenuFeedback.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMenuFeedbackStateTest, "LingTuSim.UI.MenuFeedback.Timing",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMenuFeedbackStateTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FMenuFeedbackState Feedback;
  TestEqual(TEXT("idle content is fully visible"), Feedback.PageOpacity(10.), 1.F);
  Feedback.BeginPage(10.);
  TestTrue(TEXT("new page has a brief fade and translation"), Feedback.PageOpacity(10.) < 1.F && Feedback.PageOffset(10.) > 0.F);
  TestTrue(TEXT("transition advances without waiting on an animation callback"), Feedback.PageOpacity(10.07) > Feedback.PageOpacity(10.));
  TestEqual(TEXT("page settles after 140ms"), Feedback.PageOpacity(10.15), 1.F);
  TestEqual(TEXT("settled page has no offset"), Feedback.PageOffset(10.15), 0.F);
  Feedback.BeginPage(10.06);
  TestEqual(TEXT("rapid switch still settles for the latest target"), Feedback.PageOpacity(10.21), 1.F);
  Feedback.Changed(FText::FromString(TEXT("帧率上限")), FText::FromString(TEXT("60 FPS")), true, 11.);
  TestEqual(TEXT("graphics feedback does not falsely claim applied or saved"), Feedback.ChangeText(11.).ToString(), FString(TEXT("帧率上限 → 60 FPS · 待应用")));
  TestTrue(TEXT("feedback stays readable briefly"), Feedback.ChangeOpacity(11.5) > .9F);
  TestTrue(TEXT("feedback fades before expiry"), Feedback.ChangeOpacity(12.2) < 1.F);
  TestTrue(TEXT("feedback expires instead of becoming a stale toast"), Feedback.ChangeText(12.5).IsEmpty());
  TestFalse(TEXT("idle feedback needs no further animation work"), Feedback.IsActive(12.5));
  Feedback.Changed(FText::FromString(TEXT("视角反转")), FText::FromString(TEXT("开启")), false, 13.);
  TestTrue(TEXT("player setting previews still require save"), Feedback.ChangeText(13.).ToString().EndsWith(TEXT("待保存")));
  Feedback.BeginPage(13.1);
  TestTrue(TEXT("switching category clears the prior category's transient hint"), Feedback.ChangeText(13.1).IsEmpty());
  return true;
}
#endif
