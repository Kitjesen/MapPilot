#include "LingTuSimRuntimeUIModel.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"

namespace {
using LingTuSim::UI::ERuntimeUIAction;
using LingTuSim::UI::ERuntimeUIMode;
using LingTuSim::UI::FRuntimeUIActionPolicy;

bool ResolvesTo(const FKey &Key, const ERuntimeUIAction Expected) {
  ERuntimeUIAction Action = ERuntimeUIAction::ToggleRecording;
  return FRuntimeUIActionPolicy::ResolveKey(ERuntimeUIMode::Pause, Key, Action) &&
         Action == Expected;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuFrontEndGamepadNavigationContractTest,
                                 "LingTuSim.UI.Runtime.FrontEnd.GamepadNavigationContract",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuFrontEndGamepadNavigationContractTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  TestTrue(TEXT("A confirms the current front-end selection"),
           ResolvesTo(EKeys::Gamepad_FaceButton_Bottom, ERuntimeUIAction::ConfirmGameSelection));
  TestTrue(TEXT("D-pad up selects the previous session"),
           ResolvesTo(EKeys::Gamepad_DPad_Up, ERuntimeUIAction::SelectPreviousGame));
  TestTrue(TEXT("D-pad down selects the next session"),
           ResolvesTo(EKeys::Gamepad_DPad_Down, ERuntimeUIAction::SelectNextGame));
  TestTrue(TEXT("D-pad left selects the previous asset"),
           ResolvesTo(EKeys::Gamepad_DPad_Left, ERuntimeUIAction::SelectPreviousAsset));
  TestTrue(TEXT("D-pad right selects the next asset"),
           ResolvesTo(EKeys::Gamepad_DPad_Right, ERuntimeUIAction::SelectNextAsset));
  TestTrue(TEXT("LB selects the previous reviewed asset"),
           ResolvesTo(EKeys::Gamepad_LeftShoulder, ERuntimeUIAction::SelectPreviousAsset));
  TestTrue(TEXT("RB selects the next reviewed asset"),
           ResolvesTo(EKeys::Gamepad_RightShoulder, ERuntimeUIAction::SelectNextAsset));

  ERuntimeUIAction Action = ERuntimeUIAction::ToggleRecording;
  TestFalse(TEXT("LB remains available to robot deadman outside the front end"),
            FRuntimeUIActionPolicy::ResolveKey(ERuntimeUIMode::Drive, EKeys::Gamepad_LeftShoulder,
                                               Action));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuFrontEndGamepadBackMatchesEscapeTest,
                                 "LingTuSim.UI.Runtime.FrontEnd.GamepadBackMatchesEscape",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuFrontEndGamepadBackMatchesEscapeTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FRuntimeUIModeController Controller;
  TestTrue(TEXT("B is handled as the gamepad back key"),
           Controller.HandleKey(EKeys::Gamepad_FaceButton_Right));
  TestEqual(TEXT("B enters the same menu mode as Escape"), Controller.GetMode(),
            ERuntimeUIMode::Pause);
  TestTrue(TEXT("a second B is handled"), Controller.HandleKey(EKeys::Gamepad_FaceButton_Right));
  TestEqual(TEXT("a second B restores the previous mode"), Controller.GetMode(),
            ERuntimeUIMode::Drive);
  return true;
}

#endif
