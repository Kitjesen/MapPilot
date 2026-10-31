#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimDriveKeyBindings.h"
#include "LingTuSimRobotDriveInput.h"

#include "Misc/AutomationTest.h"

using LingTuSim::UI::EDriveKeyAction;
using LingTuSim::UI::ERobotDriveInputUpdate;
using LingTuSim::UI::FDriveKeyBindings;
using LingTuSim::UI::FRobotDriveInputState;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimDriveKeyBindingsDefaultsAndValidationTest,
    "LingTuSim.UI.DriveKeyBindings.DefaultsAndValidation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimDriveKeyBindingsDefaultsAndValidationTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FDriveKeyBindings Bindings;
  TestEqual(TEXT("forward defaults to W"), Bindings.GetKey(EDriveKeyAction::Forward), EKeys::W);
  TestEqual(TEXT("backward defaults to S"), Bindings.GetKey(EDriveKeyAction::Backward), EKeys::S);
  TestEqual(TEXT("left defaults to A"), Bindings.GetKey(EDriveKeyAction::Left), EKeys::A);
  TestEqual(TEXT("right defaults to D"), Bindings.GetKey(EDriveKeyAction::Right), EKeys::D);
  TestEqual(TEXT("turn-left defaults to Q"), Bindings.GetKey(EDriveKeyAction::TurnLeft), EKeys::Q);
  TestEqual(TEXT("turn-right defaults to E"), Bindings.GetKey(EDriveKeyAction::TurnRight), EKeys::E);
  TestEqual(TEXT("forward config name is stable"),
            FString(FDriveKeyBindings::ConfigKey(EDriveKeyAction::Forward)),
            FString(TEXT("KeyForward")));

  FString Error;
  TestTrue(TEXT("binding the current key is idempotent"),
           Bindings.TryBind(EDriveKeyAction::Forward, EKeys::W, Error));
  TestTrue(TEXT("idempotent binding has no error"), Error.IsEmpty());

  const FDriveKeyBindings BeforeRejectedBindings = Bindings;
  TestFalse(TEXT("a key already used by another drive action is rejected"),
            Bindings.TryBind(EDriveKeyAction::Forward, EKeys::S, Error));
  TestTrue(TEXT("conflict rejection is readable"), !Error.IsEmpty());
  TestTrue(TEXT("conflict rejection does not mutate bindings"),
           Bindings == BeforeRejectedBindings);
  TestFalse(TEXT("reserved B is rejected"),
            Bindings.TryBind(EDriveKeyAction::Forward, EKeys::B, Error));
  TestFalse(TEXT("reserved C is rejected"),
            Bindings.TryBind(EDriveKeyAction::Forward, EKeys::C, Error));
  TestFalse(TEXT("reserved R is rejected"),
            Bindings.TryBind(EDriveKeyAction::Forward, EKeys::R, Error));
  TestFalse(TEXT("reserved X is rejected"),
            Bindings.TryBind(EDriveKeyAction::Forward, EKeys::X, Error));
  TestFalse(TEXT("non-alphanumeric keys are rejected"),
            Bindings.TryBind(EDriveKeyAction::Forward, EKeys::SpaceBar, Error));
  TestTrue(TEXT("all rejected keys leave bindings unchanged"),
           Bindings == BeforeRejectedBindings);
  TestTrue(TEXT("digits are accepted"),
           Bindings.TryBind(EDriveKeyAction::Forward, EKeys::One, Error));
  TestEqual(TEXT("accepted digit is stored"), Bindings.GetKey(EDriveKeyAction::Forward),
            EKeys::One);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimDriveKeyBindingsInputMappingTest,
    "LingTuSim.UI.DriveKeyBindings.InputMappingAndRebindFlush",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimDriveKeyBindingsInputMappingTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FDriveKeyBindings Rebound;
  FString Error;
  TestTrue(TEXT("forward can be rebound to I"),
           Rebound.TryBind(EDriveKeyAction::Forward, EKeys::I, Error));

  FRobotDriveInputState State;
  State.SetDriveMode(true);
  State.SetViewportFocused(true);
  TestEqual(TEXT("installing changed bindings emits a release"), State.SetKeyBindings(Rebound),
            ERobotDriveInputUpdate::Released);
  TestEqual(TEXT("installing unchanged bindings is handled"), State.SetKeyBindings(Rebound),
            ERobotDriveInputUpdate::Handled);
  TestEqual(TEXT("old W is no longer mapped"), State.HandleKeyDown(EKeys::W, false),
            ERobotDriveInputUpdate::Unhandled);
  TestEqual(TEXT("new I press changes drive input"), State.HandleKeyDown(EKeys::I, false),
            ERobotDriveInputUpdate::Changed);
  TestEqual(TEXT("I requests forward"), State.GetSnapshot().Forward, 1.0f);
  TestTrue(TEXT("physical I identifier is reported"),
           State.GetSnapshot().ActiveControls.Contains(TEXT("keyboard.i")));
  TestFalse(TEXT("old W identifier is not reported"),
            State.GetSnapshot().ActiveControls.Contains(TEXT("keyboard.w")));
  TestEqual(TEXT("new I key-up releases motion"), State.HandleKeyUp(EKeys::I),
            ERobotDriveInputUpdate::Released);

  FRobotDriveInputState HeldState;
  HeldState.SetDriveMode(true);
  HeldState.SetViewportFocused(true);
  HeldState.HandleKeyDown(EKeys::W, false);
  TestEqual(TEXT("rebinding while held flushes input"), HeldState.SetKeyBindings(Rebound),
            ERobotDriveInputUpdate::Released);
  TestEqual(TEXT("held rebind zeros forward"), HeldState.GetSnapshot().Forward, 0.0f);
  TestFalse(TEXT("held rebind drops the keyboard permit"), HeldState.GetSnapshot().bDeadman);
  TestEqual(TEXT("repeat after rebind is consumed without rearming"),
            HeldState.HandleKeyDown(EKeys::I, true), ERobotDriveInputUpdate::Handled);
  TestEqual(TEXT("repeat after rebind leaves forward zero"), HeldState.GetSnapshot().Forward, 0.0f);
  TestEqual(TEXT("fresh keydown rearms after flush"), HeldState.HandleKeyDown(EKeys::I, false),
            ERobotDriveInputUpdate::Changed);

  HeldState.HandleKeyUp(EKeys::I);
  HeldState.HandleAnalog(EKeys::Gamepad_LeftY, 1.0f);
  TestEqual(TEXT("gamepad axis still requires LB"), HeldState.GetSnapshot().Forward, 0.0f);
  HeldState.HandleKeyDown(EKeys::Gamepad_LeftShoulder, false);
  TestEqual(TEXT("gamepad LB permit behavior is unchanged"), HeldState.GetSnapshot().Forward,
            1.0f);
  return true;
}

#endif
