#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimGraphicsSettings.h"

#include "Engine/Engine.h"
#include "GameFramework/GameUserSettings.h"
#include "LingTuSimSessionService.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

#include <limits>

using LingTuSim::UI::FPlayerGraphicsPreview;
using LingTuSim::UI::FPlayerGraphicsSettings;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimGraphicsSettingsNormalizeTest,
    "LingTuSim.UI.GraphicsSettings.Normalize",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimGraphicsSettingsNormalizeTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FPlayerGraphicsSettings Settings;
  Settings.Resolution = FIntPoint(-1, 0);
  Settings.WindowMode = 9;
  Settings.FrameRateLimit = std::numeric_limits<float>::quiet_NaN();
  Settings.ResolutionScale = 5.0F;
  Settings.ViewDistanceQuality = -1;
  Settings.ShadowQuality = 8;
  Settings.TextureQuality = 4;
  Settings.AntiAliasingQuality = 2;
  Settings.PostProcessingQuality = 1;
  Settings.Normalize();

  TestEqual(TEXT("invalid resolution uses the player default"), Settings.Resolution,
            FIntPoint(1920, 1080));
  TestEqual(TEXT("window mode is clamped to the public enum range"), Settings.WindowMode, 2);
  TestEqual(TEXT("non-finite frame cap uses the default"), Settings.FrameRateLimit, 60.0F);
  TestEqual(TEXT("resolution scale uses the supported lower bound"),
            Settings.ResolutionScale, 50.0F);
  TestEqual(TEXT("view distance is clamped"), Settings.ViewDistanceQuality, 0);
  TestEqual(TEXT("shadow quality is clamped"), Settings.ShadowQuality, 3);
  TestEqual(TEXT("cinematic texture quality is excluded from the player list"),
            Settings.TextureQuality, 3);
  Settings.ResolutionScale = 0.F;
  Settings.Normalize();
  TestEqual(TEXT("the engine project-default sentinel is not a zero-percent render"), Settings.ResolutionScale, 0.F);

  FPlayerGraphicsSettings Same = Settings;
  TestTrue(TEXT("equal real values are equivalent"), Settings.IsEquivalentTo(Same));
  Same.FrameRateLimit += KINDA_SMALL_NUMBER * 0.5F;
  TestTrue(TEXT("float comparison tolerates insignificant engine readback noise"),
           Settings.IsEquivalentTo(Same));
  Same.FrameRateLimit += 1.0F;
  TestFalse(TEXT("a real frame-rate change is detected"), Settings.IsEquivalentTo(Same));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimGraphicsSettingsPresetTest,
    "LingTuSim.UI.GraphicsSettings.QualityPreset",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimGraphicsSettingsPresetTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FPlayerGraphicsSettings Settings;
  Settings.SetQualityPreset(3);
  TestEqual(TEXT("matching supported qualities report the preset"),
            Settings.GetQualityPreset(), 3);
  Settings.ShadowQuality = 2;
  TestEqual(TEXT("a mixed quality selection reports custom"), Settings.GetQualityPreset(),
            INDEX_NONE);
  Settings.SetQualityPreset(8);
  TestEqual(TEXT("preset input is clamped to the supported list"),
            Settings.GetQualityPreset(), 3);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimGraphicsSettingsDeadlineTest,
    "LingTuSim.UI.GraphicsSettings.PreviewDeadline",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimGraphicsSettingsDeadlineTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FPlayerGraphicsPreview Preview;
  Preview.bPending = true;
  Preview.bHasBefore = false;
  Preview.DeadlineSeconds = 25.0;
  TestEqual(TEXT("remaining time is measured from the supplied monotonic time"),
            Preview.SecondsRemaining(13.0), 12.0);
  TestFalse(TEXT("preview remains pending before its deadline"), Preview.Tick(24.999));
  TestTrue(TEXT("preview reverts exactly at its deadline"), Preview.Tick(25.0));
  TestFalse(TEXT("expired preview is no longer pending"), Preview.IsPending());
  TestEqual(TEXT("inactive preview has no remaining time"),
            Preview.SecondsRemaining(10.0), 0.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimGraphicsSettingsSessionGateTest,
    "LingTuSim.UI.GraphicsSettings.SessionGate",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimGraphicsSettingsSessionGateTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  FString PreviousSessionId;
  uint64 PreviousGeneration = 0;
  if (LingTuSim::FSessionService::GetBoundSession(PreviousSessionId, PreviousGeneration)) {
    TestTrue(TEXT("an existing session locks player graphics"),
             FPlayerGraphicsPreview::IsSessionLocked());
    return true;
  }
  if (FPlayerGraphicsPreview::IsSessionLocked()) {
    TestTrue(TEXT("an existing control binding locks player graphics"),
             FPlayerGraphicsPreview::IsSessionLocked());
    return true;
  }

  if (!TestTrue(TEXT("test session binds"),
                LingTuSim::FSessionService::RebindSession(
                    TEXT("graphics-settings-test"), 1))) {
    return false;
  }
  ON_SCOPE_EXIT { LingTuSim::FSessionService::UnbindSession(); };
  TestTrue(TEXT("a bound session locks player graphics"),
           FPlayerGraphicsPreview::IsSessionLocked());

  FPlayerGraphicsPreview Preview;
  FPlayerGraphicsSettings Settings;
  FString Error;
  TestFalse(TEXT("the session gate rejects apply before viewport mutation"),
            Preview.Apply(nullptr, Settings, 1.0, Error));
  TestFalse(TEXT("rejected apply does not start a preview"), Preview.IsPending());
  TestTrue(TEXT("rejected apply reports the session lock"), Error.Contains(TEXT("session")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimGraphicsSettingsActualReadbackTest,
    "LingTuSim.UI.GraphicsSettings.ActualReadback",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimGraphicsSettingsActualReadbackTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  if (GEngine == nullptr || GEngine->GetGameUserSettings() == nullptr) {
    AddInfo(TEXT("game user settings are unavailable in this automation context"));
    return true;
  }

  const UGameUserSettings *Actual = GEngine->GetGameUserSettings();
  const FPlayerGraphicsSettings Captured = FPlayerGraphicsSettings::Capture(nullptr);
  TestEqual(TEXT("resolution is read from UGameUserSettings"), Captured.Resolution,
            Actual->GetScreenResolution());
  TestEqual(TEXT("window mode is read from UGameUserSettings"), Captured.WindowMode,
            static_cast<int32>(Actual->GetFullscreenMode()));
  TestEqual(TEXT("frame cap is read from the effective engine limit"),
            Captured.FrameRateLimit, GEngine->GetMaxFPS());
  TestEqual(TEXT("VSync is read from UGameUserSettings"), Captured.bVSync,
            Actual->IsVSyncEnabled());
  TestEqual(TEXT("native quality is retained during capture"),
            Captured.ViewDistanceQuality, Actual->GetViewDistanceQuality());
  return true;
}

#endif
