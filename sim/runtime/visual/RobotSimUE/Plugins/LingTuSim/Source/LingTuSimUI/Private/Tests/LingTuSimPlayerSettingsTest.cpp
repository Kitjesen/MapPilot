#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimPlayerSettings.h"

#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"

#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimPlayerAudioGainTest, "LingTuSim.UI.PlayerSettings.AudioGain",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimPlayerAudioGainTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FPlayerSettings Settings;
  TestEqual(TEXT("default master preserves existing sound"), Settings.GetMasterAudioGain(), 1.F);
  TestEqual(TEXT("default UI preserves existing cues"), Settings.GetUIAudioGain(), 1.F);
  Settings.MasterVolume = .4F;
  Settings.UIVolume = .25F;
  TestEqual(TEXT("master and UI controls are independent"), Settings.GetMasterAudioGain(), .4F);
  TestEqual(TEXT("UI gain does not multiply master a second time"), Settings.GetUIAudioGain(), .25F);
  Settings.bMuteAudio = true;
  TestEqual(TEXT("mute silences the master without erasing levels"), Settings.GetMasterAudioGain(), 0.F);
  Settings.bMuteAudio = false;
  TestEqual(TEXT("unmute restores the selected master level"), Settings.GetMasterAudioGain(), .4F);
  Settings.MasterVolume = -2.F;
  Settings.UIVolume = 3.F;
  Settings.MusicVolume = -1.F;
  Settings.MenuMusicTrack = 9;
  Settings.Normalize();
  TestEqual(TEXT("master cannot be negative"), Settings.MasterVolume, 0.F);
  TestEqual(TEXT("UI cannot amplify beyond its maximum"), Settings.UIVolume, 1.F);
  TestEqual(TEXT("music cannot be negative"), Settings.MusicVolume, 0.F);
  TestEqual(TEXT("music selection resolves to an available track"), Settings.MenuMusicTrack, 1);
  Settings.MasterVolume = std::numeric_limits<float>::quiet_NaN();
  Settings.Normalize();
  TestEqual(TEXT("non-finite level returns to its default"), Settings.MasterVolume, 1.F);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimPlayerSettingsDefaultsTest, "LingTuSim.UI.PlayerSettings.Defaults",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimPlayerSettingsDefaultsTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const LingTuSim::UI::FPlayerSettings Settings;
  TestEqual(TEXT("default sensitivity preserves existing mouse look"), Settings.MouseSensitivity,
            1.0f);
  TestFalse(TEXT("mouse Y is not inverted by default"), Settings.bInvertMouseY);
  TestTrue(TEXT("control hints are shown by default"), Settings.bShowControlHints);
  TestFalse(TEXT("frame counter is hidden by default"), Settings.bShowFrameRate);
  TestEqual(TEXT("forward key defaults to W"),
            Settings.DriveKeys.GetKey(LingTuSim::UI::EDriveKeyAction::Forward), EKeys::W);
  const FVector2D Look =
      LingTuSim::UI::FPlayerSettings::ApplyMouseLook(FVector2D(2.0, -3.0), Settings);
  TestEqual(TEXT("default settings preserve yaw"), Look.X, 2.0);
  TestEqual(TEXT("default settings preserve pitch"), Look.Y, -3.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimPlayerSettingsNormalizeTest, "LingTuSim.UI.PlayerSettings.Normalize",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimPlayerSettingsNormalizeTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FPlayerSettings Settings;
  Settings.MouseSensitivity = 0.0f;
  Settings.Normalize();
  TestEqual(TEXT("sensitivity is clamped to its lower bound"), Settings.MouseSensitivity, 0.25f);
  Settings.MouseSensitivity = 4.0f;
  Settings.Normalize();
  TestEqual(TEXT("sensitivity is clamped to its upper bound"), Settings.MouseSensitivity, 3.0f);
  Settings.MouseSensitivity = std::numeric_limits<float>::quiet_NaN();
  Settings.Normalize();
  TestEqual(TEXT("non-finite sensitivity falls back to the exact default"),
            Settings.MouseSensitivity, 1.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimPlayerSettingsMouseLookTest, "LingTuSim.UI.PlayerSettings.ApplyMouseLook",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimPlayerSettingsMouseLookTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  LingTuSim::UI::FPlayerSettings Settings;
  Settings.MouseSensitivity = 2.0f;
  Settings.bInvertMouseY = true;
  const FVector2D Look =
      LingTuSim::UI::FPlayerSettings::ApplyMouseLook(FVector2D(1.5, -2.0), Settings);
  TestEqual(TEXT("sensitivity scales yaw"), Look.X, 3.0);
  TestEqual(TEXT("sensitivity scales and inversion reverses pitch"), Look.Y, 4.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimPlayerSettingsConfigRoundTripTest,
    "LingTuSim.UI.PlayerSettings.ConfigRoundTripIsolated",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimPlayerSettingsConfigRoundTripTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const FString Root = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"),
                                       TEXT("LingTuSimUI"),
                                       FGuid::NewGuid().ToString(EGuidFormats::Digits));
  const FString IniFilename = FPaths::Combine(Root, TEXT("PlayerSettings.ini"));
  ON_SCOPE_EXIT {
    GConfig->UnloadFile(IniFilename);
    IFileManager::Get().DeleteDirectory(*Root, false, true);
  };
  if (!TestTrue(TEXT("isolated settings directory is created"),
                IFileManager::Get().MakeDirectory(*Root, true))) {
    return false;
  }

  LingTuSim::UI::FPlayerSettings Saved;
  Saved.MouseSensitivity = 2.5f;
  Saved.bInvertMouseY = true;
  Saved.bShowControlHints = false;
  Saved.bShowFrameRate = true;
  Saved.MasterVolume = .4F;
  Saved.UIVolume = .7F;
  Saved.MusicVolume = .35F;
  Saved.MenuMusicTrack = 1;
  Saved.bMuteAudio = true;
  FString BindError;
  TestTrue(TEXT("isolated round-trip binding is valid"),
           Saved.DriveKeys.TryBind(LingTuSim::UI::EDriveKeyAction::Forward, EKeys::I,
                                   BindError));
  Saved.Save(IniFilename);
  if (!TestTrue(TEXT("settings are flushed to the isolated config file"),
                IFileManager::Get().FileExists(*IniFilename))) {
    return false;
  }
  GConfig->UnloadFile(IniFilename);
  GConfig->LoadFile(IniFilename);

  LingTuSim::UI::FPlayerSettings Loaded;
  Loaded.Load(IniFilename);
  TestEqual(TEXT("sensitivity round trips through isolated config"), Loaded.MouseSensitivity,
            2.5f);
  TestTrue(TEXT("inversion round trips through isolated config"), Loaded.bInvertMouseY);
  TestFalse(TEXT("hint preference round trips through isolated config"),
            Loaded.bShowControlHints);
  TestTrue(TEXT("frame counter preference round trips through isolated config"), Loaded.bShowFrameRate);
  TestEqual(TEXT("master level round trips through an unloaded config"), Loaded.MasterVolume, .4F);
  TestEqual(TEXT("UI level round trips through an unloaded config"), Loaded.UIVolume, .7F);
  TestEqual(TEXT("music level round trips through an unloaded config"), Loaded.MusicVolume, .35F);
  TestEqual(TEXT("music choice round trips through an unloaded config"), Loaded.MenuMusicTrack, 1);
  TestTrue(TEXT("mute round trips through an unloaded config"), Loaded.bMuteAudio);
  TestEqual(TEXT("drive key round trips through isolated config"),
            Loaded.DriveKeys.GetKey(LingTuSim::UI::EDriveKeyAction::Forward), EKeys::I);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimPlayerSettingsDriveKeyFallbackTest,
    "LingTuSim.UI.PlayerSettings.DriveKeyFallbackIsAtomic",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimPlayerSettingsDriveKeyFallbackTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  const FString Root = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"),
                                       TEXT("LingTuSimUI"),
                                       FGuid::NewGuid().ToString(EGuidFormats::Digits));
  const FString IniFilename = FPaths::Combine(Root, TEXT("PlayerSettings.ini"));
  ON_SCOPE_EXIT {
    GConfig->UnloadFile(IniFilename);
    IFileManager::Get().DeleteDirectory(*Root, false, true);
  };
  if (!TestTrue(TEXT("isolated fallback directory is created"),
                IFileManager::Get().MakeDirectory(*Root, true))) {
    return false;
  }

  LingTuSim::UI::FPlayerSettings Seed;
  TestTrue(TEXT("seed settings are saved"), Seed.Save(IniFilename));

  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyForward"), TEXT("I"), IniFilename);
  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyBackward"), IniFilename);
  TestTrue(TEXT("partial config is flushed"), GConfig->Flush(false, IniFilename));
  GConfig->UnloadFile(IniFilename);
  GConfig->LoadFile(IniFilename);

  LingTuSim::UI::FPlayerSettings Partial;
  Partial.Load(IniFilename);
  const LingTuSim::UI::FDriveKeyBindings Defaults;
  TestTrue(TEXT("partial drive-key config falls back as one atomic set"),
           Partial.DriveKeys == Defaults);

  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyForward"), TEXT("I"), IniFilename);
  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyBackward"), TEXT("I"), IniFilename);
  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyLeft"), TEXT("A"), IniFilename);
  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyRight"), TEXT("D"), IniFilename);
  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyTurnLeft"), TEXT("Q"), IniFilename);
  GConfig->SetString(TEXT("LingTuSim.Player"), TEXT("KeyTurnRight"), TEXT("E"), IniFilename);
  TestTrue(TEXT("corrupt config is flushed"), GConfig->Flush(false, IniFilename));
  GConfig->UnloadFile(IniFilename);
  GConfig->LoadFile(IniFilename);

  LingTuSim::UI::FPlayerSettings Corrupt;
  Corrupt.Load(IniFilename);
  TestTrue(TEXT("duplicate drive-key config falls back as one atomic set"),
           Corrupt.DriveKeys == Defaults);

  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyForward"), IniFilename);
  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyBackward"), IniFilename);
  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyLeft"), IniFilename);
  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyRight"), IniFilename);
  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyTurnLeft"), IniFilename);
  GConfig->RemoveKey(TEXT("LingTuSim.Player"), TEXT("KeyTurnRight"), IniFilename);
  TestTrue(TEXT("legacy config is flushed"), GConfig->Flush(false, IniFilename));
  GConfig->UnloadFile(IniFilename);
  GConfig->LoadFile(IniFilename);

  LingTuSim::UI::FPlayerSettings Legacy;
  Legacy.Load(IniFilename);
  TestTrue(TEXT("legacy config with no drive keys keeps defaults"), Legacy.DriveKeys == Defaults);
  return true;
}

#endif
