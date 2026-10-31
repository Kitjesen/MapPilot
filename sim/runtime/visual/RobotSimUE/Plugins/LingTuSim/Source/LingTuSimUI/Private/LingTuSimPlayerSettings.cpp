#include "LingTuSimPlayerSettings.h"

#include "Misc/ConfigCacheIni.h"

namespace LingTuSim::UI {
namespace {
constexpr TCHAR PlayerSettingsSection[] = TEXT("LingTuSim.Player");
constexpr float DefaultMouseSensitivity = 1.0f;
constexpr float MinimumMouseSensitivity = 0.25f;
constexpr float MaximumMouseSensitivity = 3.0f;
constexpr int32 DriveKeyActionCount = static_cast<int32>(EDriveKeyAction::Count);
float AudioLevel(const float Value) {
  return FMath::IsFinite(Value) ? FMath::Clamp(Value, 0.F, 1.F) : 1.F;
}
}  // namespace

void FPlayerSettings::Normalize() {
  MasterVolume = AudioLevel(MasterVolume);
  UIVolume = AudioLevel(UIVolume);
  MusicVolume = AudioLevel(MusicVolume);
  MenuMusicTrack = FMath::Clamp(MenuMusicTrack, 0, 1);
  MouseSensitivity = FMath::IsFinite(MouseSensitivity)
                         ? FMath::Clamp(MouseSensitivity, MinimumMouseSensitivity,
                                        MaximumMouseSensitivity)
                         : DefaultMouseSensitivity;
  DriveKeys.Normalize();
}

float FPlayerSettings::GetMasterAudioGain() const {
  return bMuteAudio ? 0.F : AudioLevel(MasterVolume);
}

float FPlayerSettings::GetUIAudioGain() const {
  return AudioLevel(UIVolume);
}

void FPlayerSettings::Load() {
  Load(GGameUserSettingsIni);
}

void FPlayerSettings::Load(const FString &IniFilename) {
  GConfig->GetFloat(PlayerSettingsSection, TEXT("MouseSensitivity"), MouseSensitivity,
                    IniFilename);
  GConfig->GetBool(PlayerSettingsSection, TEXT("InvertMouseY"), bInvertMouseY, IniFilename);
  GConfig->GetBool(PlayerSettingsSection, TEXT("ShowControlHints"), bShowControlHints,
                   IniFilename);
  GConfig->GetBool(PlayerSettingsSection, TEXT("ShowFrameRate"), bShowFrameRate, IniFilename);
  GConfig->GetFloat(PlayerSettingsSection, TEXT("MasterVolume"), MasterVolume, IniFilename);
  GConfig->GetFloat(PlayerSettingsSection, TEXT("UIVolume"), UIVolume, IniFilename);
  GConfig->GetFloat(PlayerSettingsSection, TEXT("MusicVolume"), MusicVolume, IniFilename);
  GConfig->GetInt(PlayerSettingsSection, TEXT("MenuMusicTrack"), MenuMusicTrack, IniFilename);
  GConfig->GetBool(PlayerSettingsSection, TEXT("MuteAudio"), bMuteAudio, IniFilename);

  FDriveKeyBindings LoadedDriveKeys;
  bool bAnyDriveKey = false;
  bool bAllDriveKeys = true;
  for (int32 Index = 0; Index < DriveKeyActionCount; ++Index) {
    const EDriveKeyAction Action = static_cast<EDriveKeyAction>(Index);
    FString KeyName;
    const bool bFound = GConfig->GetString(PlayerSettingsSection,
                                           FDriveKeyBindings::ConfigKey(Action), KeyName,
                                           IniFilename);
    bAnyDriveKey |= bFound;
    bAllDriveKeys &= bFound;
    if (bFound) {
      LoadedDriveKeys.Keys[Index] = FKey(FName(*KeyName));
    }
  }
  if (bAnyDriveKey && bAllDriveKeys) {
    LoadedDriveKeys.Normalize();
    DriveKeys = LoadedDriveKeys;
  } else {
    DriveKeys = FDriveKeyBindings();
  }
  Normalize();
}

bool FPlayerSettings::Save() const {
  return Save(GGameUserSettingsIni);
}

bool FPlayerSettings::Save(const FString &IniFilename) const {
  FPlayerSettings Normalized = *this;
  Normalized.Normalize();
  if (!GConfig->FindConfigFile(IniFilename)) {
    FConfigFile File;
    File.Read(IniFilename);
    GConfig->Add(IniFilename, File);
  }
  GConfig->SetFloat(PlayerSettingsSection, TEXT("MouseSensitivity"),
                    Normalized.MouseSensitivity, IniFilename);
  GConfig->SetBool(PlayerSettingsSection, TEXT("InvertMouseY"), Normalized.bInvertMouseY,
                   IniFilename);
  GConfig->SetBool(PlayerSettingsSection, TEXT("ShowControlHints"),
                   Normalized.bShowControlHints, IniFilename);
  GConfig->SetBool(PlayerSettingsSection, TEXT("ShowFrameRate"), Normalized.bShowFrameRate, IniFilename);
  GConfig->SetFloat(PlayerSettingsSection, TEXT("MasterVolume"), Normalized.MasterVolume, IniFilename);
  GConfig->SetFloat(PlayerSettingsSection, TEXT("UIVolume"), Normalized.UIVolume, IniFilename);
  GConfig->SetFloat(PlayerSettingsSection, TEXT("MusicVolume"), Normalized.MusicVolume, IniFilename);
  GConfig->SetInt(PlayerSettingsSection, TEXT("MenuMusicTrack"), Normalized.MenuMusicTrack, IniFilename);
  GConfig->SetBool(PlayerSettingsSection, TEXT("MuteAudio"), Normalized.bMuteAudio, IniFilename);
  for (int32 Index = 0; Index < DriveKeyActionCount; ++Index) {
    const EDriveKeyAction Action = static_cast<EDriveKeyAction>(Index);
    GConfig->SetString(PlayerSettingsSection, FDriveKeyBindings::ConfigKey(Action),
                       *Normalized.DriveKeys.GetKey(Action).GetFName().ToString(), IniFilename);
  }
  return GConfig->Flush(false, IniFilename);
}

FVector2D FPlayerSettings::ApplyMouseLook(const FVector2D &DegreeDeltas,
                                         const FPlayerSettings &Settings) {
  FPlayerSettings Normalized = Settings;
  Normalized.Normalize();
  const double PitchSign = Normalized.bInvertMouseY ? -1.0 : 1.0;
  return FVector2D(DegreeDeltas.X * Normalized.MouseSensitivity,
                   DegreeDeltas.Y * Normalized.MouseSensitivity * PitchSign);
}
}  // namespace LingTuSim::UI
