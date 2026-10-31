#pragma once

#include "CoreMinimal.h"
#include "LingTuSimDriveKeyBindings.h"

namespace LingTuSim::UI {
struct LINGTUSIMUI_API FPlayerSettings final {
  float MouseSensitivity = 1.0f;
  bool bInvertMouseY = false;
  bool bShowControlHints = true;
  bool bShowFrameRate = false;
  float MasterVolume = 1.F;
  float UIVolume = 1.F;
  float MusicVolume = .5F;
  int32 MenuMusicTrack = 0;
  bool bMuteAudio = false;
  FDriveKeyBindings DriveKeys;

  void Normalize();
  void Load();
  void Load(const FString &IniFilename);
  bool Save() const;
  bool Save(const FString &IniFilename) const;
  float GetMasterAudioGain() const;
  float GetUIAudioGain() const;

  static FVector2D ApplyMouseLook(const FVector2D &DegreeDeltas,
                                  const FPlayerSettings &Settings);
};
}  // namespace LingTuSim::UI
