#pragma once

#include "Containers/Array.h"
#include "CoreTypes.h"
#include "UObject/WeakObjectPtrTemplates.h"

class UWorld;
class UAudioComponent;

namespace LingTuSim::UI {
struct FPlayerSettings;

enum class EMenuCue { Switch, Change, Confirm };

// Original, quiet mono signed-16-bit PCM; 48/60/76 ms with silent endpoints.
// Supported sample rates are 8000..192000 Hz; invalid rates/cues return no samples.
TArray<int16> MakeMenuCueSamples(EMenuCue Cue, int32 SampleRate = 24000);

// Game-thread, non-spatial UI one-shot. Null/silent worlds and -NoSound are inert.
// No shared state or throttling: callers decide which UI events warrant a cue.
void PlayMenuCue(UWorld *World, EMenuCue Cue, float UIVolume = 1.F);

// Selected UE device only. Never changes the operating system's volume.
void ApplyPlayerAudioSettings(UWorld *World, const FPlayerSettings &Settings);

// One menu-owned player survives page switches, but not entering simulation or
// destroying the HUD. Imported original loops use normal UE audio streaming.
class FFrontEndMusicPlayer final {
 public:
  ~FFrontEndMusicPlayer();
  void Update(UWorld* World, const FPlayerSettings& Settings, bool bInFrontEnd);
  void Stop();
  bool IsPlaying() const;
 private:
  TWeakObjectPtr<UAudioComponent> Component;
  TWeakObjectPtr<UAudioComponent> FadingComponent;
  int32 CurrentTrack = INDEX_NONE;
};

}  // namespace LingTuSim::UI
