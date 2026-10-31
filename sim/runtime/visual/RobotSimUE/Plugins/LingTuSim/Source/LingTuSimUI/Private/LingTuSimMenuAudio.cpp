#include "LingTuSimMenuAudio.h"
#include "LingTuSimPlayerSettings.h"

#include "AudioDevice.h"
#include "Components/AudioComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "Math/UnrealMathUtility.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Sound/SoundWaveProcedural.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/UObjectGlobals.h"

namespace LingTuSim::UI {

TArray<int16> MakeMenuCueSamples(const EMenuCue Cue, const int32 SampleRate) {
  if (SampleRate < 8000 || SampleRate > 192000) {
    return {};
  }

  double Duration = 0.048;
  double Frequency = 900.0;
  double Sweep = -250.0;
  switch (Cue) {
    case EMenuCue::Switch:
      break;
    case EMenuCue::Change:
      Duration = 0.060;
      Frequency = 620.0;
      Sweep = 260.0;
      break;
    case EMenuCue::Confirm:
      Duration = 0.076;
      Frequency = 1040.0;
      Sweep = 520.0;
      break;
    default:
      return {};
  }

  const int32 SampleCount = FMath::RoundToInt32(SampleRate * Duration);
  TArray<int16> Samples;
  Samples.SetNumZeroed(SampleCount);
  for (int32 Index = 1; Index < SampleCount - 1; ++Index) {
    const double Time = static_cast<double>(Index) / SampleRate;
    const double Remaining = static_cast<double>(SampleCount - 1 - Index) / SampleRate;
    const double Attack = FMath::Min(Time / 0.003, 1.0);
    const double Release = FMath::Min(Remaining / 0.012, 1.0);
    const double Envelope =
        Attack * Attack * Release * Release * FMath::Exp(-2.5 * Time / Duration);
    const double Phase =
        2.0 * UE_DOUBLE_PI * (Frequency * Time + 0.5 * Sweep * Time * Time / Duration);
    const double Tone = 0.8 * FMath::Sin(Phase) + 0.2 * FMath::Sin(1.5 * Phase);
    // The weighted oscillators and envelope are <= 1: peak <= 1800 / 32767 (~5.5%).
    Samples[Index] = static_cast<int16>(FMath::RoundToInt32(1800.0 * Envelope * Tone));
  }
  return Samples;
}

void ApplyPlayerAudioSettings(UWorld *World, const FPlayerSettings &Settings) {
  if (!World || !IsInGameThread() || World->bIsTearingDown || !GEngine ||
      !GEngine->UseSound() || !World->bAllowAudioPlayback) return;
  if (FAudioDevice *Device = World->GetAudioDeviceRaw()) {
    Device->SetTransientPrimaryVolume(Settings.GetMasterAudioGain());
  }
}

void PlayMenuCue(UWorld *World, const EMenuCue Cue, const float UIVolume) {
  if (!FMath::IsFinite(UIVolume) || UIVolume <= 0.F) return;
  if (!World || !IsInGameThread() || !GEngine || !GEngine->UseSound() ||
      FParse::Param(FCommandLine::Get(), TEXT("NoSound")) || !World->bAllowAudioPlayback ||
      World->bIsTearingDown || World->IsNetMode(NM_DedicatedServer) ||
      !World->GetAudioDeviceRaw()) {
    return;
  }

  constexpr int32 SampleRate = 24000;
  const TArray<int16> Samples = MakeMenuCueSamples(Cue, SampleRate);
  if (Samples.IsEmpty()) {
    return;
  }

  // Hold the wave through spawning, then the component's UPROPERTY Sound owns its GC lifetime.
  const TStrongObjectPtr<USoundWaveProcedural> Sound(
      NewObject<USoundWaveProcedural>(GetTransientPackage(), NAME_None, RF_Transient));
  Sound->SetSampleRate(SampleRate);
  Sound->SetNumFrames(static_cast<uint32>(Samples.Num()));
  Sound->NumChannels = 1;
  Sound->Duration = static_cast<float>(Samples.Num()) / SampleRate;
  Sound->bLooping = false;
  Sound->SoundGroup = SOUNDGROUP_UI;
  Sound->QueueAudio(reinterpret_cast<const uint8 *>(Samples.GetData()),
                    Samples.Num() * static_cast<int32>(sizeof(int16)));

  if (UAudioComponent *Component =
          UGameplayStatics::SpawnSound2D(World, Sound.Get(), FMath::Min(UIVolume, 1.F), 1.0f, 0.0f, nullptr,
                                        /*bPersistAcrossLevelTransition=*/false,
                                        /*bAutoDestroy=*/true)) {
    // Procedural underflow otherwise produces silence forever. Stop on the audio side (also
    // while the game is paused), allowing a 25 ms buffer tail. Completion auto-destroys
    // the component.
    Component->StopDelayed(Sound->Duration + 0.025f);
  }
}

FFrontEndMusicPlayer::~FFrontEndMusicPlayer() { Stop(); }

bool FFrontEndMusicPlayer::IsPlaying() const {
  return Component.IsValid() && Component->IsPlaying();
}

void FFrontEndMusicPlayer::Stop() {
  for (auto* Audio : {Component.Get(), FadingComponent.Get()}) {
    if (Audio) Audio->DestroyComponent();
  }
  Component.Reset();
  FadingComponent.Reset();
  CurrentTrack = INDEX_NONE;
}

void FFrontEndMusicPlayer::Update(UWorld* World, const FPlayerSettings& Settings, const bool bInFrontEnd) {
  if (!World || World->bIsTearingDown || !GEngine || !GEngine->UseSound() ||
      !World->bAllowAudioPlayback || !World->GetAudioDeviceRaw()) {
    Stop();
    return;
  }
  if (!bInFrontEnd) {
    if (auto* Audio = Component.Get()) {
      if (auto* Previous = FadingComponent.Get()) Previous->DestroyComponent();
      FadingComponent = Component;
      Audio->FadeOut(.4F, 0.F);
      Component.Reset();
      CurrentTrack = INDEX_NONE;
    }
    return;
  }
  const int32 Track = FMath::Clamp(Settings.MenuMusicTrack, 0, 1);
  if (CurrentTrack != Track) {
    if (auto* Previous = FadingComponent.Get()) Previous->DestroyComponent();
    FadingComponent = Component;
    if (auto* Previous = Component.Get()) Previous->FadeOut(.4F, 0.F);
    const TCHAR* Paths[] = {
      TEXT("/Game/RobotSim/Audio/Menu/base-arrival.base-arrival"),
      TEXT("/Game/RobotSim/Audio/Menu/departure.departure")};
    CurrentTrack = Track;
    auto* Sound = LoadObject<USoundWave>(nullptr, Paths[Track]);
    if (Sound) {
      auto* Audio = UGameplayStatics::CreateSound2D(World, Sound, Settings.MusicVolume,
          1.F, 0.F, nullptr, false, true);
      Component = Audio;
      if (Audio) {
        Audio->FadeIn(.7F, Settings.MusicVolume);
        UE_LOG(LogTemp, Display, TEXT("LINGTU_MENU_MUSIC_STARTED track=%d duration=%.1f"), Track, Sound->Duration);
      }
    }
  } else if (auto* Audio = Component.Get()) {
    Audio->SetVolumeMultiplier(FMath::Clamp(Settings.MusicVolume, 0.F, 1.F));
  }
}

}  // namespace LingTuSim::UI
