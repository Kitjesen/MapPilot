#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimMenuAudio.h"

#include "Components/AudioComponent.h"
#include "Audio.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Math/UnrealMathUtility.h"
#include "Misc/AutomationTest.h"
#include "Sound/SoundWaveProcedural.h"
#include "UObject/UObjectHash.h"
#include "LingTuSimPlayerSettings.h"
#include "Interfaces/IPluginManager.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuAudioSwitchTest, "LingTuSim.UI.MenuAudio.SwitchPCM",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuAudioSwitchTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using namespace LingTuSim::UI;
  const TArray<int16> Samples = MakeMenuCueSamples(EMenuCue::Switch);
  if (!TestEqual(TEXT("switch is 48 ms at the default 24 kHz"), Samples.Num(), 1152)) {
    return false;
  }
  TestTrue(TEXT("PCM is deterministic"), Samples == MakeMenuCueSamples(EMenuCue::Switch));
  TestEqual(TEXT("attack starts at silence"), Samples[0], int16(0));
  TestEqual(TEXT("release ends at silence"), Samples.Last(), int16(0));
  int32 Peak = 0;
  for (const int16 Sample : Samples) {
    Peak = FMath::Max(Peak, FMath::Abs(int32(Sample)));
  }
  TestTrue(TEXT("cue has a nonzero signal"), Peak > 100);
  TestTrue(TEXT("quiet peak stays below 6 percent full scale, far from clipping"), Peak <= 1966);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuAudioDistinctTest, "LingTuSim.UI.MenuAudio.DistinctCues",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuAudioDistinctTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using namespace LingTuSim::UI;
  const TArray<int16> Cues[] = {MakeMenuCueSamples(EMenuCue::Switch),
                              MakeMenuCueSamples(EMenuCue::Change),
                              MakeMenuCueSamples(EMenuCue::Confirm)};
  TestEqual(TEXT("change is 60 ms at 24 kHz"), Cues[1].Num(), 1440);
  TestEqual(TEXT("confirm is 76 ms at 24 kHz"), Cues[2].Num(), 1824);
  for (int32 Left = 0; Left < 3; ++Left) {
    for (int32 Right = Left + 1; Right < 3; ++Right) {
      bool bDifferentSignal = false;
      const int32 CommonSamples = FMath::Min(Cues[Left].Num(), Cues[Right].Num());
      for (int32 Index = 0; Index < CommonSamples; ++Index) {
        bDifferentSignal |= Cues[Left][Index] != Cues[Right][Index];
      }
      TestTrue(FString::Printf(TEXT("cues %d and %d differ in signal, not just length"), Left,
                               Right),
               bDifferentSignal);
    }
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuAudioRatesTest, "LingTuSim.UI.MenuAudio.PCMContractAcrossRates",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuAudioRatesTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using namespace LingTuSim::UI;
  const EMenuCue Cues[] = {EMenuCue::Switch, EMenuCue::Change, EMenuCue::Confirm};
  const int32 Rates[] = {8000, 24000, 44100, 48000, 192000};
  for (const EMenuCue Cue : Cues) {
    for (const int32 Rate : Rates) {
      const FString Context = FString::Printf(TEXT("cue %d at %d Hz"), int32(Cue), Rate);
      const TArray<int16> Samples = MakeMenuCueSamples(Cue, Rate);
      const double Duration = static_cast<double>(Samples.Num()) / Rate;
      if (!TestTrue(Context + TEXT(" lasts 40..80 ms"), Duration >= 0.040 && Duration <= 0.080)) {
        continue;
      }
      TestTrue(Context + TEXT(" is deterministic"), Samples == MakeMenuCueSamples(Cue, Rate));
      TestEqual(Context + TEXT(" starts at silence"), Samples[0], int16(0));
      TestEqual(Context + TEXT(" ends at silence"), Samples.Last(), int16(0));
      int32 Peak = 0;
      int32 EdgePeak = 0;
      const int32 EdgeSamples = Rate / 1000;
      for (int32 Index = 0; Index < Samples.Num(); ++Index) {
        const int32 Magnitude = FMath::Abs(int32(Samples[Index]));
        Peak = FMath::Max(Peak, Magnitude);
        if (Index < EdgeSamples || Index >= Samples.Num() - EdgeSamples) {
          EdgePeak = FMath::Max(EdgePeak, Magnitude);
        }
      }
      TestTrue(Context + TEXT(" has nonzero PCM"), Peak > 100);
      TestTrue(Context + TEXT(" is quiet and cannot clip"), Peak <= 1966);
      TestTrue(Context + TEXT(" fades at both edges, not just the endpoints"), EdgePeak < Peak / 2);
    }
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuAudioInvalidTest, "LingTuSim.UI.MenuAudio.InvalidInputIsEmpty",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuAudioInvalidTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using namespace LingTuSim::UI;
  for (const int32 Rate : {MIN_int32, -1, 0, 7999, 192001, MAX_int32}) {
    TestTrue(FString::Printf(TEXT("unsupported rate %d produces no PCM"), Rate),
             MakeMenuCueSamples(EMenuCue::Switch, Rate).IsEmpty());
  }
  TestTrue(TEXT("unknown cue produces no PCM"),
           MakeMenuCueSamples(static_cast<EMenuCue>(255)).IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuAudioNullWorldTest, "LingTuSim.UI.MenuAudio.NullWorldIsInert",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuAudioNullWorldTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using namespace LingTuSim::UI;
  TArray<UObject *> ComponentsBefore;
  TArray<UObject *> WavesBefore;
  GetObjectsOfClass(UAudioComponent::StaticClass(), ComponentsBefore);
  GetObjectsOfClass(USoundWaveProcedural::StaticClass(), WavesBefore);
  for (const EMenuCue Cue : {EMenuCue::Switch, EMenuCue::Change, EMenuCue::Confirm}) {
    PlayMenuCue(nullptr, Cue);
  }
  TArray<UObject *> ComponentsAfter;
  TArray<UObject *> WavesAfter;
  GetObjectsOfClass(UAudioComponent::StaticClass(), ComponentsAfter);
  GetObjectsOfClass(USoundWaveProcedural::StaticClass(), WavesAfter);
  TestEqual(TEXT("null world creates no audio components"), ComponentsAfter.Num(),
            ComponentsBefore.Num());
  TestEqual(TEXT("null world creates no procedural sound waves"), WavesAfter.Num(),
            WavesBefore.Num());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuAudioPreviewFilesTest, "LingTuSim.UI.MenuAudio.PreviewWavFiles",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuAudioPreviewFilesTest::RunTest(const FString &Parameters) {
  (void)Parameters;
  using namespace LingTuSim::UI;
  const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"),
      TEXT("LingTuSimUI"), TEXT("AudioSamples"));
  if (!TestTrue(TEXT("audio sample output exists"), IFileManager::Get().MakeDirectory(*Directory, true))) return false;
  const EMenuCue Cues[] = {EMenuCue::Switch, EMenuCue::Change, EMenuCue::Confirm};
  const TCHAR* Names[] = {TEXT("switch.wav"), TEXT("change.wav"), TEXT("confirm.wav")};
  for (int32 Index = 0; Index < UE_ARRAY_COUNT(Cues); ++Index) {
    const auto PCM = MakeMenuCueSamples(Cues[Index]);
    TArray<uint8> Wave;
    SerializeWaveFile(Wave, reinterpret_cast<const uint8*>(PCM.GetData()),
        PCM.Num() * int32(sizeof(int16)), 1, 24000);
    const FString Filename = FPaths::Combine(Directory, Names[Index]);
    TestTrue(TEXT("WAV contains its PCM plus format metadata"), Wave.Num() > PCM.Num() * int32(sizeof(int16)));
    TestTrue(FString::Printf(TEXT("save original cue sample %s"), Names[Index]), FFileHelper::SaveArrayToFile(Wave, *Filename));
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimMenuMusicAssetsTest, "LingTuSim.UI.MenuAudio.OriginalMusicAssets",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMenuMusicAssetsTest::RunTest(const FString&) {
  for (const TCHAR* Name : {TEXT("base-arrival"), TEXT("departure")}) {
    const FString Path = FString::Printf(TEXT("/Game/RobotSim/Audio/Menu/%s.%s"), Name, Name);
    auto* Sound = LoadObject<USoundWave>(nullptr, *Path);
    if (!TestNotNull(TEXT("original music is imported into the actual game"), Sound)) return false;
    TestTrue(TEXT("ordinary sound wave loops without creating a component per cycle"), Sound->bLooping);
    TestTrue(TEXT("zero volume preserves playback so raising it does not require restarting the track"), Sound->IsPlayWhenSilent());
    TestEqual(TEXT("authored music remains stereo"), Sound->NumChannels, 2);
    TestEqual(TEXT("authored loop is 32 seconds, not a UI click"), Sound->Duration, 32.F);
  }
  LingTuSim::UI::FFrontEndMusicPlayer Player;
  Player.Update(nullptr, LingTuSim::UI::FPlayerSettings(), true);
  TestFalse(TEXT("null presentation worlds never create background playback"), Player.IsPlaying());
  Player.Stop();
  const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("LingTuSim"));
  if (!TestTrue(TEXT("UI plugin is available"), Plugin.IsValid())) return false;
  TestTrue(TEXT("clean factory render is staged for the real welcome page"),
      IFileManager::Get().FileSize(*FPaths::Combine(Plugin->GetBaseDir(), TEXT("Resources/FrontEnd/factory-corridor.png"))) > 100000);
  return true;
}

#endif  // WITH_DEV_AUTOMATION_TESTS
