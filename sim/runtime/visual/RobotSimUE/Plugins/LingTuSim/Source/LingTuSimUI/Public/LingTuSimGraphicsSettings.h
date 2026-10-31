#pragma once

#include "CoreMinimal.h"

class UWorld;
#if WITH_DEV_AUTOMATION_TESTS
class FLingTuSimGraphicsSettingsDeadlineTest;
#endif

namespace LingTuSim::UI {

struct LINGTUSIMUI_API FPlayerGraphicsSettings final {
  FIntPoint Resolution = FIntPoint(1920, 1080);
  int32 WindowMode = 2;
  float FrameRateLimit = 60.0F;
  bool bVSync = false;
  float ResolutionScale = 100.0F;
  int32 ViewDistanceQuality = 2;
  int32 ShadowQuality = 2;
  int32 TextureQuality = 2;
  int32 AntiAliasingQuality = 2;
  int32 PostProcessingQuality = 2;

  void Normalize();
  bool IsEquivalentTo(const FPlayerGraphicsSettings &Other) const;
  void SetQualityPreset(int32 Preset);
  int32 GetQualityPreset() const;
  void Load();

  static FPlayerGraphicsSettings Capture(UWorld *World);
};

class LINGTUSIMUI_API FPlayerGraphicsPreview final {
 public:
  FPlayerGraphicsPreview() = default;
  ~FPlayerGraphicsPreview();

  FPlayerGraphicsPreview(const FPlayerGraphicsPreview &) = delete;
  FPlayerGraphicsPreview &operator=(const FPlayerGraphicsPreview &) = delete;
  FPlayerGraphicsPreview(FPlayerGraphicsPreview &&) = delete;
  FPlayerGraphicsPreview &operator=(FPlayerGraphicsPreview &&) = delete;

  bool Apply(UWorld *World, const FPlayerGraphicsSettings &Settings, double Now,
             FString &Error);
  bool Confirm(FString &Error);
  void Revert();
  bool Tick(double Now);
  bool IsPending() const;
  double SecondsRemaining(double Now) const;

  static bool IsSessionLocked();

 private:
#if WITH_DEV_AUTOMATION_TESTS
  friend class ::FLingTuSimGraphicsSettingsDeadlineTest;
#endif

  FPlayerGraphicsSettings Before;
  FPlayerGraphicsSettings Applied;
  float BeforeGameSettingsFrameRateLimit = 0.0F;
  float BeforeEngineMaxFPS = 0.0F;
  float BeforeScreenPercentage = 0.0F;
  double DeadlineSeconds = 0.0;
  bool bHasBefore = false;
  bool bPending = false;
};

LINGTUSIMUI_API TArray<FIntPoint> GraphicsResolutionOptions(UWorld *World);

}  // namespace LingTuSim::UI
