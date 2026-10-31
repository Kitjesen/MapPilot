#include "LingTuSimGraphicsSettings.h"

#include "Engine/Engine.h"
#include "Engine/GameInstance.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "GameFramework/GameUserSettings.h"
#include "Kismet/KismetSystemLibrary.h"
#include "LingTuSimSessionService.h"
#include "Misc/ConfigCacheIni.h"
#include "Widgets/SWindow.h"
#include "UnrealClient.h"
#include "HAL/IConsoleManager.h"

namespace LingTuSim::UI {
namespace {
constexpr TCHAR GraphicsSettingsSection[] = TEXT("LingTuSim.Graphics");
constexpr double PreviewDurationSeconds = 15.0;
constexpr int32 MinimumResolutionWidth = 640;
constexpr int32 MinimumResolutionHeight = 360;
constexpr int32 MaximumResolutionDimension = 16384;
constexpr float MaximumFrameRateLimit = 360.0F;
constexpr float MinimumResolutionScale = 50.0F;
constexpr float MaximumResolutionScale = 100.0F;

UGameUserSettings *GetGameUserSettings() {
  return GEngine != nullptr ? GEngine->GetGameUserSettings() : nullptr;
}

UGameViewportClient *GetGameViewport(UWorld *World) {
  if (World == nullptr || !World->IsGameWorld()) return nullptr;
  UGameInstance *GameInstance = World->GetGameInstance();
  UGameViewportClient *Viewport =
      GameInstance != nullptr ? GameInstance->GetGameViewportClient() : nullptr;
  return Viewport != nullptr && Viewport->Viewport != nullptr ? Viewport : nullptr;
}

bool HasGameViewport(UWorld *World) {
  const UGameViewportClient *Viewport = GetGameViewport(World);
  if (Viewport == nullptr) return false;
  const FIntPoint Size = Viewport->Viewport->GetSizeXY();
  const TSharedPtr<SWindow> Window = Viewport->GetWindow();
  return Size.X > 0 && Size.Y > 0 && Window.IsValid() &&
         Window->GetNativeWindow().IsValid();
}

EWindowMode::Type ToWindowMode(const int32 WindowMode) {
  return static_cast<EWindowMode::Type>(FMath::Clamp(WindowMode, 0, 2));
}

void SetScreenPercentage(const float Value) {
  if (auto* Variable = IConsoleManager::Get().FindConsoleVariable(TEXT("r.ScreenPercentage")))
    Variable->Set(Value, static_cast<EConsoleVariableFlags>(Variable->GetFlags() & ECVF_SetByMask));
}

void ApplyToEngine(const FPlayerGraphicsSettings &Settings,
                   const float GameSettingsFrameRateLimit) {
  UGameUserSettings *GameSettings = GetGameUserSettings();
  if (GameSettings == nullptr || GEngine == nullptr) return;

  GameSettings->SetScreenResolution(Settings.Resolution);
  GameSettings->SetFullscreenMode(ToWindowMode(Settings.WindowMode));
  GameSettings->ApplyResolutionSettings(false);

  GameSettings->SetFrameRateLimit(GameSettingsFrameRateLimit);
  GameSettings->SetVSyncEnabled(Settings.bVSync);
  if (Settings.ResolutionScale == 0.F) GameSettings->ScalabilityQuality.ResolutionQuality = 0.F;
  else GameSettings->SetResolutionScaleValueEx(Settings.ResolutionScale);
  GameSettings->SetViewDistanceQuality(Settings.ViewDistanceQuality);
  GameSettings->SetShadowQuality(Settings.ShadowQuality);
  GameSettings->SetTextureQuality(Settings.TextureQuality);
  GameSettings->SetAntiAliasingQuality(Settings.AntiAliasingQuality);
  GameSettings->SetPostProcessingQuality(Settings.PostProcessingQuality);
  GameSettings->ApplyNonResolutionSettings();
  if (Settings.ResolutionScale == 0.F) SetScreenPercentage(0.F);
  GEngine->SetMaxFPS(Settings.FrameRateLimit);
}

bool SaveCustomSettings(const FPlayerGraphicsSettings &Settings, FString &Error) {
  if (GConfig == nullptr || GGameUserSettingsIni.IsEmpty()) {
    Error = TEXT("game user settings config is unavailable");
    return false;
  }

  if (!GConfig->FindConfigFile(GGameUserSettingsIni)) {
    FConfigFile File;
    File.Read(GGameUserSettingsIni);
    GConfig->Add(GGameUserSettingsIni, File);
  }

  GConfig->SetInt(GraphicsSettingsSection, TEXT("ResolutionX"), Settings.Resolution.X,
                  GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("ResolutionY"), Settings.Resolution.Y,
                  GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("WindowMode"), Settings.WindowMode,
                  GGameUserSettingsIni);
  GConfig->SetFloat(GraphicsSettingsSection, TEXT("FrameRateLimit"),
                    Settings.FrameRateLimit, GGameUserSettingsIni);
  GConfig->SetBool(GraphicsSettingsSection, TEXT("VSync"), Settings.bVSync,
                   GGameUserSettingsIni);
  GConfig->SetFloat(GraphicsSettingsSection, TEXT("ResolutionScale"),
                    Settings.ResolutionScale, GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("ViewDistanceQuality"),
                  Settings.ViewDistanceQuality, GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("ShadowQuality"), Settings.ShadowQuality,
                  GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("TextureQuality"), Settings.TextureQuality,
                  GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("AntiAliasingQuality"),
                  Settings.AntiAliasingQuality, GGameUserSettingsIni);
  GConfig->SetInt(GraphicsSettingsSection, TEXT("PostProcessingQuality"),
                  Settings.PostProcessingQuality, GGameUserSettingsIni);
  if (!GConfig->Flush(false, GGameUserSettingsIni)) {
    Error = FString::Printf(TEXT("failed to flush %s"), *GGameUserSettingsIni);
    return false;
  }
  return true;
}
}  // namespace

void FPlayerGraphicsSettings::Normalize() {
  if (Resolution.X <= 0 || Resolution.Y <= 0) {
    Resolution = FIntPoint(1920, 1080);
  } else {
    Resolution.X = FMath::Clamp(Resolution.X, MinimumResolutionWidth,
                                MaximumResolutionDimension);
    Resolution.Y = FMath::Clamp(Resolution.Y, MinimumResolutionHeight,
                                MaximumResolutionDimension);
  }
  WindowMode = FMath::Clamp(WindowMode, 0, 2);
  FrameRateLimit = FMath::IsFinite(FrameRateLimit)
                       ? FMath::Clamp(FrameRateLimit, 0.0F, MaximumFrameRateLimit)
                       : 60.0F;
  ResolutionScale = ResolutionScale == 0.F ? 0.F : FMath::IsFinite(ResolutionScale)
                        ? FMath::Clamp(ResolutionScale, MinimumResolutionScale,
                                       MaximumResolutionScale)
                        : 100.0F;
  ViewDistanceQuality = FMath::Clamp(ViewDistanceQuality, 0, 3);
  ShadowQuality = FMath::Clamp(ShadowQuality, 0, 3);
  TextureQuality = FMath::Clamp(TextureQuality, 0, 3);
  AntiAliasingQuality = FMath::Clamp(AntiAliasingQuality, 0, 3);
  PostProcessingQuality = FMath::Clamp(PostProcessingQuality, 0, 3);
}

bool FPlayerGraphicsSettings::IsEquivalentTo(
    const FPlayerGraphicsSettings &Other) const {
  return Resolution == Other.Resolution && WindowMode == Other.WindowMode &&
         FMath::IsNearlyEqual(FrameRateLimit, Other.FrameRateLimit, UE_KINDA_SMALL_NUMBER) &&
         bVSync == Other.bVSync &&
         FMath::IsNearlyEqual(ResolutionScale, Other.ResolutionScale, UE_KINDA_SMALL_NUMBER) &&
         ViewDistanceQuality == Other.ViewDistanceQuality &&
         ShadowQuality == Other.ShadowQuality &&
         TextureQuality == Other.TextureQuality &&
         AntiAliasingQuality == Other.AntiAliasingQuality &&
         PostProcessingQuality == Other.PostProcessingQuality;
}

void FPlayerGraphicsSettings::SetQualityPreset(const int32 Preset) {
  const int32 Quality = FMath::Clamp(Preset, 0, 3);
  ViewDistanceQuality = Quality;
  ShadowQuality = Quality;
  TextureQuality = Quality;
  AntiAliasingQuality = Quality;
  PostProcessingQuality = Quality;
}

int32 FPlayerGraphicsSettings::GetQualityPreset() const {
  if (ViewDistanceQuality == ShadowQuality && ShadowQuality == TextureQuality &&
      TextureQuality == AntiAliasingQuality &&
      AntiAliasingQuality == PostProcessingQuality &&
      ViewDistanceQuality >= 0 && ViewDistanceQuality <= 3) {
    return ViewDistanceQuality;
  }
  return INDEX_NONE;
}

void FPlayerGraphicsSettings::Load() {
  if (GConfig == nullptr || GGameUserSettingsIni.IsEmpty()) return;

  GConfig->GetInt(GraphicsSettingsSection, TEXT("ResolutionX"), Resolution.X,
                  GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("ResolutionY"), Resolution.Y,
                  GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("WindowMode"), WindowMode,
                  GGameUserSettingsIni);
  GConfig->GetFloat(GraphicsSettingsSection, TEXT("FrameRateLimit"), FrameRateLimit,
                    GGameUserSettingsIni);
  GConfig->GetBool(GraphicsSettingsSection, TEXT("VSync"), bVSync,
                   GGameUserSettingsIni);
  GConfig->GetFloat(GraphicsSettingsSection, TEXT("ResolutionScale"), ResolutionScale,
                    GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("ViewDistanceQuality"),
                  ViewDistanceQuality, GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("ShadowQuality"), ShadowQuality,
                  GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("TextureQuality"), TextureQuality,
                  GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("AntiAliasingQuality"),
                  AntiAliasingQuality, GGameUserSettingsIni);
  GConfig->GetInt(GraphicsSettingsSection, TEXT("PostProcessingQuality"),
                  PostProcessingQuality, GGameUserSettingsIni);
  Normalize();
}

FPlayerGraphicsSettings FPlayerGraphicsSettings::Capture(UWorld *World) {
  FPlayerGraphicsSettings Result;
  const UGameUserSettings *GameSettings = GetGameUserSettings();
  if (GameSettings == nullptr) return Result;

  Result.Resolution = GameSettings->GetScreenResolution();
  Result.WindowMode = static_cast<int32>(GameSettings->GetFullscreenMode());
  if (const UGameViewportClient *Viewport = GetGameViewport(World)) {
    Result.Resolution = Viewport->Viewport->GetSizeXY();
    if (const TSharedPtr<SWindow> Window = Viewport->GetWindow();
        Window.IsValid() && Window->GetNativeWindow().IsValid()) {
      Result.WindowMode = static_cast<int32>(Window->GetWindowMode());
    }
  }
  Result.FrameRateLimit = GEngine != nullptr ? GEngine->GetMaxFPS()
                                             : GameSettings->GetFrameRateLimit();
  Result.bVSync = GameSettings->IsVSyncEnabled();
  float NormalizedScale = 1.0F;
  float MinimumScale = MinimumResolutionScale;
  float MaximumScale = MaximumResolutionScale;
  GameSettings->GetResolutionScaleInformationEx(NormalizedScale, Result.ResolutionScale,
                                                MinimumScale, MaximumScale);
  Result.ViewDistanceQuality = GameSettings->GetViewDistanceQuality();
  Result.ShadowQuality = GameSettings->GetShadowQuality();
  Result.TextureQuality = GameSettings->GetTextureQuality();
  Result.AntiAliasingQuality = GameSettings->GetAntiAliasingQuality();
  Result.PostProcessingQuality = GameSettings->GetPostProcessingQuality();
  return Result;
}

FPlayerGraphicsPreview::~FPlayerGraphicsPreview() { Revert(); }

bool FPlayerGraphicsPreview::Apply(UWorld *World,
                                   const FPlayerGraphicsSettings &Settings,
                                   const double Now, FString &Error) {
  Error.Reset();
  if (IsSessionLocked()) {
    Error = TEXT("graphics settings are locked while a simulation session is bound");
    return false;
  }
  if (!HasGameViewport(World)) {
    Error = TEXT("a running game viewport is required to apply graphics settings");
    return false;
  }
  if (GetGameUserSettings() == nullptr || GEngine == nullptr) {
    Error = TEXT("game user settings are unavailable");
    return false;
  }
  if (!FMath::IsFinite(Now)) {
    Error = TEXT("preview time is invalid");
    return false;
  }

  GetGameUserSettings()->ValidateSettings();
  if (!bPending) {
    Before = FPlayerGraphicsSettings::Capture(World);
    BeforeGameSettingsFrameRateLimit = GetGameUserSettings()->GetFrameRateLimit();
    BeforeEngineMaxFPS = GEngine->GetMaxFPS();
    if (auto* Variable = IConsoleManager::Get().FindConsoleVariable(TEXT("r.ScreenPercentage")))
      BeforeScreenPercentage = Variable->GetFloat();
    bHasBefore = true;
  }
  Applied = Settings;
  Applied.Normalize();
  ApplyToEngine(Applied, Applied.FrameRateLimit);
  DeadlineSeconds = Now + PreviewDurationSeconds;
  bPending = true;
  return true;
}

bool FPlayerGraphicsPreview::Confirm(FString &Error) {
  Error.Reset();
  if (!bPending) {
    Error = TEXT("no graphics preview is pending");
    return false;
  }
  if (!SaveCustomSettings(Applied, Error)) return false;
  if (UGameUserSettings* GameSettings = GetGameUserSettings()) GameSettings->ConfirmVideoMode();

  bPending = false;
  bHasBefore = false;
  DeadlineSeconds = 0.0;
  return true;
}

void FPlayerGraphicsPreview::Revert() {
  if (!bPending) return;
  if (bHasBefore) {
    ApplyToEngine(Before, BeforeGameSettingsFrameRateLimit);
    SetScreenPercentage(BeforeScreenPercentage);
    if (GEngine != nullptr) GEngine->SetMaxFPS(BeforeEngineMaxFPS);
  }
  bPending = false;
  bHasBefore = false;
  DeadlineSeconds = 0.0;
}

bool FPlayerGraphicsPreview::Tick(const double Now) {
  if (!bPending || !FMath::IsFinite(Now) || Now < DeadlineSeconds) return false;
  Revert();
  return true;
}

bool FPlayerGraphicsPreview::IsPending() const { return bPending; }

double FPlayerGraphicsPreview::SecondsRemaining(const double Now) const {
  if (!bPending || !FMath::IsFinite(Now)) return 0.0;
  return FMath::Max(0.0, DeadlineSeconds - Now);
}

bool FPlayerGraphicsPreview::IsSessionLocked() {
  FString SessionId;
  uint64 ModelGeneration = 0;
  if (LingTuSim::FSessionService::GetBoundSession(SessionId, ModelGeneration)) return true;
  LingTuSim::FControlTransportBinding ControlBinding;
  return LingTuSim::FSessionService::GetControlTransportBinding(ControlBinding);
}

TArray<FIntPoint> GraphicsResolutionOptions(UWorld *World) {
  TArray<FIntPoint> Resolutions;
  if (!HasGameViewport(World)) return Resolutions;

  UKismetSystemLibrary::GetSupportedFullscreenResolutions(Resolutions);
  const FIntPoint Current = FPlayerGraphicsSettings::Capture(World).Resolution;
  if (Current.X > 0 && Current.Y > 0) Resolutions.AddUnique(Current);
  Resolutions.RemoveAll([](const FIntPoint &Resolution) {
    return Resolution.X <= 0 || Resolution.Y <= 0;
  });
  Resolutions.Sort([](const FIntPoint &Left, const FIntPoint &Right) {
    const int64 LeftPixels = static_cast<int64>(Left.X) * Left.Y;
    const int64 RightPixels = static_cast<int64>(Right.X) * Right.Y;
    return LeftPixels == RightPixels ? Left.X < Right.X : LeftPixels < RightPixels;
  });
  return Resolutions;
}

}  // namespace LingTuSim::UI
