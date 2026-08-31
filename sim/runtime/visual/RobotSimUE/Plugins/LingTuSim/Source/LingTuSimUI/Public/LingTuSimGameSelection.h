#pragma once

#include "CoreMinimal.h"
#include "LingTuSimAssetReview.h"

namespace LingTuSim::UI {
enum class EGameSelectionAvailability : uint8 {
  Runnable,
  PreviewOnly,
  Quarantined,
  Unavailable,
};

LINGTUSIMUI_API const TCHAR *GameSelectionAvailabilityName(EGameSelectionAvailability Availability);

struct LINGTUSIMUI_API FGameSelectionPackageSummary final {
  FString Id;
  FString Version;
  FString Label;

  bool IsValid() const { return !Id.IsEmpty() && !Version.IsEmpty() && !Label.IsEmpty(); }
};

struct LINGTUSIMUI_API FGameSelectionAssetSummary final {
  int32 CatalogPackageCount = 0;
  int32 SourceCandidateCount = 0;
  int32 QuarantinedCount = 0;
  int32 UnverifiedCount = 0;
  bool bLibraryAvailable = false;
  FString AvailabilityReason;
};

struct LINGTUSIMUI_API FGameSelectionBundleArtifact final {
  FString Path;
};

struct LINGTUSIMUI_API FGameSelectionOption final {
  FString Id;
  FString Title;
  FString Description;
  int32 Order = 0;
  EGameSelectionAvailability Availability = EGameSelectionAvailability::Unavailable;
  FString AvailabilityReason;
  FString Mode;
  TArray<FString> Tags;
  FGameSelectionPackageSummary Robot;
  FGameSelectionPackageSummary World;
  TOptional<FGameSelectionPackageSummary> Scenario;
  FString BundleDirectory;
  FString SessionId;
  TArray<FGameSelectionBundleArtifact> BundleArtifacts;

  bool IsRunnable() const;
};

struct LINGTUSIMUI_API FGameSelectionCatalog final {
  FString Title;
  FString SourcePath;
  FString CatalogDirectory;
  FGameSelectionAssetSummary AssetSummary;
  FAssetReviewCatalog AssetReview;
  TArray<FGameSelectionOption> Options;
};

/**
 * Loads presentation metadata but validates every runnable option through the
 * canonical SessionBundle loader. The catalog never resolves package source.
 */
class LINGTUSIMUI_API FGameSelectionCatalogLoader final {
 public:
  static bool LoadFromFile(const FString &CatalogPath, FGameSelectionCatalog &OutCatalog,
                           FString &OutError);

 private:
  FGameSelectionCatalogLoader() = delete;
};

/** Pure deterministic browse/confirm state shared by Slate and tests. */
class LINGTUSIMUI_API FGameSelectionModel final {
 public:
  void SetCatalog(FGameSelectionCatalog InCatalog);
  const FGameSelectionCatalog &GetCatalog() const { return Catalog; }
  const FGameSelectionOption *GetSelectedOption() const;
  int32 GetSelectedIndex() const { return SelectedIndex; }
  bool SelectNext();
  bool SelectPrevious();
  bool SelectById(const FString &OptionId);
  bool CanConfirm(FString &OutBlocker) const;
  bool MarkConfirmed(FString &OutBlocker);
  const FString &GetConfirmedOptionId() const { return ConfirmedOptionId; }
  bool IsConfirmed() const { return !ConfirmedOptionId.IsEmpty(); }

 private:
  FGameSelectionCatalog Catalog;
  int32 SelectedIndex = INDEX_NONE;
  FString ConfirmedOptionId;
};

/** Writes one no-replace handoff for an external launcher; it does not start a runtime. */
class LINGTUSIMUI_API FGameSelectionIntentWriter final {
 public:
  static bool WriteNew(const FString &IntentPath, const FGameSelectionOption &Option,
                       FString &OutError);

 private:
  FGameSelectionIntentWriter() = delete;
};
}  // namespace LingTuSim::UI
