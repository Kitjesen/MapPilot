#pragma once

#include "CoreMinimal.h"

class FJsonObject;

namespace LingTuSim::UI {
class FGameSelectionModel;

enum class EAssetReviewDisposition : uint8 {
  Unverified,
  Quarantined,
  ProxyOnly,
  Unavailable,
};

struct LINGTUSIMUI_API FAssetReviewEvidence final {
  FString Role;
  FString Path;
  int64 Bytes = 0;
  FString Sha256;
};

struct LINGTUSIMUI_API FAssetReviewRenderPolicy final {
  FString PhysicsAuthority;
  FString Purpose;
  bool bQualifiedVisual = false;
  bool bRunnable = false;
  FString UnrealCollisionProfile;
  bool bUnrealSimulatePhysics = false;
};

struct LINGTUSIMUI_API FAssetReviewCapabilities final {
  bool bSelectableForReview = false;
  bool bRunnable = false;
  bool bQualifiedVisual = false;
};

struct LINGTUSIMUI_API FAssetReviewCard final {
  FString Id;
  FString Title;
  FString Description;
  int32 Order = 0;
  FString AssetClass;
  FString ReviewStage;
  EAssetReviewDisposition Disposition = EAssetReviewDisposition::Unavailable;
  FString ReviewReason;
  TArray<FAssetReviewEvidence> Evidence;
  FAssetReviewRenderPolicy RenderPolicy;
  FAssetReviewCapabilities Capabilities;
  bool bSelectableForReview = false;
  TArray<FString> Tags;
};

struct LINGTUSIMUI_API FAssetReviewCatalog final {
  bool bAvailable = false;
  FString AvailabilityReason;
  FString Title;
  FString Digest;
  TArray<FAssetReviewCard> Cards;
};

/** Parses the asset_review wrapper embedded in a raw-SHA-pinned selection catalog. */
class LINGTUSIMUI_API FAssetReviewCatalogLoader final {
 public:
  static bool ParseEmbedded(const TSharedPtr<FJsonObject> &SelectionRoot,
                            FAssetReviewCatalog &OutCatalog, FString &OutError);

 private:
  FAssetReviewCatalogLoader() = delete;
};

/** Read-only deterministic card browsing that freezes after the session commits. */
class LINGTUSIMUI_API FAssetReviewModel final {
 public:
  void SetCatalog(FAssetReviewCatalog InCatalog);
  void BindSessionModel(const FGameSelectionModel &InSessionModel);
  const FAssetReviewCatalog &GetCatalog() const { return Catalog; }
  const FAssetReviewCard *GetCurrentCard() const;
  int32 GetCurrentIndex() const { return CurrentIndex; }
  bool SelectNext();
  bool SelectPrevious();
  bool SelectById(const FString &CardId);
  bool IsBrowsingFrozen() const;

 private:
  FAssetReviewCatalog Catalog;
  int32 CurrentIndex = INDEX_NONE;
  const FGameSelectionModel *SessionModel = nullptr;
};
}  // namespace LingTuSim::UI
