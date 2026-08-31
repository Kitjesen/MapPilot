#include "LingTuSimAssetReview.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "LingTuSimGameSelection.h"
#include "Misc/Paths.h"

namespace LingTuSim::UI {
namespace {
constexpr const TCHAR *AssetReviewSchema = TEXT("lingtu.sim.game-asset-review-catalog.v1");

bool HasExactFields(const TSharedPtr<FJsonObject> &Object, const TArray<FString> &Expected,
                    const TCHAR *Context, FString &OutError) {
  if (!Object.IsValid() || Object->Values.Num() != Expected.Num()) {
    OutError = FString::Printf(TEXT("%s fields do not match the supported schema"), Context);
    return false;
  }
  for (const FString &Field : Expected) {
    if (!Object->HasField(Field)) {
      OutError = FString::Printf(TEXT("%s is missing %s"), Context, *Field);
      return false;
    }
  }
  return true;
}

bool ReadString(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, FString &OutValue,
                FString &OutError) {
  if (!Object.IsValid() || !Object->TryGetStringField(Field, OutValue) || OutValue.IsEmpty()) {
    OutError = FString::Printf(TEXT("%s must be a non-empty string"), Field);
    return false;
  }
  return true;
}

bool ReadObject(const TSharedPtr<FJsonObject> &Parent, const TCHAR *Field,
                TSharedPtr<FJsonObject> &OutObject, FString &OutError) {
  const TSharedPtr<FJsonObject> *Value = nullptr;
  if (!Parent.IsValid() || !Parent->TryGetObjectField(Field, Value) || Value == nullptr ||
      !Value->IsValid()) {
    OutError = FString::Printf(TEXT("%s must be an object"), Field);
    return false;
  }
  OutObject = *Value;
  return true;
}

bool ReadBool(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, bool &OutValue,
              FString &OutError) {
  if (!Object.IsValid() || !Object->TryGetBoolField(Field, OutValue)) {
    OutError = FString::Printf(TEXT("%s must be a boolean"), Field);
    return false;
  }
  return true;
}

bool ReadNonNegativeInt(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, int32 &OutValue,
                        FString &OutError) {
  double Number = 0.0;
  if (!Object.IsValid() || !Object->TryGetNumberField(Field, Number) || !FMath::IsFinite(Number) ||
      Number < 0.0 || Number > MAX_int32 || FMath::FloorToDouble(Number) != Number) {
    OutError = FString::Printf(TEXT("%s must be a non-negative integer"), Field);
    return false;
  }
  OutValue = static_cast<int32>(Number);
  return true;
}

bool ReadNonNegativeInt64(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field,
                          int64 &OutValue, FString &OutError) {
  const TSharedPtr<FJsonValue> Value = Object.IsValid() ? Object->TryGetField(Field) : nullptr;
  if (!Value.IsValid() || Value->Type != EJson::Number || !Value->TryGetNumber(OutValue) ||
      OutValue < 0) {
    OutError = FString::Printf(TEXT("%s must be a non-negative integer"), Field);
    return false;
  }
  return true;
}

bool IsLowerHexDigest(const FString &Value) {
  if (Value.Len() != 64) {
    return false;
  }
  for (const TCHAR Character : Value) {
    if (!((Character >= TEXT('0') && Character <= TEXT('9')) ||
          (Character >= TEXT('a') && Character <= TEXT('f')))) {
      return false;
    }
  }
  return true;
}

bool IsSafeEvidencePath(const FString &Path) {
  if (Path.IsEmpty() || !FPaths::IsRelative(Path) || Path.Contains(TEXT("\\"))) {
    return false;
  }
  TArray<FString> Components;
  Path.ParseIntoArray(Components, TEXT("/"), false);
  if (Components.IsEmpty()) {
    return false;
  }
  for (const FString &Component : Components) {
    if (Component.IsEmpty() || Component == TEXT(".") || Component == TEXT("..")) {
      return false;
    }
  }
  return true;
}

bool ParseDisposition(const FString &Stage, const FString &Disposition,
                      EAssetReviewDisposition &OutDisposition) {
  if (Stage == TEXT("catalog_review") && Disposition == TEXT("unverified")) {
    OutDisposition = EAssetReviewDisposition::Unverified;
    return true;
  }
  if ((Stage == TEXT("conditioned_review") || Stage == TEXT("source_review")) &&
      Disposition == TEXT("quarantined")) {
    OutDisposition = EAssetReviewDisposition::Quarantined;
    return true;
  }
  if (Stage == TEXT("proxy_only") && Disposition == TEXT("proxy_only")) {
    OutDisposition = EAssetReviewDisposition::ProxyOnly;
    return true;
  }
  if (Stage == TEXT("unavailable") && Disposition == TEXT("unavailable")) {
    OutDisposition = EAssetReviewDisposition::Unavailable;
    return true;
  }
  return false;
}

bool ParseRenderPolicy(const TSharedPtr<FJsonObject> &Object, FAssetReviewRenderPolicy &OutPolicy,
                       FString &OutError) {
  if (!HasExactFields(Object,
                      {TEXT("physics_authority"), TEXT("purpose"), TEXT("qualified_visual"),
                       TEXT("runnable"), TEXT("unreal_collision_profile"),
                       TEXT("unreal_simulate_physics")},
                      TEXT("render_policy"), OutError)) {
    return false;
  }
  if (!ReadString(Object, TEXT("physics_authority"), OutPolicy.PhysicsAuthority, OutError) ||
      !ReadString(Object, TEXT("purpose"), OutPolicy.Purpose, OutError) ||
      !ReadBool(Object, TEXT("qualified_visual"), OutPolicy.bQualifiedVisual, OutError) ||
      !ReadBool(Object, TEXT("runnable"), OutPolicy.bRunnable, OutError) ||
      !ReadString(Object, TEXT("unreal_collision_profile"), OutPolicy.UnrealCollisionProfile,
                  OutError) ||
      !ReadBool(Object, TEXT("unreal_simulate_physics"), OutPolicy.bUnrealSimulatePhysics,
                OutError) ||
      OutPolicy.PhysicsAuthority != TEXT("mujoco") ||
      OutPolicy.Purpose != TEXT("presentation_review_only") || OutPolicy.bQualifiedVisual ||
      OutPolicy.bRunnable || OutPolicy.UnrealCollisionProfile != TEXT("NoCollision") ||
      OutPolicy.bUnrealSimulatePhysics) {
    OutError = OutError.IsEmpty()
                   ? TEXT("render_policy must preserve presentation-only MuJoCo authority")
                   : OutError;
    return false;
  }
  return true;
}

bool ParseCapabilities(const TSharedPtr<FJsonObject> &Object, const bool bExpectedSelectable,
                       FAssetReviewCapabilities &OutCapabilities, FString &OutError) {
  if (!HasExactFields(Object,
                      {TEXT("selectable_for_review"), TEXT("runnable"), TEXT("qualified_visual")},
                      TEXT("capabilities"), OutError)) {
    return false;
  }
  if (!ReadBool(Object, TEXT("selectable_for_review"), OutCapabilities.bSelectableForReview,
                OutError) ||
      !ReadBool(Object, TEXT("runnable"), OutCapabilities.bRunnable, OutError) ||
      !ReadBool(Object, TEXT("qualified_visual"), OutCapabilities.bQualifiedVisual, OutError) ||
      OutCapabilities.bSelectableForReview != bExpectedSelectable || OutCapabilities.bRunnable ||
      OutCapabilities.bQualifiedVisual) {
    OutError = OutError.IsEmpty() ? TEXT("asset review capabilities violate the read-only contract")
                                  : OutError;
    return false;
  }
  return true;
}

bool ParseEvidence(const TSharedPtr<FJsonObject> &CardObject, const FString &Stage,
                   TArray<FAssetReviewEvidence> &OutEvidence, FString &OutError) {
  const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
  if (!CardObject->TryGetArrayField(TEXT("evidence"), Values) || Values == nullptr) {
    OutError = TEXT("evidence must be an array");
    return false;
  }
  TSet<FString> Roles;
  TSet<FString> Paths;
  FString PreviousSortKey;
  for (const TSharedPtr<FJsonValue> &Value : *Values) {
    const TSharedPtr<FJsonObject> Object =
        Value.IsValid() && Value->Type == EJson::Object ? Value->AsObject() : nullptr;
    FAssetReviewEvidence Evidence;
    if (!HasExactFields(Object, {TEXT("role"), TEXT("path"), TEXT("bytes"), TEXT("sha256")},
                        TEXT("evidence entry"), OutError) ||
        !ReadString(Object, TEXT("role"), Evidence.Role, OutError) ||
        !ReadString(Object, TEXT("path"), Evidence.Path, OutError) ||
        !ReadNonNegativeInt64(Object, TEXT("bytes"), Evidence.Bytes, OutError) ||
        !ReadString(Object, TEXT("sha256"), Evidence.Sha256, OutError) ||
        !IsLowerHexDigest(Evidence.Sha256) || !IsSafeEvidencePath(Evidence.Path) ||
        Roles.Contains(Evidence.Role) || Paths.Contains(Evidence.Path)) {
      OutError = OutError.IsEmpty() ? TEXT("asset review evidence is invalid") : OutError;
      return false;
    }
    const FString SortKey = Evidence.Role + TEXT("\n") + Evidence.Path;
    if (!PreviousSortKey.IsEmpty() && SortKey.Compare(PreviousSortKey) <= 0) {
      OutError = TEXT("asset review evidence must be sorted by role then path");
      return false;
    }
    PreviousSortKey = SortKey;
    Roles.Add(Evidence.Role);
    Paths.Add(Evidence.Path);
    OutEvidence.Add(MoveTemp(Evidence));
  }

  TSet<FString> ExpectedRoles;
  if (Stage == TEXT("catalog_review")) {
    ExpectedRoles = {TEXT("package_manifest"), TEXT("visual_projection")};
  } else if (Stage == TEXT("conditioned_review")) {
    ExpectedRoles = {TEXT("conditioned_mesh"), TEXT("conditioning_report")};
  } else if (Stage == TEXT("source_review")) {
    ExpectedRoles = {TEXT("source_model"), TEXT("source_task")};
  } else if (Stage == TEXT("proxy_only")) {
    ExpectedRoles = {TEXT("physics_proxy")};
  }
  if (Roles.Num() != ExpectedRoles.Num()) {
    OutError = TEXT("asset review evidence roles do not match the review stage");
    return false;
  }
  for (const FString &Role : ExpectedRoles) {
    if (!Roles.Contains(Role)) {
      OutError = TEXT("asset review evidence roles do not match the review stage");
      return false;
    }
  }
  return true;
}

bool ParseTags(const TSharedPtr<FJsonObject> &Object, TArray<FString> &OutTags, FString &OutError) {
  const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
  if (!Object->TryGetArrayField(TEXT("tags"), Values) || Values == nullptr) {
    OutError = TEXT("tags must be an array");
    return false;
  }
  FString Previous;
  for (const TSharedPtr<FJsonValue> &Value : *Values) {
    FString Tag;
    if (!Value.IsValid() || !Value->TryGetString(Tag) || Tag.IsEmpty() ||
        (!Previous.IsEmpty() && Tag.Compare(Previous) <= 0)) {
      OutError = TEXT("tags must be unique non-empty strings sorted ascending");
      return false;
    }
    Previous = Tag;
    OutTags.Add(MoveTemp(Tag));
  }
  return true;
}

bool ParseCard(const TSharedPtr<FJsonObject> &Object, FAssetReviewCard &OutCard,
               FString &OutError) {
  if (!HasExactFields(Object,
                      {TEXT("id"), TEXT("title"), TEXT("description"), TEXT("order"),
                       TEXT("asset_class"), TEXT("review"), TEXT("evidence"), TEXT("render_policy"),
                       TEXT("capabilities"), TEXT("tags")},
                      TEXT("asset review card"), OutError) ||
      !ReadString(Object, TEXT("id"), OutCard.Id, OutError) ||
      !ReadString(Object, TEXT("title"), OutCard.Title, OutError) ||
      !ReadString(Object, TEXT("description"), OutCard.Description, OutError) ||
      !ReadNonNegativeInt(Object, TEXT("order"), OutCard.Order, OutError) ||
      !ReadString(Object, TEXT("asset_class"), OutCard.AssetClass, OutError)) {
    return false;
  }
  static const TSet<FString> AssetClasses = {TEXT("robot"),          TEXT("payload"),
                                             TEXT("vegetation"),     TEXT("terrain_detail"),
                                             TEXT("scenario_proxy"), TEXT("movable_object")};
  if (!AssetClasses.Contains(OutCard.AssetClass)) {
    OutError = TEXT("asset_class is unsupported");
    return false;
  }

  TSharedPtr<FJsonObject> Review;
  FString Disposition;
  if (!ReadObject(Object, TEXT("review"), Review, OutError) ||
      !HasExactFields(Review, {TEXT("stage"), TEXT("disposition"), TEXT("reason")}, TEXT("review"),
                      OutError) ||
      !ReadString(Review, TEXT("stage"), OutCard.ReviewStage, OutError) ||
      !ReadString(Review, TEXT("disposition"), Disposition, OutError) ||
      !ReadString(Review, TEXT("reason"), OutCard.ReviewReason, OutError) ||
      !ParseDisposition(OutCard.ReviewStage, Disposition, OutCard.Disposition)) {
    OutError = OutError.IsEmpty() ? TEXT("review stage/disposition is unsupported") : OutError;
    return false;
  }
  OutCard.bSelectableForReview = OutCard.Disposition != EAssetReviewDisposition::Unavailable;

  TSharedPtr<FJsonObject> RenderPolicy;
  TSharedPtr<FJsonObject> Capabilities;
  return ParseEvidence(Object, OutCard.ReviewStage, OutCard.Evidence, OutError) &&
         ReadObject(Object, TEXT("render_policy"), RenderPolicy, OutError) &&
         ParseRenderPolicy(RenderPolicy, OutCard.RenderPolicy, OutError) &&
         ReadObject(Object, TEXT("capabilities"), Capabilities, OutError) &&
         ParseCapabilities(Capabilities, OutCard.bSelectableForReview, OutCard.Capabilities,
                           OutError) &&
         ParseTags(Object, OutCard.Tags, OutError);
}

bool ParseCatalog(const TSharedPtr<FJsonObject> &Object, FAssetReviewCatalog &OutCatalog,
                  FString &OutError) {
  if (!HasExactFields(Object,
                      {TEXT("schema"), TEXT("title"), TEXT("policy"), TEXT("coverage"),
                       TEXT("cards"), TEXT("digest")},
                      TEXT("asset review catalog"), OutError)) {
    return false;
  }
  FString Schema;
  if (!ReadString(Object, TEXT("schema"), Schema, OutError) || Schema != AssetReviewSchema ||
      !ReadString(Object, TEXT("title"), OutCatalog.Title, OutError) ||
      !ReadString(Object, TEXT("digest"), OutCatalog.Digest, OutError) ||
      !IsLowerHexDigest(OutCatalog.Digest)) {
    OutError = OutError.IsEmpty() ? TEXT("asset review schema or digest is invalid") : OutError;
    return false;
  }

  TSharedPtr<FJsonObject> Policy;
  FAssetReviewRenderPolicy CatalogPolicy;
  TSharedPtr<FJsonObject> Coverage;
  TSharedPtr<FJsonObject> MovableCoverage;
  bool bMovableAvailable = true;
  FString MovableReason;
  if (!ReadObject(Object, TEXT("policy"), Policy, OutError) ||
      !ParseRenderPolicy(Policy, CatalogPolicy, OutError) ||
      !ReadObject(Object, TEXT("coverage"), Coverage, OutError) ||
      !HasExactFields(Coverage, {TEXT("qualified_movable_object_visual")}, TEXT("coverage"),
                      OutError) ||
      !ReadObject(Coverage, TEXT("qualified_movable_object_visual"), MovableCoverage, OutError) ||
      !HasExactFields(MovableCoverage, {TEXT("available"), TEXT("reason")},
                      TEXT("qualified_movable_object_visual"), OutError) ||
      !ReadBool(MovableCoverage, TEXT("available"), bMovableAvailable, OutError) ||
      !ReadString(MovableCoverage, TEXT("reason"), MovableReason, OutError) || bMovableAvailable) {
    OutError = OutError.IsEmpty()
                   ? TEXT("asset review movable-object coverage must remain unavailable")
                   : OutError;
    return false;
  }

  const TArray<TSharedPtr<FJsonValue>> *Cards = nullptr;
  if (!Object->TryGetArrayField(TEXT("cards"), Cards) || Cards == nullptr || Cards->IsEmpty()) {
    OutError = TEXT("available asset review catalog cards must be non-empty");
    return false;
  }
  TSet<FString> CardIds;
  int32 PreviousOrder = INDEX_NONE;
  FString PreviousId;
  for (const TSharedPtr<FJsonValue> &Value : *Cards) {
    const TSharedPtr<FJsonObject> CardObject =
        Value.IsValid() && Value->Type == EJson::Object ? Value->AsObject() : nullptr;
    FAssetReviewCard Card;
    if (!ParseCard(CardObject, Card, OutError) || CardIds.Contains(Card.Id) ||
        (PreviousOrder != INDEX_NONE &&
         (Card.Order < PreviousOrder ||
          (Card.Order == PreviousOrder && Card.Id.Compare(PreviousId) <= 0)))) {
      OutError = OutError.IsEmpty()
                     ? TEXT("asset review cards must have unique ids sorted by order then id")
                     : OutError;
      return false;
    }
    CardIds.Add(Card.Id);
    PreviousOrder = Card.Order;
    PreviousId = Card.Id;
    OutCatalog.Cards.Add(MoveTemp(Card));
  }
  OutCatalog.bAvailable = true;
  return true;
}
}  // namespace

bool FAssetReviewCatalogLoader::ParseEmbedded(const TSharedPtr<FJsonObject> &SelectionRoot,
                                              FAssetReviewCatalog &OutCatalog, FString &OutError) {
  OutCatalog = FAssetReviewCatalog{};
  TSharedPtr<FJsonObject> Wrapper;
  TSharedPtr<FJsonObject> Availability;
  FString State;
  if (!ReadObject(SelectionRoot, TEXT("asset_review"), Wrapper, OutError) ||
      !HasExactFields(Wrapper, {TEXT("availability"), TEXT("catalog")}, TEXT("asset_review"),
                      OutError) ||
      !ReadObject(Wrapper, TEXT("availability"), Availability, OutError) ||
      !HasExactFields(Availability, {TEXT("state"), TEXT("reason")},
                      TEXT("asset_review.availability"), OutError) ||
      !ReadString(Availability, TEXT("state"), State, OutError) ||
      !ReadString(Availability, TEXT("reason"), OutCatalog.AvailabilityReason, OutError)) {
    return false;
  }

  if (State == TEXT("unavailable")) {
    if (!Wrapper->HasTypedField<EJson::Null>(TEXT("catalog"))) {
      OutError = TEXT("unavailable asset_review requires catalog=null");
      return false;
    }
    return true;
  }
  if (State != TEXT("available")) {
    OutError = TEXT("asset_review availability state is unsupported");
    return false;
  }

  TSharedPtr<FJsonObject> Catalog;
  if (!ReadObject(Wrapper, TEXT("catalog"), Catalog, OutError) ||
      !ParseCatalog(Catalog, OutCatalog, OutError)) {
    return false;
  }
  return true;
}

void FAssetReviewModel::SetCatalog(FAssetReviewCatalog InCatalog) {
  Catalog = MoveTemp(InCatalog);
  CurrentIndex = Catalog.bAvailable && !Catalog.Cards.IsEmpty() ? 0 : INDEX_NONE;
}

void FAssetReviewModel::BindSessionModel(const FGameSelectionModel &InSessionModel) {
  SessionModel = &InSessionModel;
}

const FAssetReviewCard *FAssetReviewModel::GetCurrentCard() const {
  return Catalog.Cards.IsValidIndex(CurrentIndex) ? &Catalog.Cards[CurrentIndex] : nullptr;
}

bool FAssetReviewModel::IsBrowsingFrozen() const {
  return SessionModel != nullptr && SessionModel->IsConfirmed();
}

bool FAssetReviewModel::SelectNext() {
  if (Catalog.Cards.IsEmpty() || IsBrowsingFrozen()) {
    return false;
  }
  CurrentIndex = (CurrentIndex + 1 + Catalog.Cards.Num()) % Catalog.Cards.Num();
  return true;
}

bool FAssetReviewModel::SelectPrevious() {
  if (Catalog.Cards.IsEmpty() || IsBrowsingFrozen()) {
    return false;
  }
  CurrentIndex = (CurrentIndex - 1 + Catalog.Cards.Num()) % Catalog.Cards.Num();
  return true;
}

bool FAssetReviewModel::SelectById(const FString &CardId) {
  if (IsBrowsingFrozen()) {
    return false;
  }
  const int32 Index = Catalog.Cards.IndexOfByPredicate(
      [&CardId](const FAssetReviewCard &Card) { return Card.Id == CardId; });
  if (Index == INDEX_NONE) {
    return false;
  }
  CurrentIndex = Index;
  return true;
}
}  // namespace LingTuSim::UI
