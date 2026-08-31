#include "LingTuSimGameSelection.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "HAL/FileManager.h"
#include "LingTuSimBundleLoader.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

#if PLATFORM_WINDOWS
// clang-format off
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
// clang-format on
#endif

namespace LingTuSim::UI {
namespace {
constexpr const TCHAR *CatalogSchema = TEXT("lingtu.sim.game-selection-catalog.v1");
constexpr const TCHAR *IntentSchema = TEXT("lingtu.sim.game-selection-intent.v1");

FString CanonicalAbsolutePath(const FString &Path) {
  FString Result = FPaths::ConvertRelativePathToFull(Path);
  FPaths::NormalizeFilename(Result);
  FPaths::CollapseRelativeDirectories(Result);
  return Result;
}

bool LoadFileBytes(const FString &Path, TArray<uint8> &OutBytes, FString &OutError) {
  OutBytes.Reset();
  if (!FFileHelper::LoadFileToArray(OutBytes, *Path)) {
    OutError = TEXT("file bytes could not be read");
    return false;
  }
  return true;
}

bool DecodeUtf8Json(const TArray<uint8> &Bytes, FString &OutJson, FString &OutError) {
  if (Bytes.IsEmpty()) {
    OutError = TEXT("game selection catalog is empty");
    return false;
  }
  const FUTF8ToTCHAR Converted(reinterpret_cast<const ANSICHAR *>(Bytes.GetData()), Bytes.Num());
  OutJson = FString(Converted.Length(), Converted.Get());
  return true;
}

#if PLATFORM_WINDOWS
FString RemoveWindowsDevicePrefix(FString Path) {
  if (Path.StartsWith(TEXT("\\\\?\\UNC\\"), ESearchCase::IgnoreCase)) {
    Path = TEXT("\\\\") + Path.Mid(8);
  } else if (Path.StartsWith(TEXT("\\\\?\\"), ESearchCase::IgnoreCase)) {
    Path.RightChopInline(4, EAllowShrinking::No);
  }
  return Path;
}

bool ReadFinalPath(HANDLE Handle, FString &OutPath) {
  TArray<WCHAR> Buffer;
  Buffer.SetNumUninitialized(32'768);
  const DWORD Length =
      ::GetFinalPathNameByHandleW(Handle, Buffer.GetData(), static_cast<DWORD>(Buffer.Num()),
                                  FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
  if (Length == 0 || Length >= static_cast<DWORD>(Buffer.Num())) {
    return false;
  }
  Buffer[static_cast<int32>(Length)] = 0;
  OutPath = CanonicalAbsolutePath(RemoveWindowsDevicePrefix(FString(Buffer.GetData())));
  return true;
}

bool InspectWindowsPlainPath(const FString &Path, const bool bExpectedDirectory,
                             FString &OutError) {
  const DWORD Flags =
      FILE_FLAG_OPEN_REPARSE_POINT | (bExpectedDirectory ? FILE_FLAG_BACKUP_SEMANTICS : 0);
  HANDLE Handle = ::CreateFileW(*Path, FILE_READ_ATTRIBUTES, FILE_SHARE_READ, nullptr,
                                OPEN_EXISTING, Flags, nullptr);
  if (Handle == INVALID_HANDLE_VALUE) {
    OutError = TEXT("bundle path could not be opened without following links");
    return false;
  }
  BY_HANDLE_FILE_INFORMATION Information{};
  FString FinalPath;
  const bool bValid =
      ::GetFileInformationByHandle(Handle, &Information) != 0 &&
      (Information.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) == 0 &&
      (((Information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) == bExpectedDirectory) &&
      ReadFinalPath(Handle, FinalPath) &&
      FPaths::IsSamePath(FinalPath, CanonicalAbsolutePath(Path));
  ::CloseHandle(Handle);
  if (!bValid) {
    OutError = TEXT("bundle path is a link, reparse point, wrong type, or changed target");
  }
  return bValid;
}

bool ReadWindowsPlainFile(const FString &Path, TArray<uint8> &OutBytes, FString &OutError) {
  HANDLE Handle = ::CreateFileW(*Path, GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
  if (Handle == INVALID_HANDLE_VALUE) {
    OutError = TEXT("bundle artifact could not be opened without following links");
    return false;
  }
  BY_HANDLE_FILE_INFORMATION Information{};
  LARGE_INTEGER Size{};
  FString FinalPath;
  const bool bMetadataValid = ::GetFileInformationByHandle(Handle, &Information) != 0 &&
                              (Information.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) == 0 &&
                              (Information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0 &&
                              ::GetFileSizeEx(Handle, &Size) != 0 && Size.QuadPart >= 0 &&
                              Size.QuadPart <= MAX_int32 && ReadFinalPath(Handle, FinalPath) &&
                              FPaths::IsSamePath(FinalPath, CanonicalAbsolutePath(Path));
  if (!bMetadataValid) {
    ::CloseHandle(Handle);
    OutError = TEXT("bundle artifact is a link, wrong type, too large, or changed target");
    return false;
  }
  OutBytes.SetNumUninitialized(static_cast<int32>(Size.QuadPart));
  int32 Offset = 0;
  while (Offset < OutBytes.Num()) {
    const DWORD Requested =
        static_cast<DWORD>(FMath::Min<int64>(OutBytes.Num() - Offset, MAX_uint32));
    DWORD Read = 0;
    if (::ReadFile(Handle, OutBytes.GetData() + Offset, Requested, &Read, nullptr) == 0 ||
        Read != Requested) {
      ::CloseHandle(Handle);
      OutBytes.Reset();
      OutError = TEXT("bundle artifact bytes could not be read completely");
      return false;
    }
    Offset += static_cast<int32>(Read);
  }
  ::CloseHandle(Handle);
  return true;
}
#endif

bool ValidatePlainPath(const FString &Path, const bool bExpectedDirectory, FString &OutError) {
  if (IFileManager::Get().IsSymlink(*Path)) {
    OutError = TEXT("bundle path contains a symbolic link");
    return false;
  }
#if PLATFORM_WINDOWS
  return InspectWindowsPlainPath(Path, bExpectedDirectory, OutError);
#else
  const bool bExists = bExpectedDirectory ? IFileManager::Get().DirectoryExists(*Path)
                                          : IFileManager::Get().FileExists(*Path);
  if (!bExists) {
    OutError = TEXT("bundle path component is missing or has the wrong type");
  }
  return bExists;
#endif
}

bool IsValidSessionId(const FString &Value) {
  if (Value.IsEmpty() || Value.Len() > 63) {
    return false;
  }
  for (const TCHAR Character : Value) {
    if (FChar::IsWhitespace(Character) || Character == TEXT('\0')) {
      return false;
    }
  }
  return true;
}

bool ReadRequiredString(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field,
                        FString &OutValue, FString &OutError) {
  if (!Object.IsValid() || !Object->TryGetStringField(Field, OutValue) || OutValue.IsEmpty()) {
    OutError = FString::Printf(TEXT("%s must be a non-empty string"), Field);
    return false;
  }
  return true;
}

bool ReadNonNegativeInt(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field, int32 &OutValue,
                        FString &OutError) {
  double Number = 0.0;
  if (!Object.IsValid() || !Object->TryGetNumberField(Field, Number) || !FMath::IsFinite(Number) ||
      Number < 0.0 || Number > static_cast<double>(MAX_int32) ||
      FMath::FloorToDouble(Number) != Number) {
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

bool ReadPackage(const TSharedPtr<FJsonObject> &Parent, const TCHAR *Field,
                 FGameSelectionPackageSummary &OutPackage, FString &OutError) {
  TSharedPtr<FJsonObject> Object;
  return ReadObject(Parent, Field, Object, OutError) &&
         ReadRequiredString(Object, TEXT("id"), OutPackage.Id, OutError) &&
         ReadRequiredString(Object, TEXT("version"), OutPackage.Version, OutError) &&
         ReadRequiredString(Object, TEXT("label"), OutPackage.Label, OutError);
}

bool ReadAuthoritativePackage(const TSharedPtr<FJsonObject> &Parent, const TCHAR *Field,
                              FString &OutId, FString &OutVersion, FString &OutError) {
  TSharedPtr<FJsonObject> Object;
  return ReadObject(Parent, Field, Object, OutError) &&
         ReadRequiredString(Object, TEXT("id"), OutId, OutError) &&
         ReadRequiredString(Object, TEXT("version"), OutVersion, OutError);
}

bool PackageMatches(const FGameSelectionPackageSummary &Presentation, const FString &Id,
                    const FString &Version) {
  return Presentation.Id == Id && Presentation.Version == Version;
}

bool ParseAvailability(const FString &Value, EGameSelectionAvailability &OutAvailability) {
  if (Value == TEXT("runnable")) {
    OutAvailability = EGameSelectionAvailability::Runnable;
    return true;
  }
  if (Value == TEXT("preview_only")) {
    OutAvailability = EGameSelectionAvailability::PreviewOnly;
    return true;
  }
  if (Value == TEXT("quarantined")) {
    OutAvailability = EGameSelectionAvailability::Quarantined;
    return true;
  }
  if (Value == TEXT("unavailable")) {
    OutAvailability = EGameSelectionAvailability::Unavailable;
    return true;
  }
  return false;
}

bool IsContainedRelativePath(const FString &Root, const FString &Relative, FString &OutAbsolute,
                             FString &OutError) {
  if (Relative.IsEmpty() || !FPaths::IsRelative(Relative)) {
    return false;
  }
  FString NormalizedRelative = Relative;
  FPaths::NormalizeFilename(NormalizedRelative);
  FPaths::CollapseRelativeDirectories(NormalizedRelative);
  if (NormalizedRelative == TEXT("..") || NormalizedRelative.StartsWith(TEXT("../"))) {
    return false;
  }
  FString NormalizedRoot = FPaths::ConvertRelativePathToFull(Root);
  FPaths::NormalizeDirectoryName(NormalizedRoot);
  OutAbsolute = FPaths::ConvertRelativePathToFull(NormalizedRoot, NormalizedRelative);
  FPaths::NormalizeDirectoryName(OutAbsolute);
  if (!(OutAbsolute == NormalizedRoot || OutAbsolute.StartsWith(NormalizedRoot + TEXT("/")))) {
    return false;
  }
  if (!ValidatePlainPath(NormalizedRoot, true, OutError)) {
    return false;
  }
  TArray<FString> Components;
  NormalizedRelative.ParseIntoArray(Components, TEXT("/"), true);
  FString Current = NormalizedRoot;
  for (const FString &Component : Components) {
    if (Component.IsEmpty() || Component == TEXT(".") || Component == TEXT("..")) {
      return false;
    }
    Current = FPaths::Combine(Current, Component);
    if (!ValidatePlainPath(Current, true, OutError)) {
      return false;
    }
  }
  return FPaths::IsSamePath(Current, OutAbsolute);
}

bool ParseBundleArtifacts(const TSharedPtr<FJsonObject> &Bundle,
                          TArray<FGameSelectionBundleArtifact> &OutArtifacts, FString &OutError) {
  const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
  if (!Bundle->TryGetArrayField(TEXT("artifacts"), Values) || Values == nullptr ||
      Values->IsEmpty()) {
    OutError = TEXT("bundle.artifacts must be a non-empty array");
    return false;
  }
  TSet<FString> Paths;
  for (const TSharedPtr<FJsonValue> &Value : *Values) {
    const TSharedPtr<FJsonObject> Artifact = Value.IsValid() ? Value->AsObject() : nullptr;
    FGameSelectionBundleArtifact Parsed;
    if (!Artifact.IsValid() || !ReadRequiredString(Artifact, TEXT("path"), Parsed.Path, OutError) ||
        Parsed.Path != FPaths::GetCleanFilename(Parsed.Path) ||
        Parsed.Path.Contains(TEXT("/")) || Parsed.Path.Contains(TEXT("\\")) ||
        Paths.Contains(Parsed.Path)) {
      OutError = OutError.IsEmpty() ? TEXT("bundle artifact descriptor is invalid or duplicated")
                                    : OutError;
      return false;
    }
    Paths.Add(Parsed.Path);
    OutArtifacts.Add(MoveTemp(Parsed));
  }
  OutArtifacts.Sort(
      [](const FGameSelectionBundleArtifact &Left, const FGameSelectionBundleArtifact &Right) {
        return Left.Path < Right.Path;
      });
  return true;
}

bool ValidateBundleArtifacts(const FString &BundleDirectory,
                             const TArray<FGameSelectionBundleArtifact> &Artifacts,
                             FString &OutError) {
  if (!ValidatePlainPath(BundleDirectory, true, OutError)) {
    return false;
  }
  for (const FGameSelectionBundleArtifact &Artifact : Artifacts) {
    const FString Path = FPaths::Combine(BundleDirectory, Artifact.Path);
    if (!ValidatePlainPath(Path, false, OutError)) {
      return false;
    }
  }
  return true;
}

bool LoadJsonObjectFile(const FString &Path, TSharedPtr<FJsonObject> &OutObject,
                        FString &OutError) {
  FString Json;
  if (!FFileHelper::LoadFileToString(Json, *Path)) {
    OutError = FString::Printf(TEXT("compiled artifact could not be read: %s"), *Path);
    return false;
  }
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  if (!FJsonSerializer::Deserialize(Reader, OutObject) || !OutObject.IsValid()) {
    OutError = FString::Printf(TEXT("compiled artifact is not valid JSON: %s"), *Path);
    return false;
  }
  return true;
}

bool ValidatePresentationAgainstBundle(const FGameSelectionOption &Option,
                                       const LingTuSim::FSessionBundleView &BundleView,
                                       FString &OutError) {
  TSharedPtr<FJsonObject> Visual;
  if (!LoadJsonObjectFile(BundleView.VisualPlanPath, Visual, OutError)) {
    return false;
  }
  TSharedPtr<FJsonObject> VisualWorld;
  FString VisualWorldId;
  FString VisualWorldVersion;
  const TArray<TSharedPtr<FJsonValue>> *VisualRobots = nullptr;
  if (!ReadObject(Visual, TEXT("world"), VisualWorld, OutError) ||
      !ReadAuthoritativePackage(VisualWorld, TEXT("package"), VisualWorldId, VisualWorldVersion,
                                OutError) ||
      !PackageMatches(Option.World, VisualWorldId, VisualWorldVersion) ||
      !Visual->TryGetArrayField(TEXT("robots"), VisualRobots) || VisualRobots == nullptr ||
      VisualRobots->Num() != 1) {
    OutError = OutError.IsEmpty()
                   ? TEXT("catalog world/robot presentation does not match visual.plan.json")
                   : OutError;
    return false;
  }
  const TSharedPtr<FJsonObject> VisualRobot =
      (*VisualRobots)[0].IsValid() ? (*VisualRobots)[0]->AsObject() : nullptr;
  FString VisualRobotId;
  FString VisualRobotVersion;
  if (!VisualRobot.IsValid() ||
      !ReadAuthoritativePackage(VisualRobot, TEXT("package"), VisualRobotId, VisualRobotVersion,
                                OutError) ||
      !PackageMatches(Option.Robot, VisualRobotId, VisualRobotVersion)) {
    OutError = OutError.IsEmpty()
                   ? TEXT("catalog robot presentation does not match visual.plan.json")
                   : OutError;
    return false;
  }

  if (Option.Scenario.IsSet() != !BundleView.ScenarioPlanPath.IsEmpty()) {
    OutError = TEXT("catalog scenario presence does not match the compiled bundle");
    return false;
  }
  if (Option.Scenario.IsSet()) {
    TSharedPtr<FJsonObject> ScenarioPlan;
    FString PlanScenarioId;
    FString PlanScenarioVersion;
    if (!LoadJsonObjectFile(BundleView.ScenarioPlanPath, ScenarioPlan, OutError) ||
        !ReadAuthoritativePackage(ScenarioPlan, TEXT("package"), PlanScenarioId,
                                  PlanScenarioVersion, OutError) ||
        !PackageMatches(Option.Scenario.GetValue(), PlanScenarioId, PlanScenarioVersion)) {
      OutError = OutError.IsEmpty()
                     ? TEXT("catalog scenario presentation does not match compiled plans")
                     : OutError;
      return false;
    }
  }
  return true;
}

bool ParseOption(const TSharedPtr<FJsonObject> &Object, const FString &CatalogDirectory,
                 FGameSelectionOption &OutOption, FString &OutError) {
  if (!ReadRequiredString(Object, TEXT("id"), OutOption.Id, OutError) ||
      !ReadRequiredString(Object, TEXT("title"), OutOption.Title, OutError) ||
      !Object->TryGetStringField(TEXT("description"), OutOption.Description) ||
      !ReadNonNegativeInt(Object, TEXT("order"), OutOption.Order, OutError) ||
      !ReadRequiredString(Object, TEXT("mode"), OutOption.Mode, OutError) ||
      !ReadPackage(Object, TEXT("robot"), OutOption.Robot, OutError) ||
      !ReadPackage(Object, TEXT("world"), OutOption.World, OutError)) {
    return false;
  }

  TSharedPtr<FJsonObject> Availability;
  FString AvailabilityState;
  if (!ReadObject(Object, TEXT("availability"), Availability, OutError) ||
      !ReadRequiredString(Availability, TEXT("state"), AvailabilityState, OutError) ||
      !Availability->TryGetStringField(TEXT("reason"), OutOption.AvailabilityReason) ||
      !ParseAvailability(AvailabilityState, OutOption.Availability)) {
    OutError = OutError.IsEmpty() ? TEXT("availability.state is unsupported") : OutError;
    return false;
  }

  if (Object->HasTypedField<EJson::Object>(TEXT("scenario"))) {
    FGameSelectionPackageSummary Scenario;
    if (!ReadPackage(Object, TEXT("scenario"), Scenario, OutError)) {
      return false;
    }
    OutOption.Scenario = MoveTemp(Scenario);
  }

  const TArray<TSharedPtr<FJsonValue>> *Tags = nullptr;
  if (!Object->TryGetArrayField(TEXT("tags"), Tags) || Tags == nullptr) {
    OutError = TEXT("tags must be an array");
    return false;
  }
  for (const TSharedPtr<FJsonValue> &Tag : *Tags) {
    FString Value;
    if (!Tag.IsValid() || !Tag->TryGetString(Value) || Value.IsEmpty()) {
      OutError = TEXT("tags entries must be non-empty strings");
      return false;
    }
    OutOption.Tags.Add(MoveTemp(Value));
  }

  const bool bHasBundle = Object->HasTypedField<EJson::Object>(TEXT("bundle"));
  if (OutOption.Availability == EGameSelectionAvailability::Runnable && !bHasBundle) {
    OutError = TEXT("runnable option must bind one compiled SessionBundle");
    return false;
  }
  if (!bHasBundle) {
    return true;
  }

  TSharedPtr<FJsonObject> Bundle;
  FString RelativeDirectory;
  if (!ReadObject(Object, TEXT("bundle"), Bundle, OutError) ||
      !ReadRequiredString(Bundle, TEXT("directory"), RelativeDirectory, OutError) ||
      !ReadRequiredString(Bundle, TEXT("session_id"), OutOption.SessionId, OutError) ||
      !IsValidSessionId(OutOption.SessionId) ||
      !IsContainedRelativePath(CatalogDirectory, RelativeDirectory, OutOption.BundleDirectory,
                               OutError) ||
      !ParseBundleArtifacts(Bundle, OutOption.BundleArtifacts, OutError) ||
      !ValidateBundleArtifacts(OutOption.BundleDirectory, OutOption.BundleArtifacts, OutError)) {
    OutError =
        OutError.IsEmpty() ? TEXT("bundle directory or session id is invalid") : OutError;
    return false;
  }

  LingTuSim::FSessionBundleView BundleView;
  LingTuSim::FRuntimeLoadError BundleError;
  if (!LingTuSim::FSessionBundleLoader::LoadSessionBundle(OutOption.BundleDirectory, BundleView,
                                                          BundleError)) {
    OutError = FString::Printf(TEXT("compiled SessionBundle rejected: %s"), *BundleError.Message);
    return false;
  }
  if (BundleView.SessionId != OutOption.SessionId) {
    OutError = TEXT("catalog session id does not match compiled SessionBundle");
    return false;
  }
  return ValidatePresentationAgainstBundle(OutOption, BundleView, OutError) &&
         ValidateBundleArtifacts(OutOption.BundleDirectory, OutOption.BundleArtifacts, OutError);
}
}  // namespace

const TCHAR *GameSelectionAvailabilityName(const EGameSelectionAvailability Availability) {
  switch (Availability) {
    case EGameSelectionAvailability::Runnable:
      return TEXT("RUNNABLE");
    case EGameSelectionAvailability::PreviewOnly:
      return TEXT("PREVIEW ONLY");
    case EGameSelectionAvailability::Quarantined:
      return TEXT("QUARANTINED");
    case EGameSelectionAvailability::Unavailable:
    default:
      return TEXT("UNAVAILABLE");
  }
}

bool FGameSelectionOption::IsRunnable() const {
  return Availability == EGameSelectionAvailability::Runnable && !Id.IsEmpty() &&
         !Title.IsEmpty() && Robot.IsValid() && World.IsValid() && !BundleDirectory.IsEmpty() &&
          IsValidSessionId(SessionId) && !BundleArtifacts.IsEmpty();
}

bool FGameSelectionCatalogLoader::LoadFromFile(const FString &CatalogPath,
                                                FGameSelectionCatalog &OutCatalog,
                                                FString &OutError) {
  OutError.Reset();
  const FString CanonicalCatalogPath = CanonicalAbsolutePath(CatalogPath);
  TArray<uint8> CatalogBytes;
  if (!LoadFileBytes(CanonicalCatalogPath, CatalogBytes, OutError)) {
    return false;
  }
  FString Json;
  if (!DecodeUtf8Json(CatalogBytes, Json, OutError)) {
    return false;
  }
  TSharedPtr<FJsonObject> Root;
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) {
    OutError = TEXT("game selection catalog is not valid JSON");
    return false;
  }

  FString Schema;
  FGameSelectionCatalog Candidate;
  if (!ReadRequiredString(Root, TEXT("schema"), Schema, OutError) || Schema != CatalogSchema ||
      !ReadRequiredString(Root, TEXT("title"), Candidate.Title, OutError)) {
    OutError = OutError.IsEmpty() ? TEXT("game selection catalog schema is invalid") : OutError;
    return false;
  }

  TSharedPtr<FJsonObject> AssetSummary;
  TSharedPtr<FJsonObject> AssetAvailability;
  FString AssetAvailabilityState;
  if (!ReadObject(Root, TEXT("asset_summary"), AssetSummary, OutError) ||
      !ReadNonNegativeInt(AssetSummary, TEXT("catalog_package_count"),
                          Candidate.AssetSummary.CatalogPackageCount, OutError) ||
      !ReadNonNegativeInt(AssetSummary, TEXT("source_candidate_count"),
                          Candidate.AssetSummary.SourceCandidateCount, OutError) ||
      !ReadNonNegativeInt(AssetSummary, TEXT("quarantined_count"),
                          Candidate.AssetSummary.QuarantinedCount, OutError) ||
      !ReadNonNegativeInt(AssetSummary, TEXT("unverified_count"),
                          Candidate.AssetSummary.UnverifiedCount, OutError) ||
      !ReadObject(AssetSummary, TEXT("availability"), AssetAvailability, OutError) ||
      !ReadRequiredString(AssetAvailability, TEXT("state"), AssetAvailabilityState, OutError) ||
      !ReadRequiredString(AssetAvailability, TEXT("reason"),
                          Candidate.AssetSummary.AvailabilityReason, OutError) ||
      (AssetAvailabilityState != TEXT("available") &&
       AssetAvailabilityState != TEXT("unavailable"))) {
    if (OutError.IsEmpty()) {
      OutError = TEXT("asset_summary availability state is unsupported");
    }
    return false;
  }
  Candidate.AssetSummary.bLibraryAvailable = AssetAvailabilityState == TEXT("available");

  if (!FAssetReviewCatalogLoader::ParseEmbedded(Root, Candidate.AssetReview, OutError)) {
    return false;
  }

  Candidate.SourcePath = CanonicalCatalogPath;
  Candidate.CatalogDirectory = FPaths::GetPath(CanonicalCatalogPath);
  FPaths::NormalizeDirectoryName(Candidate.CatalogDirectory);
  const TArray<TSharedPtr<FJsonValue>> *Entries = nullptr;
  if (!Root->TryGetArrayField(TEXT("entries"), Entries) || Entries == nullptr ||
      Entries->IsEmpty()) {
    OutError = TEXT("game selection catalog entries must be non-empty");
    return false;
  }

  TSet<FString> OptionIds;
  for (const TSharedPtr<FJsonValue> &EntryValue : *Entries) {
    const TSharedPtr<FJsonObject> Entry = EntryValue.IsValid() ? EntryValue->AsObject() : nullptr;
    FGameSelectionOption Option;
    if (!Entry.IsValid() || !ParseOption(Entry, Candidate.CatalogDirectory, Option, OutError) ||
        OptionIds.Contains(Option.Id)) {
      OutError = OutError.IsEmpty() ? TEXT("game selection option ids must be unique") : OutError;
      return false;
    }
    OptionIds.Add(Option.Id);
    Candidate.Options.Add(MoveTemp(Option));
  }

  Candidate.Options.Sort([](const FGameSelectionOption &Left, const FGameSelectionOption &Right) {
    return Left.Order == Right.Order ? Left.Id < Right.Id : Left.Order < Right.Order;
  });
  OutCatalog = MoveTemp(Candidate);
  return true;
}

void FGameSelectionModel::SetCatalog(FGameSelectionCatalog InCatalog) {
  Catalog = MoveTemp(InCatalog);
  SelectedIndex = Catalog.Options.IsEmpty() ? INDEX_NONE : 0;
  ConfirmedOptionId.Reset();
}

const FGameSelectionOption *FGameSelectionModel::GetSelectedOption() const {
  return Catalog.Options.IsValidIndex(SelectedIndex) ? &Catalog.Options[SelectedIndex] : nullptr;
}

bool FGameSelectionModel::SelectNext() {
  if (Catalog.Options.IsEmpty() || IsConfirmed()) {
    return false;
  }
  SelectedIndex = (SelectedIndex + 1 + Catalog.Options.Num()) % Catalog.Options.Num();
  return true;
}

bool FGameSelectionModel::SelectPrevious() {
  if (Catalog.Options.IsEmpty() || IsConfirmed()) {
    return false;
  }
  SelectedIndex = (SelectedIndex - 1 + Catalog.Options.Num()) % Catalog.Options.Num();
  return true;
}

bool FGameSelectionModel::SelectById(const FString &OptionId) {
  if (IsConfirmed()) {
    return false;
  }
  const int32 Index = Catalog.Options.IndexOfByPredicate(
      [&OptionId](const FGameSelectionOption &Option) { return Option.Id == OptionId; });
  if (Index == INDEX_NONE) {
    return false;
  }
  SelectedIndex = Index;
  return true;
}

bool FGameSelectionModel::CanConfirm(FString &OutBlocker) const {
  OutBlocker.Reset();
  if (IsConfirmed()) {
    OutBlocker = TEXT("game selection has already been committed");
    return false;
  }
  const FGameSelectionOption *Option = GetSelectedOption();
  if (Option == nullptr) {
    OutBlocker = TEXT("no compiled game option is available");
    return false;
  }
  if (!Option->IsRunnable()) {
    OutBlocker = Option->AvailabilityReason.IsEmpty() ? TEXT("selected option is not runnable")
                                                      : Option->AvailabilityReason;
    return false;
  }
  return true;
}

bool FGameSelectionModel::MarkConfirmed(FString &OutBlocker) {
  if (!CanConfirm(OutBlocker)) {
    return false;
  }
  ConfirmedOptionId = GetSelectedOption()->Id;
  return true;
}

bool FGameSelectionIntentWriter::WriteNew(const FString &IntentPath,
                                          const FGameSelectionOption &Option, FString &OutError) {
  OutError.Reset();
  if (!Option.IsRunnable()) {
    OutError = TEXT("only a validated runnable option can be handed to the launcher");
    return false;
  }
  if (IntentPath.IsEmpty() || IFileManager::Get().FileExists(*IntentPath)) {
    OutError = TEXT("selection intent path is empty or already exists");
    return false;
  }
  const FString Parent = FPaths::GetPath(FPaths::ConvertRelativePathToFull(IntentPath));
  if (Parent.IsEmpty() || !IFileManager::Get().DirectoryExists(*Parent)) {
    OutError = TEXT("selection intent parent directory does not exist");
    return false;
  }

  const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
  Root->SetStringField(TEXT("schema"), IntentSchema);
  Root->SetStringField(TEXT("selection_id"), Option.Id);
  Root->SetStringField(TEXT("bundle_directory"), Option.BundleDirectory);
  Root->SetStringField(TEXT("session_id"), Option.SessionId);
  FString Json;
  const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
      TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);
  if (!FJsonSerializer::Serialize(Root, Writer)) {
    OutError = TEXT("selection intent JSON serialization failed");
    return false;
  }
  Json += TEXT("\n");

  const FString Temporary = IntentPath + TEXT(".tmp");
  if (IFileManager::Get().FileExists(*Temporary) ||
      !FFileHelper::SaveStringToFile(Json, *Temporary,
                                     FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
                                     &IFileManager::Get(), FILEWRITE_NoReplaceExisting)) {
    OutError = TEXT("selection intent temporary file could not be created");
    return false;
  }
  if (!IFileManager::Get().Move(*IntentPath, *Temporary, false, false, false, true)) {
    IFileManager::Get().Delete(*Temporary, false, true, true);
    OutError = TEXT("selection intent could not be published without replacement");
    return false;
  }
  return true;
}
}  // namespace LingTuSim::UI
