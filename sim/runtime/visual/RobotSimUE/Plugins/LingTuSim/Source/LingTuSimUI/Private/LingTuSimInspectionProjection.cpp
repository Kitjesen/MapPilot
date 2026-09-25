#include "LingTuSimInspectionProjection.h"

#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace LingTuSim::UI {
namespace {
constexpr double MaxSafeJsonInteger = 9007199254740991.0;

bool ReadRequiredString(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field,
                        FString &OutValue, FString &OutError) {
  if (!Object.IsValid() || !Object->TryGetStringField(Field, OutValue) || OutValue.IsEmpty()) {
    OutError = FString::Printf(TEXT("missing or invalid %s"), Field);
    return false;
  }
  return true;
}

bool ReadRequiredInteger(const TSharedPtr<FJsonObject> &Object, const TCHAR *Field,
                         int64 &OutValue, FString &OutError) {
  double Number = 0.0;
  if (!Object.IsValid() || !Object->TryGetNumberField(Field, Number) || !FMath::IsFinite(Number) ||
      Number != FMath::TruncToDouble(Number) || FMath::Abs(Number) > MaxSafeJsonInteger) {
    OutError = FString::Printf(TEXT("missing or invalid %s"), Field);
    return false;
  }
  OutValue = static_cast<int64>(Number);
  return true;
}

bool IsOneOf(const FString &Value, const std::initializer_list<const TCHAR *> Allowed) {
  for (const TCHAR *Candidate : Allowed) {
    if (Value == Candidate) {
      return true;
    }
  }
  return false;
}
}  // namespace

bool FInspectionProjectionParser::ParseObject(const FString &Json,
                                              TSharedPtr<FJsonObject> &OutObject,
                                              FString &OutError) {
  OutObject.Reset();
  const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  if (!FJsonSerializer::Deserialize(Reader, OutObject) || !OutObject.IsValid()) {
    OutError = TEXT("response is not a JSON object");
    return false;
  }
  return true;
}

bool FInspectionProjectionParser::ParseTask(const FString &Json, const FString &ExpectedTaskId,
                                            const FInspectionExpectedBinding &ExpectedBinding,
                                            FInspectionProjection &OutProjection,
                                            FString &OutError) {
  FInspectionProjection Parsed;
  Parsed.TaskId = ExpectedTaskId;
  Parsed.bBound = !ExpectedTaskId.IsEmpty();

  TSharedPtr<FJsonObject> Root;
  if (!ParseObject(Json, Root, OutError)) {
    return false;
  }
  FString Schema;
  if (!Root->TryGetStringField(TEXT("schema_version"), Schema) ||
      Schema != TEXT("lingtu.inspection.task.v1")) {
    OutError = TEXT("unexpected inspection task schema");
    return false;
  }
  FString ResponseTaskId;
  if (!ReadRequiredString(Root, TEXT("task_id"), ResponseTaskId, OutError) ||
      ResponseTaskId != ExpectedTaskId) {
    OutError = TEXT("inspection task identity mismatch");
    return false;
  }
  bool bFound = false;
  if (!Root->TryGetBoolField(TEXT("found"), bFound)) {
    OutError = TEXT("inspection task found flag is missing");
    return false;
  }
  Parsed.bTaskAvailable = bFound;
  Root->TryGetStringField(TEXT("reason"), Parsed.Reason);
  if (!bFound) {
    OutProjection = MoveTemp(Parsed);
    return true;
  }

  const TSharedPtr<FJsonObject> *Identity = nullptr;
  if (!Root->TryGetObjectField(TEXT("identity"), Identity) || Identity == nullptr ||
      !Identity->IsValid()) {
    OutError = TEXT("inspection task identity is missing");
    return false;
  }
  FString IdentityTaskId;
  if (!ReadRequiredString(*Identity, TEXT("task_id"), IdentityTaskId, OutError) ||
      IdentityTaskId != ExpectedTaskId ||
      !ReadRequiredString(*Identity, TEXT("route_id"), Parsed.RouteId, OutError) ||
      !ReadRequiredString(*Identity, TEXT("map_id"), Parsed.MapId, OutError) ||
      !ReadRequiredInteger(*Identity, TEXT("route_revision"), Parsed.RouteRevision, OutError) ||
      Parsed.RouteRevision <= 0 ||
      !ReadRequiredInteger(*Identity, TEXT("map_content_epoch"), Parsed.MapContentEpoch,
                           OutError) ||
      Parsed.MapContentEpoch < 0) {
    OutError = TEXT("inspection task immutable identity is invalid");
    return false;
  }
  if (!ExpectedBinding.bConfigured) {
    Parsed.IdentityBlocker = TEXT("launcher_expected_binding_missing");
  } else if (Parsed.MapId != ExpectedBinding.MapId) {
    Parsed.IdentityBlocker = TEXT("launcher_expected_map_id_mismatch");
  } else if (Parsed.MapContentEpoch != ExpectedBinding.MapContentEpoch) {
    Parsed.IdentityBlocker = TEXT("launcher_expected_map_content_epoch_mismatch");
  } else if (Parsed.RouteRevision != ExpectedBinding.RouteRevision) {
    Parsed.IdentityBlocker = TEXT("launcher_expected_route_revision_mismatch");
  } else {
    Parsed.bLauncherBindingVerified = true;
  }

  Root->TryGetStringField(TEXT("current_state"), Parsed.State);
  Root->TryGetStringField(TEXT("phase"), Parsed.Phase);
  if (!ReadRequiredString(Root, TEXT("state_source"), Parsed.StateSource, OutError)) {
    return false;
  }
  if (!Root->TryGetBoolField(TEXT("execution_confirmed"), Parsed.bExecutionConfirmed) ||
      !Root->TryGetBoolField(TEXT("terminal"), Parsed.bTerminal)) {
    OutError = TEXT("inspection task execution flags are missing");
    return false;
  }
  const bool bStateKnown =
      IsOneOf(Parsed.State, {TEXT("PLANNING"), TEXT("EXECUTING"), TEXT("PAUSED"),
                            TEXT("RECOVERING"), TEXT("SUCCESS"), TEXT("FAILED"),
                            TEXT("CANCELLED")});
  const bool bPhaseKnown =
      IsOneOf(Parsed.Phase, {TEXT("VALIDATING"), TEXT("PLANNING"), TEXT("NAVIGATING"),
                            TEXT("DWELLING"), TEXT("PAUSED"), TEXT("RECOVERING"),
                            TEXT("SUCCEEDED"), TEXT("FAILED"), TEXT("CANCELLED"),
                            TEXT("SETTLING"), TEXT("ACTION_PENDING"), TEXT("PAUSING"),
                            TEXT("CANCELLING")});
  const bool bNativeStateSource =
      Parsed.StateSource == TEXT("native_task_event") ||
      Parsed.StateSource == TEXT("persisted_native_task_event");
  const bool bTerminalState =
      Parsed.State == TEXT("SUCCESS") || Parsed.State == TEXT("FAILED") ||
      Parsed.State == TEXT("CANCELLED");
  const bool bTerminalPhaseMatches =
      (Parsed.State == TEXT("SUCCESS") && Parsed.Phase == TEXT("SUCCEEDED")) ||
      (Parsed.State == TEXT("FAILED") && Parsed.Phase == TEXT("FAILED")) ||
      (Parsed.State == TEXT("CANCELLED") && Parsed.Phase == TEXT("CANCELLED"));
  if ((!Parsed.State.IsEmpty() && !bStateKnown) || (!Parsed.Phase.IsEmpty() && !bPhaseKnown) ||
      (Parsed.bExecutionConfirmed &&
       (!bStateKnown || !bPhaseKnown || !bNativeStateSource)) ||
      (Parsed.bTerminal &&
       (!Parsed.bExecutionConfirmed || !bTerminalState || !bTerminalPhaseMatches)) ||
      (!Parsed.bTerminal && bTerminalState)) {
    OutError = TEXT("inspection task state is invalid");
    return false;
  }

  const TSharedPtr<FJsonObject> *Delivery = nullptr;
  FString Continuity;
  if (!Root->TryGetObjectField(TEXT("delivery"), Delivery) || Delivery == nullptr ||
      !Delivery->IsValid() ||
      !(*Delivery)->TryGetBoolField(TEXT("history_complete"), Parsed.bHistoryComplete) ||
      !ReadRequiredString(*Delivery, TEXT("continuity"), Continuity, OutError)) {
    OutError = TEXT("inspection task continuity is missing");
    return false;
  }
  Parsed.bHistoryComplete = Parsed.bHistoryComplete && Continuity == TEXT("verified");

  const TSharedPtr<FJsonObject> *Progress = nullptr;
  bool bProgressKnown = false;
  if (!Root->TryGetObjectField(TEXT("progress"), Progress) || Progress == nullptr ||
      !Progress->IsValid() || !(*Progress)->TryGetBoolField(TEXT("known"), bProgressKnown)) {
    OutError = TEXT("inspection task progress is missing");
    return false;
  }
  Parsed.bProgressVerified =
      bProgressKnown && Parsed.bExecutionConfirmed && Parsed.bHistoryComplete &&
      Parsed.bLauncherBindingVerified;
  if (Parsed.bProgressVerified) {
    int64 Completed = 0;
    int64 Count = 0;
    if (!ReadRequiredInteger(*Progress, TEXT("completed_points"), Completed, OutError) ||
        !ReadRequiredInteger(*Progress, TEXT("point_count"), Count, OutError) || Completed < 0 ||
        Count < 0 || Completed > Count || Count > MAX_int32) {
      OutError = TEXT("inspection task progress values are invalid");
      return false;
    }
    Parsed.CompletedPoints = static_cast<int32>(Completed);
    Parsed.PointCount = static_cast<int32>(Count);
    double PointNumber = 0.0;
    if ((*Progress)->TryGetNumberField(TEXT("current_point_number"), PointNumber) &&
        FMath::IsFinite(PointNumber) && PointNumber == FMath::TruncToDouble(PointNumber) &&
        PointNumber > 0.0 && PointNumber <= static_cast<double>(MAX_int32)) {
      Parsed.CurrentPointNumber = static_cast<int32>(PointNumber);
    }
    (*Progress)->TryGetStringField(TEXT("current_point_id"), Parsed.CurrentPointId);
    (*Progress)->TryGetStringField(TEXT("action"), Parsed.Action);
  }

  OutProjection = MoveTemp(Parsed);
  return true;
}

bool FInspectionProjectionParser::ParseReport(const FString &Json,
                                              const FInspectionProjection &TaskProjection,
                                              FInspectionProjection &OutProjection,
                                              FString &OutError) {
  TSharedPtr<FJsonObject> Root;
  if (!ParseObject(Json, Root, OutError)) {
    return false;
  }
  FString Schema;
  FString TaskId;
  if (!Root->TryGetStringField(TEXT("schema_version"), Schema) ||
      Schema != TEXT("lingtu.inspection.report.v1") ||
      !ReadRequiredString(Root, TEXT("task_id"), TaskId, OutError) ||
      TaskId != TaskProjection.TaskId) {
    OutError = TEXT("inspection report identity mismatch");
    return false;
  }

  const TSharedPtr<FJsonObject> *Identity = nullptr;
  FString RouteId;
  FString MapId;
  int64 RouteRevision = 0;
  int64 MapContentEpoch = 0;
  if (!Root->TryGetObjectField(TEXT("identity"), Identity) || Identity == nullptr ||
      !Identity->IsValid() || !ReadRequiredString(*Identity, TEXT("route_id"), RouteId, OutError) ||
      !ReadRequiredString(*Identity, TEXT("map_id"), MapId, OutError) ||
      !ReadRequiredInteger(*Identity, TEXT("route_revision"), RouteRevision, OutError) ||
      !ReadRequiredInteger(*Identity, TEXT("map_content_epoch"), MapContentEpoch, OutError) ||
      RouteId != TaskProjection.RouteId || MapId != TaskProjection.MapId ||
      RouteRevision != TaskProjection.RouteRevision ||
      MapContentEpoch != TaskProjection.MapContentEpoch) {
    OutError = TEXT("inspection report immutable identity mismatch");
    return false;
  }

  const TSharedPtr<FJsonObject> *Execution = nullptr;
  bool bConfirmed = false;
  bool bHistoryComplete = false;
  if (!Root->TryGetObjectField(TEXT("execution"), Execution) || Execution == nullptr ||
      !Execution->IsValid() ||
      !(*Execution)->TryGetBoolField(TEXT("confirmed"), bConfirmed) ||
      !(*Execution)->TryGetBoolField(TEXT("history_complete"), bHistoryComplete) || !bConfirmed ||
      !bHistoryComplete || !TaskProjection.bExecutionConfirmed ||
      !TaskProjection.bHistoryComplete || !TaskProjection.bLauncherBindingVerified) {
    OutError = TEXT("inspection report execution is not verified");
    return false;
  }

  FInspectionProjection Parsed = TaskProjection;
  FString ExecutionState;
  if (!ReadRequiredString(*Execution, TEXT("state"), ExecutionState, OutError) ||
      ExecutionState != TaskProjection.State) {
    OutError = TEXT("inspection report execution state mismatch");
    return false;
  }
  if (!ReadRequiredString(Root, TEXT("report_status"), Parsed.ReportStatus, OutError) ||
      !ReadRequiredString(Root, TEXT("acceptance"), Parsed.Acceptance, OutError)) {
    return false;
  }
  if (!IsOneOf(Parsed.ReportStatus,
               {TEXT("IN_PROGRESS"), TEXT("COMPLETE"), TEXT("PARTIAL"), TEXT("FAILED"),
                TEXT("CANCELLED"), TEXT("UNKNOWN")}) ||
      !IsOneOf(Parsed.Acceptance,
               {TEXT("PENDING"), TEXT("ACCEPTABLE"), TEXT("REVIEW_REQUIRED"),
                TEXT("NOT_ACCEPTABLE"), TEXT("UNKNOWN")})) {
    OutError = TEXT("inspection report outcome is invalid");
    return false;
  }
  bool bReportTerminal = false;
  if (!Root->TryGetBoolField(TEXT("terminal"), bReportTerminal) ||
      bReportTerminal != TaskProjection.bTerminal) {
    OutError = TEXT("inspection report terminal state mismatch");
    return false;
  }

  const TSharedPtr<FJsonObject> *Coverage = nullptr;
  int64 RequiredEvidence = 0;
  int64 VerifiedEvidence = 0;
  if (!Root->TryGetObjectField(TEXT("coverage"), Coverage) || Coverage == nullptr ||
      !Coverage->IsValid() ||
      !ReadRequiredInteger(*Coverage, TEXT("required_evidence"), RequiredEvidence, OutError) ||
      !ReadRequiredInteger(*Coverage, TEXT("verified_evidence"), VerifiedEvidence, OutError) ||
      RequiredEvidence < 0 || VerifiedEvidence < 0 || VerifiedEvidence > RequiredEvidence ||
      RequiredEvidence > MAX_int32) {
    OutError = TEXT("inspection report coverage is invalid");
    return false;
  }
  Parsed.RequiredEvidence = static_cast<int32>(RequiredEvidence);
  Parsed.VerifiedEvidence = static_cast<int32>(VerifiedEvidence);
  Parsed.bReportVerified = true;
  OutProjection = MoveTemp(Parsed);
  return true;
}

}  // namespace LingTuSim::UI
