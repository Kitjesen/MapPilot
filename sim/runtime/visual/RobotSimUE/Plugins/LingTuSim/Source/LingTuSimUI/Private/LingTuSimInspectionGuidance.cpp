#include "LingTuSimInspectionGuidance.h"
#include "LingTuSimInspectionProjection.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace LingTuSim::UI {
namespace {
bool ReadMapPosition(const TSharedPtr<FJsonObject>& Root, const TCHAR* Field,
                     FVector& Out, TSharedPtr<FJsonObject>& Object) {
  const TSharedPtr<FJsonObject>* Value = nullptr;
  FString Frame;
  double X, Y, Z;
  if (!Root->TryGetObjectField(Field, Value) || !Value || !Value->IsValid() ||
      !(*Value)->TryGetStringField(TEXT("frame_id"), Frame) || Frame != TEXT("map") ||
      !(*Value)->TryGetNumberField(TEXT("x"), X) || !FMath::IsFinite(X) ||
      !(*Value)->TryGetNumberField(TEXT("y"), Y) || !FMath::IsFinite(Y) ||
      !(*Value)->TryGetNumberField(TEXT("z"), Z) || !FMath::IsFinite(Z)) return false;
  Object = *Value;
  Out = FVector(X, Y, Z);
  return true;
}
}

bool FInspectionGuidance::Parse(const FString& Json, const FInspectionProjection& Task,
                                FInspectionGuidance& Out, FString& Error) {
  Out = FInspectionGuidance();
  TSharedPtr<FJsonObject> Root;
  const auto Reader = TJsonReaderFactory<>::Create(Json);
  FString Schema, TaskId;
  if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid() ||
      !Root->TryGetStringField(TEXT("schema_version"), Schema) ||
      Schema != TEXT("lingtu.inspection.guidance.v1") ||
      !Root->TryGetStringField(TEXT("task_id"), TaskId) || TaskId != Task.TaskId ||
      !Root->TryGetBoolField(TEXT("available"), Out.bAvailable)) {
    Error = TEXT("invalid guidance response or task identity"); return false;
  }
  Root->TryGetStringField(TEXT("reason"), Out.Reason);
  const TSharedPtr<FJsonObject>* Runtime = nullptr;
  FString Env, Product, RuntimeState, RuntimeSession;
  Root->TryGetStringField(TEXT("product_session_id"), Out.ProductSessionId);
  Out.bSimulationRuntime = Root->TryGetObjectField(TEXT("runtime"), Runtime) && Runtime && Runtime->IsValid() &&
    (*Runtime)->TryGetStringField(TEXT("env"), Env) && Env == TEXT("sim") &&
    (*Runtime)->TryGetStringField(TEXT("product"), Product) && Product == TEXT("inspection") &&
    (*Runtime)->TryGetStringField(TEXT("state"), RuntimeState) && RuntimeState == TEXT("active") &&
    (*Runtime)->TryGetStringField(TEXT("product_session_id"), RuntimeSession) && !RuntimeSession.IsEmpty() &&
    Out.ProductSessionId == RuntimeSession && Out.ProductSessionId == Task.ProductSessionId;
  if (!Out.bSimulationRuntime) {
    Out.bAvailable = false; Out.Reason = TEXT("simulation_gateway_required"); return true;
  }
  if (!Out.bAvailable) return true;
  const TSharedPtr<FJsonObject>* Identity = nullptr;
  FString IdentityTask, Route, Map;
  double Revision = 0, Epoch = 0;
  if (!Task.bLauncherBindingVerified || !Task.bProgressVerified || Task.bTerminal ||
      !Root->TryGetObjectField(TEXT("identity"), Identity) || !Identity || !Identity->IsValid() ||
      !(*Identity)->TryGetStringField(TEXT("task_id"), IdentityTask) || IdentityTask != Task.TaskId ||
      !(*Identity)->TryGetStringField(TEXT("route_id"), Route) || Route != Task.RouteId ||
      !(*Identity)->TryGetStringField(TEXT("map_id"), Map) || Map != Task.MapId ||
      !(*Identity)->TryGetNumberField(TEXT("route_revision"), Revision) || Revision != Task.RouteRevision ||
      !(*Identity)->TryGetNumberField(TEXT("map_content_epoch"), Epoch) || Epoch != Task.MapContentEpoch ||
      !Root->TryGetStringField(TEXT("point_id"), Out.PointId) || Out.PointId != Task.CurrentPointId ||
      !Root->TryGetStringField(TEXT("phase"), Out.Phase) || Out.Phase != Task.Phase) {
    Out = FInspectionGuidance(); Error = TEXT("guidance disagrees with current native task"); return false;
  }
  TSharedPtr<FJsonObject> Goal, Robot;
  if (!ReadMapPosition(Root, TEXT("goal"), Out.GoalMapMeters, Goal) ||
      !ReadMapPosition(Root, TEXT("robot_pose"), Out.RobotMapMeters, Robot) ||
      !Robot->TryGetNumberField(TEXT("yaw"), Out.RobotMapYawRadians) ||
      !FMath::IsFinite(Out.RobotMapYawRadians)) {
    Out = FInspectionGuidance(); Error = TEXT("guidance map-frame pose unavailable"); return false;
  }
  return true;
}

bool FInspectionGuidance::IsCurrent(const FInspectionProjection& Task, double NowSeconds) const {
  return bAvailable && bSimulationRuntime && Task.bBound && Task.bTaskAvailable && Task.bProgressVerified &&
    Task.bLauncherBindingVerified && !Task.bStale && !Task.bTerminal &&
    PointId == Task.CurrentPointId && Phase == Task.Phase && ReceivedSeconds > 0 &&
    NowSeconds >= ReceivedSeconds && NowSeconds - ReceivedSeconds <= 3.0;
}

double FInspectionGuidance::DistanceMeters() const { return FVector::Distance(GoalMapMeters, RobotMapMeters); }
double FInspectionGuidance::BearingDegrees() const {
  const FVector Delta = GoalMapMeters - RobotMapMeters;
  return FMath::UnwindDegrees(FMath::RadiansToDegrees(FMath::Atan2(Delta.Y, Delta.X) - RobotMapYawRadians));
}

FVector FInspectionGuidance::EstimatedTargetUnrealCm(const FNavigationPose& Pose) const {
  const double SimYaw = FMath::DegreesToRadians(Pose.Rotation.Rotator().Yaw);
  const FVector Delta = GoalMapMeters - RobotMapMeters;
  const double Angle = SimYaw - RobotMapYawRadians;
  const FVector SimTarget = Pose.PositionMeters + FVector(
    FMath::Cos(Angle) * Delta.X - FMath::Sin(Angle) * Delta.Y,
    FMath::Sin(Angle) * Delta.X + FMath::Cos(Angle) * Delta.Y, Delta.Z);
  return FVector(SimTarget.X * 100, -SimTarget.Y * 100, SimTarget.Z * 100);
}

FVector2D FInspectionGuidance::PlaceMarker(FVector2D Position, FVector2D ViewSize) {
  const FVector2D Centre = ViewSize * .5;
  const FVector2D Extent(FMath::Max(1.0, Centre.X - 64), FMath::Max(1.0, Centre.Y - 64));
  const FVector2D Delta = Position - Centre;
  const double Scale = FMath::Max(1.0, FMath::Max(FMath::Abs(Delta.X) / Extent.X, FMath::Abs(Delta.Y) / Extent.Y));
  return Centre + Delta / Scale;
}

FText FInspectionGuidance::PhaseText(const FInspectionProjection& Task) {
  if (!Task.bBound) return FText::GetEmpty();
  if (Task.bStale || !Task.bProgressVerified) return FText::FromString(TEXT("任务状态待确认"));
  if (Task.Phase == TEXT("SETTLING")) return FText::FromString(TEXT("等待机器人停稳"));
  if (Task.Phase == TEXT("DWELLING")) return FText::FromString(TEXT("观察停留中"));
  if (Task.Phase == TEXT("ACTION_PENDING")) return FText::FromString(TEXT("采集中 · 等待证据结果"));
  if (Task.Phase == TEXT("WAITING_NEXT_LOOP")) return FText::FromString(TEXT("等待下一轮巡检 · 当前不行驶"));
  if (Task.Phase == TEXT("PAUSED")) return FText::FromString(TEXT("任务已暂停 · 请显式继续"));
  if (Task.Phase == TEXT("PAUSING")) return FText::FromString(TEXT("暂停请求处理中"));
  if (Task.Phase == TEXT("CANCELLING")) return FText::FromString(TEXT("取消请求处理中"));
  if (Task.Phase == TEXT("RECOVERING")) return FText::FromString(TEXT("恢复中 · 等待原生导航"));
  if (Task.Phase == TEXT("SUCCEEDED")) return FText::FromString(TEXT("路线执行完成 · 证据接受结果另行确认"));
  if (Task.Phase == TEXT("FAILED")) return FText::FromString(TEXT("任务失败 · 查看原因"));
  if (Task.Phase == TEXT("CANCELLED")) return FText::FromString(TEXT("任务已取消"));
  if (Task.Phase == TEXT("PLANNING") || Task.Phase == TEXT("VALIDATING")) return FText::FromString(TEXT("正在规划 · 尚未确认可通行"));
  return FText::FromString(FString::Printf(TEXT("前往巡检点 %02d"), Task.CurrentPointNumber));
}
}
