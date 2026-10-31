#include "LingTuSimMeasurementClient.h"

#include "Dom/JsonObject.h"
#include "GenericPlatform/GenericPlatformHttp.h"
#include "HAL/PlatformTime.h"
#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace LingTuSim::UI {
namespace {
bool SameBinding(const FMeasurementBinding& A, const FMeasurementBinding& B) {
  return A.ProductSessionId == B.ProductSessionId && A.SessionId == B.SessionId &&
      A.ModelGeneration == B.ModelGeneration && A.ResetGeneration == B.ResetGeneration;
}
constexpr int32 MaxPreviewBodyBytes = 2 * 1024 * 1024;
}

FMeasurementClient::~FMeasurementClient() { Shutdown(); }

void FMeasurementClient::Configure(const FString& GatewayUrl, const FMeasurementBinding& Binding) {
  FString Url = GatewayUrl.TrimStartAndEnd();
  while (Url.EndsWith(TEXT("/"))) Url.LeftChopInline(1);
  if (!(Url.StartsWith(TEXT("http://")) || Url.StartsWith(TEXT("https://")))) Url.Reset();
  if (Url.Contains(TEXT("?")) || Url.Contains(TEXT("#")) || Url.Contains(TEXT("@"))) Url.Reset();
  if (Url == BaseUrl && SameBinding(Binding, Expected)) return;
  CancelRequest();
  BaseUrl = Url;
  Expected = Binding;
  SensorRows.Reset();
  AcknowledgedSensors.Reset();
  SelectedId.Reset();
  CurrentPreview.Reset();
  LastListAtSeconds = NextListAtSeconds = NextPreviewAtSeconds = 0;
  Feedback.Reset();
  TransportError = IsConfigured() ? FString() : TEXT("未绑定当前 Product 和源会话");
}

void FMeasurementClient::SetVisible(bool bInVisible) {
  if (bVisible == bInVisible) return;
  bVisible = bInVisible;
  if (bVisible) {
    NextListAtSeconds = NextPreviewAtSeconds = 0;
    Ticker = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateSP(AsShared(), &FMeasurementClient::Tick), .25F);
  } else {
    if (bControlPending) Feedback = TEXT("页面已关闭；控制结果未确认，请重新查询设置");
    CancelRequest();
    CurrentPreview.Reset();
    if (Ticker.IsValid()) FTSTicker::GetCoreTicker().RemoveTicker(Ticker);
    Ticker.Reset();
  }
}

void FMeasurementClient::Shutdown() {
  SetVisible(false);
  CancelRequest();
  CurrentPreview.Reset();
  SensorRows.Reset();
}

void FMeasurementClient::CancelRequest() {
  ++RequestRevision;
  auto Previous = MoveTemp(Request);
  Request.Reset();
  bControlPending = false;
  if (Previous.IsValid()) {
    Previous->OnProcessRequestComplete().Unbind();
    Previous->CancelRequest();
  }
}

const FMeasurementStatus* FMeasurementClient::SelectedStatus() const {
  return SensorRows.FindByPredicate([this](const FMeasurementStatus& Row) { return Row.SensorId == SelectedId; });
}

const FMeasurementPreview* FMeasurementClient::Preview() const {
  return bVisible && CurrentPreview.IsSet() && FPlatformTime::Seconds() < ExpiresAtSeconds
      ? &CurrentPreview.GetValue() : nullptr;
}

TSharedPtr<FJsonObject> FMeasurementClient::Settings() const {
  const auto* Row = SelectedStatus();
  return Row ? Row->Settings : nullptr;
}

bool FMeasurementClient::CanQuery() const {
  const auto* Row = SelectedStatus();
  return bVisible && IsConfigured() && !bControlPending && Row && LastListAtSeconds > 0 &&
      FPlatformTime::Seconds() - LastListAtSeconds <= 3.;
}

bool FMeasurementClient::CanControl() const {
  const auto* Row = SelectedStatus();
  return CanQuery() && ((Row->bHasIdentity && SameBinding(Row->Binding, Expected)) ||
      AcknowledgedSensors.Contains(SelectedId));
}

FString FMeasurementClient::RawUrl() const {
  return Preview() ? BaseUrl + TEXT("/api/v1/measurements/") +
      FGenericPlatformHttp::UrlEncode(SelectedId) + TEXT("/raw") : FString();
}

void FMeasurementClient::SelectSensor(const FString& SensorId) {
  if (SelectedId == SensorId) return;
  CancelRequest();
  SelectedId = SensorId;
  CurrentPreview.Reset();
  Feedback.Reset();
  NextPreviewAtSeconds = 0;
}

bool FMeasurementClient::Tick(float) {
  const double Now = FPlatformTime::Seconds();
  if (CurrentPreview.IsSet() && Now >= ExpiresAtSeconds) CurrentPreview.Reset();
  if (!bVisible || !IsConfigured() || Request.IsValid()) return true;
  if (Now >= NextListAtSeconds) RequestList();
  else if (!SelectedId.IsEmpty() && Now >= NextPreviewAtSeconds) RequestPreview();
  return true;
}

void FMeasurementClient::RequestList() {
  NextListAtSeconds = FPlatformTime::Seconds() + 1.;
  BeginGet(TEXT("/api/v1/measurements"), true);
}

void FMeasurementClient::RequestPreview() {
  NextPreviewAtSeconds = FPlatformTime::Seconds() + .25;
  const auto* Row = SelectedStatus();
  if (!Row || !Row->bFresh) { CurrentPreview.Reset(); return; }
  BeginGet(TEXT("/api/v1/measurements/") + FGenericPlatformHttp::UrlEncode(SelectedId) + TEXT("/preview"), false);
}

void FMeasurementClient::BeginGet(const FString& Path, bool bList) {
  const uint64 Revision = ++RequestRevision;
  const double Started = FPlatformTime::Seconds();
  const FString SensorId = SelectedId;
  Request = FHttpModule::Get().CreateRequest();
  Request->SetVerb(TEXT("GET"));
  Request->SetURL(BaseUrl + Path);
  Request->SetHeader(TEXT("Cache-Control"), TEXT("no-cache"));
  Request->SetTimeout(3.F);
  Request->SetDelegateThreadPolicy(EHttpRequestDelegateThreadPolicy::CompleteOnGameThread);
  TWeakPtr<FMeasurementClient> Weak = AsShared();
  Request->OnProcessRequestComplete().BindLambda([Weak, Revision, Started, SensorId, bList](
      FHttpRequestPtr, FHttpResponsePtr Response, bool bOk) {
    auto Self = Weak.Pin();
    if (!Self || Self->RequestRevision != Revision) return;
    Self->Request.Reset();
    const double Now = FPlatformTime::Seconds();
    // A slow list must leave a window for the selected preview, not starve it.
    if (bList) Self->NextListAtSeconds = Now + 1.;
    if (!bOk || !Response || Response->GetResponseCode() != 200 || Response->GetContent().Num() > MaxPreviewBodyBytes) {
      Self->CurrentPreview.Reset();
      Self->TransportError = FString::Printf(TEXT("测量不可用（HTTP %d）"), Response ? Response->GetResponseCode() : 0);
      if (bList) Self->LastListAtSeconds = 0;
      return;
    }
    FString Error;
    if (bList) {
      TArray<FMeasurementStatus> Rows;
      if (!FMeasurementProjectionParser::ParseList(Response->GetContentAsString(), Self->Expected, Rows, Error)) {
        Self->SensorRows.Reset();
        Self->CurrentPreview.Reset();
        Self->LastListAtSeconds = 0;
        Self->TransportError = Error;
        return;
      }
      Self->SensorRows = MoveTemp(Rows);
      Self->LastListAtSeconds = Now;
      if (Self->SelectedId.IsEmpty() && Self->SensorRows.Num()) Self->SelectedId = Self->SensorRows[0].SensorId;
      const auto* Selected = Self->SelectedStatus();
      if (!Selected || !Selected->bFresh) Self->CurrentPreview.Reset();
      Self->TransportError.Reset();
    } else {
      FMeasurementPreview Preview;
      if (!FMeasurementProjectionParser::ParsePreview(Response->GetContentAsString(), Self->Expected, SensorId, Preview, Error)) {
        Self->CurrentPreview.Reset();
        Self->TransportError = Error;
        return;
      }
      const double Remaining = Preview.Status.StaleTimeoutSeconds - Preview.Status.ReceiptAgeSeconds - (Now - Started);
      if (!Preview.Status.bFresh || Remaining <= 0) {
        Self->CurrentPreview.Reset();
        Self->TransportError = TEXT("数据已过期，等待新测量");
        return;
      }
      Self->ExpiresAtSeconds = Now + Remaining;
      Self->CurrentPreview = MoveTemp(Preview);
      Self->TransportError.Reset();
    }
  });
  if (!Request->ProcessRequest()) {
    Request.Reset();
    CurrentPreview.Reset();
    TransportError = TEXT("无法提交测量请求");
  }
}

void FMeasurementClient::SendControl(const FString& Operation, TSharedPtr<FJsonObject> Params) {
  if (!(Operation == TEXT("query") ? CanQuery() : CanControl()) || !Params) {
    Feedback = TEXT("当前来源未就绪，未发送控制"); return;
  }
  CancelRequest();
  bControlPending = true;
  const FString RequestId = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens);
  const FString SensorId = SelectedId;
  const uint64 Revision = RequestRevision;
  auto Body = MakeShared<FJsonObject>();
  Body->SetStringField(TEXT("request_id"), RequestId);
  Body->SetStringField(TEXT("expected_product_session_id"), Expected.ProductSessionId);
  Body->SetStringField(TEXT("operation"), Operation);
  Body->SetObjectField(TEXT("params"), Params);
  FString Json;
  FJsonSerializer::Serialize(Body, TJsonWriterFactory<>::Create(&Json));
  Request = FHttpModule::Get().CreateRequest();
  Request->SetVerb(TEXT("POST"));
  Request->SetURL(BaseUrl + TEXT("/api/v1/sensors/") + FGenericPlatformHttp::UrlEncode(SensorId) + TEXT("/control"));
  Request->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
  Request->SetContentAsString(Json);
  Request->SetTimeout(3.F);
  Request->SetDelegateThreadPolicy(EHttpRequestDelegateThreadPolicy::CompleteOnGameThread);
  Feedback = TEXT("等待源端回执…");
  TWeakPtr<FMeasurementClient> Weak = AsShared();
  Request->OnProcessRequestComplete().BindLambda([Weak, Revision, RequestId, SensorId](
      FHttpRequestPtr, FHttpResponsePtr Response, bool bOk) {
    auto Self = Weak.Pin();
    if (!Self || Self->RequestRevision != Revision) return;
    Self->Request.Reset();
    Self->bControlPending = false;
    Self->NextListAtSeconds = 0;
    bool bApplied = false;
    FString Reason, Error;
    TSharedPtr<FJsonObject> Settings;
    if (!bOk || !Response || (Response->GetResponseCode() != 200 && Response->GetResponseCode() != 409) ||
        Response->GetContent().Num() > MaxPreviewBodyBytes ||
        !FMeasurementProjectionParser::ParseAck(Response->GetContentAsString(), Self->Expected, RequestId,
          SensorId, bApplied, Reason, Settings, Error)) {
      Self->Feedback = TEXT("控制结果未确认；不会自动重试，请查询设置") + (Error.IsEmpty() ? FString() : TEXT("：") + Error);
      Self->CurrentPreview.Reset();
      return;
    }
    Self->Feedback = (bApplied ? TEXT("源端已确认：") : TEXT("源端拒绝：")) + Reason;
    if (!bApplied) return;
    Self->AcknowledgedSensors.Add(SensorId);
    if (auto* Row = Self->SensorRows.FindByPredicate([&](const FMeasurementStatus& Item) { return Item.SensorId == SensorId; })) {
      Row->Settings = Settings;
      bool bEnabled = true;
      if (Settings && Settings->TryGetBoolField(TEXT("enabled"), bEnabled) && !bEnabled) {
        Row->bFresh = false;
        Row->State = TEXT("disabled");
        Self->CurrentPreview.Reset();
      }
    }
  });
  if (!Request->ProcessRequest()) {
    Request.Reset();
    bControlPending = false;
    Feedback = TEXT("无法提交控制；未确认生效");
  }
}
}  // namespace LingTuSim::UI
