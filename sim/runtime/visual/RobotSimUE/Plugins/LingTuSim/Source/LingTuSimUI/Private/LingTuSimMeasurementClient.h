#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"
#include "LingTuSimMeasurementProjection.h"

class IHttpRequest;

namespace LingTuSim::UI {

/** UI-only HTTP projection. It cannot start a Product or advance inspection. */
class FMeasurementClient final : public TSharedFromThis<FMeasurementClient> {
 public:
  ~FMeasurementClient();
  void Configure(const FString& GatewayUrl, const FMeasurementBinding& Binding);
  void SetVisible(bool bVisible);
  void Shutdown();
  void SelectSensor(const FString& SensorId);
  void SendControl(const FString& Operation, TSharedPtr<FJsonObject> Params);
  const TArray<FMeasurementStatus>& Sensors() const { return SensorRows; }
  const FString& SelectedSensorId() const { return SelectedId; }
  const FMeasurementStatus* SelectedStatus() const;
  const FMeasurementPreview* Preview() const;
  TSharedPtr<FJsonObject> Settings() const;
  bool CanQuery() const;
  bool CanControl() const;
  const FMeasurementBinding& Binding() const { return Expected; }
  bool IsPending() const { return bControlPending; }
  const FString& Message() const { return Feedback; }
  const FString& Error() const { return TransportError; }
  bool IsConfigured() const { return Expected.IsConfigured() && !BaseUrl.IsEmpty(); }
  FString RawUrl() const;

 private:
  void CancelRequest();
  bool Tick(float DeltaSeconds);
  void RequestList();
  void RequestPreview();
  void BeginGet(const FString& Path, bool bList);
  FString BaseUrl;
  FMeasurementBinding Expected;
  TArray<FMeasurementStatus> SensorRows;
  TSet<FString> AcknowledgedSensors;
  FString SelectedId;
  TOptional<FMeasurementPreview> CurrentPreview;
  TSharedPtr<IHttpRequest, ESPMode::ThreadSafe> Request;
  FTSTicker::FDelegateHandle Ticker;
  uint64 RequestRevision = 0;
  double ExpiresAtSeconds = 0;
  double NextListAtSeconds = 0;
  double NextPreviewAtSeconds = 0;
  double LastListAtSeconds = 0;
  bool bVisible = false;
  bool bControlPending = false;
  FString Feedback;
  FString TransportError;
};
}  // namespace LingTuSim::UI
