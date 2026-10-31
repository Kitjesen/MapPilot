#pragma once

#include "CoreMinimal.h"

class FJsonObject;

namespace LingTuSim::UI {

struct FSensorChoice final {
  FString Id;
  FString Package;
  FString ParentFrame;
  FString Modality;
  FString Label;
  double FrequencyHz = 0.0;
  bool bRequired = false;
  bool bEnabled = true;
  /** Zero means this row has no physics timebase metadata. */
  double PhysicsTimestepS = 0.0;
  FString Notice;
  FString UnavailableReason;
  /** Empty means no camera controls; capability comes from setup, not modality. */
  FString CameraPreset;
  double PanDeg = 0.0;
  double TiltDeg = 0.0;
  FVector2D PanLimitsDeg = FVector2D(-180.0, 180.0);
  FVector2D TiltLimitsDeg = FVector2D(-90.0, 90.0);
  /** Zero means editable. Acoustic frequency is block rate, not audio sample rate. */
  double FixedFrequencyHz = 0.0;
  /** Unset means this row has no simulated GNSS fix control. */
  TOptional<bool> GnssFixAvailable;
};

struct FSensorSetup final {
  FString InstanceId;
  FString RigId;
  FString RigVersion;
  TArray<FSensorChoice> Sensors;
};

/** Startup intent only; never changes a running sensor or reports measurements. */
class FSensorSelectionModel final {
 public:
  /** On failure, OutSetup is unchanged and OutError describes the invalid field. */
  static bool ParseSetup(TSharedPtr<FJsonObject> Json, FSensorSetup& OutSetup, FString& OutError);
  /** Supply a validated setup. All authored values become the clean baseline. */
  void Configure(FSensorSetup InSetup);
  const FSensorSetup& GetSetup() const { return Setup; }
  bool HasChanges() const;
  /** Restore authored values without changing setup identity or capabilities. */
  void ResetChanges();
  bool IsEnabled(const FString& Id) const;
  /** Unknown IDs return zero. */
  double GetRate(const FString& Id) const;
  /** Required sensors cannot be disabled; unavailable sensors cannot be enabled. */
  bool SetEnabled(const FString& Id, bool bEnabled);
  /** Honor fixed rates and physics timebase alignment. Rejection preserves the draft. */
  bool SetRate(const FString& Id, double FrequencyHz);
  /** Missing capabilities reject edits; unknown camera getters return empty/zero. */
  bool HasCameraControls(const FString& Id) const;
  FString GetCameraPreset(const FString& Id) const;
  double GetPanDeg(const FString& Id) const;
  double GetTiltDeg(const FString& Id) const;
  bool SetCameraPreset(const FString& Id, const FString& Preset);
  bool SetPanDeg(const FString& Id, double Degrees);
  bool SetTiltDeg(const FString& Id, double Degrees);
  bool HasGnssFixControl(const FString& Id) const;
  bool GetGnssFixAvailable(const FString& Id) const;
  bool SetGnssFixAvailable(const FString& Id, bool bAvailable);
  TSharedPtr<FJsonObject> ToSelectionJson() const;

 private:
  const FSensorChoice* FindSensor(const FString& Id) const;
  FSensorSetup Setup;
  TMap<FString, bool> EnabledOverrides;
  TMap<FString, double> RateOverrides;
  TMap<FString, FString> CameraPresetOverrides;
  TMap<FString, double> PanOverrides;
  TMap<FString, double> TiltOverrides;
  TMap<FString, bool> GnssFixOverrides;
};
}  // namespace LingTuSim::UI
