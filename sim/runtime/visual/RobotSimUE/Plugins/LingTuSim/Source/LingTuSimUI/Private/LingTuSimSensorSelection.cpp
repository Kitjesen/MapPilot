#include "LingTuSimSensorSelection.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include <cmath>

namespace LingTuSim::UI {
namespace {
bool ReadSensorSetupString(const TSharedPtr<FJsonObject>& Json, const TCHAR* Field, FString& Out) {
  return Json.IsValid() && Json->HasTypedField<EJson::String>(Field) &&
    Json->TryGetStringField(Field, Out) && !Out.TrimStartAndEnd().IsEmpty();
}
bool IsCameraPreset(const FString& Preset) {
  return Preset == TEXT("pinhole") || Preset == TEXT("equidistant_fisheye") || Preset == TEXT("equirectangular");
}
bool ReadFiniteNumber(const TSharedPtr<FJsonObject>& Json, const TCHAR* Field, double& Out) {
  return Json->HasTypedField<EJson::Number>(Field) && Json->TryGetNumberField(Field, Out) && FMath::IsFinite(Out);
}
bool ReadAngleLimits(const TSharedPtr<FJsonObject>& Json, const TCHAR* Field, FVector2D& Out) {
  const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
  if (!Json->TryGetArrayField(Field, Values) || !Values || Values->Num() != 2) return false;
  double Min = 0.0, Max = 0.0;
  if (!(*Values)[0].IsValid() || (*Values)[0]->Type != EJson::Number || !(*Values)[0]->TryGetNumber(Min) ||
      !(*Values)[1].IsValid() || (*Values)[1]->Type != EJson::Number || !(*Values)[1]->TryGetNumber(Max) ||
      !FMath::IsFinite(Min) || !FMath::IsFinite(Max) || Min > Max) return false;
  Out = FVector2D(Min, Max);
  return true;
}
bool InAngleLimits(double Value, const FVector2D& Limits) {
  return FMath::IsFinite(Value) && Value >= Limits.X && Value <= Limits.Y;
}
bool TrySensorSelectionNanoseconds(double Value, int64& Out) {
  // Stay within exact double integers and allow only binary arithmetic roundoff.
  // Python's rational timebase validation remains authoritative.
  constexpr double MaxExactNanoseconds = 9007199254740991.0;
  constexpr double RoundoffToleranceNs = 1.e-6;
  if (!FMath::IsFinite(Value) || Value < 1.0 || Value > MaxExactNanoseconds) return false;
  const double Rounded = std::round(Value);
  if (std::abs(Value - Rounded) > RoundoffToleranceNs) return false;
  Out = static_cast<int64>(Rounded);
  return true;
}
}

bool FSensorSelectionModel::ParseSetup(TSharedPtr<FJsonObject> Json, FSensorSetup& OutSetup, FString& OutError) {
  FSensorSetup Parsed;
  FString Schema;
  if (!ReadSensorSetupString(Json, TEXT("schema"), Schema) || Schema != TEXT("lingtu.sim.sensor-setup.v1")) {
    OutError = TEXT("Expected schema lingtu.sim.sensor-setup.v1.");
    return false;
  }
  if (!ReadSensorSetupString(Json, TEXT("instance_id"), Parsed.InstanceId)) {
    OutError = TEXT("instance_id must be a non-empty string.");
    return false;
  }
  const TSharedPtr<FJsonObject>* Rig = nullptr;
  if (!Json->TryGetObjectField(TEXT("rig"), Rig) || !Rig || !Rig->IsValid() ||
      !ReadSensorSetupString(*Rig, TEXT("id"), Parsed.RigId) ||
      !ReadSensorSetupString(*Rig, TEXT("version"), Parsed.RigVersion)) {
    OutError = TEXT("rig must contain non-empty id and version strings.");
    return false;
  }
  const TArray<TSharedPtr<FJsonValue>>* Sensors = nullptr;
  if (!Json->TryGetArrayField(TEXT("sensors"), Sensors) || !Sensors) {
    OutError = TEXT("sensors must be an array.");
    return false;
  }
  TSet<FString> SeenIds;
  for (int32 Index = 0; Index < Sensors->Num(); ++Index) {
    const auto& Value = (*Sensors)[Index];
    const auto Sensor = Value.IsValid() && Value->Type == EJson::Object ? Value->AsObject() : nullptr;
    FSensorChoice Choice;
    if (!ReadSensorSetupString(Sensor, TEXT("id"), Choice.Id) ||
        !ReadSensorSetupString(Sensor, TEXT("package"), Choice.Package) ||
        !ReadSensorSetupString(Sensor, TEXT("parent_frame"), Choice.ParentFrame) ||
        !ReadSensorSetupString(Sensor, TEXT("modality"), Choice.Modality) ||
        !ReadSensorSetupString(Sensor, TEXT("label"), Choice.Label)) {
      OutError = FString::Printf(TEXT("sensors[%d] requires non-empty id, package, parent_frame, modality and label strings."), Index);
      return false;
    }
    if (SeenIds.Contains(Choice.Id)) {
      OutError = FString::Printf(TEXT("Duplicate sensor id: %s."), *Choice.Id);
      return false;
    }
    if (!Sensor->HasTypedField<EJson::Number>(TEXT("frequency_hz")) ||
        !Sensor->TryGetNumberField(TEXT("frequency_hz"), Choice.FrequencyHz) ||
        !FMath::IsFinite(Choice.FrequencyHz) || Choice.FrequencyHz <= 0.0) {
      OutError = FString::Printf(TEXT("sensors[%d].frequency_hz must be a finite positive number."), Index);
      return false;
    }
    if (Sensor->HasField(TEXT("physics_timestep_s")) &&
        (!Sensor->HasTypedField<EJson::Number>(TEXT("physics_timestep_s")) ||
         !Sensor->TryGetNumberField(TEXT("physics_timestep_s"), Choice.PhysicsTimestepS) ||
         !FMath::IsFinite(Choice.PhysicsTimestepS) || Choice.PhysicsTimestepS <= 0.0)) {
      OutError = FString::Printf(TEXT("sensors[%d].physics_timestep_s must be a finite positive number when present."), Index);
      return false;
    }
    if (!Sensor->HasTypedField<EJson::Boolean>(TEXT("required")) ||
        !Sensor->TryGetBoolField(TEXT("required"), Choice.bRequired)) {
      OutError = FString::Printf(TEXT("sensors[%d].required must be a boolean."), Index);
      return false;
    }
    if (Sensor->HasField(TEXT("enabled")) &&
        (!Sensor->HasTypedField<EJson::Boolean>(TEXT("enabled")) ||
         !Sensor->TryGetBoolField(TEXT("enabled"), Choice.bEnabled))) {
      OutError = FString::Printf(TEXT("sensors[%d].enabled must be a boolean when present."), Index);
      return false;
    }
    if (Choice.bRequired && !Choice.bEnabled) {
      OutError = FString::Printf(TEXT("Required sensor %s cannot be disabled."), *Choice.Id);
      return false;
    }
    for (const TCHAR* Field : {TEXT("notice"), TEXT("unavailable_reason")}) {
      FString& Target = FString(Field) == TEXT("notice") ? Choice.Notice : Choice.UnavailableReason;
      if (Sensor->HasField(Field) &&
          (!Sensor->HasTypedField<EJson::String>(Field) || !Sensor->TryGetStringField(Field, Target))) {
        OutError = FString::Printf(TEXT("sensors[%d].%s must be a string when present."), Index, Field);
        return false;
      }
    }
    if (!Choice.UnavailableReason.IsEmpty() && Choice.bEnabled) {
      OutError = FString::Printf(TEXT("sensors[%d].enabled must be false when unavailable_reason is non-empty."), Index);
      return false;
    }
    if (Sensor->HasField(TEXT("fixed_frequency_hz")) &&
        (!ReadFiniteNumber(Sensor, TEXT("fixed_frequency_hz"), Choice.FixedFrequencyHz) ||
         Choice.FixedFrequencyHz <= 0.0 || Choice.FixedFrequencyHz != Choice.FrequencyHz)) {
      OutError = FString::Printf(TEXT("sensors[%d].fixed_frequency_hz must be positive and equal frequency_hz."), Index);
      return false;
    }
    if (Sensor->HasField(TEXT("gnss_fix_available"))) {
      bool bFix = false;
      if (!Sensor->HasTypedField<EJson::Boolean>(TEXT("gnss_fix_available")) ||
          !Sensor->TryGetBoolField(TEXT("gnss_fix_available"), bFix)) {
        OutError = FString::Printf(TEXT("sensors[%d].gnss_fix_available must be a boolean."), Index);
        return false;
      }
      Choice.GnssFixAvailable = bFix;
    }
    if (Sensor->HasField(TEXT("camera_preset"))) {
      if (!ReadSensorSetupString(Sensor, TEXT("camera_preset"), Choice.CameraPreset) || !IsCameraPreset(Choice.CameraPreset)) {
        OutError = FString::Printf(TEXT("sensors[%d].camera_preset must be pinhole, equidistant_fisheye or equirectangular."), Index);
        return false;
      }
      if (!ReadAngleLimits(Sensor, TEXT("pan_limits_deg"), Choice.PanLimitsDeg) ||
          !ReadAngleLimits(Sensor, TEXT("tilt_limits_deg"), Choice.TiltLimitsDeg)) {
        OutError = FString::Printf(TEXT("sensors[%d].pan_limits_deg / tilt_limits_deg must be ordered pairs of finite numbers."), Index);
        return false;
      }
      if (!ReadFiniteNumber(Sensor, TEXT("pan_deg"), Choice.PanDeg) || !InAngleLimits(Choice.PanDeg, Choice.PanLimitsDeg) ||
          !ReadFiniteNumber(Sensor, TEXT("tilt_deg"), Choice.TiltDeg) || !InAngleLimits(Choice.TiltDeg, Choice.TiltLimitsDeg)) {
        OutError = FString::Printf(TEXT("sensors[%d].pan_deg / tilt_deg must be finite numbers within their limits."), Index);
        return false;
      }
    } else if (Sensor->HasField(TEXT("pan_deg")) || Sensor->HasField(TEXT("tilt_deg")) ||
               Sensor->HasField(TEXT("pan_limits_deg")) || Sensor->HasField(TEXT("tilt_limits_deg"))) {
      OutError = FString::Printf(TEXT("sensors[%d].camera_preset is required for camera controls."), Index);
      return false;
    }
    SeenIds.Add(Choice.Id);
    Parsed.Sensors.Add(MoveTemp(Choice));
  }
  OutSetup = MoveTemp(Parsed);
  OutError.Reset();
  return true;
}
void FSensorSelectionModel::Configure(FSensorSetup InSetup) {
  Setup = MoveTemp(InSetup);
  ResetChanges();
}

void FSensorSelectionModel::ResetChanges() {
  EnabledOverrides.Reset();
  RateOverrides.Reset();
  CameraPresetOverrides.Reset();
  PanOverrides.Reset();
  TiltOverrides.Reset();
  GnssFixOverrides.Reset();
}

const FSensorChoice* FSensorSelectionModel::FindSensor(const FString& Id) const {
  return Setup.Sensors.FindByPredicate([&Id](const FSensorChoice& Sensor) { return Sensor.Id == Id; });
}

bool FSensorSelectionModel::HasChanges() const {
  return !EnabledOverrides.IsEmpty() || !RateOverrides.IsEmpty() || !CameraPresetOverrides.IsEmpty() ||
    !PanOverrides.IsEmpty() || !TiltOverrides.IsEmpty() || !GnssFixOverrides.IsEmpty();
}

bool FSensorSelectionModel::IsEnabled(const FString& Id) const {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor) return false;
  if (Sensor->bRequired) return true;
  if (const bool* Override = EnabledOverrides.Find(Id)) return *Override;
  return Sensor->bEnabled;
}

double FSensorSelectionModel::GetRate(const FString& Id) const {
  if (const double* Override = RateOverrides.Find(Id)) return *Override;
  const auto* Sensor = FindSensor(Id);
  return Sensor ? Sensor->FrequencyHz : 0.0;
}

bool FSensorSelectionModel::SetEnabled(const FString& Id, bool bEnabled) {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor || (Sensor->bRequired && !bEnabled) || (bEnabled && !Sensor->UnavailableReason.IsEmpty())) return false;
  if (bEnabled == (Sensor->bRequired || Sensor->bEnabled)) EnabledOverrides.Remove(Id);
  else EnabledOverrides.Add(Id, bEnabled);
  return true;
}

bool FSensorSelectionModel::SetRate(const FString& Id, double FrequencyHz) {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor || !FMath::IsFinite(FrequencyHz) || FrequencyHz <= 0.0) return false;
  if (Sensor->FixedFrequencyHz != 0.0 && FrequencyHz != Sensor->FixedFrequencyHz) return false;
  if (Sensor->PhysicsTimestepS != 0.0) {
    int64 PeriodNs = 0;
    int64 TimestepNs = 0;
    if (!TrySensorSelectionNanoseconds(1.e9 / FrequencyHz, PeriodNs) ||
        !TrySensorSelectionNanoseconds(Sensor->PhysicsTimestepS * 1.e9, TimestepNs) ||
        PeriodNs % TimestepNs != 0) return false;
  }
  if (FrequencyHz == Sensor->FrequencyHz) RateOverrides.Remove(Id);
  else RateOverrides.Add(Id, FrequencyHz);
  return true;
}
bool FSensorSelectionModel::HasCameraControls(const FString& Id) const {
  const auto* Sensor = FindSensor(Id);
  return Sensor && !Sensor->CameraPreset.IsEmpty();
}
FString FSensorSelectionModel::GetCameraPreset(const FString& Id) const {
  if (const auto* Override = CameraPresetOverrides.Find(Id)) return *Override;
  const auto* Sensor = FindSensor(Id);
  return Sensor ? Sensor->CameraPreset : FString();
}
double FSensorSelectionModel::GetPanDeg(const FString& Id) const {
  if (const auto* Override = PanOverrides.Find(Id)) return *Override;
  const auto* Sensor = FindSensor(Id);
  return Sensor ? Sensor->PanDeg : 0.0;
}
double FSensorSelectionModel::GetTiltDeg(const FString& Id) const {
  if (const auto* Override = TiltOverrides.Find(Id)) return *Override;
  const auto* Sensor = FindSensor(Id);
  return Sensor ? Sensor->TiltDeg : 0.0;
}
bool FSensorSelectionModel::SetCameraPreset(const FString& Id, const FString& Preset) {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor || Sensor->CameraPreset.IsEmpty() || !IsCameraPreset(Preset)) return false;
  if (Preset == Sensor->CameraPreset) CameraPresetOverrides.Remove(Id);
  else CameraPresetOverrides.Add(Id, Preset);
  return true;
}
bool FSensorSelectionModel::SetPanDeg(const FString& Id, double Degrees) {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor || Sensor->CameraPreset.IsEmpty() || !InAngleLimits(Degrees, Sensor->PanLimitsDeg)) return false;
  if (Degrees == Sensor->PanDeg) PanOverrides.Remove(Id);
  else PanOverrides.Add(Id, Degrees);
  return true;
}
bool FSensorSelectionModel::SetTiltDeg(const FString& Id, double Degrees) {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor || Sensor->CameraPreset.IsEmpty() || !InAngleLimits(Degrees, Sensor->TiltLimitsDeg)) return false;
  if (Degrees == Sensor->TiltDeg) TiltOverrides.Remove(Id);
  else TiltOverrides.Add(Id, Degrees);
  return true;
}
bool FSensorSelectionModel::HasGnssFixControl(const FString& Id) const {
  const auto* Sensor = FindSensor(Id);
  return Sensor && Sensor->GnssFixAvailable.IsSet();
}
bool FSensorSelectionModel::GetGnssFixAvailable(const FString& Id) const {
  if (const auto* Override = GnssFixOverrides.Find(Id)) return *Override;
  const auto* Sensor = FindSensor(Id);
  return Sensor && Sensor->GnssFixAvailable.Get(false);
}
bool FSensorSelectionModel::SetGnssFixAvailable(const FString& Id, bool bAvailable) {
  const auto* Sensor = FindSensor(Id);
  if (!Sensor || !Sensor->GnssFixAvailable.IsSet()) return false;
  if (bAvailable == Sensor->GnssFixAvailable.GetValue()) GnssFixOverrides.Remove(Id);
  else GnssFixOverrides.Add(Id, bAvailable);
  return true;
}
TSharedPtr<FJsonObject> FSensorSelectionModel::ToSelectionJson() const {
  auto Json = MakeShared<FJsonObject>();
  Json->SetStringField(TEXT("schema"), TEXT("lingtu.sim.sensor-selection.v1"));
  Json->SetStringField(TEXT("instance_id"), Setup.InstanceId);
  TArray<TSharedPtr<FJsonValue>> Sensors;
  for (const auto& Sensor : Setup.Sensors) {
    auto Row = MakeShared<FJsonObject>();
    Row->SetStringField(TEXT("id"), Sensor.Id);
    Row->SetBoolField(TEXT("enabled"), IsEnabled(Sensor.Id));
    Row->SetNumberField(TEXT("frequency_hz"), GetRate(Sensor.Id));
    if (!Sensor.CameraPreset.IsEmpty()) {
      Row->SetStringField(TEXT("camera_preset"), GetCameraPreset(Sensor.Id));
      Row->SetNumberField(TEXT("pan_deg"), GetPanDeg(Sensor.Id));
      Row->SetNumberField(TEXT("tilt_deg"), GetTiltDeg(Sensor.Id));
    }
    if (Sensor.GnssFixAvailable.IsSet()) Row->SetBoolField(TEXT("gnss_fix_available"), GetGnssFixAvailable(Sensor.Id));
    Sensors.Add(MakeShared<FJsonValueObject>(Row));
  }
  Json->SetArrayField(TEXT("sensors"), Sensors);
  return Json;
}
}  // namespace LingTuSim::UI
