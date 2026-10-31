#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimSensorSelection.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Misc/AutomationTest.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "SLingTuSimSensorSelection.h"
#include "Framework/Application/SlateApplication.h"
#include "Input/Events.h"
#include "InputCoreTypes.h"
#include "Layout/Children.h"
#include "Misc/ScopeExit.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/SWindow.h"
#include <limits>

namespace {
using namespace LingTuSim::UI;

TSharedPtr<FJsonObject> SensorSetupFixture() {
  const FString Json = TEXT(R"({"schema":"lingtu.sim.sensor-setup.v1","instance_id":"robot-1",
    "rig":{"id":"inspection","version":"1.2.0"},"sensors":[
    {"id":"imu","package":"imu@1","parent_frame":"base","frequency_hz":200,"physics_timestep_s":0.001,
     "modality":"imu","label":"IMU","required":true},
    {"id":"camera","package":"rgb@1","parent_frame":"head","frequency_hz":30,
     "modality":"rgb","label":"Camera","required":false}]})");
  TSharedPtr<FJsonObject> Result;
  FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), Result);
  return Result;
}

TSharedPtr<FJsonObject> SensorCapabilitiesFixture() {
  auto Json = SensorSetupFixture();
  auto Rows = Json->GetArrayField(TEXT("sensors"));
  auto Camera = Rows[1]->AsObject();
  Camera->SetStringField(TEXT("notice"), TEXT("Startup software camera; not RGBD calibration."));
  Camera->SetStringField(TEXT("camera_preset"), TEXT("pinhole"));
  Camera->SetNumberField(TEXT("pan_deg"), 10.0);
  Camera->SetNumberField(TEXT("tilt_deg"), -5.0);
  Camera->SetArrayField(TEXT("pan_limits_deg"), {MakeShared<FJsonValueNumber>(-180.0), MakeShared<FJsonValueNumber>(180.0)});
  Camera->SetArrayField(TEXT("tilt_limits_deg"), {MakeShared<FJsonValueNumber>(-90.0), MakeShared<FJsonValueNumber>(90.0)});
  const auto AddRow = [&Rows](const TCHAR* Id, const TCHAR* Modality, double Rate) {
    auto Row = MakeShared<FJsonObject>();
    Row->SetStringField(TEXT("id"), Id);
    Row->SetStringField(TEXT("package"), TEXT("sensor@1"));
    Row->SetStringField(TEXT("parent_frame"), TEXT("head"));
    Row->SetStringField(TEXT("modality"), Modality);
    Row->SetStringField(TEXT("label"), Id);
    Row->SetNumberField(TEXT("frequency_hz"), Rate);
    Row->SetBoolField(TEXT("required"), false);
    Rows.Add(MakeShared<FJsonValueObject>(Row));
    return Row;
  };
  AddRow(TEXT("rgbd"), TEXT("rgbd"), 30.0);
  AddRow(TEXT("acoustic"), TEXT("acoustic"), 100.0)->SetNumberField(TEXT("fixed_frequency_hz"), 100.0);
  auto Gnss = AddRow(TEXT("gnss"), TEXT("gnss"), 10.0);
  Gnss->SetBoolField(TEXT("gnss_fix_available"), false);
  Gnss->SetStringField(TEXT("notice"), TEXT("Synthetic WGS84 origin; not measured or RTK."));
  Json->SetArrayField(TEXT("sensors"), Rows);
  return Json;
}

TSharedPtr<SWidget> FindSensorWidget(const TSharedRef<SWidget>& Root, FName Tag) {
  if (Root->GetTag() == Tag) return Root;
  auto* Children = Root->GetChildren();
  for (int32 Index = 0; Children && Index < Children->Num(); ++Index) {
    auto Found = FindSensorWidget(Children->GetChildAt(Index), Tag);
    if (Found.IsValid()) return Found;
  }
  return nullptr;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCameraMetadataTest,
  "LingTuSim.UI.SensorSelection.CameraMetadata",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorCameraMetadataTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!TestTrue(TEXT("capabilities parse"), FSensorSelectionModel::ParseSetup(SensorCapabilitiesFixture(), Setup, Error))) return false;
  TestEqual(TEXT("notice retained"), Setup.Sensors[1].Notice, FString(TEXT("Startup software camera; not RGBD calibration.")));
  TestEqual(TEXT("projection retained"), Setup.Sensors[1].CameraPreset, FString(TEXT("pinhole")));
  TestEqual(TEXT("pan retained"), Setup.Sensors[1].PanDeg, 10.0);
  TestEqual(TEXT("tilt retained"), Setup.Sensors[1].TiltDeg, -5.0);
  TestEqual(TEXT("pan lower limit retained"), Setup.Sensors[1].PanLimitsDeg.X, -180.0);
  TestEqual(TEXT("tilt upper limit retained"), Setup.Sensors[1].TiltLimitsDeg.Y, 90.0);
  const auto Reject = [this, &Setup, &Error](const TSharedPtr<FJsonObject>& Json, const TCHAR* Field) {
    TestFalse(TEXT("invalid capability rejected atomically"), FSensorSelectionModel::ParseSetup(Json, Setup, Error));
    TestTrue(TEXT("diagnostic names field"), Error.Contains(Field));
    TestEqual(TEXT("rejection preserves camera baseline"), Setup.Sensors[1].PanDeg, 10.0);
  };
  for (const TCHAR* Field : {TEXT("pan_deg"), TEXT("tilt_deg"), TEXT("pan_limits_deg"), TEXT("tilt_limits_deg")}) {
    auto Json = SensorCapabilitiesFixture();
    Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->RemoveField(Field);
    Reject(Json, Field);
  }
  for (const auto& Invalid : TArray<TSharedPtr<FJsonValue>>{MakeShared<FJsonValueNull>(),
      MakeShared<FJsonValueBoolean>(true), MakeShared<FJsonValueNumber>(12.0),
      MakeShared<FJsonValueString>(TEXT("perspective"))}) {
    auto Json = SensorCapabilitiesFixture();
    Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetField(TEXT("camera_preset"), Invalid);
    Reject(Json, TEXT("camera_preset"));
  }
  const TArray<TSharedPtr<FJsonValue>> InvalidNumbers = {MakeShared<FJsonValueString>(TEXT("10")),
    MakeShared<FJsonValueBoolean>(true), MakeShared<FJsonValueNull>(),
    MakeShared<FJsonValueNumber>(std::numeric_limits<double>::infinity()),
    MakeShared<FJsonValueNumber>(std::numeric_limits<double>::quiet_NaN()), MakeShared<FJsonValueNumber>(181.0)};
  for (const TCHAR* Field : {TEXT("pan_deg"), TEXT("tilt_deg")}) {
    for (const auto& Invalid : InvalidNumbers) {
      auto Json = SensorCapabilitiesFixture();
      Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetField(Field, Invalid);
      Reject(Json, Field);
    }
  }
  for (const TCHAR* Field : {TEXT("pan_limits_deg"), TEXT("tilt_limits_deg")}) {
    const TArray<TArray<TSharedPtr<FJsonValue>>> InvalidLimits = {{}, {MakeShared<FJsonValueNumber>(0.0)},
      {MakeShared<FJsonValueNumber>(2.0), MakeShared<FJsonValueNumber>(1.0)},
      {MakeShared<FJsonValueString>(TEXT("-180")), MakeShared<FJsonValueNumber>(180.0)},
      {MakeShared<FJsonValueNumber>(-180.0), MakeShared<FJsonValueNumber>(std::numeric_limits<double>::infinity())},
      {MakeShared<FJsonValueNumber>(-180.0), MakeShared<FJsonValueNumber>(180.0), MakeShared<FJsonValueNumber>(0.0)}};
    for (const auto& Limits : InvalidLimits) {
      auto Json = SensorCapabilitiesFixture();
      Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetArrayField(Field, Limits);
      Reject(Json, Field);
    }
  }
  auto Orphan = SensorSetupFixture();
  Orphan->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetNumberField(TEXT("pan_deg"), 0.0);
  Reject(Orphan, TEXT("camera_preset"));
  auto Narrow = SensorCapabilitiesFixture();
  auto NarrowCamera = Narrow->GetArrayField(TEXT("sensors"))[1]->AsObject();
  NarrowCamera->SetArrayField(TEXT("pan_limits_deg"), {MakeShared<FJsonValueNumber>(-20.0), MakeShared<FJsonValueNumber>(20.0)});
  NarrowCamera->SetArrayField(TEXT("tilt_limits_deg"), {MakeShared<FJsonValueNumber>(-5.0), MakeShared<FJsonValueNumber>(-5.0)});
  TestTrue(TEXT("authored narrower and locked-axis limits parse"), FSensorSelectionModel::ParseSetup(Narrow, Setup, Error));
  FSensorSelectionModel NarrowModel;
  NarrowModel.Configure(Setup);
  TestFalse(TEXT("pan obeys authored limits rather than global defaults"), NarrowModel.SetPanDeg(TEXT("camera"), 21.0));
  TestFalse(TEXT("locked tilt rejects other angles"), NarrowModel.SetTiltDeg(TEXT("camera"), 0.0));
  TestTrue(TEXT("locked tilt accepts baseline no-op"), NarrowModel.SetTiltDeg(TEXT("camera"), -5.0));
  TestTrue(TEXT("old fixtures need no capabilities"), FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error));
  TestTrue(TEXT("old camera row stays capability-free"), Setup.Sensors[1].CameraPreset.IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCameraDraftTest,
  "LingTuSim.UI.SensorSelection.CameraDraftAndSerialization",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorCameraDraftTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorCapabilitiesFixture(), Setup, Error)) return false;
  FSensorSelectionModel Model;
  Model.Configure(Setup);
  TestFalse(TEXT("camera baseline starts clean"), Model.HasChanges());
  TestTrue(TEXT("camera advertises controls"), Model.HasCameraControls(TEXT("camera")));
  for (const TCHAR* Id : {TEXT("missing"), TEXT("rgbd"), TEXT("imu")}) {
    TestFalse(TEXT("non-camera does not advertise controls"), Model.HasCameraControls(Id));
    TestTrue(TEXT("non-camera has no projection"), Model.GetCameraPreset(Id).IsEmpty());
    TestFalse(TEXT("non-camera rejects projection"), Model.SetCameraPreset(Id, TEXT("pinhole")));
    TestFalse(TEXT("non-camera rejects pan"), Model.SetPanDeg(Id, 0.0));
    TestFalse(TEXT("non-camera rejects tilt"), Model.SetTiltDeg(Id, 0.0));
  }
  TestFalse(TEXT("unknown projection rejected"), Model.SetCameraPreset(TEXT("camera"), TEXT("other")));
  for (const double Invalid : {181.0, -181.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    TestFalse(TEXT("bad pan cannot dirty a clean model"), Model.SetPanDeg(TEXT("camera"), Invalid));
  for (const double Invalid : {91.0, -91.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    TestFalse(TEXT("bad tilt cannot dirty a clean model"), Model.SetTiltDeg(TEXT("camera"), Invalid));
  TestFalse(TEXT("all rejected edits leave clean model"), Model.HasChanges());
  for (const TCHAR* Preset : {TEXT("equidistant_fisheye"), TEXT("equirectangular"), TEXT("pinhole")}) {
    TestTrue(TEXT("all supported projections accepted"), Model.SetCameraPreset(TEXT("camera"), Preset));
    TestEqual(TEXT("projection getter"), Model.GetCameraPreset(TEXT("camera")), FString(Preset));
  }
  TestFalse(TEXT("restoring projection clears dirty"), Model.HasChanges());
  TestTrue(TEXT("lower pan boundary accepted"), Model.SetPanDeg(TEXT("camera"), -180.0));
  TestTrue(TEXT("upper pan boundary accepted"), Model.SetPanDeg(TEXT("camera"), 180.0));
  TestTrue(TEXT("upper tilt boundary accepted"), Model.SetTiltDeg(TEXT("camera"), 90.0));
  TestTrue(TEXT("lower tilt boundary accepted"), Model.SetTiltDeg(TEXT("camera"), -90.0));
  Model.SetCameraPreset(TEXT("camera"), TEXT("equidistant_fisheye"));
  TestFalse(TEXT("invalid pan does not replace dirty draft"), Model.SetPanDeg(TEXT("camera"), 181.0));
  TestEqual(TEXT("valid pan survives"), Model.GetPanDeg(TEXT("camera")), 180.0);
  TestTrue(TEXT("dirty draft survives invalid edit"), Model.HasChanges());
  const auto Json = Model.ToSelectionJson();
  const auto& Rows = Json->GetArrayField(TEXT("sensors"));
  TestEqual(TEXT("schema unchanged"), Json->GetStringField(TEXT("schema")), FString(TEXT("lingtu.sim.sensor-selection.v1")));
  const auto Camera = Rows[1]->AsObject();
  TestEqual(TEXT("camera emits exactly six intent fields"), Camera->Values.Num(), 6);
  TestEqual(TEXT("serialized projection"), Camera->GetStringField(TEXT("camera_preset")), FString(TEXT("equidistant_fisheye")));
  TestEqual(TEXT("serialized pan"), Camera->GetNumberField(TEXT("pan_deg")), 180.0);
  TestEqual(TEXT("serialized tilt"), Camera->GetNumberField(TEXT("tilt_deg")), -90.0);
  TestEqual(TEXT("RGBD retains original three fields"), Rows[2]->AsObject()->Values.Num(), 3);
  TestFalse(TEXT("notice is not selection intent"), Camera->HasField(TEXT("notice")));
  TestFalse(TEXT("limits are not selection intent"), Camera->HasField(TEXT("pan_limits_deg")));
  TestEqual(TEXT("immutable baseline pan"), Model.GetSetup().Sensors[1].PanDeg, 10.0);
  Model.SetCameraPreset(TEXT("camera"), TEXT("pinhole"));
  Model.SetPanDeg(TEXT("camera"), 10.0);
  Model.SetTiltDeg(TEXT("camera"), -5.0);
  TestFalse(TEXT("restoring all values clears dirty"), Model.HasChanges());
  Model.SetPanDeg(TEXT("camera"), 20.0);
  Model.SetTiltDeg(TEXT("camera"), 15.0);
  Model.SetCameraPreset(TEXT("camera"), TEXT("equirectangular"));
  Model.SetEnabled(TEXT("camera"), false);
  Model.SetRate(TEXT("imu"), 100.0);
  Model.SetGnssFixAvailable(TEXT("gnss"), true);
  Model.ResetChanges();
  TestFalse(TEXT("reset clears every override category"), Model.HasChanges());
  TestEqual(TEXT("reset pan"), Model.GetPanDeg(TEXT("camera")), 10.0);
  TestEqual(TEXT("reset tilt"), Model.GetTiltDeg(TEXT("camera")), -5.0);
  TestEqual(TEXT("reset projection"), Model.GetCameraPreset(TEXT("camera")), FString(TEXT("pinhole")));
  TestTrue(TEXT("reset enabled state"), Model.IsEnabled(TEXT("camera")));
  TestEqual(TEXT("reset rate"), Model.GetRate(TEXT("imu")), 200.0);
  TestFalse(TEXT("reset fix"), Model.GetGnssFixAvailable(TEXT("gnss")));
  Model.SetPanDeg(TEXT("camera"), 25.0);
  Model.Configure(Setup);
  TestFalse(TEXT("reconfigure clears camera draft"), Model.HasChanges());
  Model.SetEnabled(TEXT("camera"), false);
  const auto DisabledSelection = Model.ToSelectionJson();
  const auto DisabledCamera = DisabledSelection->GetArrayField(TEXT("sensors"))[1]->AsObject();
  TestFalse(TEXT("disabled camera remains disabled in intent"), DisabledCamera->GetBoolField(TEXT("enabled")));
  TestEqual(TEXT("disabled camera retains capability intent fields"), DisabledCamera->Values.Num(), 6);
  TestEqual(TEXT("disabled camera retains authored pan"), DisabledCamera->GetNumberField(TEXT("pan_deg")), 10.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorAvailabilityGnssAndFixedRateTest,
  "LingTuSim.UI.SensorSelection.AvailabilityGnssAndFixedRate",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorAvailabilityGnssAndFixedRateTest::RunTest(const FString&) {
  auto Json = SensorCapabilitiesFixture();
  auto Camera = Json->GetArrayField(TEXT("sensors"))[1]->AsObject();
  Camera->SetStringField(TEXT("unavailable_reason"), TEXT("World has no measurements."));
  Camera->SetBoolField(TEXT("enabled"), false);
  FSensorSetup Setup;
  FString Error;
  if (!TestTrue(TEXT("disabled unavailable sensor is valid"), FSensorSelectionModel::ParseSetup(Json, Setup, Error))) return false;
  FSensorSelectionModel Model;
  Model.Configure(Setup);
  TestFalse(TEXT("unavailable sensor cannot be enabled"), Model.SetEnabled(TEXT("camera"), true));
  TestFalse(TEXT("unavailable rejection leaves clean state"), Model.HasChanges());
  TestFalse(TEXT("unavailable sensor stays authored disabled"), Model.IsEnabled(TEXT("camera")));
  TestTrue(TEXT("disabled no-op is accepted"), Model.SetEnabled(TEXT("camera"), false));
  TestTrue(TEXT("fixed block rate no-op is accepted"), Model.SetRate(TEXT("acoustic"), 100.0));
  TestFalse(TEXT("fixed acoustic rate cannot be changed"), Model.SetRate(TEXT("acoustic"), 50.0));
  TestEqual(TEXT("acoustic block rate stays 100"), Model.GetRate(TEXT("acoustic")), 100.0);
  TestFalse(TEXT("fixed rate rejection remains clean"), Model.HasChanges());
  TestTrue(TEXT("GNSS capability retained"), Model.HasGnssFixControl(TEXT("gnss")));
  TestFalse(TEXT("authored no-fix baseline"), Model.GetGnssFixAvailable(TEXT("gnss")));
  TestFalse(TEXT("non-GNSS cannot acquire fix control"), Model.SetGnssFixAvailable(TEXT("camera"), true));
  TestFalse(TEXT("unknown fix cannot be edited"), Model.SetGnssFixAvailable(TEXT("missing"), true));
  TestTrue(TEXT("simulated fix can be enabled"), Model.SetGnssFixAvailable(TEXT("gnss"), true));
  TestTrue(TEXT("fix change is dirty"), Model.HasChanges());
  const auto Selection = Model.ToSelectionJson();
  const auto& SelectionRows = Selection->GetArrayField(TEXT("sensors"));
  TestTrue(TEXT("fix configuration is serialized"), SelectionRows[4]->AsObject()->GetBoolField(TEXT("gnss_fix_available")));
  TestEqual(TEXT("only GNSS adds fix intent"), SelectionRows[4]->AsObject()->Values.Num(), 4);
  TestFalse(TEXT("RGBD never acquires a fix field"), SelectionRows[2]->AsObject()->HasField(TEXT("gnss_fix_available")));
  TestFalse(TEXT("fixed constraint is not intent"), SelectionRows[3]->AsObject()->HasField(TEXT("fixed_frequency_hz")));
  Model.SetGnssFixAvailable(TEXT("gnss"), false);
  TestFalse(TEXT("restoring fix baseline clears dirty"), Model.HasChanges());
  Model.SetGnssFixAvailable(TEXT("gnss"), true);
  Model.ResetChanges();
  TestFalse(TEXT("reset restores no-fix baseline"), Model.GetGnssFixAvailable(TEXT("gnss")));
  TestFalse(TEXT("reset clears fix dirty"), Model.HasChanges());
  Model.SetGnssFixAvailable(TEXT("gnss"), true);
  Model.Configure(Setup);
  TestFalse(TEXT("reconfigure clears fix overrides"), Model.GetGnssFixAvailable(TEXT("gnss")));
  Setup.Sensors[4].GnssFixAvailable = true;
  Model.Configure(Setup);
  TestTrue(TEXT("authored fix true baseline retained"), Model.GetGnssFixAvailable(TEXT("gnss")));
  Model.SetGnssFixAvailable(TEXT("gnss"), false);
  TestTrue(TEXT("disabling authored fix is dirty"), Model.HasChanges());
  TestFalse(TEXT("false fix intent is explicit"), Model.ToSelectionJson()->GetArrayField(TEXT("sensors"))[4]->AsObject()->GetBoolField(TEXT("gnss_fix_available")));
  Model.SetGnssFixAvailable(TEXT("gnss"), true);
  TestFalse(TEXT("restoring authored true fix cleans draft"), Model.HasChanges());
  Camera->SetBoolField(TEXT("enabled"), true);
  TestFalse(TEXT("setup cannot silently disable enabled unavailable row"), FSensorSelectionModel::ParseSetup(Json, Setup, Error));
  TestTrue(TEXT("blocked setup diagnostic"), Error.Contains(TEXT("unavailable_reason")));
  TestFalse(TEXT("failed parse leaves disabled baseline unchanged"), Setup.Sensors[1].bEnabled);
  const TArray<TSharedPtr<FJsonValue>> WrongTypes = {MakeShared<FJsonValueNull>(), MakeShared<FJsonValueNumber>(1.0),
    MakeShared<FJsonValueString>(TEXT("false")), MakeShared<FJsonValueBoolean>(true)};
  for (const TCHAR* Field : {TEXT("notice"), TEXT("unavailable_reason"), TEXT("gnss_fix_available"), TEXT("fixed_frequency_hz")}) {
    for (const auto& Wrong : WrongTypes) {
      if ((FString(Field) == TEXT("notice") || FString(Field) == TEXT("unavailable_reason")) && Wrong->Type == EJson::String) continue;
      if (FString(Field) == TEXT("gnss_fix_available") && Wrong->Type == EJson::Boolean) continue;
      auto Invalid = SensorCapabilitiesFixture();
      Invalid->GetArrayField(TEXT("sensors"))[4]->AsObject()->SetField(Field, Wrong);
      TestFalse(TEXT("optional metadata requires exact JSON types and compatible rates"), FSensorSelectionModel::ParseSetup(Invalid, Setup, Error));
      TestTrue(TEXT("optional metadata diagnostic names field"), Error.Contains(Field));
    }
  }
  for (const double Invalid : {0.0, -100.0, 50.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    auto Bad = SensorCapabilitiesFixture();
    Bad->GetArrayField(TEXT("sensors"))[3]->AsObject()->SetNumberField(TEXT("fixed_frequency_hz"), Invalid);
    TestFalse(TEXT("fixed rate must be finite positive and match baseline"), FSensorSelectionModel::ParseSetup(Bad, Setup, Error));
  }
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  Model.Configure(Setup);
  TestFalse(TEXT("legacy fixture has no GNSS capability"), Model.HasGnssFixControl(TEXT("camera")));
  TestFalse(TEXT("legacy row has no synthesized fix intent"), Model.ToSelectionJson()->GetArrayField(TEXT("sensors"))[1]->AsObject()->HasField(TEXT("gnss_fix_available")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSetupParseTest,
  "LingTuSim.UI.SensorSelection.ParseSetup",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorSetupParseTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error = TEXT("old error");
  if (!TestTrue(TEXT("valid setup parses"), FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error))) return false;
  TestTrue(TEXT("success clears stale errors"), Error.IsEmpty());
  TestEqual(TEXT("instance is preserved"), Setup.InstanceId, FString(TEXT("robot-1")));
  TestEqual(TEXT("rig ID is preserved"), Setup.RigId, FString(TEXT("inspection")));
  TestEqual(TEXT("rig version is preserved"), Setup.RigVersion, FString(TEXT("1.2.0")));
  if (!TestEqual(TEXT("both sensors are retained in source order"), Setup.Sensors.Num(), 2)) return false;
  TestEqual(TEXT("stable sensor ID"), Setup.Sensors[0].Id, FString(TEXT("imu")));
  TestEqual(TEXT("package"), Setup.Sensors[0].Package, FString(TEXT("imu@1")));
  TestEqual(TEXT("parent frame"), Setup.Sensors[0].ParentFrame, FString(TEXT("base")));
  TestEqual(TEXT("modality"), Setup.Sensors[0].Modality, FString(TEXT("imu")));
  TestEqual(TEXT("label"), Setup.Sensors[0].Label, FString(TEXT("IMU")));
  TestEqual(TEXT("frequency"), Setup.Sensors[0].FrequencyHz, 200.0);
  TestEqual(TEXT("physics timestep is retained"), Setup.Sensors[0].PhysicsTimestepS, 0.001);
  TestEqual(TEXT("visual rows have no physics timebase"), Setup.Sensors[1].PhysicsTimestepS, 0.0);
  TestTrue(TEXT("required is retained"), Setup.Sensors[0].bRequired);
  TestFalse(TEXT("optional stays optional"), Setup.Sensors[1].bRequired);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorPhysicsTimestepMetadataTest,
  "LingTuSim.UI.SensorSelection.PhysicsTimestepMetadata",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorPhysicsTimestepMetadataTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  const TArray<TSharedPtr<FJsonValue>> InvalidValues = {
    MakeShared<FJsonValueNumber>(0.0), MakeShared<FJsonValueNumber>(-0.001),
    MakeShared<FJsonValueNumber>(std::numeric_limits<double>::infinity()),
    MakeShared<FJsonValueNumber>(std::numeric_limits<double>::quiet_NaN()),
    MakeShared<FJsonValueString>(TEXT("0.001")), MakeShared<FJsonValueBoolean>(true),
    MakeShared<FJsonValueNull>()
  };
  for (const auto& Invalid : InvalidValues) {
    auto Json = SensorSetupFixture();
    Json->GetArrayField(TEXT("sensors"))[0]->AsObject()->SetField(TEXT("physics_timestep_s"), Invalid);
    TestFalse(TEXT("present physics timestep must be finite positive numeric data"),
      FSensorSelectionModel::ParseSetup(Json, Setup, Error));
    TestTrue(TEXT("invalid timestep identifies the field"), Error.Contains(TEXT("physics_timestep_s")));
    TestEqual(TEXT("invalid timestep leaves the previous setup unchanged"), Setup.Sensors[0].PhysicsTimestepS, 0.001);
  }
  auto Legacy = SensorSetupFixture();
  Legacy->GetArrayField(TEXT("sensors"))[0]->AsObject()->RemoveField(TEXT("physics_timestep_s"));
  TestTrue(TEXT("physics timestep remains optional"), FSensorSelectionModel::ParseSetup(Legacy, Setup, Error));
  TestEqual(TEXT("omitted timestep has the zero sentinel"), Setup.Sensors[0].PhysicsTimestepS, 0.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionChangesTest,
  "LingTuSim.UI.SensorSelection.ChangesAndRequiredProtection",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorSelectionChangesTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  FSensorSelectionModel Model;
  Model.Configure(Setup);
  TestFalse(TEXT("configure starts clean"), Model.HasChanges());
  TestTrue(TEXT("optional sensors start enabled"), Model.IsEnabled(TEXT("camera")));
  TestTrue(TEXT("required sensors start enabled"), Model.IsEnabled(TEXT("imu")));
  TestFalse(TEXT("required sensor cannot be disabled"), Model.SetEnabled(TEXT("imu"), false));
  TestTrue(TEXT("required sensor remains enabled"), Model.IsEnabled(TEXT("imu")));
  TestFalse(TEXT("rejection leaves baseline clean"), Model.HasChanges());
  TestTrue(TEXT("valid no-op succeeds"), Model.SetEnabled(TEXT("imu"), true));
  TestTrue(TEXT("optional sensor can be disabled"), Model.SetEnabled(TEXT("camera"), false));
  TestFalse(TEXT("disabled state is observable"), Model.IsEnabled(TEXT("camera")));
  TestTrue(TEXT("toggle marks a change"), Model.HasChanges());
  TestTrue(TEXT("restoring enabled state succeeds"), Model.SetEnabled(TEXT("camera"), true));
  TestFalse(TEXT("restoring baseline clears changes"), Model.HasChanges());
  TestFalse(TEXT("unknown ID is not enabled"), Model.IsEnabled(TEXT("missing")));
  TestFalse(TEXT("unknown ID cannot be enabled"), Model.SetEnabled(TEXT("missing"), true));
  TestFalse(TEXT("unknown ID cannot be disabled"), Model.SetEnabled(TEXT("missing"), false));
  TestEqual(TEXT("unknown ID has no rate"), Model.GetRate(TEXT("missing")), 0.0);
  TestFalse(TEXT("unknown ID cannot receive a rate"), Model.SetRate(TEXT("missing"), 20.0));
  for (const double Invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
    TestFalse(TEXT("invalid frequency is rejected"), Model.SetRate(TEXT("camera"), Invalid));
    TestEqual(TEXT("invalid frequency does not mutate state"), Model.GetRate(TEXT("camera")), 30.0);
  }
  TestTrue(TEXT("positive fractional frequency is accepted"), Model.SetRate(TEXT("camera"), 12.5));
  TestEqual(TEXT("changed frequency is observable"), Model.GetRate(TEXT("camera")), 12.5);
  TestTrue(TEXT("frequency marks changes"), Model.HasChanges());
  TestEqual(TEXT("setup remains the immutable baseline"), Model.GetSetup().Sensors[1].FrequencyHz, 30.0);
  TestTrue(TEXT("restoring frequency succeeds"), Model.SetRate(TEXT("camera"), 30.0));
  TestFalse(TEXT("restoring frequency clears changes"), Model.HasChanges());
  TestTrue(TEXT("required sensor rate remains configurable"), Model.SetRate(TEXT("imu"), 100.0));
  Model.SetEnabled(TEXT("camera"), false);
  Model.Configure(Setup);
  TestFalse(TEXT("reconfigure discards previous edits"), Model.HasChanges());
  TestTrue(TEXT("reconfigure enables optional sensors"), Model.IsEnabled(TEXT("camera")));
  TestEqual(TEXT("reconfigure restores rates"), Model.GetRate(TEXT("imu")), 200.0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorPhysicsRatePreflightTest,
  "LingTuSim.UI.SensorSelection.PhysicsRatePreflight",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorPhysicsRatePreflightTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  FSensorSelectionModel Model;
  Model.Configure(Setup);
  TestFalse(TEXT("30 Hz cannot enter a clean physics draft"), Model.SetRate(TEXT("imu"), 30.0));
  TestFalse(TEXT("a rejected rate leaves the baseline clean"), Model.HasChanges());
  for (const double Valid : {200.0, 100.0, 125.0}) {
    TestTrue(TEXT("integer periods aligned with the 1 ms step are accepted"), Model.SetRate(TEXT("imu"), Valid));
    TestEqual(TEXT("aligned rate is retained"), Model.GetRate(TEXT("imu")), Valid);
  }
  for (const double Invalid : {30.0, 400.0, 0.0, -1.0,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::max(), std::numeric_limits<double>::min(),
      1.e-12, 1.e9 / 9007199254740992.0, 1.e9 / 5000000.0001}) {
    TestFalse(TEXT("incompatible or out-of-range physics rate is rejected"), Model.SetRate(TEXT("imu"), Invalid));
    TestEqual(TEXT("rejection preserves the previous valid draft"), Model.GetRate(TEXT("imu")), 125.0);
    TestTrue(TEXT("rejection preserves the existing dirty state"), Model.HasChanges());
  }
  const auto PhysicsJson = Model.ToSelectionJson()->GetArrayField(TEXT("sensors"))[0]->AsObject();
  TestEqual(TEXT("rejected rates are never serialized"), PhysicsJson->GetNumberField(TEXT("frequency_hz")), 125.0);
  TestFalse(TEXT("timebase metadata is not user intent"), PhysicsJson->HasField(TEXT("physics_timestep_s")));
  for (const double VisualRate : {15.0, 30.0}) {
    TestTrue(TEXT("visual rate does not need physics alignment"), Model.SetRate(TEXT("camera"), VisualRate));
    TestEqual(TEXT("visual rate is retained"), Model.GetRate(TEXT("camera")), VisualRate);
  }
  TestTrue(TEXT("restoring the physics baseline succeeds"), Model.SetRate(TEXT("imu"), 200.0));
  TestFalse(TEXT("restoring both baselines clears the draft"), Model.HasChanges());

  // 1250 ns can pick up a tiny binary floating-point multiplication error.
  Setup.Sensors[0].PhysicsTimestepS = 0.00000125;
  Model.Configure(Setup);
  TestTrue(TEXT("sub-nanosecond floating roundoff does not reject an aligned period"), Model.SetRate(TEXT("imu"), 125.0));
  for (const double InvalidStep : {0.0010000001, 0.0000000005,
      9007199254740992.0 / 1.e9, std::numeric_limits<double>::max()}) {
    Setup.Sensors[0].PhysicsTimestepS = InvalidStep;
    Model.Configure(Setup);
    TestFalse(TEXT("fractional or unsafe timestep nanoseconds cannot be converted"), Model.SetRate(TEXT("imu"), 100.0));
    TestEqual(TEXT("invalid timestep leaves the baseline rate intact"), Model.GetRate(TEXT("imu")), 200.0);
    TestFalse(TEXT("invalid timestep does not create a draft"), Model.HasChanges());
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionJsonTest,
  "LingTuSim.UI.SensorSelection.SelectionJson",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorSelectionJsonTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  FSensorSelectionModel Model;
  Model.Configure(Setup);
  Model.SetEnabled(TEXT("camera"), false);
  Model.SetRate(TEXT("camera"), 12.5);
  const auto Json = Model.ToSelectionJson();
  TestEqual(TEXT("selection schema"), Json->GetStringField(TEXT("schema")), FString(TEXT("lingtu.sim.sensor-selection.v1")));
  TestEqual(TEXT("selection instance"), Json->GetStringField(TEXT("instance_id")), Setup.InstanceId);
  TestEqual(TEXT("only selection envelope fields are emitted"), Json->Values.Num(), 3);
  const auto& Sensors = Json->GetArrayField(TEXT("sensors"));
  if (!TestEqual(TEXT("disabled sensors are included"), Sensors.Num(), 2)) return false;
  const auto Required = Sensors[0]->AsObject();
  const auto Optional = Sensors[1]->AsObject();
  TestEqual(TEXT("source order and stable ID are preserved"), Required->GetStringField(TEXT("id")), FString(TEXT("imu")));
  TestTrue(TEXT("required sensor is enabled"), Required->GetBoolField(TEXT("enabled")));
  TestEqual(TEXT("required rate"), Required->GetNumberField(TEXT("frequency_hz")), 200.0);
  TestEqual(TEXT("optional stable ID"), Optional->GetStringField(TEXT("id")), FString(TEXT("camera")));
  TestFalse(TEXT("optional disabled state is serialized"), Optional->GetBoolField(TEXT("enabled")));
  TestEqual(TEXT("disabled sensor retains edited rate"), Optional->GetNumberField(TEXT("frequency_hz")), 12.5);
  TestEqual(TEXT("only selection row fields are emitted"), Optional->Values.Num(), 3);
  TestTrue(TEXT("serialization does not reset dirty state"), Model.HasChanges());
  Optional->SetBoolField(TEXT("enabled"), true);
  TestFalse(TEXT("returned JSON does not alias model state"), Model.IsEnabled(TEXT("camera")));
  Model.Configure(FSensorSetup());
  TestTrue(TEXT("empty setup has an empty selection"), Model.ToSelectionJson()->GetArrayField(TEXT("sensors")).IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionAuthoredDefaultsTest,
  "LingTuSim.UI.SensorSelection.AuthoredEnabledDefaults",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorSelectionAuthoredDefaultsTest::RunTest(const FString&) {
  auto Json = SensorSetupFixture();
  Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetBoolField(TEXT("enabled"), false);
  FSensorSetup Setup;
  FString Error;
  if (!TestTrue(TEXT("previously authored disabled optional sensor parses"), FSensorSelectionModel::ParseSetup(Json, Setup, Error))) return false;
  TestTrue(TEXT("omitted enabled defaults to true"), Setup.Sensors[0].bEnabled);
  TestFalse(TEXT("authored disabled default is preserved"), Setup.Sensors[1].bEnabled);
  FSensorSelectionModel Model;
  Model.Configure(Setup);
  TestFalse(TEXT("authored disabled sensor starts disabled"), Model.IsEnabled(TEXT("camera")));
  TestFalse(TEXT("authored defaults start clean"), Model.HasChanges());
  TestFalse(TEXT("initial selection retains authored disabled state"), Model.ToSelectionJson()->GetArrayField(TEXT("sensors"))[1]->AsObject()->GetBoolField(TEXT("enabled")));
  TestTrue(TEXT("disabled default may be enabled"), Model.SetEnabled(TEXT("camera"), true));
  TestTrue(TEXT("enabling a disabled default marks changes"), Model.HasChanges());
  TestTrue(TEXT("disabled default may be restored"), Model.SetEnabled(TEXT("camera"), false));
  TestFalse(TEXT("restoring disabled default clears changes"), Model.HasChanges());
  Model.SetEnabled(TEXT("camera"), true);
  Model.SetRate(TEXT("camera"), 15.0);
  Model.Configure(Setup);
  TestFalse(TEXT("reopen restores authored enabled state"), Model.IsEnabled(TEXT("camera")));
  TestEqual(TEXT("reopen restores authored frequency"), Model.GetRate(TEXT("camera")), 30.0);
  TestFalse(TEXT("reopen is clean"), Model.HasChanges());
  Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetStringField(TEXT("enabled"), TEXT("false"));
  TestFalse(TEXT("enabled must be a JSON boolean when present"), FSensorSelectionModel::ParseSetup(Json, Setup, Error));
  Json = SensorSetupFixture();
  Json->GetArrayField(TEXT("sensors"))[0]->AsObject()->SetBoolField(TEXT("enabled"), false);
  TestFalse(TEXT("authored state cannot disable a required sensor"), FSensorSelectionModel::ParseSetup(Json, Setup, Error));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSetupRejectInvalidTest,
  "LingTuSim.UI.SensorSelection.RejectInvalidSetupAtomically",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorSetupRejectInvalidTest::RunTest(const FString&) {
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  const auto Reject = [this, &Setup, &Error](const TSharedPtr<FJsonObject>& Json) {
    TestFalse(TEXT("malformed setup is rejected"), FSensorSelectionModel::ParseSetup(Json, Setup, Error));
    TestFalse(TEXT("failure has a diagnostic"), Error.IsEmpty());
    TestEqual(TEXT("failure preserves previous instance"), Setup.InstanceId, FString(TEXT("robot-1")));
    TestEqual(TEXT("failure preserves previous rig"), Setup.RigId, FString(TEXT("inspection")));
    if (TestEqual(TEXT("failure preserves previous sensor count"), Setup.Sensors.Num(), 2))
      TestEqual(TEXT("failure preserves previous frequency"), Setup.Sensors[1].FrequencyHz, 30.0);
  };
  Reject(nullptr);
  for (const TCHAR* Field : {TEXT("schema"), TEXT("instance_id"), TEXT("rig"), TEXT("sensors")}) {
    auto Json = SensorSetupFixture();
    Json->RemoveField(Field);
    Reject(Json);
  }
  auto Json = SensorSetupFixture();
  Json->SetStringField(TEXT("schema"), TEXT("lingtu.sim.sensor-selection.v1"));
  Reject(Json);
  Json = SensorSetupFixture();
  Json->GetObjectField(TEXT("rig"))->SetStringField(TEXT("version"), TEXT(" "));
  Reject(Json);
  Json = SensorSetupFixture();
  Json->SetNumberField(TEXT("instance_id"), 42);
  Reject(Json);
  for (const TCHAR* Field : {TEXT("id"), TEXT("package"), TEXT("parent_frame"), TEXT("modality"), TEXT("label"), TEXT("frequency_hz"), TEXT("required")}) {
    Json = SensorSetupFixture();
    Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->RemoveField(Field);
    Reject(Json);
  }
  Json = SensorSetupFixture();
  Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetStringField(TEXT("id"), TEXT("imu"));
  Reject(Json);
  for (const double Invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    Json = SensorSetupFixture();
    Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetNumberField(TEXT("frequency_hz"), Invalid);
    Reject(Json);
  }
  Json = SensorSetupFixture();
  Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetStringField(TEXT("frequency_hz"), TEXT("30"));
  Reject(Json);
  Json = SensorSetupFixture();
  Json->GetArrayField(TEXT("sensors"))[1]->AsObject()->SetStringField(TEXT("required"), TEXT("true"));
  Reject(Json);
  Json = SensorSetupFixture();
  Json->SetArrayField(TEXT("sensors"), {MakeShared<FJsonValueNull>()});
  Reject(Json);
  Json = SensorSetupFixture();
  Json->SetArrayField(TEXT("sensors"), {});
  TestTrue(TEXT("a rig without selectable sensors is valid"), FSensorSelectionModel::ParseSetup(Json, Setup, Error));
  TestTrue(TEXT("empty sensor list is preserved"), Setup.Sensors.IsEmpty());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionFocusTest,
  "LingTuSim.UI.SensorSelection.WidgetFocusAndEditing",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorSelectionFocusTest::RunTest(const FString&) {
  if (!FSlateApplication::IsInitialized()) {
    AddError(TEXT("Slate must be initialized to verify focus."));
    return false;
  }
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(SensorSetupFixture(), Setup, Error)) return false;
  // Unknown modality deliberately exercises the missing-image placeholder.
  Setup.Sensors[1].Modality = TEXT("sensor-selection-test-missing");
  auto Model = MakeShared<FSensorSelectionModel>();
  Model->Configure(Setup);
  int32 Changes = 0;
  int32 Closes = 0;
  auto Panel = SNew(SLingTuSimSensorSelection).Model(Model)
    .OnChanged_Lambda([&Changes]() { ++Changes; }).OnClose_Lambda([&Closes]() { ++Closes; });
  const auto FindTag = [](const TSharedRef<SWidget>& Root, FName Tag) {
    TFunction<TSharedPtr<SWidget>(TSharedRef<SWidget>)> Find;
    Find = [&Find, Tag](TSharedRef<SWidget> Widget) -> TSharedPtr<SWidget> {
      if (Widget->GetTag() == Tag) return Widget;
      auto* Children = Widget->GetChildren();
      for (int32 Index = 0; Children && Index < Children->Num(); ++Index) {
        auto Found = Find(Children->GetChildAt(Index));
        if (Found.IsValid()) return Found;
      }
      return nullptr;
    };
    return Find(Root);
  };
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool bWasActive = App.IsActive();
  auto Window = SNew(SWindow).ClientSize(FVector2D(600, 420))[Panel];
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(bWasActive);
    App.RequestDestroyWindow(Window);
  };
  Panel->SlatePrepass(1.F);
  const auto Required = FindTag(Panel, TEXT("LingTu.Sensors.Enabled.imu"));
  const auto Optional = FindTag(Panel, TEXT("LingTu.Sensors.Enabled.camera"));
  const auto Rate = FindTag(Panel, TEXT("LingTu.Sensors.Rate.camera"));
  const auto Close = FindTag(Panel, TEXT("LingTu.Sensors.Close"));
  if (!TestTrue(TEXT("each row has a checkbox and frequency control"), Required.IsValid() && Optional.IsValid() && Rate.IsValid() && Close.IsValid())) return false;
  TestFalse(TEXT("required checkbox is protected"), Required->IsEnabled());
  TestTrue(TEXT("optional checkbox is editable"), Optional->IsEnabled());
  auto InitialError = FindTag(Panel, TEXT("LingTu.Sensors.Error.imu"));
  TestTrue(TEXT("valid initial frequency has no visible error icon"), InitialError.IsValid() && InitialError->GetVisibility() == EVisibility::Collapsed);
  TestTrue(TEXT("missing image has an explicit placeholder"), FindTag(Panel, TEXT("LingTu.Sensors.Placeholder.camera")).IsValid());
  Panel->FocusInitialControl();
  TestTrue(TEXT("initial focus skips protected required checkbox"), Optional->HasKeyboardFocus());
  TestFalse(TEXT("checkbox focus is not text editing"), Panel->IsTextEntryEditing());
  App.SetKeyboardFocus(Rate, EFocusCause::SetDirectly);
  TestTrue(TEXT("inner editable text focus is recognized"), Panel->IsTextEntryEditing());
  // Real Slate text events exercise the commit binding, not a private model callback.
  for (const TCHAR Character : FString(TEXT("12.5")))
    App.ProcessKeyCharEvent(FCharacterEvent(Character, FModifierKeysState(), 0, false));
  App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  TestEqual(TEXT("committed frequency reaches the model"), Model->GetRate(TEXT("camera")), 12.5);
  TestEqual(TEXT("one actual edit fires OnChanged once"), Changes, 1);
  App.SetKeyboardFocus(Close, EFocusCause::SetDirectly);
  TestFalse(TEXT("leaving text entry clears editing state"), Panel->IsTextEntryEditing());
  App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  TestEqual(TEXT("close only delegates to the parent"), Closes, 1);
  TestEqual(TEXT("unchanged blur does not duplicate OnChanged"), Changes, 1);
  const auto RateBox = StaticCastSharedPtr<SEditableTextBox>(Rate);
  for (const TCHAR* Invalid : {TEXT("0"), TEXT("-1"), TEXT("nan"), TEXT("inf"), TEXT("12oops"), TEXT("")}) {
    App.SetKeyboardFocus(Rate, EFocusCause::SetDirectly);
    RateBox->SetText(FText::FromString(Invalid));
    App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
    App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
    TestEqual(TEXT("invalid text never changes the model"), Model->GetRate(TEXT("camera")), 12.5);
    TestEqual(TEXT("invalid text never fires OnChanged"), Changes, 1);
  }
  RateBox->SetText(FText::FromString(TEXT("12.5")));
  App.SetKeyboardFocus(Optional, EFocusCause::SetDirectly);
  StaticCastSharedPtr<SCheckBox>(Optional)->ToggleCheckedState();
  Rate->MarkPrepassAsDirty(); Rate->SlatePrepass(1.F);
  TestFalse(TEXT("checkbox edits the model"), Model->IsEnabled(TEXT("camera")));
  TestFalse(TEXT("disabled sensor frequency cannot be edited"), Rate->IsEnabled());
  TestEqual(TEXT("toggle fires OnChanged once"), Changes, 2);
  StaticCastSharedPtr<SCheckBox>(Required)->ToggleCheckedState();
  TestTrue(TEXT("model also protects required checkbox callback"), Model->IsEnabled(TEXT("imu")));
  TestEqual(TEXT("protected callback does not fire OnChanged"), Changes, 2);
  Setup.Sensors[1].bEnabled = false;
  auto AuthoredModel = MakeShared<FSensorSelectionModel>();
  AuthoredModel->Configure(Setup);
  auto AuthoredPanel = SNew(SLingTuSimSensorSelection).Model(AuthoredModel);
  AuthoredPanel->SlatePrepass(1.F);
  const auto AuthoredToggle = StaticCastSharedPtr<SCheckBox>(FindTag(AuthoredPanel, TEXT("LingTu.Sensors.Enabled.camera")));
  const auto AuthoredRate = FindTag(AuthoredPanel, TEXT("LingTu.Sensors.Rate.camera"));
  TestTrue(TEXT("reopened optional checkbox reflects authored disabled state"), AuthoredToggle.IsValid() && !AuthoredToggle->IsChecked());
  TestTrue(TEXT("reopened disabled sensor frequency starts non-editable"), AuthoredRate.IsValid() && !AuthoredRate->IsEnabled());
  Setup.Sensors.SetNum(1);
  auto RequiredOnlyModel = MakeShared<FSensorSelectionModel>();
  RequiredOnlyModel->Configure(Setup);
  int32 PhysicsChanges = 0;
  auto RequiredOnly = SNew(SLingTuSimSensorSelection).Model(RequiredOnlyModel)
    .OnChanged_Lambda([&PhysicsChanges]() { ++PhysicsChanges; });
  Window->SetContent(RequiredOnly);
  RequiredOnly->SlatePrepass(1.F);
  RequiredOnly->FocusInitialControl();
  TestTrue(TEXT("all-required setup focuses an editable rate"), RequiredOnly->IsTextEntryEditing());
  const auto PhysicsRateBox = StaticCastSharedPtr<SEditableTextBox>(FindTag(RequiredOnly, TEXT("LingTu.Sensors.Rate.imu")));
  if (!TestTrue(TEXT("physics rate control exists"), PhysicsRateBox.IsValid())) return false;
  PhysicsRateBox->SetText(FText::FromString(TEXT("30")));
  App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  TestTrue(TEXT("physics incompatibility is shown at commit"), PhysicsRateBox->HasError());
  TestEqual(TEXT("30 Hz never enters the physics draft"), RequiredOnlyModel->GetRate(TEXT("imu")), 200.0);
  TestFalse(TEXT("rejected physics edit keeps the draft clean"), RequiredOnlyModel->HasChanges());
  TestEqual(TEXT("rejected physics edit never notifies the parent"), PhysicsChanges, 0);
  PhysicsRateBox->SetText(FText::FromString(TEXT("100")));
  App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  TestFalse(TEXT("aligned physics rate clears the error"), PhysicsRateBox->HasError());
  TestEqual(TEXT("aligned physics rate enters the draft"), RequiredOnlyModel->GetRate(TEXT("imu")), 100.0);
  TestEqual(TEXT("aligned physics edit notifies the parent once"), PhysicsChanges, 1);
  // Empty setups must still offer a focusable way back.
  auto Empty = SNew(SLingTuSimSensorSelection).Model(MakeShared<FSensorSelectionModel>());
  Window->SetContent(Empty);
  Empty->SlatePrepass(1.F);
  Empty->FocusInitialControl();
  const auto EmptyClose = FindTag(Empty, TEXT("LingTu.Sensors.Close"));
  TestTrue(TEXT("empty setup focuses close"), EmptyClose.IsValid() && EmptyClose->HasKeyboardFocus());
  TestFalse(TEXT("empty setup cannot be text editing"), Empty->IsTextEntryEditing());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCapabilityWidgetTest,
  "LingTuSim.UI.SensorSelection.CapabilityWidgetEditing",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FSensorCapabilityWidgetTest::RunTest(const FString&) {
  if (!FSlateApplication::IsInitialized()) {
    AddError(TEXT("Slate must be initialized to verify camera editing."));
    return false;
  }
  auto Json = SensorCapabilitiesFixture();
  auto Rows = Json->GetArrayField(TEXT("sensors"));
  auto Blocked = SensorSetupFixture()->GetArrayField(TEXT("sensors"))[1]->AsObject();
  Blocked->SetStringField(TEXT("id"), TEXT("blocked"));
  Blocked->SetStringField(TEXT("unavailable_reason"), TEXT("World has no measurements."));
  Blocked->SetBoolField(TEXT("enabled"), false);
  Rows.Add(MakeShared<FJsonValueObject>(Blocked));
  Json->SetArrayField(TEXT("sensors"), Rows);
  FSensorSetup Setup;
  FString Error;
  if (!FSensorSelectionModel::ParseSetup(Json, Setup, Error)) return false;
  auto Model = MakeShared<FSensorSelectionModel>();
  Model->Configure(Setup);
  int32 Changes = 0;
  auto Panel = SNew(SLingTuSimSensorSelection).Model(Model).OnChanged_Lambda([&Changes]() { ++Changes; });
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool bWasActive = App.IsActive();
  auto Window = SNew(SWindow).ClientSize(FVector2D(660, 740))[Panel];
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(bWasActive);
    App.RequestDestroyWindow(Window);
  };
  Panel->SlatePrepass(1.F);
  const auto Pan = StaticCastSharedPtr<SEditableTextBox>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Pan.camera")));
  const auto Tilt = StaticCastSharedPtr<SEditableTextBox>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Tilt.camera")));
  const auto Projection = StaticCastSharedPtr<SComboBox<TSharedPtr<FString>>>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Projection.camera")));
  const auto Fix = StaticCastSharedPtr<SCheckBox>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.GnssFix.gnss")));
  if (!TestTrue(TEXT("camera and GNSS capability controls exist"), Pan.IsValid() && Tilt.IsValid() && Projection.IsValid() && Fix.IsValid())) return false;
  TestFalse(TEXT("RGBD has no projection control"), FindSensorWidget(Panel, TEXT("LingTu.Sensors.Projection.rgbd")).IsValid());
  TestTrue(TEXT("camera notice displayed"), FindSensorWidget(Panel, TEXT("LingTu.Sensors.Notice.camera")).IsValid());
  TestTrue(TEXT("synthetic WGS84 notice displayed"), FindSensorWidget(Panel, TEXT("LingTu.Sensors.Notice.gnss")).IsValid());
  const auto BlockedToggle = StaticCastSharedPtr<SCheckBox>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Enabled.blocked")));
  const auto AcousticRate = FindSensorWidget(Panel, TEXT("LingTu.Sensors.Rate.acoustic"));
  if (!TestTrue(TEXT("blocked and acoustic controls exist"), BlockedToggle.IsValid() && AcousticRate.IsValid())) return false;
  TestFalse(TEXT("unavailable checkbox is disabled"), BlockedToggle->IsEnabled());
  TestTrue(TEXT("unavailable reason displayed"), FindSensorWidget(Panel, TEXT("LingTu.Sensors.Unavailable.blocked")).IsValid());
  TestFalse(TEXT("fixed 100 blocks/s is not editable"), AcousticRate->IsEnabled());
  BlockedToggle->ToggleCheckedState();
  TestFalse(TEXT("blocked callback cannot enable sensor"), Model->IsEnabled(TEXT("blocked")));
  TestEqual(TEXT("blocked callback never notifies"), Changes, 0);
  const auto Commit = [&App](const TSharedPtr<SEditableTextBox>& Box, const TCHAR* Value) {
    App.SetKeyboardFocus(Box, EFocusCause::SetDirectly);
    Box->SetText(FText::FromString(Value));
    App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
    App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 0, 0));
  };
  Commit(Pan, TEXT("25.5"));
  TestTrue(TEXT("pan focus suppresses parent shortcuts"), Panel->IsTextEntryEditing());
  TestEqual(TEXT("pan commit reaches draft"), Model->GetPanDeg(TEXT("camera")), 25.5);
  TestEqual(TEXT("pan commit notifies once"), Changes, 1);
  for (const TCHAR* Invalid : {TEXT("181"), TEXT("-181"), TEXT("nan"), TEXT("inf"), TEXT("12oops"), TEXT("")}) {
    Commit(Pan, Invalid);
    TestTrue(TEXT("bad pan displays validation error"), Pan->HasError());
    TestEqual(TEXT("bad pan preserves valid draft"), Model->GetPanDeg(TEXT("camera")), 25.5);
    TestEqual(TEXT("bad pan does not notify"), Changes, 1);
  }
  Commit(Pan, TEXT("10"));
  TestFalse(TEXT("valid pan clears error"), Pan->HasError());
  TestFalse(TEXT("restoring pan baseline clears dirty"), Model->HasChanges());
  Commit(Tilt, TEXT("91"));
  TestTrue(TEXT("out-of-range tilt error"), Tilt->HasError());
  TestFalse(TEXT("invalid tilt cannot dirty clean baseline"), Model->HasChanges());
  Commit(Tilt, TEXT("-90"));
  TestTrue(TEXT("tilt focus suppresses parent shortcuts"), Panel->IsTextEntryEditing());
  TestEqual(TEXT("tilt lower limit commit"), Model->GetTiltDeg(TEXT("camera")), -90.0);
  Commit(Tilt, TEXT("-5"));
  TestFalse(TEXT("restored angles clean"), Model->HasChanges());
  App.SetKeyboardFocus(Projection, EFocusCause::SetDirectly);
  TestTrue(TEXT("projection focus also suppresses parent shortcuts"), Panel->IsTextEntryEditing());
  const int32 BeforeProjection = Changes;
  const auto ProjectionKey = [&App](FKey Key) {
    App.ProcessKeyDownEvent(FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0));
    App.ProcessKeyUpEvent(FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0));
  };
  ProjectionKey(EKeys::Down);
  TestEqual(TEXT("projection keyboard selects fisheye"), Model->GetCameraPreset(TEXT("camera")), FString(TEXT("equidistant_fisheye")));
  ProjectionKey(EKeys::Down);
  TestEqual(TEXT("projection keyboard selects panorama"), Model->GetCameraPreset(TEXT("camera")), FString(TEXT("equirectangular")));
  ProjectionKey(EKeys::Up);
  ProjectionKey(EKeys::Up);
  TestEqual(TEXT("projection changes notify once per selection"), Changes, BeforeProjection + 4);
  TestFalse(TEXT("restoring projection baseline cleans draft"), Model->HasChanges());
  App.SetKeyboardFocus(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Close")), EFocusCause::SetDirectly);
  TestFalse(TEXT("leaving controls clears editing suppression"), Panel->IsTextEntryEditing());
  const int32 BeforeFix = Changes;
  Fix->ToggleCheckedState();
  TestTrue(TEXT("GNSS checkbox sets startup fix"), Model->GetGnssFixAvailable(TEXT("gnss")));
  TestEqual(TEXT("fix toggle notifies once"), Changes, BeforeFix + 1);
  Fix->ToggleCheckedState();
  TestFalse(TEXT("restored fix cleans draft"), Model->HasChanges());
  const auto GnssToggle = StaticCastSharedPtr<SCheckBox>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Enabled.gnss")));
  GnssToggle->ToggleCheckedState();
  Panel->SlatePrepass(1.F);
  TestFalse(TEXT("disabled GNSS fix control locked"), Fix->IsEnabled());
  const int32 BeforeDisabledFix = Changes;
  Fix->ToggleCheckedState();
  TestFalse(TEXT("disabled fix callback cannot mutate intent"), Model->GetGnssFixAvailable(TEXT("gnss")));
  TestEqual(TEXT("disabled fix callback cannot notify"), Changes, BeforeDisabledFix);
  GnssToggle->ToggleCheckedState();
  TestFalse(TEXT("restoring GNSS enable state cleans draft"), Model->HasChanges());
  const auto Toggle = StaticCastSharedPtr<SCheckBox>(FindSensorWidget(Panel, TEXT("LingTu.Sensors.Enabled.camera")));
  Toggle->ToggleCheckedState();
  Panel->SlatePrepass(1.F);
  TestFalse(TEXT("disabled camera pan locked"), Pan->IsEnabled());
  TestFalse(TEXT("disabled camera tilt locked"), Tilt->IsEnabled());
  TestFalse(TEXT("disabled camera projection locked"), Projection->IsEnabled());
  Toggle->ToggleCheckedState();
  Panel->SlatePrepass(1.F);
  TestTrue(TEXT("reenabled camera pan editable"), Pan->IsEnabled());
  TestFalse(TEXT("restoring enabled state cleans draft"), Model->HasChanges());
  return true;
}

#endif
