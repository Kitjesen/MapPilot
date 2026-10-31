#if WITH_DEV_AUTOMATION_TESTS
#include "LingTuSimGameSelection.h"
#include "LingTuSimSensorSelection.h"
#include "SLingTuSimRobotMapPicker.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"

namespace {
LingTuSim::UI::FGameSelectionCatalog SensorCatalog() {
  using namespace LingTuSim::UI;
  FSensorSetup Setup;
  Setup.InstanceId = TEXT("robot"); Setup.RigId = TEXT("navigation"); Setup.RigVersion = TEXT("1");
  Setup.Sensors = {{TEXT("imu"),TEXT("imu@1"),TEXT("imu"),TEXT("imu"),TEXT("IMU"),200,true},
      {TEXT("rgb"),TEXT("rgb@1"),TEXT("camera"),TEXT("rgb"),TEXT("RGB"),30,false}};
  FGameSelectionOption A;
  A.Id = TEXT("one"); A.Title = TEXT("One"); A.Robot = {TEXT("robot"),TEXT("1"),TEXT("Robot")};
  A.World = {TEXT("world1"),TEXT("1"),TEXT("World1")};
  A.SensorSelection = MakeShared<FSensorSelectionModel>(); A.SensorSelection->Configure(Setup);
  auto B = A; B.Id = TEXT("two"); B.World.Id = TEXT("world2");
  B.SensorSelection = MakeShared<FSensorSelectionModel>(); B.SensorSelection->Configure(Setup);
  FGameSelectionCatalog Catalog; Catalog.Options = {A,B}; return Catalog;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionAcrossWorldTest, "LingTuSim.UI.SensorSelection.AcrossWorlds",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSensorSelectionAcrossWorldTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FGameSelectionModel Model; Model.SetCatalog(SensorCatalog()); Model.SelectById(TEXT("one"));
  Model.GetSelectedOption()->SensorSelection->SetEnabled(TEXT("rgb"), false);
  Model.SelectWorldById(TEXT("world2"));
  TestFalse(TEXT("same robot/rig choices survive map selection"), Model.GetSelectedOption()->SensorSelection->IsEnabled(TEXT("rgb")));
  TestTrue(TEXT("changed selection still requires authoring"), Model.GetSelectedOption()->SensorSelection->HasChanges());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionPickerTest, "LingTuSim.UI.SensorSelection.PickerFlow",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSensorSelectionPickerTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Model = MakeShared<FGameSelectionModel>(); Model->SetCatalog(SensorCatalog());
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true).CanConfirm(false);
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280,900))[Picker];
  auto& App = FSlateApplication::Get(); App.AddWindow(Window,false);
  ON_SCOPE_EXIT { App.RequestDestroyWindow(Window); };
  TestTrue(TEXT("sensor configuration opens within robot step"), Picker->OpenSensorSelection());
  TestTrue(TEXT("sensor panel owns text/navigation input"), Picker->IsSearchEditing());
  Picker->PrimaryAction();
  TestEqual(TEXT("open configuration cannot launch or advance"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::Robot);
  TestTrue(TEXT("Escape closes the drawer first"), Picker->GoBack());
  TestFalse(TEXT("drawer is closed"), Picker->IsSensorSelectionOpen());
  TestEqual(TEXT("only the same two steps remain"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::Robot);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorSelectionDifferentTimebaseTest, "LingTuSim.UI.SensorSelection.DifferentWorldTimebases",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSensorSelectionDifferentTimebaseTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = SensorCatalog();
  auto Fast = Catalog.Options[0].SensorSelection->GetSetup();
  Fast.Sensors[0].PhysicsTimestepS = .001;
  auto Slow = Fast; Slow.Sensors[0].PhysicsTimestepS = .005;
  Catalog.Options[0].SensorSelection->Configure(Fast);
  Catalog.Options[1].SensorSelection->Configure(Slow);
  FGameSelectionModel Model; Model.SetCatalog(MoveTemp(Catalog)); Model.SelectById(TEXT("one"));
  auto First = Model.GetSelectedOption()->SensorSelection;
  TestTrue(TEXT("125 Hz is compatible with 1 ms world"), First->SetRate(TEXT("imu"),125.));
  Model.SelectWorldById(TEXT("world2"));
  auto Second = Model.GetSelectedOption()->SensorSelection;
  TestTrue(TEXT("different world timebases do not share editable model"), First != Second);
  TestFalse(TEXT("125 Hz is incompatible with 5 ms world"), Second->SetRate(TEXT("imu"),125.));
  TestFalse(TEXT("rejected edit does not modify second world's defaults"), Second->HasChanges());
  Model.SelectWorldById(TEXT("world1"));
  TestEqual(TEXT("returning to original world retains valid draft"), Model.GetSelectedOption()->SensorSelection->GetRate(TEXT("imu")),125.);
  return true;
}
#endif
