#if WITH_DEV_AUTOMATION_TESTS
#include "LingTuSimGameSelection.h"
#include "LingTuSimSensorSelection.h"
#include "SLingTuSimRobotMapPicker.h"
#include "Framework/Application/SlateApplication.h"
#include "ImageUtils.h"
#include "Input/Events.h"
#include "InputCoreTypes.h"
#include "Layout/Children.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SWindow.h"

namespace LingTuSim::UI {
namespace {
TSharedPtr<SWidget> FindSensorTag(TSharedRef<SWidget> Widget, FName Tag) {
  if (Widget->GetTag() == Tag) return Widget;
  auto* Children = Widget->GetChildren();
  for (int32 Index = 0; Children && Index < Children->Num(); ++Index) {
    auto Found = FindSensorTag(Children->GetChildAt(Index), Tag);
    if (Found) return Found;
  }
  return nullptr;
}

class FSensorSelectionRenderCommand final : public IAutomationLatentCommand {
 public:
  explicit FSensorSelectionRenderCommand(FAutomationTestBase* InTest) : Test(InTest) {}
  ~FSensorSelectionRenderCommand() { Close(); }
  bool Update() override {
    if (!Window) {
      FString CatalogPath, Error;
      FParse::Value(FCommandLine::Get(), TEXT("LingTuSensorTestCatalog="), CatalogPath);
      FParse::Value(FCommandLine::Get(), TEXT("LingTuSensorTestOutput="), Output);
      FGameSelectionCatalog Catalog;
      if (!Test->TestTrue(*Error, FGameSelectionCatalogLoader::LoadFromFile(CatalogPath, Catalog, Error))) {
        Test->AddError(Error); return true;
      }
      Model = MakeShared<FGameSelectionModel>(); Model->SetCatalog(MoveTemp(Catalog));
      if (!Test->TestTrue(TEXT("real catalog has editable Thunder rig"),
          Model->GetSelectedOption() && Model->GetSelectedOption()->SensorSelection.IsValid())) return true;
      Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true).CanConfirm(true);
      Window = SNew(SWindow).Title(FText::FromString(TEXT("LingTu sensor selection verification")))
        .ClientSize(FVector2D(1600,900)).SizingRule(ESizingRule::FixedSize)[Picker.ToSharedRef()];
      auto& App = FSlateApplication::Get(); PreviousFocus = App.GetKeyboardFocusedWidget();
      bWasActive = App.IsActive(); bOwnsActivation = true;
      App.AddWindow(Window.ToSharedRef(), true); App.OnApplicationActivationChanged(true);
      Deadline = FPlatformTime::Seconds() + 90.; return false;
    }
    if (FPlatformTime::Seconds() > Deadline) {
      Test->AddError(TEXT("Sensor selector rendered verification timed out.")); Close(); return true;
    }
    if (Stage == 0) {
      if (!Picker->IsShowroomReady()) return false;
      Test->TestTrue(TEXT("drawer opens on actual catalog"), Picker->OpenSensorSelection());
      Stage = 1; ReadyAt = FPlatformTime::Seconds() + 1.; return false;
    }
    if (FPlatformTime::Seconds() < ReadyAt) return false;
    if (Stage == 1) {
      Picker->SlatePrepass(1.F);
      auto Toggle = FindSensorTag(Picker.ToSharedRef(), TEXT("LingTu.Sensors.Enabled.front_depth"));
      auto Rate = FindSensorTag(Picker.ToSharedRef(), TEXT("LingTu.Sensors.Rate.front_rgb"));
      if (!Test->TestTrue(TEXT("actual controls exist"), Toggle.IsValid() && Rate.IsValid())) { Close(); return true; }
      StaticCastSharedPtr<SCheckBox>(Toggle)->ToggleCheckedState();
      auto& App = FSlateApplication::Get(); App.SetKeyboardFocus(Rate, EFocusCause::SetDirectly);
      StaticCastSharedPtr<SEditableTextBox>(Rate)->SetText(FText::FromString(TEXT("15")));
      App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter,FModifierKeysState(),0,false,0,0));
      App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter,FModifierKeysState(),0,false,0,0));
      auto Sensors = Model->GetSelectedOption()->SensorSelection;
      Test->TestFalse(TEXT("depth UI edit reaches model"), Sensors->IsEnabled(TEXT("front_depth")));
      Test->TestEqual(TEXT("RGB UI rate edit reaches model"), Sensors->GetRate(TEXT("front_rgb")), 15.);
      if (Sensors->HasGnssFixControl(TEXT("gnss"))) {
        Test->TestEqual(TEXT("versioned suite exposes all nine real sensor choices"), Sensors->GetSetup().Sensors.Num(), 9);
        auto Gnss = FindSensorTag(Picker.ToSharedRef(), TEXT("LingTu.Sensors.Enabled.gnss"));
        auto Fix = FindSensorTag(Picker.ToSharedRef(), TEXT("LingTu.Sensors.GnssFix.gnss"));
        if (!Test->TestTrue(TEXT("actual GNSS controls exist"), Gnss.IsValid() && Fix.IsValid())) { Close(); return true; }
        StaticCastSharedPtr<SCheckBox>(Gnss)->ToggleCheckedState();
        Picker->SlatePrepass(1.F);
        StaticCastSharedPtr<SCheckBox>(Fix)->ToggleCheckedState();
        Test->TestTrue(TEXT("GNSS selected through actual widget"), Sensors->IsEnabled(TEXT("gnss")));
        Test->TestTrue(TEXT("simulation fix selected through actual widget"), Sensors->GetGnssFixAvailable(TEXT("gnss")));
        App.SetKeyboardFocus(Fix, EFocusCause::SetDirectly);
        auto Scroll = FindSensorTag(Picker.ToSharedRef(), TEXT("LingTu.Sensors.Scroll"));
        if (Test->TestTrue(TEXT("configuration scroll container exists"), Scroll.IsValid()))
          StaticCastSharedPtr<SScrollBox>(Scroll)->ScrollToEnd();
      }
      Stage = 2; ReadyAt = FPlatformTime::Seconds() + .5; return false;
    }
    TArray<FColor> Pixels; FIntVector Size;
    if (Test->TestTrue(TEXT("capture actual rendered selector"), FSlateApplication::Get().TakeScreenshot(Window.ToSharedRef(), Pixels, Size))) {
      TArray64<uint8> Png;
      FImageUtils::PNGCompressImageArray(Size.X, Size.Y, Pixels, Png);
      Test->TestTrue(TEXT("save actual selector screenshot"), FFileHelper::SaveArrayToFile(Png, *FPaths::Combine(Output,TEXT("sensor-selection.png"))));
    }
    Test->TestTrue(TEXT("back closes configuration before leaving robot step"), Picker->GoBack());
    Picker->PrimaryAction();
    Test->TestEqual(TEXT("two-step flow advances to map"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
    FString Error;
    Test->TestTrue(TEXT("production writer saves modified intent"), FGameSelectionIntentWriter::WriteNew(
        FPaths::Combine(Output,TEXT("selection.intent.json")), *Model->GetSelectedOption(), Error));
    if (!Error.IsEmpty()) Test->AddError(Error);
    Close(); return true;
  }
 private:
  void Close() {
    if (FSlateApplication::IsInitialized() && bOwnsActivation) {
      auto& App = FSlateApplication::Get();
      if (PreviousFocus) App.SetKeyboardFocus(PreviousFocus); else App.ClearKeyboardFocus();
      App.OnApplicationActivationChanged(bWasActive); bOwnsActivation = false;
    }
    if (Window) Window->RequestDestroyWindow();
    Window.Reset(); Picker.Reset(); Model.Reset();
  }
  FAutomationTestBase* Test;
  TSharedPtr<FGameSelectionModel> Model;
  TSharedPtr<SLingTuSimRobotMapPicker> Picker;
  TSharedPtr<SWindow> Window;
  TSharedPtr<SWidget> PreviousFocus;
  FString Output;
  bool bWasActive = false, bOwnsActivation = false;
  int32 Stage = 0;
  double Deadline = 0, ReadyAt = 0;
};
}
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FSensorSelectionRenderTest, "LingTuSim.UI.SensorSelection.Render",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
void FSensorSelectionRenderTest::GetTests(TArray<FString>& Names, TArray<FString>& Commands) const {
  FString Catalog, Output;
  if (FParse::Value(FCommandLine::Get(),TEXT("LingTuSensorTestCatalog="),Catalog) &&
      FParse::Value(FCommandLine::Get(),TEXT("LingTuSensorTestOutput="),Output)) {
    Names.Add(TEXT("ActualCatalogAndIntent")); Commands.Add(TEXT("fixture"));
  }
}
bool FSensorSelectionRenderTest::RunTest(const FString&) {
  ADD_LATENT_AUTOMATION_COMMAND(LingTuSim::UI::FSensorSelectionRenderCommand(this)); return true;
}
#endif
