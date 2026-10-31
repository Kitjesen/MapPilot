#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimMeasurementClient.h"
#include "SLingTuSimMeasurements.h"
#include "SLingTuSimRuntimeHUD.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Dom/JsonObject.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/PlatformTime.h"
#include "ImageUtils.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Widgets/SWindow.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Input/Events.h"

// Opt-in real HTTP/native-control fixture, never registered without explicit test binding.
// It is not a Product lifecycle, robot motion, or factory qualification test.
namespace LingTuSim::UI {
namespace {
bool ContainsText(const TSharedRef<SWidget>& Widget, const FString& Caption) {
  if (Widget->GetType() == TEXT("STextBlock") && StaticCastSharedRef<STextBlock>(Widget)->GetText().ToString() == Caption) return true;
  auto* Children = Widget->GetChildren();
  for (int32 I = 0; I < Children->Num(); ++I) if (ContainsText(Children->GetChildAt(I), Caption)) return true;
  return false;
}
TSharedPtr<SWidget> FindControl(const TSharedRef<SWidget>& Widget, const TCHAR* Type, const FString& Caption) {
  if (!Widget->GetVisibility().IsVisible()) return nullptr;
  if (Widget->GetType() == Type && ContainsText(Widget, Caption)) return Widget;
  auto* Children = Widget->GetChildren();
  for (int32 I = 0; I < Children->Num(); ++I) if (auto Found = FindControl(Children->GetChildAt(I), Type, Caption)) return Found;
  return nullptr;
}
}
class FMeasurementLiveCommand final : public IAutomationLatentCommand {
 public:
  explicit FMeasurementLiveCommand(FAutomationTestBase* InTest) : Test(InTest) {
    FString Url, Product;
    FParse::Value(FCommandLine::Get(), TEXT("LingTuMeasurementTestGateway="), Url);
    FParse::Value(FCommandLine::Get(), TEXT("LingTuMeasurementTestProduct="), Product);
    FParse::Value(FCommandLine::Get(), TEXT("LingTuMeasurementTestOutput="), Output);
    Client = MakeShared<FMeasurementClient>();
    Client->Configure(Url, {Product, TEXT("compiled-source"), 2, 1});
    Window = SNew(SWindow).Title(FText::FromString(TEXT("LingTu measurement integration fixture")))
        .ClientSize(FVector2D(1040, 760)).SupportsMaximize(false).SupportsMinimize(false);
    Mode = MakeShared<FRuntimeUIModeController>(); Mode->TogglePause();
    SAssignNew(HUD, SLingTuSimRuntimeHUD).ModeController(Mode).Measurements(Client)
        .LocalState(MakeShared<FRuntimeUILocalState>()).PlayerSettings(MakeShared<FPlayerSettings>())
        .LoginModel(MakeShared<FFrontEndLoginModel>()).SelectionModel(MakeShared<FGameSelectionModel>())
        .AssetReviewModel(MakeShared<FAssetReviewModel>()).InspectionProjection(MakeShared<FInspectionProjection>())
        .SelectionFeedback(MakeShared<FString>());
    Window->SetContent(HUD.ToSharedRef());
    FSlateApplication::Get().AddWindow(Window.ToSharedRef());
    HUD->PreviewPlayerMenuPage(TEXT("measurements"));
    Advance(0);
  }
  virtual ~FMeasurementLiveCommand() { Close(); }
  virtual bool Update() override {
    if (FPlatformTime::Seconds() > Deadline) {
      Test->AddError(FString::Printf(TEXT("Measurement fixture timed out at stage %d: %s / %s"),
          Stage, *Client->Error(), *Client->Message()));
      Close(); return true;
    }
    switch (Stage) {
      case 0:
        if (Client->Sensors().Num() != 3) return false;
        Client->SelectSensor(TEXT("robot.semantic"));
        Advance(19); break;
      case 19:
        if (FPlatformTime::Seconds() - StageStarted < .5 || !Client->CanQuery()) return false;
        Test->TestFalse(TEXT("Injected first-frame loss has no measurement identity"), Client->SelectedStatus()->bHasIdentity);
        Test->TestTrue(TEXT("Declared sensor can query before its first sample"), Client->CanQuery());
        Activate(TEXT("SButton"), TEXT("查询设置")); Advance(20); break;
      case 20:
        if (Client->IsPending() || !Client->Message().StartsWith(TEXT("源端已确认："))) return false;
        Client->SelectSensor(TEXT("robot.thermal")); Advance(1); break;
      case 1:
        if (!Ready(TEXT("thermal"))) return false;
        HUD->RestoreSelectionFocus();
        Test->TestTrue(TEXT("Initial keyboard focus belongs to the HUD"), HUD->HasFocusedDescendants());
        Test->TestTrue(TEXT("Real HUD remains in live measurement mode"), HUD->IsMeasurementPageOpen());
        Test->TestEqual(TEXT("Opening UI does not pause source"), Mode->GetMode(), ERuntimeUIMode::Tactical);
        Activate(TEXT("SButton"), TEXT("查询设置")); Advance(21); break;
      case 21:
        if (!ConfirmedBool(TEXT("enabled"), true) || !Ready(TEXT("thermal"))) return false;
        Capture(TEXT("thermal")); SendBool(TEXT("recording"), true); Advance(2); break;
      case 2:
        if (!ConfirmedBool(TEXT("recording"), true)) return false;
        Advance(3); break;
      case 3:
        if (FPlatformTime::Seconds() - StageStarted < .7) return false;
        SendBool(TEXT("recording"), false); Advance(4); break;
      case 4:
        if (!ConfirmedBool(TEXT("recording"), false)) return false;
        Client->SelectSensor(TEXT("robot.acoustic")); Advance(5); break;
      case 5:
        if (!Ready(TEXT("acoustic"))) return false;
        Test->TestEqual(TEXT("Actual microphone sample rate"), Client->Preview()->SampleRateHz, 8000);
        Activate(TEXT("SButton"), TEXT("查询设置")); Advance(22); break;
      case 22:
        if (!ConfirmedBool(TEXT("enabled"), true) || !Ready(TEXT("acoustic"))) return false;
        Capture(TEXT("acoustic")); SendBool(TEXT("recording"), true); Advance(6); break;
      case 6:
        if (!ConfirmedBool(TEXT("recording"), true)) return false;
        Advance(7); break;
      case 7:
        if (FPlatformTime::Seconds() - StageStarted < .7) return false;
        SendBool(TEXT("recording"), false); Advance(8); break;
      case 8:
        if (!ConfirmedBool(TEXT("recording"), false)) return false;
        Client->SelectSensor(TEXT("robot.semantic")); Advance(9); break;
      case 9:
        if (!Ready(TEXT("semantic"))) return false;
        Capture(TEXT("semantic")); SendBool(TEXT("enabled"), false); Advance(10); break;
      case 10:
        if (!ConfirmedBool(TEXT("enabled"), false)) return false;
        Test->TestNull(TEXT("Disabled source clears the preview"), Client->Preview());
        Advance(11); break;
      case 11:
        if (FPlatformTime::Seconds() - StageStarted < .5 || !Client->CanControl()) return false;
        Capture(TEXT("disabled")); SendBool(TEXT("enabled"), true); Advance(12); break;
      case 12:
        if (!ConfirmedBool(TEXT("enabled"), true) || !Ready(TEXT("semantic"))) return false;
        HUD->HandleRuntimeMenuBack();
        Test->TestEqual(TEXT("Back restores paused menu"), Mode->GetMode(), ERuntimeUIMode::Pause);
        Test->TestNull(TEXT("Closing page clears measurements"), Client->Preview());
        Test->TestFalse(TEXT("Hidden page cannot control sensors"), Client->CanControl());
        Test->TestFalse(TEXT("Hidden page has no pending request"), Client->IsPending());
        Close(); return true;
    }
    return false;
  }
 private:
  void Advance(int32 Next) { Stage = Next; StageStarted = FPlatformTime::Seconds(); Deadline = StageStarted + 20.; }
  bool Ready(const TCHAR* Kind) const {
    return Client->Preview() && Client->Preview()->Kind == Kind && Client->CanControl()
        && FPlatformTime::Seconds() - StageStarted > .5;
  }
  void SendBool(const TCHAR* Operation, bool Value) {
    bool Before = Value;
    Test->TestTrue(TEXT("Control has confirmed initial state"), Client->Settings() && Client->Settings()->TryGetBoolField(Operation, Before));
    Test->TestTrue(TEXT("Checkbox is changing the actual state"), Before != Value);
    Activate(TEXT("SCheckBox"), Operation);
  }
  void Activate(const TCHAR* Type, const TCHAR* Caption) {
    HUD->SlatePrepass(1.F);
    auto Control = FindControl(HUD.ToSharedRef(), Type, Caption);
    if (!Test->TestTrue(FString::Printf(TEXT("Found actual %s control"), Caption), Control.IsValid())) return;
    Test->TestTrue(TEXT("Control is enabled"), Control->IsEnabled());
    FSlateApplication::Get().SetKeyboardFocus(Control, EFocusCause::SetDirectly);
    const FKeyEvent Accept(EKeys::Enter, FModifierKeysState(), 0, false, 13, 13);
    Control->OnKeyDown(FGeometry(), Accept); Control->OnKeyUp(FGeometry(), Accept);
  }
  bool ConfirmedBool(const TCHAR* Name, bool Expected) const {
    bool Actual = !Expected;
    return !Client->IsPending() && Client->Settings()
        && Client->Settings()->TryGetBoolField(Name, Actual) && Actual == Expected
        && Client->Message().StartsWith(TEXT("源端已确认："));
  }
  void Capture(const TCHAR* Name) {
    TArray<FColor> Pixels; FIntVector Size;
    const bool bCaptured = FSlateApplication::Get().TakeScreenshot(Window.ToSharedRef(), Pixels, Size);
    Test->TestTrue(FString::Printf(TEXT("Rendered %s panel"), Name), bCaptured);
    if (!bCaptured || Size.X <= 0 || Size.Y <= 0 || Output.IsEmpty()) return;
    TArray64<uint8> Png;
    FImageUtils::PNGCompressImageArray(Size.X, Size.Y, Pixels, Png);
    Test->TestTrue(TEXT("Saved rendered panel"), FFileHelper::SaveArrayToFile(
        Png, *FPaths::Combine(Output, FString(Name) + TEXT(".png"))));
  }
  void Close() {
    if (Client) Client->Shutdown();
    if (Window && FSlateApplication::IsInitialized()) Window->RequestDestroyWindow();
    HUD.Reset(); Window.Reset(); Client.Reset(); Mode.Reset();
  }
  FAutomationTestBase* Test;
  TSharedPtr<FMeasurementClient> Client;
  TSharedPtr<SLingTuSimRuntimeHUD> HUD;
  TSharedPtr<FRuntimeUIModeController> Mode;
  TSharedPtr<SWindow> Window;
  FString Output;
  int32 Stage = 0;
  double StageStarted = 0, Deadline = 0;
};
}

IMPLEMENT_COMPLEX_AUTOMATION_TEST(FLingTuMeasurementLiveTest, "LingTuSim.UI.Measurements.LiveFixture",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
void FLingTuMeasurementLiveTest::GetTests(TArray<FString>& Names, TArray<FString>& Commands) const {
  FString Url, Product, Output;
  if (FParse::Value(FCommandLine::Get(), TEXT("LingTuMeasurementTestGateway="), Url)
      && FParse::Value(FCommandLine::Get(), TEXT("LingTuMeasurementTestProduct="), Product)
      && FParse::Value(FCommandLine::Get(), TEXT("LingTuMeasurementTestOutput="), Output)) {
    Names.Add(TEXT("RealHTTPAndNativeControls")); Commands.Add(TEXT("fixture"));
  }
}
bool FLingTuMeasurementLiveTest::RunTest(const FString&) {
  ADD_LATENT_AUTOMATION_COMMAND(LingTuSim::UI::FMeasurementLiveCommand(this));
  return true;
}
#endif
