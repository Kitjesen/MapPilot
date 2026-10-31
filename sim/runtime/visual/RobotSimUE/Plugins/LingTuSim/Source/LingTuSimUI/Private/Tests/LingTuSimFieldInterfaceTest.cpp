#if WITH_DEV_AUTOMATION_TESTS
#include "SLingTuSimRuntimeHUD.h"
#include "SLingTuSimRobotMapPicker.h"
#include "SLingTuSimShowroom.h"
#include "LingTuSimRuntimeUIModel.h"
#include "LingTuSimRuntimeUIInputProcessor.h"
#include "Framework/Application/SlateApplication.h"
#include "Engine/World.h"
#include "Engine/WorldInitializationValues.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Misc/Paths.h"
#include "Misc/ConfigCacheIni.h"
#include "HAL/FileManager.h"
#include "Input/Events.h"
#include "Layout/ArrangedChildren.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSlider.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/SWindow.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShowroomIdentifierTest, "LingTuSim.UI.Runtime.Showroom.Identifier",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomIdentifierTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  TestEqual(TEXT("actual library ID resolves only to showroom asset"), SLingTuSimShowroom::MeshPath(TEXT("unitree_g1")),
    FString(TEXT("/Game/RobotSim/FrontEnd/Showroom/Robots/SM_unitree_g1.SM_unitree_g1")));
  TestTrue(TEXT("empty selection has no substitute robot"), SLingTuSimShowroom::MeshPath(TEXT("")).IsEmpty());
  TestTrue(TEXT("object path is not a robot ID"), SLingTuSimShowroom::MeshPath(TEXT("/Game/Other" )).IsEmpty());
  auto Widget = SNew(SLingTuSimShowroom).RobotId(TEXT("unitree_g1"));
  TestFalse(TEXT("construction is lazy, no world is started by UI tests"), Widget->HasRobot());
  return true;
}

namespace {
LingTuSim::UI::FGameSelectionCatalog RobotMapCatalog() {
  using namespace LingTuSim::UI;
  FGameSelectionOption Factory;
  Factory.Id = TEXT("thunder_factory");
  Factory.Title = TEXT("Factory patrol");
  Factory.Availability = EGameSelectionAvailability::Runnable;
  Factory.Robot = {TEXT("thunder"), TEXT("1.0.3"), TEXT("雷霆 V4"), TEXT("wheeled_legged")};
  Factory.World = {TEXT("factory"), TEXT("1.2.1"), TEXT("工业园区")};
  Factory.BundleDirectory = TEXT("C:/compiled/factory");
  Factory.SessionId = TEXT("factory_session");
  Factory.BundleArtifacts = {{TEXT("physics.plan.json")}};
  auto Open = Factory;
  Open.Id = TEXT("thunder_open");
  Open.World = {TEXT("open"), TEXT("1.1.0"), TEXT("开放场地")};
  Open.BundleDirectory = TEXT("C:/compiled/open");
  Open.SessionId = TEXT("open_session");
  auto Cart = Factory;
  Cart.Id = TEXT("cart_factory");
  Cart.Robot = {TEXT("cart"), TEXT("1.0.0"), TEXT("OmniCart"), TEXT("wheeled")};
  Cart.World.Version = TEXT("1.1.0");
  Cart.Availability = EGameSelectionAvailability::PreviewOnly;
  Cart.AvailabilityReason = TEXT("Unreal controller is not bound");
  Cart.BundleDirectory.Reset();
  FGameSelectionCatalog Catalog;
  Catalog.Options = {Factory, Open, Cart};
  return Catalog;
}

struct FFieldUIFixture {
  TSharedPtr<LingTuSim::UI::FRuntimeUIModeController> Mode = MakeShared<LingTuSim::UI::FRuntimeUIModeController>();
  TSharedPtr<LingTuSim::UI::FPlayerSettings> Settings = MakeShared<LingTuSim::UI::FPlayerSettings>();
  TSharedPtr<LingTuSim::UI::FInspectionProjection> Inspection = MakeShared<LingTuSim::UI::FInspectionProjection>();
  TSharedRef<LingTuSim::UI::SLingTuSimRuntimeHUD> Widget(UWorld* World = nullptr) {
    auto HUD = SNew(LingTuSim::UI::SLingTuSimRuntimeHUD).World(World).ModeController(Mode)
      .LocalState(MakeShared<LingTuSim::UI::FRuntimeUILocalState>())
      .PlayerSettings(Settings)
      .LoginModel(MakeShared<LingTuSim::UI::FFrontEndLoginModel>())
      .SelectionModel(MakeShared<LingTuSim::UI::FGameSelectionModel>())
      .AssetReviewModel(MakeShared<LingTuSim::UI::FAssetReviewModel>())
      .InspectionProjection(Inspection)
      .SelectionFeedback(MakeShared<FString>());
    // UE 5.8 registered visibility/enabled attributes are evaluated in prepass.
    HUD->SlatePrepass(1.0F);
    return HUD;
  }
};

class FScopedFieldUITestWorld final {
 public:
  FScopedFieldUITestWorld() {
    const FWorldInitializationValues Values = FWorldInitializationValues()
        .AllowAudioPlayback(false).RequiresHitProxies(false).CreatePhysicsScene(false)
        .CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false)
        .SetTransactional(false);
    World = UWorld::CreateWorld(EWorldType::Game, false,
        FName(*FString::Printf(TEXT("LingTuSimFieldUITest_%s"),
            *FGuid::NewGuid().ToString(EGuidFormats::Digits))), nullptr, true,
        ERHIFeatureLevel::Num, &Values);
  }
  ~FScopedFieldUITestWorld() { if (World != nullptr) World->DestroyWorld(false); }
  UWorld* Get() const { return World; }

 private:
  UWorld* World = nullptr;
};

bool HasVisibleText(TSharedRef<SWidget> Widget, const FString& Text) {
  if (!Widget->GetVisibility().IsVisible()) return false;
  if (Widget->GetType() == TEXT("STextBlock") && StaticCastSharedRef<STextBlock>(Widget)->GetText().ToString() == Text) return true;
  FChildren* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index)
    if (HasVisibleText(Children->GetChildAt(Index), Text)) return true;
  return false;
}

TSharedPtr<SButton> FindButton(TSharedRef<SWidget> Widget, const FString& Caption) {
  if (!Widget->GetVisibility().IsVisible()) return nullptr;
  if ((Widget->GetType() == TEXT("SButton") || Widget->GetType() == TEXT("SPlayerSettingChoice")) &&
      HasVisibleText(Widget, Caption))
    return StaticCastSharedRef<SButton>(Widget);
  FChildren* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index)
    if (auto Button = FindButton(Children->GetChildAt(Index), Caption)) return Button;
  return nullptr;
}

TSharedPtr<SBorder> FindFocusMarker(TSharedRef<SWidget> Widget) {
  if (Widget->GetType() == TEXT("SBorder") && Widget->GetTag() == TEXT("LingTu.Menu.FocusMarker"))
    return StaticCastSharedRef<SBorder>(Widget);
  FChildren* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index)
    if (auto Marker = FindFocusMarker(Children->GetChildAt(Index))) return Marker;
  return nullptr;
}

TSharedPtr<SButton> FindPreviewButton(TSharedRef<SWidget> Widget) {
  if (Widget->GetType() == TEXT("SButton") && Widget->GetTag() == TEXT("LingTu.Selection.Preview"))
    return StaticCastSharedRef<SButton>(Widget);
  auto* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index)
    if (auto Found = FindPreviewButton(Children->GetChildAt(Index))) return Found;
  return nullptr;
}

void AcceptButton(const TSharedRef<SButton>& Button) {
  FKeyEvent Accept(EKeys::Enter, FModifierKeysState(), 0, false, 13, 13);
  Button->OnKeyDown(FGeometry(), Accept);
  Button->OnKeyUp(FGeometry(), Accept);
}

TSharedPtr<SWidget> FindTaggedWidget(TSharedRef<SWidget> Widget, FName Tag) {
  if (Widget->GetTag() == Tag) return Widget;
  auto* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index)
    if (auto Found = FindTaggedWidget(Children->GetChildAt(Index), Tag)) return Found;
  return nullptr;
}

TSharedPtr<SCheckBox> FindCheckBox(TSharedRef<SWidget> Widget, const FString& Caption) {
  if (!Widget->GetVisibility().IsVisible()) return nullptr;
  if (Widget->GetType() == TEXT("SCheckBox") && HasVisibleText(Widget, Caption)) return StaticCastSharedRef<SCheckBox>(Widget);
  auto* Children = Widget->GetChildren();
  for (int32 Index = 0; Index < Children->Num(); ++Index)
    if (auto Check = FindCheckBox(Children->GetChildAt(Index), Caption)) return Check;
  return nullptr;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWelcomeLocalEntryTest, "LingTuSim.UI.Runtime.Welcome.LocalEntry",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FWelcomeLocalEntryTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  for (const bool bKeyboard : {false, true}) {
    auto Mode = MakeShared<FRuntimeUIModeController>();
    Mode->TogglePause();
    auto Login = MakeShared<FFrontEndLoginModel>();
    auto Selection = MakeShared<FGameSelectionModel>();
    Selection->SetCatalog(RobotMapCatalog());
    int32 LaunchCount = 0;
    auto HUD = SNew(SLingTuSimRuntimeHUD).ModeController(Mode)
      .LocalState(MakeShared<FRuntimeUILocalState>()).PlayerSettings(MakeShared<FPlayerSettings>())
      .LoginModel(Login).SelectionModel(Selection).AssetReviewModel(MakeShared<FAssetReviewModel>())
      .InspectionProjection(MakeShared<FInspectionProjection>()).SelectionFeedback(MakeShared<FString>())
      .FrontEndLoginRequired(true).OnSelectionConfirmed_Lambda([&LaunchCount]() { ++LaunchCount; });
    auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[HUD];
    auto& App = FSlateApplication::Get();
    const auto PreviousFocus = App.GetKeyboardFocusedWidget();
    const bool WasActive = App.IsActive();
    App.AddWindow(Window, false);
    App.OnApplicationActivationChanged(true);
    ON_SCOPE_EXIT {
      App.ClearKeyboardFocus(EFocusCause::SetDirectly);
      if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
      App.OnApplicationActivationChanged(WasActive);
      App.RequestDestroyWindow(Window);
    };
    HUD->SlatePrepass(1.F);
    TestTrue(TEXT("short Start label is visible"), HasVisibleText(HUD, TEXT("开始")));
    TestFalse(TEXT("old Start simulation label is absent"), HasVisibleText(HUD, TEXT("开始仿真")));
    TestFalse(TEXT("forest-specific title is removed"), HasVisibleText(HUD, TEXT("林地行动终端")));
    TestFalse(TEXT("operator form is removed from welcome"), HasVisibleText(HUD, TEXT("本地操作员")));
    TestFalse(TEXT("footer key legend is removed"), HasVisibleText(HUD, TEXT("↑ ↓  选择     Enter  确认")));
    TestFalse(TEXT("catalog diagnostics do not dominate the main menu"), HasVisibleText(HUD, TEXT("已载入 2 款机器人  /  2 张地图")));
    auto NameWidget = FindTaggedWidget(HUD, TEXT("LingTu.Welcome.Operator"));
    auto StartWidget = FindTaggedWidget(HUD, TEXT("LingTu.Welcome.Start"));
    TestFalse(TEXT("operator text entry is not hidden in the widget tree"), NameWidget.IsValid());
    TestTrue(TEXT("approved factory background replaces the old welcome stage"), FindTaggedWidget(HUD, TEXT("LingTu.Welcome.FactoryBackground")).IsValid());
    if (!TestTrue(TEXT("entry control exists"), StartWidget.IsValid())) return false;
    auto Start = StaticCastSharedPtr<SButton>(StartWidget);
    TestFalse(TEXT("showing the menu does not sign in"), Login->IsLoggedIn());
    TestTrue(TEXT("entry requires no form"), Start->IsEnabled());
    if (bKeyboard) {
      App.SetKeyboardFocus(Start, EFocusCause::SetDirectly);
      App.ProcessKeyDownEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 13, 13));
      App.ProcessKeyUpEvent(FKeyEvent(EKeys::Enter, FModifierKeysState(), 0, false, 13, 13));
    } else {
      AcceptButton(Start.ToSharedRef());
    }
    HUD->SlatePrepass(1.F);
    TestEqual(TEXT("entry still uses local identity, not online authentication"), Login->GetMethod(), EFrontEndLoginMethod::LocalOperator);
    TestEqual(TEXT("existing local default is retained"), Login->GetDisplayName(), FString(TEXT("Operator")));
    TestFalse(TEXT("entry never confirms a simulation"), Selection->IsConfirmed());
    TestEqual(TEXT("entry never dispatches launch"), LaunchCount, 0);
    TestFalse(TEXT("welcome hides after entry"), HasVisibleText(HUD, TEXT("开始")));
    TestTrue(TEXT("robot selection follows welcome"), HasVisibleText(HUD, TEXT("选择机器人")));
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWelcomeEmptyCatalogTest, "LingTuSim.UI.Runtime.Welcome.EmptyCatalog",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FWelcomeEmptyCatalogTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Mode = MakeShared<FRuntimeUIModeController>();
  Mode->TogglePause();
  auto Login = MakeShared<FFrontEndLoginModel>();
  auto HUD = SNew(SLingTuSimRuntimeHUD).ModeController(Mode)
    .LocalState(MakeShared<FRuntimeUILocalState>()).PlayerSettings(MakeShared<FPlayerSettings>())
    .LoginModel(Login).SelectionModel(MakeShared<FGameSelectionModel>())
    .AssetReviewModel(MakeShared<FAssetReviewModel>()).InspectionProjection(MakeShared<FInspectionProjection>())
    .SelectionFeedback(MakeShared<FString>()).FrontEndLoginRequired(true);
  HUD->SlatePrepass(1.F);
  auto Start = FindButton(HUD, TEXT("开始"));
  if (!TestTrue(TEXT("entry remains visible"), Start.IsValid())) return false;
  TestFalse(TEXT("no configured session cannot enter"), Start->IsEnabled());
  TestTrue(TEXT("missing catalog is explained"), HasVisibleText(HUD, TEXT("尚未载入场景配置，请检查本地目录。")));
  TestFalse(TEXT("missing catalog never authenticates"), Login->IsLoggedIn());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWelcomeMenusTest, "LingTuSim.UI.Runtime.Welcome.SettingsAndExit",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FWelcomeMenusTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Mode = MakeShared<FRuntimeUIModeController>();
  Mode->TogglePause();
  auto Login = MakeShared<FFrontEndLoginModel>();
  auto Selection = MakeShared<FGameSelectionModel>();
  Selection->SetCatalog(RobotMapCatalog());
  auto Settings = MakeShared<FPlayerSettings>();
  int32 ExitCount = 0;
  auto HUD = SNew(SLingTuSimRuntimeHUD).ModeController(Mode)
    .LocalState(MakeShared<FRuntimeUILocalState>()).PlayerSettings(Settings)
    .LoginModel(Login).SelectionModel(Selection).AssetReviewModel(MakeShared<FAssetReviewModel>())
    .InspectionProjection(MakeShared<FInspectionProjection>()).SelectionFeedback(MakeShared<FString>())
    .FrontEndLoginRequired(true).OnRequestExit_Lambda([&ExitCount]() { ++ExitCount; });
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[HUD];
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool WasActive = App.IsActive();
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(WasActive);
    App.RequestDestroyWindow(Window);
  };
  HUD->SlatePrepass(1.F);
  auto OpenSettings = FindButton(HUD, TEXT("设置"));
  if (!TestTrue(TEXT("settings entry exists before login"), OpenSettings.IsValid())) return false;
  AcceptButton(OpenSettings.ToSharedRef());
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("existing graphics settings are visible"), HasVisibleText(HUD, TEXT("显示模式")));
  TestFalse(TEXT("main menu is not an overlay intercepting settings"), HasVisibleText(HUD, TEXT("开始")));
  auto Controls = FindButton(HUD, TEXT("操作"));
  if (!TestTrue(TEXT("controls tab exists"), Controls.IsValid())) return false;
  AcceptButton(Controls.ToSharedRef());
  auto Forward = FindButton(HUD, TEXT("前进"));
  if (!TestTrue(TEXT("real key binding control exists"), Forward.IsValid())) return false;
  App.SetKeyboardFocus(Forward, EFocusCause::SetDirectly);
  int32 Actions = 0;
  FLingTuSimRuntimeUIInputProcessor Processor(Mode, HUD, HUD, {},
    [&Actions](auto) { ++Actions; }, []() { return true; }, []() { return false; }, {},
    [HUD]() { return HUD->HandleRuntimeMenuBack(); }, Settings,
    [HUD](const FKeyEvent& Event) { return HUD->HandleKeyBindingCapture(Event); },
    [HUD]() { HUD->CancelKeyBindingCapture(); });
  AcceptButton(Forward.ToSharedRef());
  TestTrue(TEXT("front-end input routes to active settings capture"),
    Processor.HandleKeyDownEvent(App, FKeyEvent(EKeys::I, FModifierKeysState(), 0, false, 0, 0)));
  TestEqual(TEXT("pre-session key binding changes"), Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::I);
  TestEqual(TEXT("binding capture does not publish game actions"), Actions, 0);
  TestTrue(TEXT("back handles the unsaved settings decision"), HUD->HandleRuntimeMenuBack());
  auto Discard = FindButton(HUD, TEXT("放弃修改"));
  if (!TestTrue(TEXT("welcome settings protects unsaved bindings too"), Discard.IsValid())) return false;
  AcceptButton(Discard.ToSharedRef());
  TestEqual(TEXT("discard restores the prior binding before returning home"), Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("focus returns to settings entry"), OpenSettings->HasKeyboardFocus());
  auto Quit = FindButton(HUD, TEXT("退出"));
  if (!TestTrue(TEXT("quit entry exists"), Quit.IsValid())) return false;
  AcceptButton(Quit.ToSharedRef());
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("pre-session quit explanation is truthful"), HasVisibleText(HUD, TEXT("退出灵途？当前尚未启动仿真。")));
  TestEqual(TEXT("opening confirmation does not exit"), ExitCount, 0);
  TestTrue(TEXT("cancel returns without exit"), HUD->HandleRuntimeMenuBack());
  TestTrue(TEXT("focus returns to quit entry"), Quit->HasKeyboardFocus());
  AcceptButton(Quit.ToSharedRef());
  auto Confirm = FindButton(HUD, TEXT("确认"));
  if (!TestTrue(TEXT("selector can confirm quit without a fake runtime binding"), Confirm.IsValid() && Confirm->IsEnabled())) return false;
  AcceptButton(Confirm.ToSharedRef());
  TestEqual(TEXT("quit uses the existing owner delegate"), ExitCount, 1);
  TestFalse(TEXT("settings and quit never authenticate"), Login->IsLoggedIn());
  TestFalse(TEXT("settings and quit never commit a session"), Selection->IsConfirmed());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotMapSelectionModelTest, "LingTuSim.UI.Runtime.RobotMapPicker.ExactPair",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotMapSelectionModelTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FGameSelectionModel Model;
  Model.SetCatalog(RobotMapCatalog());
  FString Blocker;
  TestEqual(TEXT("robots deduplicate by package identity"), Model.GetRobotChoices().Num(), 2);
  TestEqual(TEXT("maps deduplicate without inventing bundles"), Model.GetWorldChoices().Num(), 2);
  TestTrue(TEXT("selecting the other robot is supported"), Model.SelectRobotById(TEXT("cart")));
  if (!TestNotNull(TEXT("the declared cart pair exists"), Model.GetSelectedOption())) return false;
  TestFalse(TEXT("preview robot cannot launch"), Model.CanConfirm(Blocker));
  TestEqual(TEXT("exact version comes from selected session"), Model.GetSelectedOption()->World.Version, FString(TEXT("1.1.0")));
  TestTrue(TEXT("map selection can express an unsupported pair"), Model.SelectWorldById(TEXT("open")));
  TestNull(TEXT("missing cart/open pair never falls back to Thunder"), Model.GetSelectedOption());
  TestFalse(TEXT("unconfigured pair cannot launch"), Model.CanConfirm(Blocker));
  TestEqual(TEXT("requested robot remains visible"), Model.GetRequestedRobotId(), FString(TEXT("cart")));
  TestEqual(TEXT("requested map remains visible"), Model.GetRequestedWorldId(), FString(TEXT("open")));
  TestFalse(TEXT("unknown robot is rejected"), Model.SelectRobotById(TEXT("missing")));
  TestEqual(TEXT("unknown selection leaves request unchanged"), Model.GetRequestedRobotId(), FString(TEXT("cart")));
  TestTrue(TEXT("selecting Thunder preserves requested map"), Model.SelectRobotById(TEXT("thunder")));
  if (!TestNotNull(TEXT("Thunder/open resolves"), Model.GetSelectedOption())) return false;
  TestEqual(TEXT("handoff uses the exact open session"), Model.GetSelectedOption()->SessionId, FString(TEXT("open_session")));
  TestTrue(TEXT("real pair can confirm"), Model.MarkConfirmed(Blocker));
  TestFalse(TEXT("robot is frozen after handoff"), Model.SelectRobotById(TEXT("cart")));
  TestFalse(TEXT("map is frozen after handoff"), Model.SelectWorldById(TEXT("factory")));
  TestEqual(TEXT("committed option remains exact"), Model.GetConfirmedOptionId(), FString(TEXT("thunder_open")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotSourceLibrarySelectionTest, "LingTuSim.UI.Runtime.RobotMapPicker.ImportedBodies",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotSourceLibrarySelectionTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = RobotMapCatalog();
  Catalog.RobotLibrary = {{TEXT("unitree_go2"), TEXT("2.3.2"), TEXT("Unitree Go2")},
                         {TEXT("thunder"), TEXT("source"), TEXT("Source Thunder")}};
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(MoveTemp(Catalog));
  TestEqual(TEXT("imported robot appears once; runnable metadata takes precedence"), Model->GetRobotChoices().Num(), 3);
  TestEqual(TEXT("session robot label remains authoritative"), Model->GetRobotChoices()[0].Label, FString(TEXT("雷霆 V4")));
  TestTrue(TEXT("imported body can be browsed"), Model->SelectRobotById(TEXT("unitree_go2")));
  TestNull(TEXT("import does not manufacture a session"), Model->GetSelectedOption());
  FString Blocker;
  TestFalse(TEXT("source body alone cannot launch"), Model->CanConfirm(Blocker));
  int32 LaunchCount = 0;
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true)
    .CanConfirm_Lambda([Model]() { FString Reason; return Model->CanConfirm(Reason); })
    .OnConfirm_Lambda([&LaunchCount]() { ++LaunchCount; return FReply::Handled(); });
  Picker->SlatePrepass(1.F);
  TestTrue(TEXT("imported robot is a real list control"), FindButton(Picker, TEXT("Unitree Go2")).IsValid());
  TestTrue(TEXT("unconnected control is explicit"), HasVisibleText(Picker, TEXT("模型已入库 · 驾驶控制尚未接入")));
  Picker->PrimaryAction();
  Picker->PrimaryAction();
  TestEqual(TEXT("no compatible session prevents review/launch"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  TestEqual(TEXT("no launch callback"), LaunchCount, 0);
  Model->SelectRobotById(TEXT("thunder"));
  TestTrue(TEXT("returning to a configured robot restores existing launch eligibility"), Model->CanConfirm(Blocker));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotCategorySearchTest, "LingTuSim.UI.Runtime.RobotMapPicker.CategorySearch",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotCategorySearchTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = RobotMapCatalog();
  Catalog.RobotLibrary = {
    {TEXT("unitree_go2"), TEXT("2.3.2"), TEXT("Unitree Go2"), TEXT("quadruped")},
    {TEXT("lite3"), TEXT("2.3.2"), TEXT("Lite3"), TEXT("quadruped")},
    {TEXT("unitree_h1"), TEXT("1"), TEXT("Unitree H1"), TEXT("humanoid")},
    {TEXT("ur5e"), TEXT("1"), TEXT("UR5e"), TEXT("manipulator")}
  };
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(MoveTemp(Catalog));
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true).CanConfirm(false);
  Picker->SlatePrepass(1.F);
  TestEqual(TEXT("all categories initially visible"), Picker->GetVisibleRobotCount(), 6);
  auto HumanTab = FindButton(Picker, TEXT("人形"));
  if (!TestTrue(TEXT("category is an actual control"), HumanTab.IsValid())) return false;
  AcceptButton(HumanTab.ToSharedRef());
  TestEqual(TEXT("human filter has one match"), Picker->GetVisibleRobotCount(), 1);
  TestEqual(TEXT("preview selection follows filter"), Model->GetRequestedRobotId(), FString(TEXT("unitree_h1")));
  TestFalse(TEXT("wheel-legged row is not focusable in human list"), FindButton(Picker, TEXT("雷霆 V4")).IsValid());
  TestNull(TEXT("filter cannot manufacture a playable session"), Model->GetSelectedOption());
  const auto SearchWidget = FindTaggedWidget(Picker, TEXT("LingTu.Selection.Search"));
  if (!TestTrue(TEXT("search exists"), SearchWidget.IsValid())) return false;
  const auto Search = StaticCastSharedPtr<SEditableTextBox>(SearchWidget);
  Search->SetText(FText::FromString(TEXT("UR5E")));
  TestEqual(TEXT("search intersects the category"), Picker->GetVisibleRobotCount(), 0);
  Picker->PrimaryAction();
  TestEqual(TEXT("empty results cannot advance"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::Robot);
  TestTrue(TEXT("empty result is explicit"), HasVisibleText(Picker, TEXT("没有匹配结果")));
  Picker->SetRobotCategory(TEXT("all"));
  TestEqual(TEXT("search is case insensitive"), Picker->GetVisibleRobotCount(), 1);
  TestEqual(TEXT("matching robot becomes selected"), Model->GetRequestedRobotId(), FString(TEXT("ur5e")));
  TestTrue(TEXT("back clears search before leaving picker"), Picker->GoBack());
  TestEqual(TEXT("clearing search restores the full list"), Picker->GetVisibleRobotCount(), 6);
  Picker->SetRobotCategory(TEXT("wheeled_legged"));
  if (!TestNotNull(TEXT("filtered robot has the existing session"), Model->GetSelectedOption())) return false;
  TestEqual(TEXT("wheel-legged category restores actual matching session"), Model->GetSelectedOption()->Id, FString(TEXT("thunder_factory")));
  Picker->PrimaryAction();
  Picker->GoBack();
  TestEqual(TEXT("filter survives map and back"), Picker->GetRobotCategory(), FString(TEXT("wheeled_legged")));

  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[Picker];
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool WasActive = App.IsActive();
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(WasActive);
    App.RequestDestroyWindow(Window);
  };
  Picker->SlatePrepass(1.F);
  App.SetKeyboardFocus(Search, EFocusCause::SetDirectly);
  TestTrue(TEXT("search owns text focus"), Picker->IsSearchEditing());
  auto Mode = MakeShared<FRuntimeUIModeController>();
  Mode->TogglePause();
  int32 Actions = 0;
  FLingTuSimRuntimeUIInputProcessor Input(Mode, Picker, Picker, {}, [&](ERuntimeUIAction) { ++Actions; },
    []() { return true; }, [Picker]() { return Picker->IsSearchEditing(); });
  TestFalse(TEXT("B while searching is text, not a menu shortcut"), Input.HandleKeyDownEvent(App,
    FKeyEvent(EKeys::B, FModifierKeysState(), 0, false, 0, 0)));
  TestEqual(TEXT("typing never sends a lifecycle action"), Actions, 0);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotMapPickerInteractionTest, "LingTuSim.UI.Runtime.RobotMapPicker.SelectionAndReadOnly",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotMapPickerInteractionTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(RobotMapCatalog());
  bool bEditable = true;
  FString Committed;
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model)
    .CanEdit_Lambda([&]() { return bEditable; })
    .CanConfirm_Lambda([&]() { FString Blocker; return Model->CanConfirm(Blocker); })
    .OnConfirm_Lambda([&]() { Committed = Model->GetSelectedOption()->SessionId; return FReply::Handled(); });
  Picker->SlatePrepass(1.F);
  auto Cart = FindButton(Picker, TEXT("OmniCart"));
  auto Thunder = FindButton(Picker, TEXT("雷霆 V4"));
  if (!Cart || !Thunder) { AddError(TEXT("robot controls are missing")); return false; }
  TestFalse(TEXT("maps are on a separate step"), FindButton(Picker, TEXT("开放场地")).IsValid());
  AcceptButton(Cart.ToSharedRef());
  Picker->PrimaryAction();
  TestEqual(TEXT("robot next opens map page"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  auto Open = FindButton(Picker, TEXT("开放场地"));
  if (!TestTrue(TEXT("map button is visible on step two"), Open.IsValid())) return false;
  Picker->PrimaryAction();
  auto Enter = FindButton(Picker, TEXT("进入场景"));
  if (!TestTrue(TEXT("map page has the enter button"), Enter.IsValid())) return false;
  TestFalse(TEXT("preview cannot launch from map page"), Enter->IsEnabled());
  AcceptButton(Open.ToSharedRef());
  TestNull(TEXT("widget cannot synthesize a missing bundle"), Model->GetSelectedOption());
  Picker->PrimaryAction();
  TestEqual(TEXT("unsupported pair cannot advance"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  Picker->GoBack();
  AcceptButton(Thunder.ToSharedRef());
  Picker->PrimaryAction();
  Enter = FindButton(Picker, TEXT("进入场景"));
  if (!TestTrue(TEXT("map launch is reachable for supported pair"), Enter.IsValid())) return false;
  TestTrue(TEXT("steps did not dispatch a premature launch"), Committed.IsEmpty());
  TestTrue(TEXT("real pair enables enter"), Enter->IsEnabled());
  AcceptButton(Enter.ToSharedRef());
  TestEqual(TEXT("actual confirm delegates the chosen session"), Committed, FString(TEXT("open_session")));
  bEditable = false;
  Picker->GoBack();
  Picker->SlatePrepass(1.F);
  TestFalse(TEXT("runtime browsing disables robot changes"), Cart->IsEnabled());
  AcceptButton(Cart.ToSharedRef());
  TestEqual(TEXT("disabled callback cannot change robot"), Model->GetRequestedRobotId(), FString(TEXT("thunder")));
  Picker->PrimaryAction();
  auto ReadOnlyEnter = FindButton(Picker, TEXT("进入场景"));
  if (!TestTrue(TEXT("read-only map remains reachable"), ReadOnlyEnter.IsValid())) return false;
  TestFalse(TEXT("runtime browsing disables handoff"), ReadOnlyEnter->IsEnabled());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotMapCompatibilityTest, "LingTuSim.UI.Runtime.RobotMapPicker.CurrentRobotCompatibility",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotMapCompatibilityTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = RobotMapCatalog();
  Catalog.RobotLibrary.Add({TEXT("g1"), TEXT("1"), TEXT("G1"), TEXT("humanoid")});
  auto Isolated = Catalog.Options[0];
  Isolated.Id = TEXT("isolated");
  Isolated.World = {TEXT("isolated_world"), TEXT("1"), TEXT("隔离场地")};
  Isolated.Availability = EGameSelectionAvailability::Quarantined;
  Catalog.Options.Add(Isolated);
  auto Missing = Isolated;
  Missing.Id = TEXT("missing");
  Missing.World = {TEXT("missing_world"), TEXT("1"), TEXT("缺失场地")};
  Missing.Availability = EGameSelectionAvailability::Unavailable;
  Catalog.Options.Add(Missing);
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(MoveTemp(Catalog));
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true).CanConfirm(false);
  Picker->PrimaryAction();
  auto Factory = FindButton(Picker, TEXT("工业园区"));
  auto Open = FindButton(Picker, TEXT("开放场地"));
  auto Quarantined = FindButton(Picker, TEXT("隔离场地"));
  auto Unavailable = FindButton(Picker, TEXT("缺失场地"));
  if (!Factory || !Open || !Quarantined || !Unavailable) { AddError(TEXT("map rows missing")); return false; }
  TestTrue(TEXT("runnable count is for Thunder only"), HasVisibleText(Factory.ToSharedRef(), TEXT("可运行 · 1 项配置")));
  TestTrue(TEXT("quarantine stays explicit"), HasVisibleText(Quarantined.ToSharedRef(), TEXT("已隔离 · 不可运行")));
  TestTrue(TEXT("unavailable is not disguised as preview"), HasVisibleText(Unavailable.ToSharedRef(), TEXT("配置不可用")));
  Model->SelectRobotById(TEXT("cart"));
  Picker->SlatePrepass(1.F);
  TestTrue(TEXT("same row now describes Cart preview"), HasVisibleText(Factory.ToSharedRef(), TEXT("仅预览 · 暂不可驾驶")));
  TestTrue(TEXT("Cart cannot inherit Thunder open-field readiness"), HasVisibleText(Open.ToSharedRef(), TEXT("当前机器人未适配")));
  Model->SelectRobotById(TEXT("g1"));
  Picker->SlatePrepass(1.F);
  TestTrue(TEXT("source-only G1 has no runnable map"), HasVisibleText(Factory.ToSharedRef(), TEXT("当前机器人未适配")));
  Picker->PrimaryAction();
  TestEqual(TEXT("source-only robot stays on map page"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  Model->SelectRobotById(TEXT("thunder"));
  Picker->SlatePrepass(1.F);
  TestTrue(TEXT("returning to Thunder updates without rebuilding rows"), HasVisibleText(Factory.ToSharedRef(), TEXT("可运行 · 1 项配置")));
  TestFalse(TEXT("browsing never commits"), Model->IsConfirmed());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotMapDirectLaunchTest, "LingTuSim.UI.Runtime.RobotMapPicker.TwoStepLaunch",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotMapDirectLaunchTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = RobotMapCatalog();
  auto Variant = Catalog.Options[0];
  Variant.Id = TEXT("variant");
  Variant.Title = TEXT("指定巡检配置");
  Variant.Description = TEXT("由当前目录提供的任务说明");
  Variant.SessionId = TEXT("variant_session");
  Catalog.Options.Add(Variant);
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(MoveTemp(Catalog));
  Model->SelectById(TEXT("variant"));
  bool bEditable = true;
  int32 Launches = 0;
  FString LaunchedSession;
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit_Lambda([&]() { return bEditable; })
    .CanConfirm_Lambda([Model]() { FString Blocker; return Model->CanConfirm(Blocker); })
    .OnConfirm_Lambda([&]() {
      LaunchedSession = Model->GetSelectedOption()->SessionId;
      FString Blocker;
      Model->MarkConfirmed(Blocker);
      ++Launches;
      return FReply::Handled();
    });
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[Picker];
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool WasActive = App.IsActive();
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    if (PreviousFocus) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(WasActive);
    App.RequestDestroyWindow(Window);
  };
  Picker->PrimaryAction();
  TestEqual(TEXT("one next reaches final page"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  TestTrue(TEXT("map page shows the selected description"), HasVisibleText(Picker, Variant.Description));
  TestFalse(TEXT("no redundant third step"), HasVisibleText(Picker, TEXT("03  出发")));
  TestFalse(TEXT("no redundant confirm-page button"), FindButton(Picker, TEXT("下一步：确认配置")).IsValid());
  for (const FString Action : {TEXT("01  机器人"), TEXT("上一步")}) {
    auto Back = FindButton(Picker, Action);
    if (!TestTrue(TEXT("return is a real button"), Back.IsValid())) return false;
    AcceptButton(Back.ToSharedRef());
    TestEqual(TEXT("return goes directly to robot"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::Robot);
    const auto Row = FindButton(Picker, TEXT("雷霆 V4"));
    TestTrue(TEXT("focus returns to the selected row"), Row.IsValid() && Row->HasKeyboardFocus());
    if (!TestNotNull(TEXT("edit preserves the exact variant"), Model->GetSelectedOption())) return false;
    TestEqual(TEXT("edit never silently chooses the first configuration"), Model->GetSelectedOption()->Id, Variant.Id);
    Picker->PrimaryAction();
    TestEqual(TEXT("back at map with exact task preserved"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  }
  TestEqual(TEXT("editing never launches"), Launches, 0);
  bEditable = false;
  Picker->SlatePrepass(1.F);
  auto Enter = FindButton(Picker, TEXT("进入场景"));
  if (!TestTrue(TEXT("map contains the final launch action"), Enter.IsValid())) return false;
  TestFalse(TEXT("read-only session cannot launch"), Enter->IsEnabled());
  AcceptButton(Enter.ToSharedRef());
  TestEqual(TEXT("read-only click does not launch"), Launches, 0);
  bEditable = true;
  Picker->SlatePrepass(1.F);
  AcceptButton(Enter.ToSharedRef());
  TestEqual(TEXT("map click hands off without another page"), Launches, 1);
  TestEqual(TEXT("handoff uses the exact selected task"), LaunchedSession, Variant.SessionId);
  Picker->SlatePrepass(1.F);
  TestFalse(TEXT("committed selection cannot launch twice"), Enter->IsEnabled());
  Picker->PrimaryAction();
  TestEqual(TEXT("repeated launch is rejected"), Launches, 1);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotMapPickerExactConfigurationTest, "LingTuSim.UI.Runtime.RobotMapPicker.ExplicitConfiguration",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotMapPickerExactConfigurationTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = RobotMapCatalog();
  auto Crossing = Catalog.Options[0];
  Crossing.Id = TEXT("thunder_crossing");
  Crossing.Title = TEXT("行人穿越");
  Crossing.SessionId = TEXT("crossing_session");
  Crossing.World.Version = TEXT("1.3.0");
  auto Review = Crossing;
  Review.Id = TEXT("thunder_review");
  Review.Title = TEXT("预览配置");
  Review.Availability = EGameSelectionAvailability::PreviewOnly;
  Review.BundleDirectory.Reset();
  Catalog.Options.Add(Crossing);
  Catalog.Options.Add(Review);
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(MoveTemp(Catalog));
  Model->SelectById(TEXT("thunder_review"));
  Model->SelectRobotById(TEXT("thunder"));
  TestEqual(TEXT("reselecting a package preserves exact preview configuration"), Model->GetSelectedOption()->Id, FString(TEXT("thunder_review")));
  Model->SelectWorldById(TEXT("open"));
  Model->SelectWorldById(TEXT("factory"));
  TestNull(TEXT("ambiguous pair cannot silently select the first runnable option"), Model->GetSelectedOption());
  FString Blocker;
  TestFalse(TEXT("ambiguous pair blocks confirmation"), Model->CanConfirm(Blocker));
  FString Session;
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true)
    .CanConfirm_Lambda([&]() { return Model->CanConfirm(Blocker); })
    .OnConfirm_Lambda([&]() { Session = Model->GetSelectedOption()->SessionId; return FReply::Handled(); });
  Picker->SlatePrepass(1.F);
  Picker->PrimaryAction();
  auto Choice = FindButton(Picker, TEXT("行人穿越"));
  if (!Choice) { AddError(TEXT("exact configuration control is missing")); return false; }
  Picker->PrimaryAction();
  TestEqual(TEXT("ambiguous pair blocks advance"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  AcceptButton(Choice.ToSharedRef());
  Model->SelectWorldById(TEXT("factory"));
  TestEqual(TEXT("reselecting map retains the chosen variant"), Model->GetSelectedOption()->Id, FString(TEXT("thunder_crossing")));
  TestEqual(TEXT("selected variant owns its version"), Model->GetSelectedOption()->World.Version, FString(TEXT("1.3.0")));
  auto Enter = FindButton(Picker, TEXT("进入场景"));
  if (!TestTrue(TEXT("map shows enter action"), Enter.IsValid())) return false;
  AcceptButton(Enter.ToSharedRef());
  TestEqual(TEXT("explicit configuration determines exact handoff"), Session, FString(TEXT("crossing_session")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotMapPreviewStepFlowTest, "LingTuSim.UI.Runtime.RobotMapPicker.PreviewStepFlow",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FRobotMapPreviewStepFlowTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  auto Catalog = RobotMapCatalog();
  Catalog.Options.SetNum(1);
  Catalog.Options[0].Robot.Id = TEXT("thunderv4");
  Catalog.Options[0].World.Id = TEXT("factory_park_hf");
  auto Model = MakeShared<FGameSelectionModel>();
  Model->SetCatalog(MoveTemp(Catalog));
  int32 Launches = 0;
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(Model).CanEdit(true).CanConfirm(true)
    .OnConfirm_Lambda([&]() { ++Launches; return FReply::Handled(); });
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720)).SupportsMaximize(false).SupportsMinimize(false)[Picker];
  auto& App = FSlateApplication::Get();
  const bool bWasActive = App.IsActive();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(bWasActive);
    App.RequestDestroyWindow(Window);
  };
  Picker->SlatePrepass(1.F);
  Picker->FocusInitialControl();
  TestTrue(TEXT("initial control owns keyboard focus"), Picker->HasFocusedDescendants());
  auto Preview = FindPreviewButton(Picker);
  TestFalse(TEXT("robot inspection stays inline, without a full-size overlay"), Picker->IsPreviewExpanded());
  TestTrue(TEXT("inline model interaction remains"), FindTaggedWidget(Picker, TEXT("LingTu.Selection.Model")).IsValid());
  TestFalse(TEXT("no redundant reset toolbar"), FindButton(Picker, TEXT("复位")).IsValid());
  TestFalse(TEXT("no redundant view preset toolbar"), FindButton(Picker, TEXT("正面")).IsValid());
  TestFalse(TEXT("no persistent gesture instructions"), HasVisibleText(Picker, TEXT("拖动旋转")));
  auto Next = FindButton(Picker, TEXT("下一步：选择地图"));
  if (!TestTrue(TEXT("robot next action exists"), Next.IsValid())) return false;
  AcceptButton(Next.ToSharedRef());
  TestEqual(TEXT("next opens only map selection"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  if (!TestTrue(TEXT("map preview resource is available"), Preview.IsValid() && Preview->IsEnabled())) return false;
  AcceptButton(Preview.ToSharedRef());
  TestTrue(TEXT("map still supports expanded image preview"), Picker->IsPreviewExpanded());
  Picker->PrimaryAction();
  TestEqual(TEXT("expanded map cannot advance the underlying page"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::World);
  TestTrue(TEXT("back closes map preview first"), Picker->GoBack());
  TestFalse(TEXT("map preview closed"), Picker->IsPreviewExpanded());
  auto Enter = FindButton(Picker, TEXT("进入场景"));
  if (!TestTrue(TEXT("map directly exposes launch"), Enter.IsValid())) return false;
  TestTrue(TEXT("valid map can launch after closing preview"), Enter->IsEnabled());
  TestEqual(TEXT("preview and back never launch"), Launches, 0);
  auto Mode = MakeShared<FRuntimeUIModeController>();
  Mode->TogglePause();
  int32 FallbackActions = 0;
  FLingTuSimRuntimeUIInputProcessor Input(Mode, Picker, Picker, {},
    [&](ERuntimeUIAction) { ++FallbackActions; }, []() { return true; }, []() { return false; }, {},
    [Picker]() { return Picker->GoBack(); }, {}, {}, {}, [Picker]() { Picker->FocusInitialControl(); });
  TestTrue(TEXT("selector Escape is consumed by the current step"), Input.HandleKeyDownEvent(App,
    FKeyEvent(EKeys::Escape, FModifierKeysState(), 0, false, 0, 0)));
  TestEqual(TEXT("Escape returns directly to robot selection"), Picker->GetStep(), SLingTuSimRobotMapPicker::EStep::Robot);
  TestEqual(TEXT("step back never logs out or dispatches a lifecycle action"), FallbackActions, 0);
  TestEqual(TEXT("selection survives navigation"), Model->GetSelectedOption()->SessionId, FString(TEXT("factory_session")));
  for (int32 Page = 0; Page < 2; ++Page) {
    App.SetKeyboardFocus(Picker, EFocusCause::SetDirectly);
    TestFalse(TEXT("root focus has no focused child"), Picker->HasFocusedDescendants());
    TestFalse(TEXT("selector arrows go to Slate after restoring focus"), Input.HandleKeyDownEvent(App,
      FKeyEvent(EKeys::Down, FModifierKeysState(), 0, false, 0, 0)));
    TestTrue(TEXT("current step regains child focus"), Picker->HasFocusedDescendants());
    TestEqual(TEXT("lost focus cannot dispatch a legacy session change"), FallbackActions, 0);
    TestEqual(TEXT("navigation never changes the selected robot"), Model->GetRequestedRobotId(), FString(TEXT("thunderv4")));
    if (Page == 0) Picker->PrimaryAction();
    else Picker->GoBack();
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerRobotMapLibraryEntryTest, "LingTuSim.UI.Runtime.PlayerMenu.RobotMapLibraryEntry",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerRobotMapLibraryEntryTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Entry = FindButton(HUD, TEXT("机器人与地图"));
  if (!TestTrue(TEXT("player menu exposes the robot and map library"), Entry.IsValid())) return false;
  AcceptButton(Entry.ToSharedRef());
  TestTrue(TEXT("robot selection is the initial page"), HasVisibleText(HUD, TEXT("选择机器人")));
  TestFalse(TEXT("map selection is a separate page"), HasVisibleText(HUD, TEXT("选择地图")));
  TestEqual(TEXT("browsing does not enter drive"), Fixture.Mode->GetMode(), LingTuSim::UI::ERuntimeUIMode::Pause);
  TestTrue(TEXT("back returns before resuming gameplay"), HUD->HandleRuntimeMenuBack());
  TestTrue(TEXT("menu root returns"), FindButton(HUD, TEXT("继续游戏")).IsValid());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerTaskJournalNavigationTest, "LingTuSim.UI.Runtime.PlayerMenu.TaskJournalNavigation",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerTaskJournalNavigationTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Journal = FindButton(HUD, TEXT("任务日志"));
  if (!TestTrue(TEXT("task journal has a player-facing menu entry"), Journal.IsValid())) return false;
  AcceptButton(Journal.ToSharedRef());
  TestTrue(TEXT("unbound run explains why no task is shown"), HasVisibleText(HUD, TEXT("当前会话未绑定巡检任务")));
  TestEqual(TEXT("opening a journal never resumes drive"), Fixture.Mode->GetMode(), LingTuSim::UI::ERuntimeUIMode::Pause);
  TestFalse(TEXT("reading a journal cannot fabricate task binding"), Fixture.Inspection->bBound);
  TestTrue(TEXT("Escape pops journal before closing menu"), HUD->HandleRuntimeMenuBack());
  TestTrue(TEXT("root menu is restored"), FindButton(HUD, TEXT("继续游戏")).IsValid());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerMeasurementsNavigationTest, "LingTuSim.UI.Runtime.PlayerMenu.MeasurementsNavigation",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerMeasurementsNavigationTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 900))[HUD];
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool bWasActive = App.IsActive();
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  ON_SCOPE_EXIT {
    if (PreviousFocus) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(bWasActive);
    App.RequestDestroyWindow(Window);
  };
  TArray<ERuntimeUIMode> PublishedModes;
  bool bMotionPublished = false;
  FLingTuSimRuntimeUIInputProcessor Input(Fixture.Mode.ToSharedRef(), HUD, HUD,
    [&](const FRobotDriveInputSnapshot& Snapshot, ERobotDrivePublishReason) {
      PublishedModes.Add(Fixture.Mode->GetMode()); bMotionPublished |= Snapshot.bDeadman;
    }, {}, []() { return false; }, [HUD]() { return HUD->IsTextEntryEditing(); }, {},
    [HUD]() { return HUD->HandleRuntimeMenuBack(); });
  auto Entry = FindButton(HUD, TEXT("传感器"));
  if (!TestTrue(TEXT("real runtime menu exposes measurements"), Entry.IsValid())) return false;
  AcceptButton(Entry.ToSharedRef());
  TestTrue(TEXT("measurements stay visible outside pause"), HUD->IsMeasurementPageOpen());
  TestEqual(TEXT("source can keep sampling in non-driving mode"), Fixture.Mode->GetMode(), ERuntimeUIMode::Tactical);
  TestTrue(TEXT("measurement page owns Tab and editing shortcuts"), HUD->IsTextEntryEditing());
  TestFalse(TEXT("Tab passes to Slate navigation"), Input.HandleKeyDownEvent(App,
      FKeyEvent(EKeys::Tab, FModifierKeysState(), 0, false, 0, 0)));
  TestEqual(TEXT("Tab cannot leave sensor page"), Fixture.Mode->GetMode(), ERuntimeUIMode::Tactical);
  TestTrue(TEXT("non-driving mode transition is published"), PublishedModes.Contains(ERuntimeUIMode::Tactical));
  TestFalse(TEXT("opening sensors never publishes driving deadman"), bMotionPublished);
  TestTrue(TEXT("Back returns to the menu"), HUD->HandleRuntimeMenuBack());
  TestEqual(TEXT("return restores paused menu"), Fixture.Mode->GetMode(), ERuntimeUIMode::Pause);
  TestFalse(TEXT("sensor controls no longer own keys"), HUD->IsTextEntryEditing());
  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  HUD->RestoreSelectionFocus();
  TestTrue(TEXT("input-mode synchronization restores the sensor menu button"), Entry->HasKeyboardFocus());
  Fixture.Mode->HandleKey(EKeys::Escape);
  TestEqual(TEXT("closing paused menu restores original driving mode"), Fixture.Mode->GetMode(), ERuntimeUIMode::Drive);
  for (const auto Original : {ERuntimeUIMode::Drive, ERuntimeUIMode::Build, ERuntimeUIMode::Tactical}) {
    FRuntimeUIModeController Mode;
    if (Original == ERuntimeUIMode::Build) Mode.ToggleBuild();
    if (Original == ERuntimeUIMode::Tactical) Mode.ToggleTactical();
    Mode.TogglePause();
    TestTrue(TEXT("live viewer opens from pause"), Mode.SetLiveMenuView(true));
    TestTrue(TEXT("live viewer returns to same pause"), Mode.SetLiveMenuView(false));
    Mode.TogglePause();
    TestEqual(TEXT("live viewing preserves original resume mode"), Mode.GetMode(), Original);
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerTaskJournalResultTest, "LingTuSim.UI.Runtime.PlayerMenu.TaskJournalResults",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerTaskJournalResultTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  auto& Task = *Fixture.Inspection;
  Task.bBound = Task.bTaskAvailable = Task.bLauncherBindingVerified = Task.bProgressVerified = true;
  Task.bExecutionConfirmed = Task.bHistoryComplete = Task.bTerminal = Task.bReportVerified = true;
  Task.State = TEXT("SUCCESS"); Task.Phase = TEXT("SUCCEEDED");
  Task.CompletedPoints = Task.PointCount = 3;
  Task.ReportStatus = TEXT("PARTIAL"); Task.Acceptance = TEXT("REVIEW_REQUIRED");
  Task.VerifiedEvidence = 2; Task.RequiredEvidence = 3;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Journal = FindButton(HUD, TEXT("任务日志"));
  if (!TestTrue(TEXT("task journal entry exists"), Journal.IsValid())) return false;
  AcceptButton(Journal.ToSharedRef());
  TestTrue(TEXT("native completion is shown separately"), HasVisibleText(HUD, TEXT("路线执行完成 · 证据接受结果另行确认")));
  TestTrue(TEXT("partial evidence is never presented as accepted completion"),
    HasVisibleText(HUD, TEXT("报告：部分完成  ·  验收：需要复核  ·  证据 2/3")));
  auto Resume = FindButton(HUD, TEXT("继续巡检"));
  TestTrue(TEXT("terminal task action remains disabled"), Resume.IsValid() && !Resume->IsEnabled());
  Task.bReportStale = true;
  Task.Acceptance = TEXT("ACCEPTABLE");
  HUD->Invalidate(EInvalidateWidgetReason::Layout); HUD->SlatePrepass(1.F);
  TestTrue(TEXT("stale acceptance explains refresh"), HasVisibleText(HUD, TEXT("验收报告等待刷新")));
  TestFalse(TEXT("stale report hides prior evidence values"), HasVisibleText(HUD, TEXT("报告：部分完成  ·  验收：通过  ·  证据 2/3")));
  Task.bStale = true;
  HUD->Invalidate(EInvalidateWidgetReason::Layout); HUD->SlatePrepass(1.F);
  TestTrue(TEXT("disconnected task shows uncertainty rather than old success"), HasVisibleText(HUD, TEXT("任务状态待确认")));
  TestEqual(TEXT("reading never advances native progress"), Task.CompletedPoints, 3);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerMenuSettingsInteractionTest, "LingTuSim.UI.Runtime.PlayerMenu.SettingsInteraction",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerMenuSettingsInteractionTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Settings = FindButton(HUD, TEXT("设置"));
  if (!TestTrue(TEXT("settings menu exists"), Settings.IsValid())) return false;
  AcceptButton(Settings.ToSharedRef());
  TestTrue(TEXT("graphics is the default category"), HasVisibleText(HUD, TEXT("显示模式")));
  auto Controls = FindButton(HUD, TEXT("操作"));
  if (!TestTrue(TEXT("controls category exists"), Controls.IsValid())) return false;
  AcceptButton(Controls.ToSharedRef());
  auto Invert = FindCheckBox(HUD, TEXT("反转垂直视角"));
  if (!TestTrue(TEXT("real mouse control exists"), Invert.IsValid())) return false;
  Invert->ToggleCheckedState();
  TestTrue(TEXT("invert widget changes shared camera preferences"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("keyboard bindings are actionable"), FindButton(HUD, TEXT("前进")).IsValid());
  TestTrue(TEXT("actual keyboard key is visible"), HasVisibleText(HUD, TEXT("W")));
  auto Interface = FindButton(HUD, TEXT("界面"));
  if (!TestTrue(TEXT("interface category exists"), Interface.IsValid())) return false;
  AcceptButton(Interface.ToSharedRef());
  auto Hints = FindCheckBox(HUD, TEXT("显示操作提示"));
  auto FPS = FindCheckBox(HUD, TEXT("显示帧率与帧时间"));
  if (!TestTrue(TEXT("interface controls exist"), Hints.IsValid() && FPS.IsValid())) return false;
  Hints->ToggleCheckedState();
  FPS->ToggleCheckedState();
  TestTrue(TEXT("frame timing can be shown independently"), Fixture.Settings->bShowFrameRate);
  TestFalse(TEXT("hint widget changes shared HUD preferences"), Fixture.Settings->bShowControlHints);
  Fixture.Settings->MouseSensitivity = 2.F;
  auto Defaults = FindButton(HUD, TEXT("恢复默认"));
  if (!TestTrue(TEXT("defaults is actionable"), Defaults.IsValid())) return false;
  AcceptButton(Defaults.ToSharedRef());
  TestEqual(TEXT("interface defaults do not alter mouse sensitivity"), Fixture.Settings->MouseSensitivity, 2.F);
  TestTrue(TEXT("interface defaults do not alter mouse inversion"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("default hints restored"), Fixture.Settings->bShowControlHints);
  TestFalse(TEXT("default frame counter restored"), Fixture.Settings->bShowFrameRate);
  AcceptButton(Controls.ToSharedRef());
  AcceptButton(Defaults.ToSharedRef());
  TestEqual(TEXT("controls defaults restore sensitivity"), Fixture.Settings->MouseSensitivity, 1.F);
  TestFalse(TEXT("controls defaults restore inversion"), Fixture.Settings->bInvertMouseY);
  HUD->OnKeyDown(FGeometry(), FKeyEvent(EKeys::Gamepad_LeftShoulder, FModifierKeysState(), 0, false, 0, 0));
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("gamepad shoulder returns to graphics category"), HasVisibleText(HUD, TEXT("显示模式")));
  TestEqual(TEXT("editing preferences cannot enter drive"), Fixture.Mode->GetMode(), LingTuSim::UI::ERuntimeUIMode::Pause);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerSettingsUndoContractTest,
  "LingTuSim.UI.Runtime.PlayerMenu.SettingsUndoContract",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerSettingsUndoContractTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  const FString Root = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"),
      TEXT("LingTuSimUI"), FGuid::NewGuid().ToString(EGuidFormats::Digits));
  const FString IniPath = FPaths::Combine(Root, TEXT("PlayerSettings.ini"));
  const FString BlockedIniPath = FPaths::Combine(Root, TEXT("BlockedSettings.ini"));
  if (!TestTrue(TEXT("isolated settings directory"), IFileManager::Get().MakeDirectory(*Root, true))) return false;
  if (!TestTrue(TEXT("save target is blocked by a directory"), IFileManager::Get().MakeDirectory(*BlockedIniPath, true))) return false;
  TGuardValue<FString> Guard(GGameUserSettingsIni, IniPath);
  ON_SCOPE_EXIT {
    GConfig->UnloadFile(IniPath);
    GConfig->UnloadFile(BlockedIniPath);
    IFileManager::Get().DeleteDirectory(*Root, false, true);
  };

  GConfig->SetFloat(TEXT("LingTuSim.Graphics"), TEXT("FrameRateLimit"), 60.F, IniPath);

  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  FScopedFieldUITestWorld TestWorld;
  if (!TestNotNull(TEXT("settings contract test world exists"), TestWorld.Get())) return false;
  auto HUD = Fixture.Widget(TestWorld.Get());

  HUD->PreviewPlayerMenuPage(TEXT("settings"));
  auto ApplyGraphics = FindButton(HUD, TEXT("应用画面"));
  auto FrameRate = FindButton(HUD, TEXT("帧率上限"));
  if (!TestTrue(TEXT("graphics draft controls exist"), ApplyGraphics.IsValid() && FrameRate.IsValid())) return false;
  TestFalse(TEXT("unchanged graphics draft cannot be applied"), ApplyGraphics->IsEnabled());
  HUD->ApplyGraphicsSettingsForTest();
  TestFalse(TEXT("direct unchanged apply is a no-op, not a preview"), HUD->IsGraphicsPreviewPendingForTest());
  TestTrue(TEXT("direct no-op reports unchanged values"), HasVisibleText(HUD, TEXT("画面设置未变化")));
  AcceptButton(FrameRate.ToSharedRef());
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("a real frame-rate change marks the graphics draft"), HUD->HasGraphicsDraftChangesForTest());
  TestTrue(TEXT("changed graphics draft enables apply without disabling editing"), ApplyGraphics->IsEnabled());
  AcceptButton(ApplyGraphics.ToSharedRef());
  TestFalse(TEXT("test world without a viewport cannot start the safety preview"), HUD->IsGraphicsPreviewPendingForTest());
  AcceptButton(FindButton(HUD, TEXT("操作")).ToSharedRef());
  AcceptButton(FindButton(HUD, TEXT("画面")).ToSharedRef());
  TestTrue(TEXT("unapplied graphics state is visible after feedback clears"), HasVisibleText(HUD, TEXT("有未应用的画面修改")));
  AcceptButton(FindButton(HUD, TEXT("操作")).ToSharedRef());
  TestTrue(TEXT("graphics draft survives a settings tab round trip"), HUD->HasGraphicsDraftChangesForTest());
  AcceptButton(FindButton(HUD, TEXT("撤销修改")).ToSharedRef());
  TestFalse(TEXT("undo restores the graphics checkpoint without applying"), HUD->HasGraphicsDraftChangesForTest());
  AcceptButton(FindButton(HUD, TEXT("画面")).ToSharedRef());
  ApplyGraphics = FindButton(HUD, TEXT("应用画面"));
  TestFalse(TEXT("restored graphics values disable apply again"), ApplyGraphics->IsEnabled());
  FrameRate = FindButton(HUD, TEXT("帧率上限"));
  AcceptButton(FrameRate.ToSharedRef());
  for (int32 Step = 0; Step < 6 && HUD->HasGraphicsDraftChangesForTest(); ++Step)
    AcceptButton(FrameRate.ToSharedRef());
  HUD->SlatePrepass(1.F);
  TestFalse(TEXT("returning to the original frame-rate value clears dirty"), HUD->HasGraphicsDraftChangesForTest());
  TestFalse(TEXT("value-based dirty guard disables apply after restoring the original"), ApplyGraphics->IsEnabled());

  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  auto Invert = FindCheckBox(HUD, TEXT("反转垂直视角"));
  if (!TestTrue(TEXT("controls edit is available"), Invert.IsValid())) return false;
  Invert->ToggleCheckedState();
  AcceptButton(FindButton(HUD, TEXT("界面")).ToSharedRef());
  auto FPS = FindCheckBox(HUD, TEXT("显示帧率与帧时间"));
  if (!TestTrue(TEXT("interface edit is available"), FPS.IsValid())) return false;
  FPS->ToggleCheckedState();
  AcceptButton(FindButton(HUD, TEXT("操作")).ToSharedRef());
  TestTrue(TEXT("controls edit survives a settings tab round trip"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("interface edit survives a settings tab round trip"), Fixture.Settings->bShowFrameRate);

  auto Undo = FindButton(HUD, TEXT("撤销修改"));
  if (!TestTrue(TEXT("explicit undo is available"), Undo.IsValid())) return false;
  AcceptButton(Undo.ToSharedRef());
  TestFalse(TEXT("undo restores the entry controls value"), Fixture.Settings->bInvertMouseY);
  TestFalse(TEXT("undo restores the entry interface value"), Fixture.Settings->bShowFrameRate);
  TestTrue(TEXT("undo keeps settings open"), HasVisibleText(HUD, TEXT("鼠标灵敏度")));

  Invert = FindCheckBox(HUD, TEXT("反转垂直视角"));
  Invert->ToggleCheckedState();
  auto Save = FindButton(HUD, TEXT("保存设置"));
  if (!TestTrue(TEXT("save is available"), Save.IsValid())) return false;
  AcceptButton(Save.ToSharedRef());
  TestTrue(TEXT("successful save reports completion"), HasVisibleText(HUD, TEXT("设置已保存")));
  TestTrue(TEXT("successful save creates the requested ini file"), IFileManager::Get().FileExists(*IniPath));
  Invert->ToggleCheckedState();
  TestFalse(TEXT("later edit is live before close"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("Back opens an unsaved changes decision"), HUD->HandleRuntimeMenuBack());
  AcceptButton(FindButton(HUD, TEXT("放弃修改")).ToSharedRef());
  TestTrue(TEXT("Back restores the successful save checkpoint"), Fixture.Settings->bInvertMouseY);

  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  Invert = FindCheckBox(HUD, TEXT("反转垂直视角"));
  Invert->ToggleCheckedState();
  GGameUserSettingsIni = BlockedIniPath;
  AcceptButton(FindButton(HUD, TEXT("保存设置")).ToSharedRef());
  TestTrue(TEXT("failed save is observable"),
      HasVisibleText(HUD, TEXT("保存失败，请检查配置文件是否可写；当前修改仍未保存")));
  TestTrue(TEXT("Back requires a decision after failed save"), HUD->HandleRuntimeMenuBack());
  AcceptButton(FindButton(HUD, TEXT("放弃修改")).ToSharedRef());
  TestTrue(TEXT("failed save does not advance the checkpoint"), Fixture.Settings->bInvertMouseY);

  GGameUserSettingsIni = IniPath;
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  FindCheckBox(HUD, TEXT("反转垂直视角"))->ToggleCheckedState();
  Fixture.Mode->TogglePause();
  HUD->Tick(FGeometry(), 0.0, 0.016F);
  TestTrue(TEXT("hiding the menu restores unsaved player settings"), Fixture.Settings->bInvertMouseY);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerAudioSettingsContractTest,
  "LingTuSim.UI.Runtime.PlayerMenu.AudioSettingsContract",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerAudioSettingsContractTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  const FString Root = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"),
      TEXT("LingTuSimUI"), FGuid::NewGuid().ToString(EGuidFormats::Digits));
  const FString IniPath = FPaths::Combine(Root, TEXT("PlayerSettings.ini"));
  if (!TestTrue(TEXT("isolated audio preferences directory"), IFileManager::Get().MakeDirectory(*Root, true))) return false;
  TGuardValue<FString> Guard(GGameUserSettingsIni, IniPath);
  ON_SCOPE_EXIT {
    GConfig->UnloadFile(IniPath);
    IFileManager::Get().DeleteDirectory(*Root, false, true);
  };
  FFieldUIFixture Fixture;
  Fixture.Settings->MouseSensitivity = 2.F;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  HUD->PreviewPlayerMenuPage(TEXT("settings"));
  auto AudioTab = FindButton(HUD, TEXT("声音"));
  if (!TestTrue(TEXT("audio is a real settings category"), AudioTab.IsValid())) return false;
  AcceptButton(AudioTab.ToSharedRef());
  HUD->SlatePrepass(1.F);
  auto MasterWidget = FindTaggedWidget(HUD, TEXT("LingTu.Settings.MasterVolume"));
  auto UIWidget = FindTaggedWidget(HUD, TEXT("LingTu.Settings.UIVolume"));
  auto MusicWidget = FindTaggedWidget(HUD, TEXT("LingTu.Settings.MusicVolume"));
  if (!TestTrue(TEXT("music has an independent live volume control"), MusicWidget.IsValid())) return false;
  auto TrackChoice = FindButton(HUD, TEXT("背景音乐"));
  if (!TestTrue(TEXT("two original music drafts can be selected"), TrackChoice.IsValid())) return false;
  auto Mute = FindCheckBox(HUD, TEXT("静音"));
  if (!TestTrue(TEXT("both audio sliders and mute are visible"), MasterWidget.IsValid() && UIWidget.IsValid() && Mute.IsValid())) return false;
  const FGeometry SliderGeometry = FGeometry::MakeRoot(FVector2D(200, 20), FSlateLayoutTransform());
  const FPointerEvent MiddleClick(0, FVector2D(100, 10), FVector2D(100, 10),
      TSet<FKey>{EKeys::LeftMouseButton}, EKeys::LeftMouseButton, 0, FModifierKeysState());
  auto Master = StaticCastSharedPtr<SSlider>(MasterWidget);
  auto UI = StaticCastSharedPtr<SSlider>(UIWidget);
  auto Music = StaticCastSharedPtr<SSlider>(MusicWidget);
  Master->OnMouseButtonDown(SliderGeometry, MiddleClick);
  UI->OnMouseButtonDown(SliderGeometry, MiddleClick);
  Music->OnMouseButtonDown(SliderGeometry, MiddleClick);
  AcceptButton(TrackChoice.ToSharedRef());
  TestEqual(TEXT("music slider is not the UI-cue slider"), Fixture.Settings->MusicVolume, .5F);
  TestEqual(TEXT("music selection changes independently of cue volume"), Fixture.Settings->MenuMusicTrack, 1);
  TestEqual(TEXT("master slider changes the live preference"), Fixture.Settings->MasterVolume, .5F);
  TestEqual(TEXT("UI slider changes its separate live preference"), Fixture.Settings->UIVolume, .5F);
  Mute->ToggleCheckedState();
  TestEqual(TEXT("mute produces silence without changing the two sliders"), Fixture.Settings->GetMasterAudioGain(), 0.F);
  HUD->SlatePrepass(1.F);
  auto Preview = FindButton(HUD, TEXT("试听界面音效"));
  if (!TestTrue(TEXT("the real preview action exists"), Preview.IsValid())) return false;
  TestFalse(TEXT("preview cannot pretend to be audible while muted"), Preview->IsEnabled());
  AcceptButton(FindButton(HUD, TEXT("撤销修改")).ToSharedRef());
  HUD->SlatePrepass(1.F);
  TestEqual(TEXT("undo restores master"), Fixture.Settings->MasterVolume, 1.F);
  TestEqual(TEXT("undo restores UI level"), Fixture.Settings->UIVolume, 1.F);
  TestFalse(TEXT("undo restores mute"), Fixture.Settings->bMuteAudio);
  TestEqual(TEXT("undo restores music volume"), Fixture.Settings->MusicVolume, .5F);
  TestEqual(TEXT("undo restores music selection"), Fixture.Settings->MenuMusicTrack, 0);
  TestEqual(TEXT("audio editing does not reset mouse sensitivity"), Fixture.Settings->MouseSensitivity, 2.F);
  Master->OnMouseButtonDown(SliderGeometry, MiddleClick);
  UI->OnMouseButtonDown(SliderGeometry, MiddleClick);
  Music->OnMouseButtonDown(SliderGeometry, MiddleClick);
  AcceptButton(TrackChoice.ToSharedRef());
  Mute->ToggleCheckedState();
  AcceptButton(FindButton(HUD, TEXT("保存设置")).ToSharedRef());
  FPlayerSettings Loaded;
  Loaded.Load(IniPath);
  TestEqual(TEXT("saved master survives loading preferences"), Loaded.MasterVolume, .5F);
  TestEqual(TEXT("saved UI level survives loading preferences"), Loaded.UIVolume, .5F);
  TestTrue(TEXT("saved mute survives loading preferences"), Loaded.bMuteAudio);
  TestEqual(TEXT("saved music volume survives loading preferences"), Loaded.MusicVolume, .5F);
  TestEqual(TEXT("saved music choice survives loading preferences"), Loaded.MenuMusicTrack, 1);
  Mute->ToggleCheckedState();
  HUD->HandleRuntimeMenuBack();
  auto Discard = FindButton(HUD, TEXT("放弃修改"));
  if (!TestTrue(TEXT("leaving an audio edit requires the unsaved decision"), Discard.IsValid())) return false;
  AcceptButton(Discard.ToSharedRef());
  TestTrue(TEXT("discard returns to the saved audio checkpoint"), Fixture.Settings->bMuteAudio);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldMenuNotWorkspaceTest, "LingTuSim.UI.Runtime.FieldInterface.MenuNotWorkspace",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldMenuNotWorkspaceTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  TestTrue(TEXT("runtime menu is visible"), HasVisibleText(HUD, TEXT("菜单")));
  TestTrue(TEXT("pause menu exposes current task context"), HasVisibleText(HUD, TEXT("当前任务")));
  TestTrue(TEXT("pause menu exposes session and control context"), HasVisibleText(HUD, TEXT("会话未连接   /   控制未连接")));
  TestFalse(TEXT("runtime menu does not expose the offline selection workspace"), HasVisibleText(HUD, TEXT("部署准备  /  素材审阅")));
  for (const auto* Caption : {TEXT("继续游戏"), TEXT("重新开始"), TEXT("设置"), TEXT("返回主菜单"), TEXT("退出游戏")})
    TestTrue(TEXT("player menu action exists"), FindButton(HUD, Caption).IsValid());
  for (const auto* Caption : {TEXT("切换视角"), TEXT("运行信息"), TEXT("场景预览"), TEXT("会话概况")})
    TestFalse(TEXT("technical actions are not player menu entries"), HasVisibleText(HUD, Caption));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldMenuAcceptTest, "LingTuSim.UI.Runtime.FieldInterface.MenuAccept",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldMenuAcceptTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Resume = FindButton(HUD, TEXT("继续游戏"));
  if (!TestTrue(TEXT("resume button exists"), Resume.IsValid())) return false;
  AcceptButton(Resume.ToSharedRef());
  TestEqual(TEXT("Enter returns to Drive through the actual button delegate"), Fixture.Mode->GetMode(), LingTuSim::UI::ERuntimeUIMode::Drive);
  Fixture.Mode->TogglePause();
  HUD->Invalidate(EInvalidateWidgetReason::Layout);
  HUD->SlatePrepass(1.0F);
  auto Settings = FindButton(HUD, TEXT("设置"));
  if (!TestTrue(TEXT("settings button exists"), Settings.IsValid())) return false;
  AcceptButton(Settings.ToSharedRef());
  TestTrue(TEXT("settings page opens on graphics"), HasVisibleText(HUD, TEXT("显示模式")));
  TestEqual(TEXT("settings does not resume robot drive"), Fixture.Mode->GetMode(), LingTuSim::UI::ERuntimeUIMode::Pause);
  TestTrue(TEXT("Escape can pop settings to root"), HUD->HandleRuntimeMenuBack());
  TestTrue(TEXT("root restored"), FindButton(HUD, TEXT("继续游戏")).IsValid());
  TestFalse(TEXT("root back is left to existing resume policy"), HUD->HandleRuntimeMenuBack());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldUnavailableActionsTest, "LingTuSim.UI.Runtime.FieldInterface.UnavailableActions",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldUnavailableActionsTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  for (const auto* Caption : {TEXT("重新开始"), TEXT("返回主菜单"), TEXT("退出游戏")}) {
    auto Action = FindButton(HUD, Caption);
    if (!TestTrue(TEXT("lifecycle entry exists"), Action.IsValid())) return false;
    AcceptButton(Action.ToSharedRef());
    auto Confirm = FindButton(HUD, TEXT("确认"));
    if (!TestTrue(TEXT("confirmation exists"), Confirm.IsValid())) return false;
    TestFalse(TEXT("unbound lifecycle cannot pretend to execute"), Confirm->IsEnabled());
    TestTrue(TEXT("cancel returns without executing"), HUD->HandleRuntimeMenuBack());
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldMissingTruthTest, "LingTuSim.UI.Runtime.FieldInterface.MissingTruth",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldMissingTruthTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  auto HUD = Fixture.Widget();
  TestTrue(TEXT("missing speed is not zero"), HasVisibleText(HUD, TEXT("—")) && HasVisibleText(HUD, TEXT("m/s")));
  TestFalse(TEXT("missing speed is not presented as measured zero"), HasVisibleText(HUD, TEXT("0.00 m/s")));
  TestTrue(TEXT("unbound view is honestly named a model preview"), HasVisibleText(HUD, TEXT("模型预览")));
  TestTrue(TEXT("the primary link status remains explicitly unavailable"), HasVisibleText(HUD, TEXT("离线预览")));
  TestFalse(TEXT("turn rate is moved out of the compact driving HUD"), HasVisibleText(HUD, TEXT("— °/s")));
  TestFalse(TEXT("long control legend is absent by default"), HasVisibleText(HUD, TEXT("W / A / S / D")));
  TestFalse(TEXT("unbound task diagnostic is not a fake objective"), HasVisibleText(HUD, TEXT("巡检任务未绑定")));
  Fixture.Mode->SetDriveHelpVisible(true);
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("explicit help reveals actionable mapped keys"), HasVisibleText(HUD, TEXT("W / A / S / D")));
  TestTrue(TEXT("explicit help names the details shortcut"), HasVisibleText(HUD, TEXT("运行详情")));
  Fixture.Mode->SetDriveHelpVisible(false);
  HUD->SlatePrepass(1.F);
  TestFalse(TEXT("release hides the guide again"), HasVisibleText(HUD, TEXT("操作指南")));
  TestFalse(TEXT("binding diagnostic is not duplicated on the driving HUD"),
            HasVisibleText(HUD, TEXT("会话未连接   /   控制未连接")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldDrivingReadoutsTest, "LingTuSim.UI.Runtime.FieldInterface.DrivingReadouts",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldDrivingReadoutsTest::RunTest(const FString&) {
  using LingTuSim::UI::SLingTuSimRuntimeHUD;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status;
  Status.bObservedBaseVelocityAvailable = true;
  Status.bIdentityCoherent = true;
  Status.ObservedBaseLinearVelocityMps = FVector(.3, .4, 2.0);
  Status.ObservedBaseAngularVelocityRadps = FVector(1, 2, .5);
  TestEqual(TEXT("driving speed excludes suspension vertical motion"),
            SLingTuSimRuntimeHUD::FormatFieldSpeed(Status).ToString(), FString(TEXT("0.50 m/s")));
  TestEqual(TEXT("compact value separates the unit from horizontal speed"),
            SLingTuSimRuntimeHUD::FormatDriveSpeedValue(Status).ToString(), FString(TEXT("0.5")));
  Status.bFullStatusAvailable = true;
  TestEqual(TEXT("stale status cannot keep a live-looking compact speed"),
            SLingTuSimRuntimeHUD::FormatDriveSpeedValue(Status).ToString(), FString(TEXT("—")));
  Status.bFullStatusAvailable = false;
  TestEqual(TEXT("actual positive yaw rate is readable in degrees"),
            SLingTuSimRuntimeHUD::FormatFieldTurnRate(Status).ToString(), FString(TEXT("+28.6 °/s")));
  Status.ObservedBaseAngularVelocityRadps.Z = -.5;
  TestEqual(TEXT("turn direction remains signed"),
            SLingTuSimRuntimeHUD::FormatFieldTurnRate(Status).ToString(), FString(TEXT("-28.6 °/s")));
  Status.bIdentityCoherent = false;
  TestEqual(TEXT("wrong-generation turn rate cannot look like measured zero"),
            SLingTuSimRuntimeHUD::FormatFieldTurnRate(Status).ToString(), FString(TEXT("— °/s")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldLinkReadinessTest, "LingTuSim.UI.Runtime.FieldInterface.LinkReadiness",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldLinkReadinessTest::RunTest(const FString&) {
  using namespace LingTuSim;
  using LingTuSim::UI::SLingTuSimRuntimeHUD;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status;
  Status.bSessionAvailable = Status.bControlBindingAvailable = Status.bIdentityCoherent = true;
  TestEqual(TEXT("bound is not running"), SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("等待运行状态")));
  Status.bFullStatusAvailable = Status.bFullStatusFresh = Status.bFullStatusIdentityCoherent = true;
  Status.bLatestAppliedTruthAvailable = true;
  Status.FullStatus.Runtime.RuntimeState = EControlStatusRuntimeState::Running;
  Status.FullStatus.Runtime.SafeStopState = EControlSafeStopState::Clear;
  FControlStatusReadinessFacet* Facets[] = {&Status.FullStatus.Readiness.Physics, &Status.FullStatus.Readiness.Control,
      &Status.FullStatus.Readiness.Visual, &Status.FullStatus.Readiness.Sensors};
  for (auto* Facet : Facets) { Facet->bRequired = true; Facet->State = EControlBindingState::Active; }
  FControlStatusSensor Sensor;
  Sensor.State = EControlSensorState::Active;
  Status.FullStatus.Sensors.Add(Sensor);
  TestEqual(TEXT("fresh complete runtime can show running"), SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("联动运行")));
  Status.FullStatus.Readiness.Sensors.State = EControlBindingState::Prepared;
  TestEqual(TEXT("required prepared facet is not a healthy link"), SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("等待运行组件")));
  Status.FullStatus.Readiness.Sensors.State = EControlBindingState::Active;
  Status.FullStatus.Readiness.Physics.bRequired = false;
  TestEqual(TEXT("omitted expected physics readiness is not a healthy link"),
            SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("等待运行组件")));
  Status.FullStatus.Readiness.Physics.bRequired = true;
  Status.FullStatus.Readiness.Control.Blocker = TEXT("control owner unavailable");
  TestEqual(TEXT("active facet with a blocker is not a healthy link"),
            SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("等待运行组件")));
  Status.FullStatus.Readiness.Control.Blocker.Reset();
  Status.FullStatus.Sensors[0].Blocker = TEXT("producer heartbeat stalled");
  TestEqual(TEXT("active sensor with a blocker is not a healthy link"),
            SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("等待运行组件")));
  Status.FullStatus.Sensors[0].Blocker.Reset();
  TestEqual(TEXT("clearing the blockers restores the healthy label"),
            SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("联动运行")));
  Status.bFullStatusFresh = false;
  TestEqual(TEXT("stale authority remains visible"), SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("运行状态过期")));
  Status.bFullStatusFresh = true;
  Status.FullStatus.Runtime.RuntimeState = EControlStatusRuntimeState::Failed;
  TestEqual(TEXT("failed runtime is not hidden by simplified presentation"), SLingTuSimRuntimeHUD::FormatDriveLink(Status).ToString(), FString(TEXT("仿真异常")));
  TestEqual(TEXT("new compact notice retains the same failure"), SLingTuSimRuntimeHUD::FormatDriveNotice(Status).ToString(), FString(TEXT("仿真异常")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldTruthIdentityTest, "LingTuSim.UI.Runtime.FieldInterface.TruthIdentity",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldTruthIdentityTest::RunTest(const FString&) {
  using LingTuSim::UI::SLingTuSimRuntimeHUD;
  LingTuSim::UI::FRuntimeUIStatusSnapshot Status;
  Status.bLatestAppliedTruthAvailable = true;
  Status.bObservedBaseVelocityAvailable = true;
  Status.ObservedBaseLinearVelocityMps = FVector(1.25, 0, 0);
  Status.SimTimeNs = 5'000'000'000;
  TestEqual(TEXT("old identity speed is unavailable"), SLingTuSimRuntimeHUD::FormatFieldSpeed(Status).ToString(), FString(TEXT("— m/s")));
  TestEqual(TEXT("old identity time is unavailable"), SLingTuSimRuntimeHUD::FormatFieldTime(Status).ToString(), FString(TEXT("仿真时间  等待真值")));
  Status.bIdentityCoherent = true;
  TestEqual(TEXT("current identity speed remains measurable"), SLingTuSimRuntimeHUD::FormatFieldSpeed(Status).ToString(), FString(TEXT("1.25 m/s")));
  TestEqual(TEXT("current identity time remains measurable"), SLingTuSimRuntimeHUD::FormatFieldTime(Status).ToString(), FString(TEXT("仿真时间  5.0 s")));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldMenuNavigationTest, "LingTuSim.UI.Runtime.FieldInterface.MenuNavigation",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldMenuNavigationTest::RunTest(const FString&) {
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto Resume = FindButton(HUD, TEXT("继续游戏"));
  if (!TestTrue(TEXT("resume button exists"), Resume.IsValid())) return false;
  auto Tactical = FindButton(HUD, TEXT("设置"));
  if (!TestTrue(TEXT("settings button exists"), Tactical.IsValid())) return false;
  auto ResumeMarker = FindFocusMarker(Resume.ToSharedRef());
  auto TacticalMarker = FindFocusMarker(Tactical.ToSharedRef());
  if (!TestTrue(TEXT("menu rows have explicit focus markers"), ResumeMarker.IsValid() && TacticalMarker.IsValid())) return false;
  auto Window = SNew(SWindow).ClientSize(FVector2D(1000, 700))[HUD];
  auto& App = FSlateApplication::Get();
  const bool bPreviouslyActive = App.IsActive();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  App.AddWindow(Window, false);
  App.OnApplicationActivationChanged(true);
  App.SetKeyboardFocus(Resume, EFocusCause::SetDirectly);
  HUD->Invalidate(EInvalidateWidgetReason::Layout);
  HUD->SlatePrepass(1.0F);
  TestTrue(TEXT("production menu child owns focus"), App.GetKeyboardFocusedWidget() == Resume);
  TestTrue(TEXT("HUD focus path is active"), App.IsActive() && HUD->HasFocusedDescendants());
  const auto FocusedColor = ResumeMarker->GetBorderBackgroundColor().GetSpecifiedColor();
  TestTrue(TEXT("keyboard selection has a visible neutral marker"), FocusedColor.A > .9F && FMath::IsNearlyEqual(FocusedColor.B, FocusedColor.R));
  TestEqual(TEXT("unfocused row has no selection marker"), TacticalMarker->GetBorderBackgroundColor().GetSpecifiedColor().A, 0.0F);
  App.SetKeyboardFocus(Tactical, EFocusCause::Navigation);
  HUD->Invalidate(EInvalidateWidgetReason::Layout);
  HUD->SlatePrepass(1.0F);
  TestEqual(TEXT("selection marker leaves the previous row"), ResumeMarker->GetBorderBackgroundColor().GetSpecifiedColor().A, 0.0F);
  TestTrue(TEXT("selection marker follows keyboard focus"), TacticalMarker->GetBorderBackgroundColor().GetSpecifiedColor().A > .9F);
  App.SetKeyboardFocus(Resume, EFocusCause::SetDirectly);
  int32 OfflineActions = 0;
  {
    LingTuSim::UI::FLingTuSimRuntimeUIInputProcessor Processor(Fixture.Mode.ToSharedRef(), HUD, HUD, {},
      [&OfflineActions](LingTuSim::UI::ERuntimeUIAction) { ++OfflineActions; }, []() { return false; }, []() { return false; });
    for (const auto& Key : {EKeys::Up, EKeys::Down, EKeys::Left, EKeys::Right,
                           EKeys::Gamepad_DPad_Up, EKeys::Gamepad_DPad_Down,
                           EKeys::Gamepad_DPad_Left, EKeys::Gamepad_DPad_Right}) {
      TestFalse(TEXT("runtime menu routes navigation to Slate"), Processor.HandleKeyDownEvent(App, FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0)));
    }
    TestEqual(TEXT("no offline selection was dispatched"), OfflineActions, 0);
    LingTuSim::UI::FLingTuSimRuntimeUIInputProcessor Selector(Fixture.Mode.ToSharedRef(), HUD, HUD, {},
      [&OfflineActions](LingTuSim::UI::ERuntimeUIAction) { ++OfflineActions; }, []() { return true; }, []() { return false; });
    TestFalse(TEXT("focused selector routes Down to Slate"), Selector.HandleKeyDownEvent(App, FKeyEvent(EKeys::Down, FModifierKeysState(), 0, false, 0, 0)));
    TestEqual(TEXT("navigation does not secretly change an entire session"), OfflineActions, 0);
  }
  if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
  else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
  App.OnApplicationActivationChanged(bPreviouslyActive);
  App.RequestDestroyWindow(Window);
  return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldKeyCaptureSaveTest, "LingTuSim.UI.Runtime.FieldInterface.SaveCancelsKeyCapture",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldKeyCaptureSaveTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  const FString Root = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation"),
      TEXT("LingTuSimUI"), FGuid::NewGuid().ToString(EGuidFormats::Digits));
  const FString IniPath = FPaths::Combine(Root, TEXT("PlayerSettings.ini"));
  if (!TestTrue(TEXT("isolated settings directory"), IFileManager::Get().MakeDirectory(*Root, true))) return false;
  TGuardValue<FString> Guard(GGameUserSettingsIni, IniPath);
  ON_SCOPE_EXIT {
    GConfig->UnloadFile(IniPath);
    IFileManager::Get().DeleteDirectory(*Root, false, true);
  };
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  auto Forward = FindButton(HUD, TEXT("前进"));
  auto Save = FindButton(HUD, TEXT("保存设置"));
  if (!TestTrue(TEXT("binding and save controls exist"), Forward.IsValid() && Save.IsValid())) return false;
  AcceptButton(Forward.ToSharedRef());
  TestTrue(TEXT("capture armed before save"), HasVisibleText(HUD, TEXT("按下新键…")));
  AcceptButton(Save.ToSharedRef());
  TestTrue(TEXT("save succeeded"), HasVisibleText(HUD, TEXT("设置已保存")));
  TestFalse(TEXT("save cancels pending capture"), HUD->HandleKeyBindingCapture(FKeyEvent(EKeys::I, FModifierKeysState(), 0, false, 0, 0)));
  TestEqual(TEXT("subsequent key cannot silently alter live mapping"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  FPlayerSettings Readback;
  Readback.Load(IniPath);
  TestEqual(TEXT("saved and live mapping agree"), Readback.DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldKeyRebindingTest, "LingTuSim.UI.Runtime.FieldInterface.KeyRebinding",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldKeyRebindingTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  auto Forward = FindButton(HUD, TEXT("前进"));
  if (!TestTrue(TEXT("forward binding row exists"), Forward.IsValid())) return false;
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  const bool bPreviouslyActive = App.IsActive();
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[HUD];
  App.AddWindow(Window, false);
  ON_SCOPE_EXIT {
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.OnApplicationActivationChanged(bPreviouslyActive);
    App.RequestDestroyWindow(Window);
  };
  App.OnApplicationActivationChanged(true);
  App.SetKeyboardFocus(Forward, EFocusCause::SetDirectly);
  TArray<FRobotDriveInputSnapshot> Published;
  int32 ActionCount = 0;
  FLingTuSimRuntimeUIInputProcessor Processor(Fixture.Mode.ToSharedRef(), HUD, HUD,
      [&Published](const auto& Input, auto) { Published.Add(Input); },
      [&ActionCount](auto) { ++ActionCount; }, []() { return false; }, []() { return false; }, {},
      [HUD]() { return HUD->HandleRuntimeMenuBack(); }, Fixture.Settings,
      [HUD](const FKeyEvent& Event) { return HUD->HandleKeyBindingCapture(Event); },
      [HUD]() { HUD->CancelKeyBindingCapture(); });
  const auto Press = [&Processor, &App](const FKey& Key, bool bRepeat = false) {
    return Processor.HandleKeyDownEvent(App, FKeyEvent(Key, FModifierKeysState(), 0, bRepeat, 0, 0));
  };
  const auto Release = [&Processor, &App](const FKey& Key) {
    Processor.HandleKeyUpEvent(App, FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0));
  };
  AcceptButton(Forward.ToSharedRef());
  TestTrue(TEXT("reserved recording key is captured"), Press(EKeys::R));
  TestEqual(TEXT("recording cannot fire during rebinding"), ActionCount, 0);
  TestEqual(TEXT("reserved key leaves old binding intact"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  Press(EKeys::S);
  TestEqual(TEXT("duplicate cannot steal backward binding"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  Press(EKeys::I, true);
  TestEqual(TEXT("repeated key cannot bind"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  Press(EKeys::Escape);
  TestEqual(TEXT("Escape cancels capture, not the settings page"), Fixture.Mode->GetMode(), ERuntimeUIMode::Pause);
  TestTrue(TEXT("controls remain visible after cancel"), HasVisibleText(HUD, TEXT("鼠标灵敏度")));
  AcceptButton(Forward.ToSharedRef());
  Press(EKeys::I);
  Release(EKeys::I);
  TestEqual(TEXT("input processor routes a new physical binding"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::I);
  for (const auto& Sample : Published)
    TestTrue(TEXT("binding in menu never creates robot intent"), !Sample.bDeadman && Sample.Forward == 0 && Sample.Left == 0 && Sample.YawLeft == 0);

  AcceptButton(Forward.ToSharedRef());
  App.OnApplicationActivationChanged(false);
  Press(EKeys::J);
  App.OnApplicationActivationChanged(true);
  TestFalse(TEXT("loss of application focus cancels pending capture"), HUD->HandleKeyBindingCapture(FKeyEvent(EKeys::J, FModifierKeysState(), 0, false, 0, 0)));
  TestEqual(TEXT("focus loss cannot silently replace mapping"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::I);
  Fixture.Mode->TogglePause();
  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  Fixture.Mode->SetDriveHelpVisible(true);
  HUD->SlatePrepass(1.F);
  TestTrue(TEXT("Drive hints reflect remapping"), HasVisibleText(HUD, TEXT("I / A / S / D")));
  Fixture.Mode->SetDriveHelpVisible(false);
  TestFalse(TEXT("old physical key no longer drives"), Press(EKeys::W));
  TestTrue(TEXT("new physical key drives"), Press(EKeys::I));
  if (!TestFalse(TEXT("intent was published"), Published.IsEmpty())) return false;
  TestEqual(TEXT("new binding produces forward intent"), Published.Last().Forward, 1.F);
  TestTrue(TEXT("evidence names actual physical key"), Published.Last().ActiveControls.Contains(TEXT("keyboard.i")));
  Release(EKeys::I);
  TestTrue(TEXT("release stops input"), Published.Last().bRelease && !Published.Last().bDeadman && Published.Last().Forward == 0);
  Press(EKeys::I);
  App.OnApplicationActivationChanged(false);
  Press(EKeys::I, true);
  TestTrue(TEXT("focus loss releases remapped input"), Published.Last().bRelease && Published.Last().Forward == 0);
  App.OnApplicationActivationChanged(true);
  Press(EKeys::I, true);
  TestEqual(TEXT("held repeat cannot rearm on focus restore"), Published.Last().Forward, 0.F);

  Fixture.Mode->TogglePause();
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  Forward = FindButton(HUD, TEXT("前进"));
  AcceptButton(Forward.ToSharedRef());
  AcceptButton(FindButton(HUD, TEXT("界面")).ToSharedRef());
  TestFalse(TEXT("changing category cancels capture"), HUD->HandleKeyBindingCapture(FKeyEvent(EKeys::J, FModifierKeysState(), 0, false, 0, 0)));
  AcceptButton(FindButton(HUD, TEXT("恢复默认")).ToSharedRef());
  TestEqual(TEXT("interface defaults preserve drive mapping"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::I);
  AcceptButton(FindButton(HUD, TEXT("操作")).ToSharedRef());
  AcceptButton(FindButton(HUD, TEXT("恢复默认")).ToSharedRef());
  TestEqual(TEXT("controls defaults restore original key"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  return true;
}
namespace {
bool FindArrangedWidget(const TSharedRef<SWidget>& Widget, const FGeometry& Geometry,
    const TSharedRef<SWidget>& Target, FGeometry& Result) {
  if (Widget == Target) { Result = Geometry; return true; }
  FArrangedChildren Children(EVisibility::Visible);
  Widget->ArrangeChildren(Geometry, Children);
  for (int32 Index = 0; Index < Children.Num(); ++Index)
    if (FindArrangedWidget(Children[Index].Widget, Children[Index].Geometry, Target, Result)) return true;
  return false;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerSettingsScreenLayoutTest, "LingTuSim.UI.Runtime.PlayerMenu.ScreenLayout",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerSettingsScreenLayoutTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  HUD->PreviewPlayerMenuPage(TEXT("settings"));
  auto Category = FindButton(HUD, TEXT("画面"));
  auto Back = FindButton(HUD, TEXT("返回"));
  auto Apply = FindButton(HUD, TEXT("应用画面"));
  if (!TestTrue(TEXT("settings navigation and actions exist"), Category.IsValid() && Back.IsValid() && Apply.IsValid())) return false;
  for (const FVector2D Size : {FVector2D(1280, 720), FVector2D(1600, 900), FVector2D(2560, 1080)}) {
    HUD->SlatePrepass(1.F);
    const auto Root = FGeometry::MakeRoot(Size, FSlateLayoutTransform(1.F));
    FGeometry CategoryBounds, BackBounds, ApplyBounds;
    if (!TestTrue(TEXT("settings controls are arranged on screen"),
        FindArrangedWidget(HUD, Root, Category.ToSharedRef(), CategoryBounds) &&
        FindArrangedWidget(HUD, Root, Back.ToSharedRef(), BackBounds) &&
        FindArrangedWidget(HUD, Root, Apply.ToSharedRef(), ApplyBounds))) return false;
    const FVector2D ApplyEnd = ApplyBounds.LocalToAbsolute(ApplyBounds.GetLocalSize());
    const FVector2D BackEnd = BackBounds.LocalToAbsolute(BackBounds.GetLocalSize());
    TestTrue(TEXT("categories stay near the left screen edge, not in a centered card"), CategoryBounds.GetAbsolutePosition().X < Size.X * .12);
    TestTrue(TEXT("categories start in the upper part of the screen"), CategoryBounds.GetAbsolutePosition().Y < Size.Y * .3);
    TestTrue(TEXT("Back stays near the left screen edge"), BackBounds.GetAbsolutePosition().X < Size.X * .12);
    TestTrue(TEXT("actions stay at the bottom as the screen grows"), BackEnd.Y > Size.Y * .85 && ApplyEnd.Y > Size.Y * .85);
    TestTrue(TEXT("primary action stays near the right screen edge"), ApplyEnd.X > Size.X * .90 && ApplyEnd.X <= Size.X);
    TestTrue(TEXT("controls remain readable rather than scaling down the entire page"), CategoryBounds.GetAbsoluteSize().Y >= 44);
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerSettingsOptionInteractionTest, "LingTuSim.UI.Runtime.PlayerMenu.OptionInteraction",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerSettingsOptionInteractionTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  HUD->PreviewPlayerMenuPage(TEXT("settings"));
  auto Row = FindButton(HUD, TEXT("帧率上限"));
  if (!TestTrue(TEXT("frame limit row exists"), Row.IsValid())) return false;
  auto Decrease = FindTaggedWidget(Row.ToSharedRef(), TEXT("LingTu.Settings.Decrease"));
  auto Increase = FindTaggedWidget(Row.ToSharedRef(), TEXT("LingTu.Settings.Increase"));
  if (!TestTrue(TEXT("left and right arrows are real action controls"), Decrease.IsValid() && Increase.IsValid())) return false;
  const auto PriorValue = FindTaggedWidget(Row.ToSharedRef(), TEXT("LingTu.Settings.Value"));
  if (!TestTrue(TEXT("current value is readable"), PriorValue.IsValid())) return false;
  const FText Before = StaticCastSharedPtr<STextBlock>(PriorValue)->GetText();
  AcceptButton(StaticCastSharedRef<SButton>(Decrease.ToSharedRef()));
  TestFalse(TEXT("left arrow decrements the value"), StaticCastSharedPtr<STextBlock>(PriorValue)->GetText().EqualTo(Before));
  auto Feedback = FindTaggedWidget(HUD, TEXT("PlayerSettingsFeedback"));
  TestTrue(TEXT("the actual option control immediately reports the new draft value"), Feedback.IsValid() &&
      StaticCastSharedPtr<STextBlock>(Feedback)->GetText().ToString() ==
      FString(TEXT("帧率上限 → ")) + StaticCastSharedPtr<STextBlock>(PriorValue)->GetText().ToString() + TEXT(" · 待应用"));
  AcceptButton(StaticCastSharedRef<SButton>(Increase.ToSharedRef()));
  TestTrue(TEXT("opposite arrows return to the prior value"), StaticCastSharedPtr<STextBlock>(PriorValue)->GetText().EqualTo(Before));
  Row->OnMouseEnter(FGeometry(), FPointerEvent());
  TestTrue(TEXT("hover shows a setting-specific explanation"), HasVisibleText(HUD, TEXT("限制主视口帧率，降低显卡负载。不会修改传感器的采样频率。")));
  auto Preview = FindTaggedWidget(HUD, TEXT("LingTu.Settings.RobotBackdrop"));
  TestTrue(TEXT("settings uses a robot scene rather than a plain full-screen fill"), Preview.IsValid());
  if (Preview.IsValid()) TestTrue(TEXT("the background cannot capture gameplay or menu input"), Preview->GetVisibility() == EVisibility::HitTestInvisible);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlayerSettingsExitDecisionTest, "LingTuSim.UI.Runtime.PlayerMenu.SettingsExitDecision",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FPlayerSettingsExitDecisionTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  auto& App = FSlateApplication::Get();
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[HUD];
  App.AddWindow(Window, false);
  ON_SCOPE_EXIT {
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.RequestDestroyWindow(Window);
  };
  auto Backdrop = FindTaggedWidget(HUD, TEXT("LingTu.Settings.Backdrop"));
  TestTrue(TEXT("settings retains a readable fallback behind its robot scene"), Backdrop.IsValid());
  if (Backdrop.IsValid()) {
    const auto Color = StaticCastSharedPtr<SBorder>(Backdrop)->GetBorderBackgroundColor().GetSpecifiedColor();
    TestEqual(TEXT("fallback prevents an unrelated active world bleeding through an unloaded scene"), Color.A, 1.F);
    TestTrue(TEXT("settings follows the light front-end palette"), Color.R > .7F && Color.G > .7F && Color.B > .7F);
  }
  auto Invert = FindCheckBox(HUD, TEXT("反转垂直视角"));
  if (!TestTrue(TEXT("control setting exists"), Invert.IsValid())) return false;
  App.SetKeyboardFocus(Invert, EFocusCause::SetDirectly);
  Invert->ToggleCheckedState();
  TestTrue(TEXT("unsaved edits are visible before returning"), HasVisibleText(HUD, TEXT("反转垂直视角 → 开启 · 待保存")));
  AcceptButton(FindButton(HUD, TEXT("返回")).ToSharedRef());
  auto Continue = FindButton(HUD, TEXT("继续设置"));
  auto Discard = FindButton(HUD, TEXT("放弃修改"));
  if (!TestTrue(TEXT("back requires a decision before discarding edits"), Continue.IsValid() && Discard.IsValid())) return false;
  TestTrue(TEXT("opening the decision preserves the live edit"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("safe continue action receives keyboard focus"), App.GetKeyboardFocusedWidget() == Continue);
  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  HUD->RestoreSelectionFocus();
  TestTrue(TEXT("restoring child focus stays inside the decision"), App.GetKeyboardFocusedWidget() == Continue);
  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  HUD->FocusRuntimeMenuCTA();
  TestTrue(TEXT("menu focus cannot select the hidden resume action"), App.GetKeyboardFocusedWidget() == Continue);
  TestTrue(TEXT("gamepad shoulder is contained while deciding"), HUD->OnKeyDown(FGeometry(),
      FKeyEvent(EKeys::Gamepad_RightShoulder, FModifierKeysState(), 0, false, 0, 0)).IsEventHandled());
  TestEqual(TEXT("decision never resumes robot drive"), Fixture.Mode->GetMode(), ERuntimeUIMode::Pause);
  TestFalse(TEXT("settings underneath cannot be activated"), FindButton(HUD, TEXT("恢复默认")).IsValid());
  AcceptButton(Continue.ToSharedRef());
  TestTrue(TEXT("continue restores the same controls page"), HasVisibleText(HUD, TEXT("鼠标灵敏度")));
  TestTrue(TEXT("continue preserves the unsaved value"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("continue restores focus to the edited control"), App.GetKeyboardFocusedWidget() == Invert);
  TestTrue(TEXT("Escape opens the same decision"), HUD->HandleRuntimeMenuBack());
  TestTrue(TEXT("Escape cancels the decision before leaving settings"), HUD->HandleRuntimeMenuBack());
  TestTrue(TEXT("Escape at the decision preserves edits"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("the original settings page is still available"), HasVisibleText(HUD, TEXT("鼠标灵敏度")));
  HUD->HandleRuntimeMenuBack();
  Discard = FindButton(HUD, TEXT("放弃修改"));
  if (!TestTrue(TEXT("discard remains available on a second attempt"), Discard.IsValid())) return false;
  AcceptButton(Discard.ToSharedRef());
  TestFalse(TEXT("discard restores the entry checkpoint"), Fixture.Settings->bInvertMouseY);
  TestTrue(TEXT("discard returns to the menu, not to drive"), FindButton(HUD, TEXT("继续游戏")).IsValid());
  TestEqual(TEXT("discard keeps the session paused"), Fixture.Mode->GetMode(), ERuntimeUIMode::Pause);
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  HUD->HandleRuntimeMenuBack();
  TestTrue(TEXT("unchanged settings return without an unnecessary prompt"), FindButton(HUD, TEXT("继续游戏")).IsValid());

  HUD->PreviewPlayerMenuPage(TEXT("settings"));
  AcceptButton(FindButton(HUD, TEXT("帧率上限")).ToSharedRef());
  TestTrue(TEXT("graphics draft has actually changed"), HUD->HasGraphicsDraftChangesForTest());
  HUD->HandleRuntimeMenuBack();
  Continue = FindButton(HUD, TEXT("继续设置"));
  if (!TestTrue(TEXT("unapplied graphics also requires a decision"), Continue.IsValid())) return false;
  AcceptButton(Continue.ToSharedRef());
  TestTrue(TEXT("continue keeps the graphics draft"), HUD->HasGraphicsDraftChangesForTest());
  HUD->HandleRuntimeMenuBack();
  AcceptButton(FindButton(HUD, TEXT("放弃修改")).ToSharedRef());
  HUD->PreviewPlayerMenuPage(TEXT("settings"));
  TestFalse(TEXT("discarded graphics is not carried into the next edit"), HUD->HasGraphicsDraftChangesForTest());
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFieldKeyCaptureFeedbackTest, "LingTuSim.UI.Runtime.FieldInterface.KeyCaptureFeedbackAndFocus",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FFieldKeyCaptureFeedbackTest::RunTest(const FString&) {
  using namespace LingTuSim::UI;
  FFieldUIFixture Fixture;
  Fixture.Mode->TogglePause();
  auto HUD = Fixture.Widget();
  auto& App = FSlateApplication::Get();
  HUD->Tick(FGeometry(), App.GetCurrentTime(), 0.F);
  HUD->PreviewPlayerMenuPage(TEXT("controls"));
  auto Forward = FindButton(HUD, TEXT("前进"));
  if (!TestTrue(TEXT("binding row exists"), Forward.IsValid())) return false;
  const auto PreviousFocus = App.GetKeyboardFocusedWidget();
  auto Window = SNew(SWindow).ClientSize(FVector2D(1280, 720))[HUD];
  App.AddWindow(Window, false);
  ON_SCOPE_EXIT {
    if (PreviousFocus.IsValid()) App.SetKeyboardFocus(PreviousFocus, EFocusCause::SetDirectly);
    else App.ClearKeyboardFocus(EFocusCause::SetDirectly);
    App.RequestDestroyWindow(Window);
  };
  const auto Capture = [&HUD](const FKey& Key) {
    return HUD->HandleKeyBindingCapture(FKeyEvent(Key, FModifierKeysState(), 0, false, 0, 0));
  };
  AcceptButton(Forward.ToSharedRef());
  const double ErrorTime = App.GetCurrentTime();
  Capture(EKeys::S);
  HUD->SlatePrepass(1.F);
  const FString Conflict = TEXT("按键 S 已绑定到后退 · 请换一个键，Esc 取消");
  TestTrue(TEXT("conflict names the occupied action"), HasVisibleText(HUD, Conflict));
  auto FeedbackWidget = FindTaggedWidget(HUD, TEXT("PlayerSettingsFeedback"));
  if (!TestTrue(TEXT("settings feedback exists"), FeedbackWidget.IsValid())) return false;
  auto Feedback = StaticCastSharedPtr<STextBlock>(FeedbackWidget);
  TestEqual(TEXT("conflict uses a readable warning color"), Feedback->GetColorAndOpacity().GetSpecifiedColor(), FLinearColor(.48F, .035F, .025F, 1.F));
  TestEqual(TEXT("conflict preserves the original binding"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::W);
  HUD->Tick(FGeometry(), ErrorTime + 1.0, 0.F);
  TestTrue(TEXT("error remains readable for one second"), HasVisibleText(HUD, Conflict));
  HUD->Tick(FGeometry(), ErrorTime + 1.6, 0.F);
  HUD->SlatePrepass(1.F);
  TestFalse(TEXT("transient error clears after 1.5 seconds"), HasVisibleText(HUD, Conflict));
  TestTrue(TEXT("capture prompt returns without cancelling capture"), HasVisibleText(HUD, TEXT("为“前进”按一个新键 · Esc 取消")));
  TestEqual(TEXT("prompt restores normal color"), Feedback->GetColorAndOpacity().GetSpecifiedColor(), FLinearColor(.10F, .12F, .14F, 1.F));

  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  TestTrue(TEXT("Escape is consumed by capture"), Capture(EKeys::Escape));
  TestTrue(TEXT("cancel returns focus to its original row"), App.GetKeyboardFocusedWidget() == Forward);
  TestTrue(TEXT("cancel feedback remains visible"), HasVisibleText(HUD, TEXT("已取消改键")));

  AcceptButton(Forward.ToSharedRef());
  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  TestTrue(TEXT("new key is consumed by capture"), Capture(EKeys::I));
  TestTrue(TEXT("successful binding returns focus to its original row"), App.GetKeyboardFocusedWidget() == Forward);
  TestEqual(TEXT("successful binding updates the mapping"), Fixture.Settings->DriveKeys.GetKey(EDriveKeyAction::Forward), EKeys::I);
  Forward->OnKeyUp(FGeometry(), FKeyEvent(EKeys::I, FModifierKeysState(), 0, false, 0, 0));
  TestFalse(TEXT("releasing the bound letter cannot re-open capture"), Capture(EKeys::J));

  AcceptButton(Forward.ToSharedRef());
  App.SetKeyboardFocus(HUD, EFocusCause::SetDirectly);
  HUD->CancelKeyBindingCapture();
  TestTrue(TEXT("programmatic cancellation does not steal focus"), App.GetKeyboardFocusedWidget() == HUD);
  return true;
}
#endif
