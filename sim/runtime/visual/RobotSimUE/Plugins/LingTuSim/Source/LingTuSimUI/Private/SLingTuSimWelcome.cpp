#include "SLingTuSimRuntimeHUD.h"
#include "SLingTuSimShowroom.h"
#include "SLingTuSimShowroomInput.h"

#include "Brushes/SlateColorBrush.h"
#include "Brushes/SlateDynamicImageBrush.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/SLeafWidget.h"
#include "Rendering/DrawElements.h"
#include "Input/Events.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScaleBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace LingTuSim::UI {
namespace {
const FLinearColor Ink(.015F, .022F, .027F, 1);
const FLinearColor Muted(.09F, .11F, .13F, 1);
const FLinearColor Accent(.012F, .13F, .23F, 1);
// Resolution-independent route / L mark. No external image or font glyph dependency.
class SWelcomeMark final : public SLeafWidget {
 public:
  SLATE_BEGIN_ARGS(SWelcomeMark) {} SLATE_END_ARGS()
  void Construct(const FArguments&) { SetVisibility(EVisibility::HitTestInvisible); }
  virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(42, 42); }
  virtual int32 OnPaint(const FPaintArgs&, const FGeometry& Geometry, const FSlateRect&,
      FSlateWindowElementList& Elements, int32 Layer, const FWidgetStyle&, bool) const override {
    const TArray<FVector2D> Route = {FVector2D(8, 6), FVector2D(8, 34), FVector2D(34, 34)};
    const TArray<FVector2D> Forward = {FVector2D(21, 8), FVector2D(34, 8), FVector2D(34, 21)};
    FSlateDrawElement::MakeLines(Elements, Layer, Geometry.ToPaintGeometry(), Route, ESlateDrawEffect::None, Ink, true, 3.F);
    FSlateDrawElement::MakeLines(Elements, Layer, Geometry.ToPaintGeometry(), Forward, ESlateDrawEffect::None, Accent, true, 3.F);
    return Layer;
  }
};
TSharedRef<STextBlock> WelcomeText(const TCHAR* Value, int32 Size, FLinearColor Color = Ink) {
  return SNew(STextBlock).Text(FText::FromString(Value))
      .Font(FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), Size)).ColorAndOpacity(Color);
}
TSharedRef<SButton> WelcomeButton(const TCHAR* Caption, int32 Size, FOnClicked Click,
                                 TAttribute<bool> Enabled = true) {
  static const FButtonStyle Style = FButtonStyle()
      .SetNormal(FSlateColorBrush(FLinearColor::Transparent))
      .SetHovered(FSlateColorBrush(FLinearColor(1, 1, 1, .16F)))
      .SetPressed(FSlateColorBrush(FLinearColor(.58F, .65F, .68F, .22F)))
      .SetDisabled(FSlateColorBrush(FLinearColor::Transparent))
      .SetNormalPadding(FMargin(0)).SetPressedPadding(FMargin(0));
  auto Button = SNew(SButton).ButtonStyle(&Style).ContentPadding(FMargin(0, 16))
      .HAlign(HAlign_Fill).IsEnabled(Enabled).OnClicked(Click);
  const TWeakPtr<SButton> Weak = Button;
  Button->SetContent(SNew(SHorizontalBox)
    + SHorizontalBox::Slot().FillWidth(1)
      [SNew(STextBlock).Text(FText::FromString(Caption))
        .Font(FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), Size))
        .ColorAndOpacity_Lambda([Weak]() {
          auto Control = Weak.Pin();
          return Control && (Control->IsHovered() || Control->HasKeyboardFocus()) ? Accent : Ink;
        })]
    + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
      [SNew(STextBlock).Text(FText::FromString(TEXT("→")))
        .Font(FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), 22)).ColorAndOpacity(Accent)
        .Visibility_Lambda([Weak]() {
          auto Control = Weak.Pin();
          return Control && (Control->IsHovered() || Control->HasKeyboardFocus()) ? EVisibility::Visible : EVisibility::Hidden;
        })]);
  return Button;
}
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildLoginScreen() {
  WelcomeShowroom = SNew(SLingTuSimShowroom).Visibility(FactoryBackgroundBrush.IsValid() ? EVisibility::Collapsed : EVisibility::HitTestInvisible)
      .Environment(EShowroomEnvironment::OperationsBase)
      .RobotId_Lambda([this]() { return SelectionModel->GetRequestedRobotId(); });
  WelcomeShowroomInput = SNew(SLingTuSimShowroomInput)
      .CanInteract_Lambda([this]() {
        return !FactoryBackgroundBrush.IsValid() && GetLoginVisibility() == EVisibility::Visible && MenuPage == EPlayerMenuPage::Root && WelcomeShowroom->HasRobot();
      })
      .OnGesture([this](EShowroomGesture Gesture, FVector2D Delta) {
        switch (Gesture) {
          case EShowroomGesture::Orbit: WelcomeShowroom->Orbit(Delta); break;
          case EShowroomGesture::Pan: WelcomeShowroom->Pan(Delta); break;
          case EShowroomGesture::Zoom: WelcomeShowroom->Zoom(Delta.X); break;
          case EShowroomGesture::Reset: WelcomeShowroom->SetView(EShowroomView::Perspective); break;
        }
      });
  LocalWorkspaceButton = WelcomeButton(TEXT("开始"), 30,
      FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::HandleEnterLocalWorkspace),
      TAttribute<bool>::CreateSP(this, &SLingTuSimRuntimeHUD::CanEnterLocalWorkspace));
  LocalWorkspaceButton->SetTag(TEXT("LingTu.Welcome.Start"));
  WelcomeSettingsButton = WelcomeButton(TEXT("设置"), 23, FOnClicked::CreateLambda([this]() {
    ShowMenuPage(EPlayerMenuPage::Settings); return FReply::Handled();
  }));
  WelcomeSettingsButton->SetTag(TEXT("LingTu.Welcome.Settings"));
  WelcomeQuitButton = WelcomeButton(TEXT("退出"), 23, FOnClicked::CreateLambda([this]() {
    ShowMenuPage(EPlayerMenuPage::Quit); return FReply::Handled();
  }));
  WelcomeQuitButton->SetTag(TEXT("LingTu.Welcome.Quit"));
  auto Menu = SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight()[LocalWorkspaceButton.ToSharedRef()]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 25)
      [WelcomeText(TEXT("选择机器人与地图"), 13, Muted)]
    + SVerticalBox::Slot().AutoHeight()[WelcomeSettingsButton.ToSharedRef()]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 3)[WelcomeQuitButton.ToSharedRef()]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 22, 0, 0)
      [SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), 12))
        .ColorAndOpacity(FLinearColor(.48F, .12F, .06F)).AutoWrapText(true)
        .Text_Lambda([this]() {
          return SelectionModel->GetCatalog().Options.IsEmpty() ? GetLoginCatalogStatusText() : GetLoginFeedbackText();
        })];
  auto Brand = SNew(SHorizontalBox)
    + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)[SNew(SWelcomeMark)]
    + SHorizontalBox::Slot().AutoWidth().Padding(16, 0, 0, 0)[SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight()[WelcomeText(TEXT("灵途"), 28)]
      + SVerticalBox::Slot().AutoHeight().Padding(1, 3, 0, 0)[WelcomeText(TEXT("LINGTU"), 11, Muted)]];
  auto Page = SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)[Brand]
    + SVerticalBox::Slot().FillHeight(1).VAlign(VAlign_Center)
      [SNew(SHorizontalBox)
        + SHorizontalBox::Slot().AutoWidth().Padding(100, 0, 0, 0)[SNew(SBox).WidthOverride(240)[Menu]]
        + SHorizontalBox::Slot().FillWidth(1)]
    + SVerticalBox::Slot().AutoHeight()[SNew(SBox).HeightOverride(84)];
  auto ReadabilityVeil = MakeShowroomVeil();
  ReadabilityVeil->SetRenderOpacity(FactoryBackgroundBrush.IsValid() ? 1.F : .50F);
  return SNew(SOverlay)
    + SOverlay::Slot()[WelcomeShowroom.ToSharedRef()]
    + SOverlay::Slot()[SNew(SBox).Tag(TEXT("LingTu.Welcome.FrontEndBackgroundBrush"))
      .Visibility(this, &SLingTuSimRuntimeHUD::GetFrontEndStaticBackdropVisibility)
      [SNew(SScaleBox).Stretch(EStretch::ScaleToFill)
        [SNew(SBox).WidthOverride(1920).HeightOverride(1080)
          [SNew(SImage).Image(FactoryBackgroundBrush.Get())]]]]
    + SOverlay::Slot()[SNew(SHorizontalBox)
      + SHorizontalBox::Slot().FillWidth(FactoryBackgroundBrush.IsValid() ? 1.F : .58F)[ReadabilityVeil]
      + SHorizontalBox::Slot().FillWidth(FactoryBackgroundBrush.IsValid() ? 0.F : .42F)]
    + SOverlay::Slot()[SNew(SHorizontalBox)
      .Visibility_Lambda([this]() { return !FactoryBackgroundBrush.IsValid() && MenuPage == EPlayerMenuPage::Root ? EVisibility::SelfHitTestInvisible : EVisibility::Collapsed; })
      + SHorizontalBox::Slot().FillWidth(.40F)
      + SHorizontalBox::Slot().FillWidth(.60F)[SNew(SBox).Tag(TEXT("LingTu.Welcome.Viewport"))[WelcomeShowroomInput.ToSharedRef()]]]
    + SOverlay::Slot().Padding(86, 72, 86, 54)
      [SNew(SScaleBox).Visibility_Lambda([this]() {
          return MenuPage == EPlayerMenuPage::Root ? EVisibility::SelfHitTestInvisible : EVisibility::Collapsed;
        }).Stretch(EStretch::ScaleToFit).StretchDirection(EStretchDirection::DownOnly)
        [SNew(SBox).WidthOverride(1748).HeightOverride(954)[Page]]];
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
bool SLingTuSimRuntimeHUD::ProbeWelcomeInspection(bool bReset) {
  if (!WelcomeShowroom->IsReady() || !WelcomeShowroom->HasRobot()) return false;
  const FGeometry Geometry = FGeometry::MakeRoot(FVector2D(600, 400), FSlateLayoutTransform());
  const auto Mouse = [](FKey Button, TSet<FKey> Held = {}, FVector2D At = {}, FVector2D Previous = {}, float Wheel = 0) {
    return FPointerEvent(0, At, Previous, Held, Button, Wheel, FModifierKeysState());
  };
  if (bReset) {
    WelcomeShowroomInput->OnMouseButtonDoubleClick(Geometry, Mouse(EKeys::LeftMouseButton));
    const auto& View = WelcomeShowroom->GetView();
    const bool bDefault = View.YawDegrees == -25 && View.PitchDegrees == 12 && View.Zoom == 1 && View.Pan.IsNearlyZero();
    ShowMenuPage(EPlayerMenuPage::Settings);
    const bool bBlocked = !WelcomeShowroomInput->OnMouseWheel(Geometry, Mouse(EKeys::Invalid, {}, {}, {}, 1)).IsEventHandled();
    ShowMenuPage(EPlayerMenuPage::Root);
    UE_LOG(LogTemp, Display, TEXT("LINGTU_WELCOME_INTERACTION reset=%d settings_blocks_input=%d"), bDefault, bBlocked);
    return bDefault && bBlocked;
  }
  WelcomeShowroomInput->OnMouseButtonDown(Geometry, Mouse(EKeys::LeftMouseButton, {EKeys::LeftMouseButton}));
  WelcomeShowroomInput->OnMouseMove(Geometry, Mouse(EKeys::Invalid, {EKeys::LeftMouseButton}, {72, 16}));
  WelcomeShowroomInput->OnMouseButtonUp(Geometry, Mouse(EKeys::LeftMouseButton));
  WelcomeShowroomInput->OnMouseWheel(Geometry, Mouse(EKeys::Invalid, {}, {}, {}, 1));
  WelcomeShowroomInput->OnMouseButtonDown(Geometry, Mouse(EKeys::MiddleMouseButton, {EKeys::MiddleMouseButton}));
  WelcomeShowroomInput->OnMouseMove(Geometry, Mouse(EKeys::Invalid, {EKeys::MiddleMouseButton}, {24, 8}));
  WelcomeShowroomInput->OnMouseButtonUp(Geometry, Mouse(EKeys::MiddleMouseButton));
  const auto& View = WelcomeShowroom->GetView();
  const bool bChanged = View.YawDegrees != -25 && View.PitchDegrees != 12 && View.Zoom > 1 && !View.Pan.IsNearlyZero();
  UE_LOG(LogTemp, Display, TEXT("LINGTU_WELCOME_INTERACTION changed=%d yaw=%.2f pitch=%.2f zoom=%.2f pan=%s physics=false"),
      bChanged, View.YawDegrees, View.PitchDegrees, View.Zoom, *View.Pan.ToString());
  return bChanged && !WelcomeShowroomInput->IsDragging();
}
#endif
}
