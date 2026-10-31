#include "SLingTuSimRuntimeHUD.h"
#include "SLingTuSimNavigationOverlay.h"
#include "SLingTuSimRobotMapPicker.h"
#include "SLingTuSimShowroom.h"
#include "SLingTuSimMeasurements.h"
#include "LingTuSimMeasurementClient.h"

#include "Brushes/SlateColorBrush.h"
#include "Brushes/SlateRoundedBoxBrush.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "HAL/IConsoleManager.h"
#include "Engine/Engine.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SSlider.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SScaleBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/SLeafWidget.h"
#include "Widgets/Text/STextBlock.h"

namespace LingTuSim::UI {
namespace {
const FLinearColor Paper(0.90F, 0.93F, 0.96F, 1.0F);
const FLinearColor Secondary(0.55F, 0.61F, 0.68F, 1.0F);
const FLinearColor Accent(0.40F, 0.65F, 0.82F, 1.0F);
const FLinearColor Panel(0.018F, 0.024F, 0.034F, 0.94F);
const FLinearColor RuleColor(0.20F, 0.26F, 0.32F, 0.45F);
const FLinearColor Shadow(0.0F, 0.0F, 0.0F, 0.90F);
const FLinearColor HudPanel(0.012F, 0.018F, 0.022F, 0.62F);
const FLinearColor SettingsSurface(.91F, .93F, .94F, 1.F);
const FLinearColor SettingsInk(.022F, .029F, .036F, 1.F);
const FLinearColor SettingsMuted(.10F, .12F, .14F, 1.F);
const FLinearColor SettingsAccent(.015F, .17F, .28F, 1.F);

FSlateFontInfo FieldFont(int32 Size, bool bBold = false) {
  return FCoreStyle::GetDefaultFontStyle(bBold ? TEXT("Bold") : TEXT("Regular"), Size);
}

TSharedRef<STextBlock> Label(const TCHAR* Text, int32 Size = 12,
                           FLinearColor Color = Paper, bool bBold = false) {
  return SNew(STextBlock).Text(FText::FromString(Text)).Font(FieldFont(Size, bBold))
      .ColorAndOpacity(Color).ShadowOffset(FVector2D(1, 1)).ShadowColorAndOpacity(Shadow);
}

const FSlateBrush* Fill() { return FCoreStyle::Get().GetBrush(TEXT("WhiteBrush")); }

TSharedRef<STextBlock> SettingsText(const TCHAR* Text, int32 Size = 14,
                                 FLinearColor Color = SettingsInk, bool bBold = false) {
  return SNew(STextBlock).Text(FText::FromString(Text)).Font(FieldFont(Size, bBold)).ColorAndOpacity(Color);
}

TSharedRef<SWidget> SettingsRule() {
  return SNew(SBox).HeightOverride(1)[SNew(SBorder).BorderImage(Fill()).Padding(0)
      .BorderBackgroundColor(FLinearColor(.70F, .75F, .78F, 1.F))];
}

TSharedRef<SWidget> Rule() {
  return SNew(SBox).HeightOverride(1)[SNew(SBorder).BorderImage(Fill())
      .BorderBackgroundColor(RuleColor).Padding(0)];
}

TSharedRef<SWidget> KeyHint(TAttribute<FText> Key, const TCHAR* Action) {
  return SNew(SHorizontalBox)
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
          [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(HudPanel).Padding(FMargin(7, 4))
              [SNew(STextBlock).Text(Key).Font(FieldFont(10, true)).ColorAndOpacity(Paper)]]
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(7, 0, 20, 0)
          [Label(Action, 11, Secondary)];
}

TSharedRef<SWidget> KeyHint(const TCHAR* Key, const TCHAR* Action) {
  return KeyHint(TAttribute<FText>(FText::FromString(Key)), Action);
}

TSharedRef<SWidget> QuietKeyHint(const TCHAR* Key, const TCHAR* Action) {
  static const FSlateRoundedBoxBrush Outline(FLinearColor::Transparent, 2.F,
      FLinearColor(.83F, .83F, .83F, .6F), 1.F);
  return SNew(SHorizontalBox)
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
          [SNew(SBorder).BorderImage(&Outline).Padding(FMargin(6, 3))[Label(Key, 11, FLinearColor::White)]]
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(9, 0, 24, 0)
          [Label(Action, 12, FLinearColor(.82F, .82F, .82F, 1))];
}

const FButtonStyle* MenuButtonStyle() {
  static const FButtonStyle Style = FButtonStyle()
      .SetNormal(FSlateColorBrush(FLinearColor::Transparent))
      .SetHovered(FSlateColorBrush(FLinearColor::Transparent))
      .SetPressed(FSlateColorBrush(FLinearColor::Transparent))
      .SetDisabled(FSlateColorBrush(FLinearColor::Transparent))
      .SetNormalPadding(FMargin(0)).SetPressedPadding(FMargin(0));
  return &Style;
}

TSharedRef<SButton> MenuButton(TSharedRef<SWidget> Content, FOnClicked Click,
                              TAttribute<bool> Enabled = true) {
  auto Button = SNew(SButton).ButtonStyle(MenuButtonStyle()).ContentPadding(0)
      .HAlign(HAlign_Fill).VAlign(VAlign_Fill).IsEnabled(Enabled).OnClicked(Click);
  const TWeakPtr<SButton> WeakButton = Button;
  auto FocusMarker = SNew(SBorder).BorderImage(Fill()).Padding(0)
      .BorderBackgroundColor_Lambda([WeakButton]() {
        const auto Row = WeakButton.Pin();
        return Row.IsValid() && Row->IsEnabled() && Row->HasKeyboardFocus()
            ? FLinearColor(.93F, .93F, .93F, 1) : FLinearColor::Transparent;
      });
  FocusMarker->SetTag(TEXT("LingTu.Menu.FocusMarker"));
  Button->SetContent(SNew(SBorder).BorderImage(Fill()).Padding(0)
      .BorderBackgroundColor_Lambda([WeakButton]() {
        const auto Row = WeakButton.Pin();
        if (!Row.IsValid() || !Row->IsEnabled()) return FLinearColor::Transparent;
        if (Row->IsPressed()) return FLinearColor(.10F, .10F, .10F, .55F);
        if (Row->HasKeyboardFocus()) return FLinearColor(.07F, .07F, .07F, .40F);
        return Row->IsHovered() ? FLinearColor(.05F, .05F, .05F, .30F)
                                : FLinearColor::Transparent;
      })
      [SNew(SHorizontalBox)
        + SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride(2)[FocusMarker]]
        + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0, 0, 0)
            [SNew(STextBlock).Text(FText::FromString(TEXT("›"))).Font(FieldFont(21)).ColorAndOpacity_Lambda([WeakButton]() {
              const auto Row = WeakButton.Pin();
              return Row.IsValid() && (Row->HasKeyboardFocus() || Row->IsHovered())
                  ? FLinearColor::White : FLinearColor::Transparent;
            })]
        + SHorizontalBox::Slot().FillWidth(1).Padding(FMargin(12, 13))[Content]]);
  return Button;
}

bool HasCurrentStatus(const FRuntimeUIStatusSnapshot& Status) {
  return Status.bFullStatusAvailable && Status.bFullStatusFresh && Status.bFullStatusIdentityCoherent;
}

const FButtonStyle* SettingRowStyle() {
  static const FButtonStyle Style = FButtonStyle()
      .SetNormal(FSlateColorBrush(FLinearColor::Transparent))
      .SetHovered(FSlateColorBrush(FLinearColor(.72F, .81F, .87F, .45F)))
      .SetPressed(FSlateColorBrush(FLinearColor(.60F, .72F, .80F, .55F)))
      .SetDisabled(FSlateColorBrush(FLinearColor::Transparent))
      .SetNormalPadding(FMargin(0)).SetPressedPadding(FMargin(0));
  return &Style;
}

TSharedRef<SButton> SettingsButton(const TCHAR* Caption, FOnClicked Click,
    bool bPrimary = false, TAttribute<bool> Enabled = true) {
  auto Button = SNew(SButton).ButtonStyle(SettingRowStyle()).ContentPadding(0).OnClicked(Click).IsEnabled(Enabled);
  const TWeakPtr<SButton> Weak = Button;
  Button->SetContent(SNew(SBorder).BorderImage(Fill()).Padding(FMargin(18, 12))
      .BorderBackgroundColor_Lambda([Weak, bPrimary]() {
        const auto Row = Weak.Pin();
        if (bPrimary) return SettingsAccent;
        return Row.IsValid() && (Row->HasKeyboardFocus() || Row->IsHovered())
            ? FLinearColor(.76F, .83F, .88F, 1.F) : FLinearColor::Transparent;
      })[SettingsText(Caption, 15, bPrimary ? FLinearColor::White : SettingsInk)]);
  return Button;
}

class SPlayerSettingChoice final : public SButton {
 public:
  SLATE_BEGIN_ARGS(SPlayerSettingChoice) {}
  SLATE_ARGUMENT(FText, Label)
  SLATE_ATTRIBUTE(FText, Value)
  SLATE_ARGUMENT(TFunction<void(int32)>, OnAdjust)
  SLATE_ARGUMENT(TFunction<void()>, OnInspect)
  SLATE_ARGUMENT(TFunction<void(const FText&, const FText&)>, OnChanged)
  SLATE_END_ARGS()
  void Construct(const FArguments& Args) {
    Adjust = Args._OnAdjust;
    Inspect = Args._OnInspect;
    Changed = Args._OnChanged;
    Label = Args._Label;
    Value = Args._Value;
    const auto Arrow = [this](const TCHAR* Caption, int32 Direction) {
      return SNew(SButton).Tag(Direction < 0 ? TEXT("LingTu.Settings.Decrease") : TEXT("LingTu.Settings.Increase"))
          .ButtonStyle(SettingRowStyle()).IsFocusable(false).ContentPadding(FMargin(10, 9))
          .OnClicked_Lambda([this, Direction]() {
            Step(Direction);
            return FReply::Handled().SetUserFocus(AsShared(), EFocusCause::Mouse);
          })[SettingsText(Caption, 18, SettingsAccent)];
    };
    SButton::Construct(SButton::FArguments().ButtonStyle(SettingRowStyle()).ContentPadding(FMargin(10, 1))
      .HAlign(HAlign_Fill).OnClicked_Lambda([this]() { Step(1); return FReply::Handled(); })
      [SNew(SBorder).BorderImage(Fill()).Padding(0).BorderBackgroundColor_Lambda([this]() {
          return HasKeyboardFocus() ? FLinearColor(.70F, .80F, .87F, .75F) : FLinearColor(1, 1, 1, .32F);
        })[SNew(SHorizontalBox)
          + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center).Padding(8, 0)
              [SNew(STextBlock).Text(Args._Label).Font(FieldFont(15)).ColorAndOpacity(SettingsInk)]
          + SHorizontalBox::Slot().AutoWidth()[Arrow(TEXT("‹"), -1)]
          + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
              [SNew(SBox).MinDesiredWidth(100).HAlign(HAlign_Center)
                [SNew(STextBlock).Tag(TEXT("LingTu.Settings.Value")).Text(Args._Value).Font(FieldFont(15)).ColorAndOpacity(SettingsInk)]]
          + SHorizontalBox::Slot().AutoWidth()[Arrow(TEXT("›"), 1)]]]);
  }
  virtual void OnMouseEnter(const FGeometry& Geometry, const FPointerEvent& Event) override {
    SButton::OnMouseEnter(Geometry, Event);
    if (Inspect) Inspect();
  }
  virtual FReply OnFocusReceived(const FGeometry& Geometry, const FFocusEvent& Event) override {
    if (Inspect) Inspect();
    return SButton::OnFocusReceived(Geometry, Event);
  }
  virtual FReply OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Event) override {
    const auto& Key = Event.GetKey();
    if (Key == EKeys::Left || Key == EKeys::Right || Key == EKeys::Gamepad_DPad_Left || Key == EKeys::Gamepad_DPad_Right) {
      Step(Key == EKeys::Left || Key == EKeys::Gamepad_DPad_Left ? -1 : 1);
      return FReply::Handled();
    }
    return SButton::OnKeyDown(Geometry, Event);
  }
 private:
  void Step(int32 Direction) {
    if (Inspect) Inspect();
    const FText Before = Value.Get();
    if (Adjust) Adjust(Direction);
    const FText After = Value.Get();
    if (Changed && !Before.EqualTo(After)) Changed(Label, After);
  }
  FText Label;
  TAttribute<FText> Value;
  TFunction<void(int32)> Adjust;
  TFunction<void()> Inspect;
  TFunction<void(const FText&, const FText&)> Changed;
};

int32 StepOption(int32 Current, int32 Direction, int32 Count) {
  if (Current < 0) return Direction > 0 ? 0 : Count - 1;
  return (Current + Direction + Count) % Count;
}

FText QualityLabel(int32 Quality) {
  const TCHAR* Names[] = {TEXT("低"), TEXT("中"), TEXT("高"), TEXT("极高")};
  return FText::FromString(Quality >= 0 && Quality < 4 ? Names[Quality] : TEXT("自定义"));
}

const FSliderStyle* PlayerSliderStyle() {
  static const FSliderStyle Style = FSliderStyle(FCoreStyle::Get().GetWidgetStyle<FSliderStyle>(TEXT("Slider")))
      .SetNormalBarImage(FSlateColorBrush(FLinearColor(.38F, .38F, .38F, 1)))
      .SetHoveredBarImage(FSlateColorBrush(FLinearColor(.65F, .65F, .65F, 1)))
      .SetDisabledBarImage(FSlateColorBrush(FLinearColor(.15F, .15F, .15F, 1)))
      .SetBarThickness(2.F);
  return &Style;
}

class SPlayerMenuBackdrop final : public SLeafWidget {
 public:
  SLATE_BEGIN_ARGS(SPlayerMenuBackdrop) {} SLATE_END_ARGS()
  void Construct(const FArguments&) { SetVisibility(EVisibility::HitTestInvisible); }
  virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D::ZeroVector; }
  virtual int32 OnPaint(const FPaintArgs&, const FGeometry& Geometry, const FSlateRect&,
      FSlateWindowElementList& Elements, int32 Layer, const FWidgetStyle&, bool) const override {
    const float Width = Geometry.GetLocalSize().X;
    TArray<FSlateGradientStop> Stops;
    Stops.Emplace(FVector2D(0, 0), FLinearColor(0, 0, 0, .91F));
    Stops.Emplace(FVector2D(Width * .38F, 0), FLinearColor(0, 0, 0, .73F));
    Stops.Emplace(FVector2D(Width, 0), FLinearColor(0, 0, 0, .12F));
    // Slate names the stop orientation: vertical stops produce a left-to-right fade.
    FSlateDrawElement::MakeGradient(Elements, Layer, Geometry.ToPaintGeometry(), MoveTemp(Stops), Orient_Vertical);
    return Layer;
  }
};

class SSettingsVeil final : public SLeafWidget {
 public:
  SLATE_BEGIN_ARGS(SSettingsVeil) {} SLATE_END_ARGS()
  void Construct(const FArguments&) { SetVisibility(EVisibility::HitTestInvisible); }
  virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D::ZeroVector; }
  virtual int32 OnPaint(const FPaintArgs&, const FGeometry& Geometry, const FSlateRect&,
      FSlateWindowElementList& Elements, int32 Layer, const FWidgetStyle&, bool) const override {
    const float Width = Geometry.GetLocalSize().X;
    TArray<FSlateGradientStop> Stops;
    Stops.Emplace(FVector2D(0, 0), FLinearColor(.93F, .94F, .95F, .99F));
    Stops.Emplace(FVector2D(Width * .40F, 0), FLinearColor(.93F, .94F, .95F, .96F));
    Stops.Emplace(FVector2D(Width * .65F, 0), FLinearColor(.93F, .94F, .95F, .92F));
    Stops.Emplace(FVector2D(Width * .76F, 0), FLinearColor(.93F, .94F, .95F, .12F));
    Stops.Emplace(FVector2D(Width, 0), FLinearColor(.93F, .94F, .95F, .04F));
    FSlateDrawElement::MakeGradient(Elements, Layer, Geometry.ToPaintGeometry(), MoveTemp(Stops), Orient_Vertical);
    return Layer;
  }
};
}  // namespace

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildDriveInterface() {
  return SNew(SOverlay)
      + SOverlay::Slot()[SNew(SLingTuSimNavigationOverlay).World(World.Get()).Task(InspectionProjection)
        .Pose_Lambda([this]() {
          FNavigationPose Pose;
          Pose.bAvailable = CachedStatus.bIdentityCoherent && CachedStatus.bLatestAppliedTruthAvailable &&
            CachedStatus.bObservedBasePoseAvailable && CachedStatus.bFullStatusFresh && CachedStatus.bFullStatusIdentityCoherent;
          Pose.PositionMeters = CachedStatus.ObservedBasePositionMeters;
          Pose.Rotation = CachedStatus.ObservedBaseRotation;
          return Pose;
        })]
      + SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Top).Padding(48, 42, 0, 0)
          [SNew(SBox).MaxDesiredWidth(380)
            [SNew(SHorizontalBox)
              + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Top).Padding(0, 0, 14, 0)
                  [Label(TEXT("◇"), 21, FLinearColor::White)]
              + SHorizontalBox::Slot().FillWidth(1)
                  [SNew(SVerticalBox)
                    + SVerticalBox::Slot().AutoHeight()
                        [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetDriveMissionText)
                          .Font(FieldFont(18)).ColorAndOpacity(FLinearColor::White).AutoWrapText(true)
                          .ShadowOffset(FVector2D(1, 1)).ShadowColorAndOpacity(Shadow)]
                    + SVerticalBox::Slot().AutoHeight().Padding(0, 7, 0, 0)
                        [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetInspectionProgressText)
                          .Visibility_Lambda([this]() { return InspectionProjection.IsValid() && InspectionProjection->bBound
                              ? EVisibility::HitTestInvisible : EVisibility::Collapsed; })
                          .Font(FieldFont(12)).ColorAndOpacity(this, &SLingTuSimRuntimeHUD::GetInspectionStatusColor)
                          .AutoWrapText(true).ShadowOffset(FVector2D(1, 1)).ShadowColorAndOpacity(Shadow)]
                    + SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
                        [SNew(STextBlock).Text_Lambda([this]() { return FInspectionGuidance::PhaseText(*InspectionProjection); })
                          .Font(FieldFont(12)).ColorAndOpacity(FLinearColor(.90F, .74F, .43F, 1))
                          .AutoWrapText(true).ShadowOffset(FVector2D(1, 1)).ShadowColorAndOpacity(Shadow)]]]]
      + SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Top).Padding(0, 44, 48, 0)
          [SNew(SBox).MaxDesiredWidth(290)
              [SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
                    [SNew(SHorizontalBox)
                      + SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 9, 0)
                          [SNew(STextBlock).Text(FText::FromString(TEXT("○"))).Font(FieldFont(14))
                            .ColorAndOpacity(this, &SLingTuSimRuntimeHUD::GetDriveNoticeColor)]
                      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
                          [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetDriveNoticeText)
                            .Font(FieldFont(12)).ColorAndOpacity(this, &SLingTuSimRuntimeHUD::GetDriveNoticeColor)
                            .ShadowOffset(FVector2D(1, 1)).ShadowColorAndOpacity(Shadow)]]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0).HAlign(HAlign_Right)
                    [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetPlayerFrameRateText)
                      .Visibility_Lambda([this]() { return PlayerSettings->bShowFrameRate ? EVisibility::SelfHitTestInvisible : EVisibility::Collapsed; })
                      .Font(FieldFont(11)).ColorAndOpacity(Secondary)]]]
      + SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Bottom).Padding(0, 0, 48, 42)
          [SNew(SBox).WidthOverride(190)
              [SNew(SVerticalBox)
              + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
                  [SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().AutoWidth()
                        [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetDriveSpeedValueText)
                          .Font(FieldFont(40)).ColorAndOpacity(FLinearColor::White)
                          .ShadowOffset(FVector2D(1, 1)).ShadowColorAndOpacity(Shadow)]
                    + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Bottom).Padding(10, 0, 0, 8)
                        [Label(TEXT("m/s"), 14, FLinearColor(.87F, .87F, .87F, 1))]]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 7, 0, 0)
                  [SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center).Padding(0, 0, 12, 0)[Rule()]
                    + SHorizontalBox::Slot().AutoWidth()[Label(TEXT("速度"), 11, FLinearColor(.78F, .78F, .78F, 1))]
                    + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center).Padding(12, 0, 0, 0)[Rule()]]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0).HAlign(HAlign_Right)
                  [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetDriveRecordingText)
                    .Visibility_Lambda([this]() {
                      if (!HasCurrentStatus(CachedStatus)) return EVisibility::Collapsed;
                      const auto State = CachedStatus.FullStatus.Recording.State;
                      return State == LingTuSim::EControlRecordingState::Recording || State == LingTuSim::EControlRecordingState::Failed ||
                          State == LingTuSim::EControlRecordingState::Committed ? EVisibility::HitTestInvisible : EVisibility::Collapsed;
                    }).Font(FieldFont(11)).ColorAndOpacity(Secondary)]]]
      + SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Bottom).Padding(48, 0, 0, 42)
          [SNew(SVerticalBox)
            + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 18)
                [SNew(SBox).MaxDesiredWidth(340).MaxDesiredHeight(310)
                  .Visibility_Lambda([this]() { return ModeController->IsDriveHelpVisible() ? EVisibility::HitTestInvisible : EVisibility::Collapsed; })
                  [BuildDriveHelp()]]
            + SVerticalBox::Slot().AutoHeight()
                [SNew(SHorizontalBox).Visibility_Lambda([this]() {
                  return PlayerSettings->bShowControlHints || ModeController->IsDriveHelpVisible()
                      ? EVisibility::HitTestInvisible : EVisibility::Collapsed;
                })
                  + SHorizontalBox::Slot().AutoWidth()[QuietKeyHint(TEXT("F1"), TEXT("操作"))]
                  + SHorizontalBox::Slot().AutoWidth()[QuietKeyHint(TEXT("Esc"), TEXT("菜单"))]]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildDriveHelp() {
  const auto Row = [](const TCHAR* Action, TAttribute<FText> Keys) -> TSharedRef<SWidget> {
    return SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1)[Label(Action, 12, FLinearColor::White)]
        + SHorizontalBox::Slot().AutoWidth().Padding(18, 0, 0, 0)
            [SNew(STextBlock).Text(Keys).Font(FieldFont(12)).ColorAndOpacity(FLinearColor(.8F, .8F, .8F, 1))];
  };
  return SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor(0, 0, 0, .72F)).Padding(18)
      [SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 14)[Label(TEXT("操作指南"), 15, FLinearColor::White)]
        + SVerticalBox::Slot().AutoHeight()[Row(TEXT("移动"), TAttribute<FText>::CreateLambda([this]() { return GetDriveKeysText(false); }))]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 10)[Row(TEXT("转向"), TAttribute<FText>::CreateLambda([this]() { return GetDriveKeysText(true); }))]
        + SVerticalBox::Slot().AutoHeight()[Row(TEXT("旋转视角"), FText::FromString(TEXT("右键拖动")))]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 10)[Row(TEXT("缩放 / 前后移动"), FText::FromString(TEXT("滚轮")))]
        + SVerticalBox::Slot().AutoHeight()[Row(TEXT("切换视角"), FText::FromString(TEXT("C / 中键")))]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 10)[Row(TEXT("运行详情"), FText::FromString(TEXT("Tab")))]
        + SVerticalBox::Slot().AutoHeight()[Row(TEXT("开始 / 停止录制"), FText::FromString(TEXT("R")))]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 14, 0, 0)
            [Label(TEXT("按住 F1 / 手柄 View 展开，松开收起"), 10, Secondary)]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildTacticalInterface() {
  const auto Readout = [this](const TCHAR* Title, FText (SLingTuSimRuntimeHUD::*Value)() const) -> TSharedRef<SWidget> {
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[Label(Title, 12, Secondary, true)]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
            [SNew(STextBlock).Text(this, Value).Font(FieldFont(11)).ColorAndOpacity(Paper).AutoWrapText(true)];
  };
  return SNew(SOverlay)
      + SOverlay::Slot().HAlign(HAlign_Fill).VAlign(VAlign_Top).Padding(44, 36, 44, 0)
          [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1)[Label(TEXT("运行信息"), 24, Paper, true)]
            + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)[KeyHint(TEXT("Tab"), TEXT("返回操作"))]]
      + SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Fill).Padding(44, 100, 0, 90)
          [SNew(SBox).WidthOverride(370)
            [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(Panel).Padding(24)
              [SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()[Readout(TEXT("操作请求"), &SLingTuSimRuntimeHUD::GetRequestedAxesText)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 22, 0, 22)[Rule()]
                + SVerticalBox::Slot().AutoHeight()[Readout(TEXT("控制端受理"), &SLingTuSimRuntimeHUD::GetAckText)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
                    [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetAcceptedMotionText)
                      .Font(FieldFont(10)).ColorAndOpacity(Secondary).AutoWrapText(true)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 22, 0, 22)[Rule()]
                + SVerticalBox::Slot().AutoHeight()[Readout(TEXT("物理观测"), &SLingTuSimRuntimeHUD::GetObservedVelocityText)]
                + SVerticalBox::Slot().FillHeight(1)[SNew(SSpacer)]
                + SVerticalBox::Slot().AutoHeight()[Label(TEXT("请求、受理与真实运动分别显示。"), 10, Secondary)]]]]
      + SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Fill).Padding(0, 100, 44, 90)
          [SNew(SBox).WidthOverride(370)
            [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(Panel).Padding(24)
              [SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()[Readout(TEXT("会话绑定"), &SLingTuSimRuntimeHUD::GetDriveSessionText)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 20)[Rule()]
                + SVerticalBox::Slot().AutoHeight()[Readout(TEXT("传感器与记录"), &SLingTuSimRuntimeHUD::GetSensorsText)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
                    [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetRecordingText)
                      .Font(FieldFont(11)).ColorAndOpacity(Secondary).AutoWrapText(true)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 20)[Rule()]
                + SVerticalBox::Slot().AutoHeight()[Readout(TEXT("真值与任务"), &SLingTuSimRuntimeHUD::GetTruthFrameText)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
                    [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetInspectionProgressText)
                      .Font(FieldFont(11)).ColorAndOpacity(this, &SLingTuSimRuntimeHUD::GetInspectionStatusColor).AutoWrapText(true)]
                + SVerticalBox::Slot().AutoHeight().Padding(0, 7, 0, 0)
                    [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetInspectionReportText)
                      .Font(FieldFont(10)).ColorAndOpacity(Secondary).AutoWrapText(true)]]]]
      + SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Bottom).Padding(0, 0, 0, 35)
          [Label(TEXT("只读信息视图  /  中央保留真实场景，不绘制未绑定的地图或路线"), 11, Secondary)];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildRuntimeMenu() {
  const auto OpenPage = [this](const TCHAR* Title, EPlayerMenuPage Page) {
    return MenuButton(Label(Title, 22), FOnClicked::CreateLambda([this, Page]() {
      ShowMenuPage(Page); return FReply::Handled();
    }));
  };
  ResumeMenuButton = MenuButton(Label(TEXT("继续游戏"), 22),
      FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::HandleMenuResume));
  SettingsMenuButton = OpenPage(TEXT("设置"), EPlayerMenuPage::Settings);
  JournalMenuButton = OpenPage(TEXT("任务日志"), EPlayerMenuPage::Journal);
  MeasurementsMenuButton = OpenPage(TEXT("传感器"), EPlayerMenuPage::Measurements);
  SAssignNew(MeasurementPanel, SLingTuSimMeasurements).Client(Measurements)
      .OnBack(FSimpleDelegate::CreateLambda([this]() { ShowMenuPage(EPlayerMenuPage::Root); }));
  LibraryMenuButton = OpenPage(TEXT("机器人与地图"), EPlayerMenuPage::Library);
  auto Root = SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 0, 36)[Label(TEXT("菜单"), 13, FLinearColor(.65F, .65F, .65F, 1))]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 0, 8)
          [Label(TEXT("当前任务"), 11, FLinearColor(.55F, .61F, .68F, 1))]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 0, 7)
          [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetDriveMissionText)
            .Font(FieldFont(15)).ColorAndOpacity(FLinearColor::White).AutoWrapText(true)]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 0, 24)
          [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetDriveSessionText)
            .Font(FieldFont(11)).ColorAndOpacity(FLinearColor(.55F, .61F, .68F, 1)).AutoWrapText(true)]
      + SVerticalBox::Slot().AutoHeight()[ResumeMenuButton.ToSharedRef()]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 8)[JournalMenuButton.ToSharedRef()]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 8)[MeasurementsMenuButton.ToSharedRef()]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 8)[LibraryMenuButton.ToSharedRef()]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 8)[OpenPage(TEXT("重新开始"), EPlayerMenuPage::Restart)]
      + SVerticalBox::Slot().AutoHeight()[SettingsMenuButton.ToSharedRef()]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 8)[OpenPage(TEXT("返回主菜单"), EPlayerMenuPage::MainMenu)]
      + SVerticalBox::Slot().AutoHeight()[OpenPage(TEXT("退出游戏"), EPlayerMenuPage::Quit)];
  return SNew(SOverlay)
      + SOverlay::Slot()[SNew(SPlayerMenuBackdrop)]
      + SOverlay::Slot()[SNew(SOverlay)
          .Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Settings ? EVisibility::HitTestInvisible : EVisibility::Collapsed; })
          + SOverlay::Slot()[SNew(SBorder).Tag(TEXT("LingTu.Settings.Backdrop")).BorderImage(Fill())
              .BorderBackgroundColor(SettingsSurface).Padding(0)]
          + SOverlay::Slot()[SNew(SLingTuSimShowroom).Tag(TEXT("LingTu.Settings.RobotBackdrop"))
              .Visibility(EVisibility::HitTestInvisible).RobotId_Lambda([this]() { return SelectionModel->GetRequestedRobotId(); })]
          + SOverlay::Slot()[SNew(SSettingsVeil)]
          + SOverlay::Slot().VAlign(VAlign_Bottom)[SNew(SBox).HeightOverride(130)
              [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor(.93F, .94F, .95F, .92F)).Padding(0)]]]
      + SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Center).Padding(100, 48, 48, 80)
          [SNew(SBox).Visibility_Lambda([this]() { return MenuPage != EPlayerMenuPage::Settings ? EVisibility::Visible : EVisibility::Collapsed; })
            [SNew(SScaleBox).Stretch(EStretch::ScaleToFit).StretchDirection(EStretchDirection::DownOnly)
              [SNew(SBox).WidthOverride_Lambda([this]() {
                  return FOptionalSize(MenuPage == EPlayerMenuPage::Library ? 1200.F :
                      MenuPage == EPlayerMenuPage::Journal || MenuPage == EPlayerMenuPage::Measurements ? 1000.F : 480.F);
                })[SNew(SOverlay)
                  + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Root ? EVisibility::Visible : EVisibility::Collapsed; })[Root]]
                  + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Journal ? EVisibility::Visible : EVisibility::Collapsed; })[BuildTaskJournal()]]
                  + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Measurements ? EVisibility::Visible : EVisibility::Collapsed; })[MeasurementPanel.ToSharedRef()]]
                  + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Library ? EVisibility::Visible : EVisibility::Collapsed; })[BuildRobotMapPicker(false)]]
                  + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() {
                      return MenuPage == EPlayerMenuPage::Restart || MenuPage == EPlayerMenuPage::MainMenu || MenuPage == EPlayerMenuPage::Quit
                          ? EVisibility::Visible : EVisibility::Collapsed;
                    })[BuildMenuConfirmation()]]]]]]
      + SOverlay::Slot().HAlign(HAlign_Fill).VAlign(VAlign_Fill).Padding(64, 40, 64, 56)
          [SNew(SBox).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Settings ? EVisibility::Visible : EVisibility::Collapsed; })
            [BuildPlayerSettings()]]
      + SOverlay::Slot().HAlign(HAlign_Left).VAlign(VAlign_Bottom).Padding(80, 0, 0, 32)
          [SNew(SOverlay)
            + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Settings ? EVisibility::Collapsed : EVisibility::Visible; })
                [SNew(SHorizontalBox)
                  + SHorizontalBox::Slot().AutoWidth()[KeyHint(TEXT("↑ ↓"), TEXT("选择"))]
                  + SHorizontalBox::Slot().AutoWidth()[KeyHint(TEXT("Enter"), TEXT("确认"))]
                  + SHorizontalBox::Slot().AutoWidth()[KeyHint(TEXT("Esc"), TEXT("返回"))]]]
            + SOverlay::Slot()[SNew(STextBlock).Visibility_Lambda([this]() { return MenuPage == EPlayerMenuPage::Settings ? EVisibility::Visible : EVisibility::Collapsed; })
                .Text(FText::FromString(TEXT("↑ ↓ 选择     ← → 调整     Enter 确认     Esc 返回"))).Font(FieldFont(12)).ColorAndOpacity(SettingsMuted)]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildRobotMapPicker(bool bFrontEnd) {
  auto Picker = SNew(SLingTuSimRobotMapPicker).Model(SelectionModel)
    .CanEdit_Lambda([this, bFrontEnd]() {
      return bFrontEnd && !CachedStatus.bSessionAvailable && LoginModel->IsLoggedIn()
          && LoginModel->CanEdit() && !SelectionModel->IsConfirmed();
    })
    .CanConfirm_Lambda([this]() { return !CachedStatus.bSessionAvailable && OnSelectionConfirmed.IsBound() && CanConfirmSelection(); })
    .Status(TAttribute<FText>::CreateSP(this, &SLingTuSimRuntimeHUD::GetSelectionStatusText))
    .OnSelectionChanged(FSimpleDelegate::CreateLambda([this]() {
      SelectionFeedback->Reset();
      Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
      SlatePrepass(1.F);
    }))
    .OnConfirm(FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::HandleConfirmSelection))
    .OnBack(FOnClicked::CreateLambda([this, bFrontEnd]() {
      if (bFrontEnd) return HandleLogout();
      ShowMenuPage(EPlayerMenuPage::Root); return FReply::Handled();
    }))
    .OnBrowseAssets(bFrontEnd ? FOnClicked::CreateLambda([this]() {
      bAssetReviewWorkspace = true;
      Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
      SlatePrepass(1.F);
      return FReply::Handled();
    }) : FOnClicked());
  if (bFrontEnd) WorkspaceRobotMapPicker = Picker;
  else RuntimeRobotMapPicker = Picker;
  return Picker;
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildWorkspaceScreen() {
  return SNew(SOverlay)
    + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() {
        return bAssetReviewWorkspace ? EVisibility::Collapsed : EVisibility::Visible;
      })[BuildRobotMapPicker(true)]]
    + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() {
        return bAssetReviewWorkspace ? EVisibility::Visible : EVisibility::Collapsed;
      })[BuildAssetReviewWorkspace()]]
    + SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Top).Padding(0, 22)
      [SNew(SBox).Visibility_Lambda([this]() { return bAssetReviewWorkspace ? EVisibility::Visible : EVisibility::Collapsed; })
        [MenuButton(Label(TEXT("返回机器人与地图"), 14), FOnClicked::CreateLambda([this]() {
          bAssetReviewWorkspace = false;
          Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
          SlatePrepass(1.F);
          if (WorkspaceRobotMapPicker.IsValid()) WorkspaceRobotMapPicker->FocusInitialControl();
          return FReply::Handled();
        }))]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildTaskJournal() {
  JournalBackButton = MenuButton(Label(TEXT("返回"), 18), FOnClicked::CreateLambda([this]() {
    ShowMenuPage(EPlayerMenuPage::Root); return FReply::Handled();
  }));
  const auto TaskButton = [this](const TCHAR* Caption, const TCHAR* Command, FSimpleDelegate Callback) {
    return MenuButton(Label(Caption, 16), FOnClicked::CreateLambda([this, Command, Callback]() {
      if (InspectionProjection->CanRequestControl(Command)) Callback.ExecuteIfBound();
      return FReply::Handled();
    }), TAttribute<bool>::CreateLambda([this, Command, Callback]() {
      return Callback.IsBound() && InspectionProjection->CanRequestControl(Command);
    }));
  };
  return SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 0, 22)[Label(TEXT("任务日志"), 28)]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0)[Rule()]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 22)
          [SNew(STextBlock).Font(FieldFont(20)).ColorAndOpacity(FLinearColor::White).AutoWrapText(true)
            .Text_Lambda([this]() {
              return !InspectionProjection->bBound
                  ? FText::FromString(TEXT("当前会话未绑定巡检任务"))
                  : FInspectionGuidance::PhaseText(*InspectionProjection);
            })]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 18, 22)
          [SNew(STextBlock).Font(FieldFont(12)).ColorAndOpacity(Secondary).AutoWrapText(true)
            .Text_Lambda([this]() { return FText::FromString(InspectionProjection->bBound
                ? TEXT("当前任务摘要 · 不生成未收到的历史事件")
                : TEXT("可以继续自由驾驶。只有明确绑定任务后，这里才会显示巡检记录。")); })]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SBox).Visibility_Lambda([this]() { return InspectionProjection->bBound ? EVisibility::Visible : EVisibility::Collapsed; })
            [SNew(SVerticalBox)
              + SVerticalBox::Slot().AutoHeight()
                [SNew(SHorizontalBox)
                  + SHorizontalBox::Slot().FillWidth(1).Padding(18, 0, 36, 0)
                    [SNew(SVerticalBox)
                      + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)[Label(TEXT("执行进度"), 12, Secondary)]
                      + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetInspectionProgressText)
                          .Font(FieldFont(16)).ColorAndOpacity(Paper).AutoWrapText(true)]
                      + SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 0)
                        [SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(InspectionProjection->Reason); })
                          .Font(FieldFont(12)).ColorAndOpacity(Secondary).AutoWrapText(true)]]
                  + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 18, 0)
                    [SNew(SVerticalBox)
                      + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)[Label(TEXT("证据与验收"), 12, Secondary)]
                      + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetInspectionReportText)
                          .Font(FieldFont(16)).ColorAndOpacity(Paper).AutoWrapText(true)]
                      + SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 0)
                        [SNew(STextBlock).Text(FText::FromString(TEXT("执行完成不等于证据齐全，也不等于验收通过。")))
                          .Font(FieldFont(12)).ColorAndOpacity(Secondary).AutoWrapText(true)]]]
              + SVerticalBox::Slot().AutoHeight().Padding(18, 26)
                [SNew(SHorizontalBox)
                  + SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 22, 0)[TaskButton(TEXT("暂停巡检"), TEXT("pause"), OnInspectionPause)]
                  + SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 22, 0)[TaskButton(TEXT("继续巡检"), TEXT("resume"), OnInspectionResume)]
                  + SHorizontalBox::Slot().AutoWidth()[TaskButton(TEXT("取消巡检"), TEXT("cancel"), OnInspectionCancel)]]
              + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 18, 12)
                [SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(InspectionProjection->CommandFeedback); })
                  .Font(FieldFont(12)).ColorAndOpacity(Secondary).AutoWrapText(true)]]]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 18, 18)[Rule()]
      + SVerticalBox::Slot().AutoHeight()[JournalBackButton.ToSharedRef()];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildPlayerSettings() {
  const auto Category = [this](ESettingsTab Tab, const TCHAR* Caption, TSharedPtr<SButton>& Button) -> TSharedRef<SWidget> {
    Button = SNew(SButton).ButtonStyle(SettingRowStyle()).ContentPadding(0)
        .OnClicked_Lambda([this, Tab]() { SelectSettingsTab(Tab); return FReply::Handled(); });
    const TWeakPtr<SButton> Weak = Button;
    Button->SetContent(SNew(SBorder).BorderImage(Fill()).Padding(FMargin(14, 16))
        .BorderBackgroundColor_Lambda([this, Tab, Weak]() {
          const auto Row = Weak.Pin();
          return SettingsTab == Tab || (Row.IsValid() && Row->HasKeyboardFocus())
              ? FLinearColor(.77F, .84F, .89F, 1.F) : FLinearColor::Transparent;
        })
        [SNew(SHorizontalBox)
          + SHorizontalBox::Slot().FillWidth(1)[SNew(STextBlock).Text(FText::FromString(Caption)).Font(FieldFont(18))
              .ColorAndOpacity_Lambda([this, Tab]() { return SettingsTab == Tab ? SettingsAccent : SettingsInk; })]
          + SHorizontalBox::Slot().AutoWidth()[SNew(STextBlock).Text(FText::FromString(TEXT("›"))).Font(FieldFont(19))
              .ColorAndOpacity(SettingsAccent).Visibility_Lambda([this, Tab]() {
                return SettingsTab == Tab ? EVisibility::Visible : EVisibility::Hidden;
              })]]);
    return Button.ToSharedRef();
  };
  SettingsBackButton = SettingsButton(TEXT("返回"), FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::RequestSettingsBack));
  SettingsContinueButton = SettingsButton(TEXT("继续设置"),
      FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::ContinueSettingsEdit), true);

  auto Form = SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1)[SettingsText(TEXT("设置"), 36, SettingsInk, true)]
            + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)[SettingsText(TEXT("LINGTU"), 12, SettingsMuted)]]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 26)[SettingsRule()]
      + SVerticalBox::Slot().FillHeight(1)
          [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 32, 0)
                [SNew(SBox).WidthOverride(160)[SNew(SVerticalBox)
                  + SVerticalBox::Slot().AutoHeight().Padding(14, 0, 0, 12)[SettingsText(TEXT("设置分类"), 12, SettingsMuted)]
                  + SVerticalBox::Slot().AutoHeight()[Category(ESettingsTab::Graphics, TEXT("画面"), GraphicsTabButton)]
                  + SVerticalBox::Slot().AutoHeight().Padding(0, 4)[Category(ESettingsTab::Controls, TEXT("操作"), ControlsTabButton)]
                  + SVerticalBox::Slot().AutoHeight()[Category(ESettingsTab::Interface, TEXT("界面"), InterfaceTabButton)]
                  + SVerticalBox::Slot().AutoHeight().Padding(0, 4)[Category(ESettingsTab::Audio, TEXT("声音"), AudioTabButton)]]]
            + SHorizontalBox::Slot().FillWidth(.68F)[SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 0, 10)
                    [SNew(STextBlock).Font(FieldFont(21)).ColorAndOpacity(SettingsInk).Text_Lambda([this]() {
                      return FText::FromString(SettingsTab == ESettingsTab::Graphics ? TEXT("显示与画质")
                          : SettingsTab == ESettingsTab::Audio ? TEXT("声音与反馈")
                          : SettingsTab == ESettingsTab::Controls ? TEXT("视角与驾驶") : TEXT("驾驶界面"));
                    })]
                + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 0, 18)
                    [SNew(STextBlock).Font(FieldFont(12)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)
                      .Text_Lambda([this]() { return FText::FromString(SettingsTab == ESettingsTab::Graphics
                          ? TEXT("画面修改需应用并确认。不影响传感器输出配置。")
                          : TEXT("修改立即预览。保存后保留；返回前可选择是否放弃修改。")); })]
                + SVerticalBox::Slot().FillHeight(1)[SAssignNew(SettingsAnimatedContent, SBox)[SNew(SOverlay)
                    + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return SettingsTab == ESettingsTab::Graphics ? EVisibility::Visible : EVisibility::Collapsed; })[BuildGraphicsSettingsPage()]]
                    + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return SettingsTab == ESettingsTab::Controls ? EVisibility::Visible : EVisibility::Collapsed; })[BuildControlsSettingsPage()]]
                    + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return SettingsTab == ESettingsTab::Interface ? EVisibility::Visible : EVisibility::Collapsed; })[BuildInterfaceSettingsPage()]]
                    + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return SettingsTab == ESettingsTab::Audio ? EVisibility::Visible : EVisibility::Collapsed; })[BuildAudioSettingsPage()]]]]
                + SVerticalBox::Slot().AutoHeight()
                    [SNew(SBox).Visibility_Lambda([this]() { return GraphicsPreview.IsPending() ? EVisibility::Visible : EVisibility::Collapsed; })
                      [SNew(SHorizontalBox)
                        + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center)
                            [SNew(STextBlock).Font(FieldFont(13)).ColorAndOpacity(SettingsInk).Text_Lambda([this]() {
                              return FText::FromString(FString::Printf(TEXT("保留当前画面？%.0f 秒后自动还原"), FMath::CeilToDouble(GraphicsPreview.SecondsRemaining(FPlatformTime::Seconds()))));
                            })]
                        + SHorizontalBox::Slot().AutoWidth()[SettingsButton(TEXT("保留"), FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::ConfirmGraphicsSettings), true)]
                        + SHorizontalBox::Slot().AutoWidth()[SettingsButton(TEXT("还原"), FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::RevertGraphicsSettings))]]]]
            + SHorizontalBox::Slot().FillWidth(.32F).VAlign(VAlign_Bottom).Padding(36, 0, 0, 0)
                [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor(1, 1, 1, .88F)).Padding(24)
                  [SNew(SVerticalBox)
                    + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Font(FieldFont(19, true)).ColorAndOpacity(SettingsInk)
                        .Text_Lambda([this]() { return SettingsTab == ESettingsTab::Graphics
                            ? (SettingsHelpTitle.IsEmpty() ? FText::FromString(TEXT("画面设置")) : SettingsHelpTitle)
                            : FText::FromString(SettingsTab == ESettingsTab::Audio ? TEXT("声音与反馈")
                                : SettingsTab == ESettingsTab::Controls ? TEXT("视角与驾驶") : TEXT("驾驶界面")); })]
                    + SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
                        [SNew(STextBlock).Tag(TEXT("LingTu.Settings.Help")).Font(FieldFont(13)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)
                          .Text_Lambda([this]() { return SettingsTab == ESettingsTab::Graphics
                              ? (SettingsHelpBody.IsEmpty() ? FText::FromString(TEXT("选择一项查看说明。画面应用后有 15 秒确认时间。")) : SettingsHelpBody)
                              : FText::FromString(SettingsTab == ESettingsTab::Audio
                                  ? TEXT("主音量控制本应用的声音，界面音量只控制操作提示音。静音会保留原音量。修改可即时试听；保存后保留，撤销或放弃可恢复。")
                                  : SettingsTab == ESettingsTab::Controls
                                  ? TEXT("点击驾驶动作可重新绑定按键。Esc 取消改键；保存后保留。")
                                  : TEXT("设置驾驶时显示的信息。任务、传感器和机器人控制不会被这些显示开关更改。")); })]]]]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 12)[SettingsRule()]
      + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
          + SHorizontalBox::Slot().AutoWidth()[SettingsBackButton.ToSharedRef()]
          + SHorizontalBox::Slot().FillWidth(1)[SNew(SSpacer)]
          + SHorizontalBox::Slot().AutoWidth()[SettingsButton(TEXT("恢复默认"), FOnClicked::CreateLambda([this]() {
              if (SettingsTab == ESettingsTab::Graphics) {
                if (GraphicsPreview.IsPending()) {
                  GraphicsPreview.Revert();
                  GraphicsDraft = FPlayerGraphicsSettings::Capture(World.Get());
                  GraphicsSettingsCheckpoint = GraphicsDraft;
                }
                GraphicsDraft = FPlayerGraphicsSettings();
                MenuFeedback = TEXT("默认画质已暂存，点击应用画面后生效");
              } else {
                const FPlayerSettings Defaults;
                if (SettingsTab == ESettingsTab::Controls) {
                  CancelKeyBindingCapture();
                  PlayerSettings->MouseSensitivity = Defaults.MouseSensitivity;
                  PlayerSettings->bInvertMouseY = Defaults.bInvertMouseY;
                  PlayerSettings->DriveKeys = Defaults.DriveKeys;
                } else if (SettingsTab == ESettingsTab::Audio) {
                  PlayerSettings->MasterVolume = Defaults.MasterVolume;
                  PlayerSettings->UIVolume = Defaults.UIVolume;
                  PlayerSettings->MusicVolume = Defaults.MusicVolume;
                  PlayerSettings->MenuMusicTrack = Defaults.MenuMusicTrack;
                  PlayerSettings->bMuteAudio = Defaults.bMuteAudio;
                  ApplyPlayerAudioSettings(World.Get(), *PlayerSettings);
                } else {
                  PlayerSettings->bShowControlHints = Defaults.bShowControlHints;
                  PlayerSettings->bShowFrameRate = Defaults.bShowFrameRate;
                }
                MenuFeedback = TEXT("本页已恢复默认，点击保存可保留");
              }
              return FReply::Handled();
            }))]
          + SHorizontalBox::Slot().AutoWidth()[SettingsButton(TEXT("撤销修改"), FOnClicked::CreateLambda([this]() {
              CancelKeyBindingCapture();
              if (GraphicsPreview.IsPending()) {
                GraphicsPreview.Revert();
                GraphicsDraft = FPlayerGraphicsSettings::Capture(World.Get());
                GraphicsSettingsCheckpoint = GraphicsDraft;
              } else GraphicsDraft = GraphicsSettingsCheckpoint;
              RestoreUnsavedPlayerSettings(true);
              MenuFeedback = TEXT("已撤销尚未应用或保存的修改");
              return FReply::Handled();
            }))]
          + SHorizontalBox::Slot().AutoWidth().Padding(16, 0, 0, 0)
              [SNew(SOverlay)
                + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return SettingsTab == ESettingsTab::Graphics ? EVisibility::Visible : EVisibility::Collapsed; })
                    [SettingsButton(TEXT("应用画面"), FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::ApplyGraphicsSettings), true,
                        TAttribute<bool>::CreateSP(this, &SLingTuSimRuntimeHUD::CanApplyGraphicsSettings))]]
                + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() { return SettingsTab != ESettingsTab::Graphics ? EVisibility::Visible : EVisibility::Collapsed; })
                    [SettingsButton(TEXT("保存设置"), FOnClicked::CreateLambda([this]() {
                      CancelKeyBindingCapture();
                      if (PlayerSettings->Save()) {
                        AdvancePlayerSettingsCheckpoint();
                        MenuFeedback = TEXT("设置已保存");
                        EmitMenuCue(EMenuCue::Confirm);
                      } else MenuFeedback = TEXT("保存失败，请检查配置文件是否可写；当前修改仍未保存");
                      return FReply::Handled();
                    }), true)]]]]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)[SNew(SBox).MinDesiredHeight(22)
          [SNew(STextBlock).Tag(TEXT("PlayerSettingsFeedback")).Text_Lambda([this]() {
            if (!MenuFeedback.IsEmpty()) return FText::FromString(MenuFeedback);
            const auto Change = MenuMotion.ChangeText(FSlateApplication::Get().GetCurrentTime());
            if (!Change.IsEmpty()) return Change;
            return FText::FromString(GraphicsPreview.IsPending() ? TEXT("画面已应用，等待确认")
                : HasGraphicsDraftChanges() ? TEXT("有未应用的画面修改")
                : HasSettingsChanges() ? TEXT("有未保存的修改") : TEXT(""));
          }).Font(FieldFont(12)).ColorAndOpacity_Lambda([this]() {
            if (KeyBindingErrorUntil > 0.) return FLinearColor(.48F, .035F, .025F, 1.F);
            const float Alpha = MenuMotion.ChangeOpacity(FSlateApplication::Get().GetCurrentTime());
            return MenuFeedback.IsEmpty() && Alpha > 0.F ? SettingsAccent.CopyWithNewOpacity(Alpha) : SettingsMuted;
          }).AutoWrapText(true)]];

  return SNew(SOverlay)
      + SOverlay::Slot()[SNew(SBox).Visibility_Lambda([this]() {
          return bSettingsExitDecision ? EVisibility::Collapsed : EVisibility::Visible;
        })[Form]]
      + SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Center)[SNew(SBox).MaxDesiredWidth(850)
        [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor::White)
          .Padding(48).Visibility_Lambda([this]() { return bSettingsExitDecision ? EVisibility::Visible : EVisibility::Collapsed; })
          [SNew(SVerticalBox)
            + SVerticalBox::Slot().AutoHeight()[SettingsText(TEXT("放弃未保存的修改？"), 28, SettingsInk, true)]
            + SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 32)
                [SNew(STextBlock).Text(FText::FromString(TEXT("未保存的操作和界面设置将还原；未应用的画面修改也会丢弃。已保存的设置不会受影响。")))
                  .Font(FieldFont(15)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)]
            + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
                + SHorizontalBox::Slot().AutoWidth()[SettingsContinueButton.ToSharedRef()]
                + SHorizontalBox::Slot().AutoWidth().Padding(16, 0, 0, 0)[SettingsButton(TEXT("放弃修改"),
                    FOnClicked::CreateLambda([this]() { ShowMenuPage(EPlayerMenuPage::Root); return FReply::Handled(); }))]]]]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildGraphicsSettingsPage() {
  const auto Changed = [this](const FText& Name, const FText& Value) { NotifySettingChanged(Name, Value, true); };
  const auto Help = [this](const TCHAR* Title, const TCHAR* Description) {
    return [this, Title, Description]() {
      SettingsHelpTitle = FText::FromString(Title);
      SettingsHelpBody = FText::FromString(Description);
    };
  };
  const auto Quality = [this, Help, Changed](const TCHAR* Title, const TCHAR* Description, int32 FPlayerGraphicsSettings::*Member) -> TSharedRef<SWidget> {
    return SNew(SPlayerSettingChoice).Label(FText::FromString(Title))
      .OnChanged(Changed)
      .OnInspect(Help(Title, Description))
      .Value_Lambda([this, Member]() { return QualityLabel(GraphicsDraft.*Member); })
      .OnAdjust([this, Member](int32 Direction) { GraphicsDraft.*Member = StepOption(GraphicsDraft.*Member, Direction, 4); });
  };
  auto Display = SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 0, 12)[SettingsText(TEXT("显示"), 12, SettingsMuted)]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("显示模式")))
            .OnChanged(Changed)
            .OnInspect(Help(TEXT("显示模式"), TEXT("切换全屏、无边框或窗口显示。应用后需确认，未确认会恢复原来的模式。")))
            .Value_Lambda([this]() { const TCHAR* Names[] = {TEXT("全屏"), TEXT("无边框"), TEXT("窗口")}; return FText::FromString(Names[FMath::Clamp(GraphicsDraft.WindowMode, 0, 2)]); })
            .OnAdjust([this](int32 Direction) { GraphicsDraft.WindowMode = StepOption(GraphicsDraft.WindowMode, Direction, 3); })]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("分辨率")))
            .OnChanged(Changed)
            .OnInspect(Help(TEXT("分辨率"), TEXT("调整主视口的显示尺寸。更高分辨率更清晰，也会增加显卡负载。")))
            .Value_Lambda([this]() { return FText::FromString(FString::Printf(TEXT("%d × %d"), GraphicsDraft.Resolution.X, GraphicsDraft.Resolution.Y)); })
            .OnAdjust([this](int32 Direction) {
              if (!GraphicsResolutions.IsEmpty()) GraphicsDraft.Resolution = GraphicsResolutions[StepOption(GraphicsResolutions.IndexOfByKey(GraphicsDraft.Resolution), Direction, GraphicsResolutions.Num())];
            })]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("帧率上限")))
            .OnChanged(Changed)
            .OnInspect(Help(TEXT("帧率上限"), TEXT("限制主视口帧率，降低显卡负载。不会修改传感器的采样频率。")))
            .Value_Lambda([this]() { return FText::FromString(GraphicsDraft.FrameRateLimit <= 0 ? TEXT("不限") : FString::Printf(TEXT("%.0f FPS"), GraphicsDraft.FrameRateLimit)); })
            .OnAdjust([this](int32 Direction) {
              const TArray<float> Options{30.F, 60.F, 90.F, 120.F, 144.F, 0.F};
              GraphicsDraft.FrameRateLimit = Options[StepOption(Options.IndexOfByKey(GraphicsDraft.FrameRateLimit), Direction, Options.Num())];
            })]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("垂直同步")))
            .OnChanged(Changed)
            .OnInspect(Help(TEXT("垂直同步"), TEXT("减少画面撕裂，可能增加输入延迟。开启后帧率还会受到显示器刷新率限制。")))
            .Value_Lambda([this]() { return FText::FromString(GraphicsDraft.bVSync ? TEXT("开启") : TEXT("关闭")); })
            .OnAdjust([this](int32) { GraphicsDraft.bVSync = !GraphicsDraft.bVSync; })]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("渲染比例")))
            .OnChanged(Changed)
            .OnInspect(Help(TEXT("渲染比例"), TEXT("降低内部渲染分辨率可提升性能，但会使画面变软。项目默认保留原有配置。")))
            .Value_Lambda([this]() { return FText::FromString(GraphicsDraft.ResolutionScale == 0.F ? TEXT("项目默认") : FString::Printf(TEXT("%.0f %%"), GraphicsDraft.ResolutionScale)); })
            .OnAdjust([this](int32 Direction) {
              const TArray<float> Options{0.F, 50.F, 60.F, 70.F, 80.F, 90.F, 100.F};
              GraphicsDraft.ResolutionScale = Options[StepOption(Options.IndexOfByKey(GraphicsDraft.ResolutionScale), Direction, Options.Num())];
            })];
  auto QualityOptions = SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 0, 12)[SettingsText(TEXT("画质"), 12, SettingsMuted)]
      + SVerticalBox::Slot().AutoHeight()
          [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("画质预设")))
            .OnChanged(Changed)
            .OnInspect(Help(TEXT("画质预设"), TEXT("同时调整下方的画质选项。单独修改某项后，预设可能显示为自定义。")))
            .Value_Lambda([this]() { return QualityLabel(GraphicsDraft.GetQualityPreset()); })
            .OnAdjust([this](int32 Direction) { GraphicsDraft.SetQualityPreset(StepOption(GraphicsDraft.GetQualityPreset(), Direction, 4)); })]
      + SVerticalBox::Slot().AutoHeight()[Quality(TEXT("视距"), TEXT("调整远处物体的显示距离，不会移除物理场景中的碰撞。"), &FPlayerGraphicsSettings::ViewDistanceQuality)]
      + SVerticalBox::Slot().AutoHeight()[Quality(TEXT("纹理"), TEXT("调整纹理细节等级。较高设置需要更多显存。"), &FPlayerGraphicsSettings::TextureQuality)]
      + SVerticalBox::Slot().AutoHeight()[Quality(TEXT("阴影"), TEXT("调整阴影精细度。降低后可以减轻复杂场景的渲染负担。"), &FPlayerGraphicsSettings::ShadowQuality)]
      + SVerticalBox::Slot().AutoHeight()[Quality(TEXT("抗锯齿"), TEXT("减少物体边缘闪烁和锯齿。实际算法由项目渲染配置决定。"), &FPlayerGraphicsSettings::AntiAliasingQuality)]
      + SVerticalBox::Slot().AutoHeight()[Quality(TEXT("后期处理"), TEXT("调整画面后期效果的质量等级，不更改传感器与物理参数。"), &FPlayerGraphicsSettings::PostProcessingQuality)];
  return SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 10, 12)
          [SNew(STextBlock).Font(FieldFont(11)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)
            .Text_Lambda([]() { return FText::FromString(FPlayerGraphicsPreview::IsSessionLocked()
                ? TEXT("当前仿真会话控制渲染配置。请结束会话后修改画面；传感器分辨率与采样率不在这里调整。")
                : TEXT("左右键调整，点击可切换。画面应用后需确认；未确认将在 15 秒后自动还原。")); })]
      + SVerticalBox::Slot().FillHeight(1)
          [SAssignNew(SettingsScrollBoxes[0], SScrollBox).Tag(TEXT("LingTu.Settings.GraphicsScroll"))
            + SScrollBox::Slot()
                [SNew(SHorizontalBox).IsEnabled_Lambda([this]() { return CanEditGraphicsSettings(); })
                  + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 30, 0)[Display]
                  + SHorizontalBox::Slot().FillWidth(1)[QualityOptions]]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildControlsSettingsPage() {
  const auto Binding = [](const TCHAR* Action, const TCHAR* Keys) -> TSharedRef<SWidget> {
    return SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1)[SettingsText(Action, 14)]
        + SHorizontalBox::Slot().AutoWidth()[SettingsText(Keys, 14, SettingsMuted)];
  };
  return SAssignNew(SettingsScrollBoxes[1], SScrollBox).Tag(TEXT("LingTu.Settings.ControlsScroll"))
      + SScrollBox::Slot()[SNew(SHorizontalBox)
          + SHorizontalBox::Slot().FillWidth(1).Padding(10, 0, 40, 0)[SNew(SVerticalBox)
              + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 18)[SettingsText(TEXT("鼠标与视角"), 12, SettingsMuted)]
              + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
                  + SHorizontalBox::Slot().FillWidth(1)[SettingsText(TEXT("鼠标灵敏度"), 16)]
                  + SHorizontalBox::Slot().AutoWidth()[SNew(STextBlock).Font(FieldFont(15)).ColorAndOpacity(SettingsInk)
                      .Text_Lambda([this]() { return FText::FromString(FString::Printf(TEXT("%.2f ×"), PlayerSettings->MouseSensitivity)); })]]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 22)
                  [SNew(SSlider).Style(PlayerSliderStyle()).MinValue(.25F).MaxValue(3.F).StepSize(.05F).MouseUsesStep(true)
                    .SliderHandleColor(SettingsAccent)
                    .Value_Lambda([this]() { return PlayerSettings->MouseSensitivity; })
                    .OnValueChanged_Lambda([this](float Value) {
                      const float Before = PlayerSettings->MouseSensitivity;
                      PlayerSettings->MouseSensitivity = Value; PlayerSettings->Normalize();
                      if (!FMath::IsNearlyEqual(Before, PlayerSettings->MouseSensitivity))
                        NotifySettingChanged(FText::FromString(TEXT("鼠标灵敏度")), FText::FromString(FString::Printf(TEXT("%.2f ×"), PlayerSettings->MouseSensitivity)), false);
                    })]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 22)
                  [SNew(SCheckBox).ForegroundColor(SettingsInk)
                    .IsChecked_Lambda([this]() { return PlayerSettings->bInvertMouseY ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
                    .OnCheckStateChanged_Lambda([this](ECheckBoxState State) {
                      PlayerSettings->bInvertMouseY = State == ECheckBoxState::Checked;
                      NotifySettingChanged(FText::FromString(TEXT("反转垂直视角")), FText::FromString(PlayerSettings->bInvertMouseY ? TEXT("开启") : TEXT("关闭")), false);
                    })
                      [SettingsText(TEXT("反转垂直视角"), 16)]]
              + SVerticalBox::Slot().AutoHeight()[Binding(TEXT("旋转视角"), TEXT("右键拖动"))]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 12)[Binding(TEXT("缩放 / 前后移动"), TEXT("滚轮"))]
              + SVerticalBox::Slot().AutoHeight()[Binding(TEXT("切换视角"), TEXT("C / 中键"))]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 22, 0, 0)
                  [SNew(STextBlock).Text(FText::FromString(TEXT("Tab 运行信息   ·   R 记录   ·   Esc 菜单 / 返回")))
                    .Font(FieldFont(12)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 0)
                  [SNew(STextBlock).Text(FText::FromString(TEXT("点击右侧动作后按新键。支持单个字母或数字；重复键不会覆盖。Esc 取消。")))
                    .Font(FieldFont(12)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)]]
          + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 10, 0)[SNew(SVerticalBox)
              + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 0, 10)[SettingsText(TEXT("驾驶按键"), 12, SettingsMuted)]
              + SVerticalBox::Slot().AutoHeight()[BuildDriveKeyBindingRow(EDriveKeyAction::Forward)]
              + SVerticalBox::Slot().AutoHeight()[BuildDriveKeyBindingRow(EDriveKeyAction::Backward)]
              + SVerticalBox::Slot().AutoHeight()[BuildDriveKeyBindingRow(EDriveKeyAction::Left)]
              + SVerticalBox::Slot().AutoHeight()[BuildDriveKeyBindingRow(EDriveKeyAction::Right)]
              + SVerticalBox::Slot().AutoHeight()[BuildDriveKeyBindingRow(EDriveKeyAction::TurnLeft)]
              + SVerticalBox::Slot().AutoHeight()[BuildDriveKeyBindingRow(EDriveKeyAction::TurnRight)]
              + SVerticalBox::Slot().AutoHeight().Padding(10, 16, 0, 0)[SettingsText(TEXT("键盘松键停止，无需 Shift。"), 12, SettingsMuted)]
              + SVerticalBox::Slot().AutoHeight().Padding(10, 18, 0, 0)
                  [SNew(STextBlock).Text(FText::FromString(TEXT("手柄：按住 LB，左摇杆移动，LT / RT 转向。右摇杆调整自由相机。")))
                    .Font(FieldFont(12)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)]]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildDriveKeyBindingRow(const EDriveKeyAction Action) {
  auto Row = SNew(SButton).ButtonStyle(SettingRowStyle()).ContentPadding(FMargin(10, 8)).HAlign(HAlign_Fill)
      [SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1)[SettingsText(FDriveKeyBindings::ActionLabel(Action), 16)]
        + SHorizontalBox::Slot().AutoWidth()
            [SNew(STextBlock).Font(FieldFont(16)).ColorAndOpacity(SettingsInk)
              .Text_Lambda([this, Action]() {
                return PendingDriveKey.IsSet() && PendingDriveKey.GetValue() == Action
                    ? FText::FromString(TEXT("按下新键…")) : PlayerSettings->DriveKeys.GetKey(Action).GetDisplayName();
              })]];
  const TWeakPtr<SWidget> WeakRow = Row;
  Row->SetOnClicked(FOnClicked::CreateLambda([this, Action, WeakRow]() {
    PendingDriveKey = Action;
    KeyBindingReturnFocus = WeakRow;
    KeyBindingErrorUntil = 0.0;
    MenuFeedback = FString::Printf(TEXT("为“%s”按一个新键 · Esc 取消"), FDriveKeyBindings::ActionLabel(Action));
    return FReply::Handled();
  }));
  return Row;
}

bool SLingTuSimRuntimeHUD::HandleKeyBindingCapture(const FKeyEvent& Event) {
  if (!PendingDriveKey.IsSet()) return false;
  if (MenuPage != EPlayerMenuPage::Settings || SettingsTab != ESettingsTab::Controls ||
      GetRuntimeMenuVisibility() != EVisibility::Visible) {
    CancelKeyBindingCapture();
    return false;
  }
  if (Event.IsRepeat()) return true;
  if (Event.GetKey() == EKeys::Escape || Event.GetKey() == EKeys::Gamepad_FaceButton_Right) {
    CancelKeyBindingCapture(true);
    return true;
  }
  if (Event.IsShiftDown() || Event.IsControlDown() || Event.IsAltDown() || Event.IsCommandDown()) {
    MenuFeedback = TEXT("请使用单个字母或数字，不支持组合键 · Esc 取消");
    KeyBindingErrorUntil = FSlateApplication::Get().GetCurrentTime() + 1.5;
    return true;
  }
  FString Error;
  const auto Action = PendingDriveKey.GetValue();
  if (PlayerSettings->DriveKeys.TryBind(Action, Event.GetKey(), Error)) {
    FinishKeyBindingCapture(true);
    MenuFeedback = FString::Printf(TEXT("%s → %s · 已生效，点击保存可保留"),
        FDriveKeyBindings::ActionLabel(Action), *Event.GetKey().GetDisplayName().ToString());
    EmitMenuCue(EMenuCue::Confirm);
  } else {
    MenuFeedback = Error + TEXT(" · 请换一个键，Esc 取消");
    KeyBindingErrorUntil = FSlateApplication::Get().GetCurrentTime() + 1.5;
  }
  return true;
}

void SLingTuSimRuntimeHUD::FinishKeyBindingCapture(const bool bRestoreFocus) {
  PendingDriveKey.Reset();
  KeyBindingErrorUntil = 0.0;
  const auto Target = KeyBindingReturnFocus.Pin();
  KeyBindingReturnFocus.Reset();
  if (bRestoreFocus && Target.IsValid() && FSlateApplication::IsInitialized())
    FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
}

void SLingTuSimRuntimeHUD::CancelKeyBindingCapture(const bool bRestoreFocus) {
  if (!PendingDriveKey.IsSet()) return;
  FinishKeyBindingCapture(bRestoreFocus);
  MenuFeedback = TEXT("已取消改键");
}

FText SLingTuSimRuntimeHUD::GetDriveKeysText(const bool bTurn) const {
  const auto& Keys = PlayerSettings->DriveKeys;
  const auto Name = [&Keys](EDriveKeyAction Action) { return Keys.GetKey(Action).GetDisplayName().ToString(); };
  return FText::FromString(bTurn
      ? Name(EDriveKeyAction::TurnLeft) + TEXT(" / ") + Name(EDriveKeyAction::TurnRight)
      : Name(EDriveKeyAction::Forward) + TEXT(" / ") + Name(EDriveKeyAction::Left) + TEXT(" / ") +
        Name(EDriveKeyAction::Backward) + TEXT(" / ") + Name(EDriveKeyAction::Right));
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildAudioSettingsPage() {
  const auto VolumeRow = [this](const TCHAR* Caption, FName RowTag, float FPlayerSettings::*Member) -> TSharedRef<SWidget> {
    return SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1)[SettingsText(Caption, 16)]
        + SHorizontalBox::Slot().AutoWidth()[SNew(STextBlock).Font(FieldFont(16)).ColorAndOpacity(SettingsInk)
          .Text_Lambda([this, Member]() { return FText::AsPercent((*PlayerSettings).*Member); })]]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 32)
        [SNew(SSlider).Tag(RowTag).Style(PlayerSliderStyle()).MinValue(0.F).MaxValue(1.F)
          .StepSize(.05F).MouseUsesStep(true).SliderHandleColor(SettingsAccent)
          .Value_Lambda([this, Member]() { return (*PlayerSettings).*Member; })
          .OnValueChanged_Lambda([this, Member, Caption](float Value) {
            float& Level = (*PlayerSettings).*Member;
            const float Before = Level;
            Level = Value;
            PlayerSettings->Normalize();
            ApplyPlayerAudioSettings(World.Get(), *PlayerSettings);
            if (!FMath::IsNearlyEqual(Before, Level))
              NotifySettingChanged(FText::FromString(Caption), FText::AsPercent(Level), false);
          })];
  };
  return SAssignNew(SettingsScrollBoxes[static_cast<int32>(ESettingsTab::Audio)], SScrollBox)
      .Tag(TEXT("LingTu.Settings.AudioScroll"))
    + SScrollBox::Slot().Padding(10, 0, 40, 0)[SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 22)[SettingsText(TEXT("播放音量"), 12, SettingsMuted)]
      + SVerticalBox::Slot().AutoHeight()[VolumeRow(TEXT("主音量"), TEXT("LingTu.Settings.MasterVolume"), &FPlayerSettings::MasterVolume)]
      + SVerticalBox::Slot().AutoHeight()[VolumeRow(TEXT("音乐音量"), TEXT("LingTu.Settings.MusicVolume"), &FPlayerSettings::MusicVolume)]
      + SVerticalBox::Slot().AutoHeight()[VolumeRow(TEXT("界面音量"), TEXT("LingTu.Settings.UIVolume"), &FPlayerSettings::UIVolume)]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 22)
        [SNew(SPlayerSettingChoice).Label(FText::FromString(TEXT("背景音乐")))
          .Value_Lambda([this]() { return FText::FromString(PlayerSettings->MenuMusicTrack == 0 ? TEXT("基地序曲") : TEXT("启程")); })
          .OnAdjust([this](int32 Direction) { PlayerSettings->MenuMusicTrack = StepOption(PlayerSettings->MenuMusicTrack, Direction, 2); })
          .OnChanged([this](FText Title, FText Value) { NotifySettingChanged(Title, Value, false); })]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 28)
        [SNew(SCheckBox).ForegroundColor(SettingsInk)
          .IsChecked_Lambda([this]() { return PlayerSettings->bMuteAudio ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
          .OnCheckStateChanged_Lambda([this](ECheckBoxState State) {
            PlayerSettings->bMuteAudio = State == ECheckBoxState::Checked;
            ApplyPlayerAudioSettings(World.Get(), *PlayerSettings);
            NotifySettingChanged(FText::FromString(TEXT("静音")), FText::FromString(PlayerSettings->bMuteAudio ? TEXT("开启") : TEXT("关闭")), false);
          })[SettingsText(TEXT("静音"), 16)]]
      + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)
        [SettingsButton(TEXT("试听界面音效"), FOnClicked::CreateLambda([this]() {
          EmitMenuCue(EMenuCue::Confirm);
          return FReply::Handled();
        }), false, TAttribute<bool>::CreateLambda([this]() {
          return GEngine && GEngine->UseSound() && World.IsValid() && World->bAllowAudioPlayback &&
              PlayerSettings->GetMasterAudioGain() > 0.F && PlayerSettings->GetUIAudioGain() > 0.F;
        }))]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 0)
        [SNew(STextBlock).Font(FieldFont(12)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)
          .Text_Lambda([this]() { return FText::FromString(!GEngine || !GEngine->UseSound()
              ? TEXT("当前启动配置已关闭音频，音量设置仍可保存。")
              : PlayerSettings->bMuteAudio ? TEXT("当前已静音，音量数值会保留。")
              : PlayerSettings->MasterVolume <= 0.F || PlayerSettings->UIVolume <= 0.F ? TEXT("音量为 0，调高后可试听。")
              : TEXT("背景音乐在首页和选择页循环，进入场景后淡出。界面音效独立调节。")); })]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildInterfaceSettingsPage() {
  return SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(10, 0, 0, 20)[SettingsText(TEXT("驾驶界面"), 12, SettingsMuted)]
      + SVerticalBox::Slot().AutoHeight().Padding(10, 0)
          [SNew(SCheckBox).IsChecked_Lambda([this]() { return PlayerSettings->bShowControlHints ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
            .OnCheckStateChanged_Lambda([this](ECheckBoxState State) {
              PlayerSettings->bShowControlHints = State == ECheckBoxState::Checked;
              NotifySettingChanged(FText::FromString(TEXT("显示操作提示")), FText::FromString(PlayerSettings->bShowControlHints ? TEXT("开启") : TEXT("关闭")), false);
            })[SettingsText(TEXT("显示操作提示"), 16)]]
      + SVerticalBox::Slot().AutoHeight().Padding(10, 20, 10, 0)
          [SNew(SCheckBox).IsChecked_Lambda([this]() { return PlayerSettings->bShowFrameRate ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
            .OnCheckStateChanged_Lambda([this](ECheckBoxState State) {
              PlayerSettings->bShowFrameRate = State == ECheckBoxState::Checked;
              NotifySettingChanged(FText::FromString(TEXT("显示帧率与帧时间")), FText::FromString(PlayerSettings->bShowFrameRate ? TEXT("开启") : TEXT("关闭")), false);
            })[SettingsText(TEXT("显示帧率与帧时间"), 16)]]
      + SVerticalBox::Slot().AutoHeight().Padding(10, 24, 10, 0)
          [SNew(STextBlock).Text(FText::FromString(TEXT("关闭底部键位提示，让驾驶画面更干净。安全与故障提示仍保留。")))
            .Font(FieldFont(13)).ColorAndOpacity(SettingsMuted).AutoWrapText(true)];
}

bool SLingTuSimRuntimeHUD::HasSettingsChanges() const {
  const auto& Saved = PlayerSettingsCheckpoint;
  const auto& Current = *PlayerSettings;
  return GraphicsPreview.IsPending() || HasGraphicsDraftChanges() ||
      (bPlayerSettingsEditActive &&
       (Current.MouseSensitivity != Saved.MouseSensitivity || Current.bInvertMouseY != Saved.bInvertMouseY ||
        Current.bShowControlHints != Saved.bShowControlHints || Current.bShowFrameRate != Saved.bShowFrameRate ||
        Current.MasterVolume != Saved.MasterVolume || Current.UIVolume != Saved.UIVolume || Current.bMuteAudio != Saved.bMuteAudio ||
        Current.MusicVolume != Saved.MusicVolume || Current.MenuMusicTrack != Saved.MenuMusicTrack ||
        !(Current.DriveKeys == Saved.DriveKeys)));
}

FReply SLingTuSimRuntimeHUD::RequestSettingsBack() {
  CancelKeyBindingCapture();
  if (!HasSettingsChanges()) {
    ShowMenuPage(EPlayerMenuPage::Root);
    return FReply::Handled();
  }
  if (FSlateApplication::IsInitialized()) SettingsExitReturnFocus = FSlateApplication::Get().GetKeyboardFocusedWidget();
  bSettingsExitDecision = true;
  Invalidate(EInvalidateWidgetReason::Layout);
  SlatePrepass(1.F);
  if (SettingsContinueButton.IsValid() && FSlateApplication::IsInitialized())
    FSlateApplication::Get().SetKeyboardFocus(SettingsContinueButton, EFocusCause::SetDirectly);
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::ContinueSettingsEdit() {
  bSettingsExitDecision = false;
  const auto Target = SettingsExitReturnFocus.Pin();
  SettingsExitReturnFocus.Reset();
  Invalidate(EInvalidateWidgetReason::Layout);
  SlatePrepass(1.F);
  if (FSlateApplication::IsInitialized())
    FSlateApplication::Get().SetKeyboardFocus(Target.IsValid() ? Target : SettingsBackButton, EFocusCause::SetDirectly);
  return FReply::Handled();
}

void SLingTuSimRuntimeHUD::SelectSettingsTab(ESettingsTab Tab) {
  if (SettingsTab != Tab) {
    MenuMotion.BeginPage(FSlateApplication::Get().GetCurrentTime());
    EmitMenuCue(EMenuCue::Switch);
  }
  CancelKeyBindingCapture();
  SettingsTab = Tab;
  SettingsHelpTitle = FText::GetEmpty();
  SettingsHelpBody = FText::GetEmpty();
  if (SettingsScrollBoxes[static_cast<int32>(Tab)].IsValid())
    SettingsScrollBoxes[static_cast<int32>(Tab)]->ScrollToStart();
  MenuFeedback.Reset();
  Invalidate(EInvalidateWidgetReason::Layout);
  SlatePrepass(1.F);
  const auto Focus = Tab == ESettingsTab::Graphics ? GraphicsTabButton : Tab == ESettingsTab::Controls ? ControlsTabButton
      : Tab == ESettingsTab::Audio ? AudioTabButton : InterfaceTabButton;
  if (Focus.IsValid() && FSlateApplication::IsInitialized()) FSlateApplication::Get().SetKeyboardFocus(Focus, EFocusCause::SetDirectly);
}

void SLingTuSimRuntimeHUD::BeginPlayerSettingsEdit() {
  PlayerSettingsCheckpoint = *PlayerSettings;
  bPlayerSettingsEditActive = true;
}

void SLingTuSimRuntimeHUD::RestoreUnsavedPlayerSettings(const bool bKeepEditing) {
  if (!bPlayerSettingsEditActive || !PlayerSettings.IsValid()) return;
  *PlayerSettings = PlayerSettingsCheckpoint;
  ApplyPlayerAudioSettings(World.Get(), *PlayerSettings);
  bPlayerSettingsEditActive = bKeepEditing;
}

void SLingTuSimRuntimeHUD::AdvancePlayerSettingsCheckpoint() {
  if (!bPlayerSettingsEditActive || !PlayerSettings.IsValid()) return;
  PlayerSettingsCheckpoint = *PlayerSettings;
}

FReply SLingTuSimRuntimeHUD::OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Event) {
  if (MenuPage == EPlayerMenuPage::Settings && ModeController->GetMode() == ERuntimeUIMode::Pause &&
      (Event.GetKey() == EKeys::Gamepad_LeftShoulder || Event.GetKey() == EKeys::Gamepad_RightShoulder)) {
    if (!bSettingsExitDecision) SelectSettingsTab(static_cast<ESettingsTab>(StepOption(static_cast<int32>(SettingsTab), Event.GetKey() == EKeys::Gamepad_RightShoulder ? 1 : -1, static_cast<int32>(ESettingsTab::Count))));
    return FReply::Handled();
  }
  return SCompoundWidget::OnKeyDown(Geometry, Event);
}

FText SLingTuSimRuntimeHUD::GetPlayerFrameRateText() const {
  return FText::FromString(SmoothedFrameTimeSeconds > 0.F
      ? FString::Printf(TEXT("%.0f FPS  /  %.1f ms"), 1.F / SmoothedFrameTimeSeconds, SmoothedFrameTimeSeconds * 1000.F)
      : TEXT("— FPS"));
}

bool SLingTuSimRuntimeHUD::CanEditGraphicsSettings() const {
  return World.IsValid() && !GraphicsPreview.IsPending() && !FPlayerGraphicsPreview::IsSessionLocked();
}

bool SLingTuSimRuntimeHUD::HasGraphicsDraftChanges() const {
  return !GraphicsDraft.IsEquivalentTo(GraphicsSettingsCheckpoint);
}

bool SLingTuSimRuntimeHUD::CanApplyGraphicsSettings() const {
  return CanEditGraphicsSettings() && HasGraphicsDraftChanges();
}

FReply SLingTuSimRuntimeHUD::ApplyGraphicsSettings() {
  if (!HasGraphicsDraftChanges()) {
    MenuFeedback = TEXT("画面设置未变化");
    return FReply::Handled();
  }
  FString Error;
  if (GraphicsPreview.Apply(World.Get(), GraphicsDraft, FPlatformTime::Seconds(), Error)) MenuFeedback = TEXT("画面已应用，请选择保留或还原");
  else MenuFeedback = FString::Printf(TEXT("画面未应用：%s"), *Error);
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::ConfirmGraphicsSettings() {
  FString Error;
  if (GraphicsPreview.Confirm(Error)) {
    GraphicsDraft.Normalize();
    GraphicsSettingsCheckpoint = GraphicsDraft;
    MenuFeedback = TEXT("画面设置已确认并保存");
    EmitMenuCue(EMenuCue::Confirm);
  } else {
    MenuFeedback = FString::Printf(TEXT("未能保存：%s"), *Error);
  }
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::RevertGraphicsSettings() {
  GraphicsPreview.Revert();
  GraphicsDraft = FPlayerGraphicsSettings::Capture(World.Get());
  GraphicsSettingsCheckpoint = GraphicsDraft;
  MenuFeedback = TEXT("已还原之前的画面设置");
  return FReply::Handled();
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildMenuConfirmation() {
  ConfirmationCancelButton = MenuButton(Label(TEXT("取消"), 18), FOnClicked::CreateLambda([this]() {
    ShowMenuPage(EPlayerMenuPage::Root); return FReply::Handled();
  }));
  return SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 18, 26)
          [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetMenuConfirmationTitle).Font(FieldFont(28)).ColorAndOpacity(FLinearColor::White)]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 0, 18, 26)
          [SNew(STextBlock).Text(this, &SLingTuSimRuntimeHUD::GetMenuConfirmationBody).Font(FieldFont(15))
            .ColorAndOpacity(FLinearColor(.7F, .7F, .7F, 1)).AutoWrapText(true)]
      + SVerticalBox::Slot().AutoHeight()[ConfirmationCancelButton.ToSharedRef()]
      + SVerticalBox::Slot().AutoHeight()[MenuButton(Label(TEXT("确认"), 18),
          FOnClicked::CreateSP(this, &SLingTuSimRuntimeHUD::ConfirmMenuAction),
          TAttribute<bool>::CreateSP(this, &SLingTuSimRuntimeHUD::CanConfirmMenuAction))]
      + SVerticalBox::Slot().AutoHeight().Padding(18, 24, 18, 0)
          [SNew(STextBlock).Text_Lambda([this]() {
            if (LocalState->bExitRequestPending) return FText::FromString(TEXT("正在结束会话，等待运行端确认…"));
            return FText::FromString(LocalState->ExitRequestError.IsEmpty() ? MenuFeedback
                : FString::Printf(TEXT("退出未完成：%s。可取消或重试。"), *LocalState->ExitRequestError));
          }).Font(FieldFont(12)).ColorAndOpacity(FLinearColor(.7F, .7F, .7F, 1)).AutoWrapText(true)];
}

void SLingTuSimRuntimeHUD::ShowMenuPage(EPlayerMenuPage Page) {
  if (MenuPage != Page) {
    MenuMotion.BeginPage(FSlateApplication::Get().GetCurrentTime());
    EmitMenuCue(EMenuCue::Switch);
  }
  CancelKeyBindingCapture();
  bSettingsExitDecision = false;
  SettingsExitReturnFocus.Reset();
  const auto Previous = MenuPage;
  if (Previous == EPlayerMenuPage::Settings && Page != EPlayerMenuPage::Settings) {
    GraphicsPreview.Revert();
    RestoreUnsavedPlayerSettings();
  }
  if (Page == EPlayerMenuPage::Settings && Previous != EPlayerMenuPage::Settings) {
    SettingsTab = ESettingsTab::Graphics;
    GraphicsDraft = FPlayerGraphicsSettings::Capture(World.Get());
    GraphicsDraft.Load();
    GraphicsSettingsCheckpoint = GraphicsDraft;
    BeginPlayerSettingsEdit();
  }
  MenuPage = Page;
  // Reuse non-driving Tactical mode: viewing sensors must not pause their source.
  // Returning to the regular menu restores Pause; it does not acquire driving.
  if (!bFrontEndLoginRequired && Page == EPlayerMenuPage::Measurements) {
    ModeController->SetLiveMenuView(true);
  } else if (Previous == EPlayerMenuPage::Measurements && ModeController->GetMode() == ERuntimeUIMode::Tactical) {
    ModeController->SetLiveMenuView(false);
  }
  if (Measurements.IsValid()) Measurements->SetVisible(
      Page == EPlayerMenuPage::Measurements && GetRuntimeMenuVisibility() == EVisibility::Visible);
  bRuntimeMenuWasVisible = GetRuntimeMenuVisibility() == EVisibility::Visible;
  MenuFeedback.Reset();
  Invalidate(EInvalidateWidgetReason::Layout);
  SlatePrepass(1.F);
  if (!FSlateApplication::IsInitialized()) return;
  if (Page == EPlayerMenuPage::Library && RuntimeRobotMapPicker.IsValid()) {
    RuntimeRobotMapPicker->FocusInitialControl(); return;
  }
  if (Page == EPlayerMenuPage::Measurements && MeasurementPanel.IsValid()) {
    MeasurementPanel->FocusInitialControl(); return;
  }
  if (bFrontEndLoginRequired && Page == EPlayerMenuPage::Root) {
    const auto Target = Previous == EPlayerMenuPage::Settings ? WelcomeSettingsButton : WelcomeQuitButton;
    if (Target.IsValid()) FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
    return;
  }
  const auto Target = Page == EPlayerMenuPage::Settings ? GraphicsTabButton
      : Page == EPlayerMenuPage::Journal ? JournalBackButton
      : Page == EPlayerMenuPage::Root ? (Previous == EPlayerMenuPage::Settings ? SettingsMenuButton
          : Previous == EPlayerMenuPage::Journal ? JournalMenuButton
          : Previous == EPlayerMenuPage::Measurements ? MeasurementsMenuButton
          : Previous == EPlayerMenuPage::Library ? LibraryMenuButton : ResumeMenuButton)
      : ConfirmationCancelButton;
  RuntimeMenuFocusTarget = Target;
  if (Target.IsValid()) FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
}

void SLingTuSimRuntimeHUD::NotifySettingChanged(const FText& Name, const FText& Value, bool bNeedsApply) {
  // An active capture/error owns this line; never conceal validation or capture feedback.
  if (PendingDriveKey.IsSet()) return;
  MenuFeedback.Reset();
  KeyBindingErrorUntil = 0.;
  MenuMotion.Changed(Name, Value, bNeedsApply, FSlateApplication::Get().GetCurrentTime());
  EmitMenuCue(EMenuCue::Change);
}

void SLingTuSimRuntimeHUD::EmitMenuCue(EMenuCue Cue) {
  const double Now = FSlateApplication::Get().GetCurrentTime();
  if (LastMenuCueAt >= 0. && Now - LastMenuCueAt < .05) return;
  LastMenuCueAt = Now;
  if (PlayerSettings->GetMasterAudioGain() <= 0.F) return;
  PlayMenuCue(World.Get(), Cue, PlayerSettings->GetUIAudioGain());
}

bool SLingTuSimRuntimeHUD::HandleRuntimeMenuBack() {
  if (GetWorkspaceVisibility() == EVisibility::Visible) {
    if (bAssetReviewWorkspace) {
      bAssetReviewWorkspace = false;
      Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
      SlatePrepass(1.F);
      if (WorkspaceRobotMapPicker.IsValid()) WorkspaceRobotMapPicker->FocusInitialControl();
      return true;
    }
    return WorkspaceRobotMapPicker.IsValid() && WorkspaceRobotMapPicker->GoBack();
  }
  if (PendingDriveKey.IsSet()) { CancelKeyBindingCapture(true); return true; }
  if (GetRuntimeMenuVisibility() != EVisibility::Visible || MenuPage == EPlayerMenuPage::Root) return false;
  if (MenuPage == EPlayerMenuPage::Library && RuntimeRobotMapPicker.IsValid() && RuntimeRobotMapPicker->GoBack()) return true;
  if (MenuPage == EPlayerMenuPage::Settings) {
    if (bSettingsExitDecision) ContinueSettingsEdit();
    else RequestSettingsBack();
    return true;
  }
  ShowMenuPage(EPlayerMenuPage::Root);
  return true;
}

void SLingTuSimRuntimeHUD::AdvanceFrontEndSelection() {
  if (GetWorkspaceVisibility() == EVisibility::Visible && !bAssetReviewWorkspace && WorkspaceRobotMapPicker.IsValid())
    WorkspaceRobotMapPicker->PrimaryAction();
}

void SLingTuSimRuntimeHUD::RestoreSelectionFocus() {
  if (IsMeasurementPageOpen() && MeasurementPanel.IsValid()) { MeasurementPanel->FocusInitialControl(); return; }
  if (bSettingsExitDecision) { FocusRuntimeMenuCTA(); return; }
  if (GetRuntimeMenuVisibility() == EVisibility::Visible && bFrontEndLoginRequired) {
    const auto Target = MenuPage == EPlayerMenuPage::Settings ? GraphicsTabButton : ConfirmationCancelButton;
    if (Target.IsValid()) FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
    return;
  }
  if (GetRuntimeMenuVisibility() == EVisibility::Visible && !bFrontEndLoginRequired) {
    if (auto Target = RuntimeMenuFocusTarget.Pin())
      FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
    else FocusRuntimeMenuCTA();
    return;
  }
  if (GetWorkspaceVisibility() == EVisibility::Visible && !bAssetReviewWorkspace && WorkspaceRobotMapPicker.IsValid())
    WorkspaceRobotMapPicker->FocusInitialControl();
  else if (GetLoginVisibility() == EVisibility::Visible) FocusFrontEndLoginCTA();
}

void SLingTuSimRuntimeHUD::OpenQuitConfirmation() {
  if (GetRuntimeMenuVisibility() == EVisibility::Visible) ShowMenuPage(EPlayerMenuPage::Quit);
}

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
void SLingTuSimRuntimeHUD::PreviewRobotCategory(const FString& Category) {
  if (WorkspaceRobotMapPicker.IsValid()) WorkspaceRobotMapPicker->SetRobotCategory(Category);
}
bool SLingTuSimRuntimeHUD::IsFrontEndShowroomReady() const {
  if (GetLoginVisibility() == EVisibility::Visible) return FactoryBackgroundBrush.IsValid() || (WelcomeShowroom.IsValid() && WelcomeShowroom->IsReady());
  return WorkspaceRobotMapPicker.IsValid() && WorkspaceRobotMapPicker->IsShowroomReady();
}
void SLingTuSimRuntimeHUD::PreviewShowroomInspection() {
  if (WorkspaceRobotMapPicker.IsValid()) WorkspaceRobotMapPicker->PreviewShowroomInspection();
}

void SLingTuSimRuntimeHUD::PreviewPlayerMenuPage(const FString& Page) {
  ShowMenuPage(Page == TEXT("settings") || Page == TEXT("controls") || Page == TEXT("interface") || Page == TEXT("audio") || Page == TEXT("settings_exit") ? EPlayerMenuPage::Settings
      : Page == TEXT("journal") ? EPlayerMenuPage::Journal
      : Page == TEXT("measurements") ? EPlayerMenuPage::Measurements
      : Page == TEXT("library") ? EPlayerMenuPage::Library
      : Page == TEXT("quit") ? EPlayerMenuPage::Quit : EPlayerMenuPage::Root);
  if (Page == TEXT("controls")) SelectSettingsTab(ESettingsTab::Controls);
  if (Page == TEXT("interface")) SelectSettingsTab(ESettingsTab::Interface);
  if (Page == TEXT("audio")) {
    SelectSettingsTab(ESettingsTab::Audio);
    UE_LOG(LogTemp, Display, TEXT("LINGTU_MENU_MUSIC_STATUS playing=%d track=%d"), FrontEndMusic.IsPlaying(), PlayerSettings->MenuMusicTrack);
  }
  if (Page == TEXT("settings_exit")) {
    SelectSettingsTab(ESettingsTab::Controls);
    PlayerSettings->bInvertMouseY = !PlayerSettings->bInvertMouseY;
    RequestSettingsBack();
  }
}

void SLingTuSimRuntimeHUD::ProbeGraphicsPreview(const bool bVerifyRollback) {
  if (!bVerifyRollback) {
    if (ModeController->GetMode() != ERuntimeUIMode::Pause) ModeController->TogglePause();
    ShowMenuPage(EPlayerMenuPage::Settings);
    GraphicsProbeBefore = FPlayerGraphicsSettings::Capture(World.Get());
    auto* ScreenPercentage = IConsoleManager::Get().FindConsoleVariable(TEXT("r.ScreenPercentage"));
    GraphicsProbeBeforeScreenPercentage = ScreenPercentage ? ScreenPercentage->GetFloat() : 0.F;
    auto Candidate = GraphicsProbeBefore;
    Candidate.FrameRateLimit = FMath::IsNearlyEqual(Candidate.FrameRateLimit, 30.F) ? 60.F : 30.F;
    Candidate.ShadowQuality = Candidate.ShadowQuality == 1 ? 2 : 1;
    Candidate.ResolutionScale = 75.F;
    GraphicsSettingsCheckpoint = GraphicsProbeBefore;
    GraphicsDraft = Candidate;
    const bool bCanApply = CanApplyGraphicsSettings();
    ApplyGraphicsSettings();
    const FString ApplyFeedback = MenuFeedback;
    const bool bApplied = GraphicsPreview.IsPending();
    const auto Actual = FPlayerGraphicsSettings::Capture(World.Get());
    SelectSettingsTab(ESettingsTab::Controls);
    const bool bPendingAcrossTabs = GraphicsPreview.IsPending() && HasGraphicsDraftChanges();
    SelectSettingsTab(ESettingsTab::Graphics);
    bGraphicsProbeStarted = MenuPage == EPlayerMenuPage::Settings && bCanApply && bApplied && bPendingAcrossTabs
        && FMath::IsNearlyEqual(Actual.FrameRateLimit, Candidate.FrameRateLimit)
        && Actual.ShadowQuality == Candidate.ShadowQuality
        && FMath::IsNearlyEqual(Actual.ResolutionScale, Candidate.ResolutionScale)
        && ScreenPercentage && FMath::IsNearlyEqual(ScreenPercentage->GetFloat(), Candidate.ResolutionScale);
    UE_LOG(LogTemp, Display, TEXT("LINGTU_SETTINGS_GRAPHICS_PROBE_START passed=%s pending=%s cap=%.0f shadow=%d scale=%.0f screen_percentage=%.0f feedback=%s"),
        bGraphicsProbeStarted ? TEXT("true") : TEXT("false"), GraphicsPreview.IsPending() ? TEXT("true") : TEXT("false"),
        Actual.FrameRateLimit, Actual.ShadowQuality, Actual.ResolutionScale, ScreenPercentage ? ScreenPercentage->GetFloat() : -1.F, *ApplyFeedback);
  } else {
    const auto Actual = FPlayerGraphicsSettings::Capture(World.Get());
    auto* ScreenPercentage = IConsoleManager::Get().FindConsoleVariable(TEXT("r.ScreenPercentage"));
    const bool bRestored = bGraphicsProbeStarted && !GraphicsPreview.IsPending()
        && !HasGraphicsDraftChanges()
        && Actual.Resolution == GraphicsProbeBefore.Resolution && Actual.WindowMode == GraphicsProbeBefore.WindowMode
        && FMath::IsNearlyEqual(Actual.FrameRateLimit, GraphicsProbeBefore.FrameRateLimit)
        && Actual.ShadowQuality == GraphicsProbeBefore.ShadowQuality
        && FMath::IsNearlyEqual(Actual.ResolutionScale, GraphicsProbeBefore.ResolutionScale)
        && ScreenPercentage && FMath::IsNearlyEqual(ScreenPercentage->GetFloat(), GraphicsProbeBeforeScreenPercentage);
    UE_LOG(LogTemp, Display, TEXT("LINGTU_SETTINGS_GRAPHICS_PROBE_REVERT passed=%s pending=%s cap=%.0f shadow=%d scale=%.0f screen_percentage=%.0f resolution=%dx%d expected=%dx%d mode=%d expected_mode=%d dirty=%s expected_cap=%.0f expected_shadow=%d expected_scale=%.0f expected_screen_percentage=%.0f"),
        bRestored ? TEXT("true") : TEXT("false"), GraphicsPreview.IsPending() ? TEXT("true") : TEXT("false"),
        Actual.FrameRateLimit, Actual.ShadowQuality, Actual.ResolutionScale, ScreenPercentage ? ScreenPercentage->GetFloat() : -1.F,
        Actual.Resolution.X, Actual.Resolution.Y, GraphicsProbeBefore.Resolution.X, GraphicsProbeBefore.Resolution.Y,
        Actual.WindowMode, GraphicsProbeBefore.WindowMode, HasGraphicsDraftChanges() ? TEXT("true") : TEXT("false"),
        GraphicsProbeBefore.FrameRateLimit, GraphicsProbeBefore.ShadowQuality, GraphicsProbeBefore.ResolutionScale,
        GraphicsProbeBeforeScreenPercentage);
  }
}
#endif

FText SLingTuSimRuntimeHUD::GetMenuConfirmationTitle() const {
  return FText::FromString(MenuPage == EPlayerMenuPage::Restart ? TEXT("重新开始？")
      : MenuPage == EPlayerMenuPage::MainMenu ? TEXT("返回主菜单？") : TEXT("退出游戏？"));
}

FText SLingTuSimRuntimeHUD::GetMenuConfirmationBody() const {
  if (bFrontEndLoginRequired && MenuPage == EPlayerMenuPage::Quit)
    return FText::FromString(TEXT("退出灵途？当前尚未启动仿真。"));
  if (MenuPage == EPlayerMenuPage::Restart && !OnRequestRestart.IsBound())
    return FText::FromString(TEXT("当前运行端尚未接通场景、机器人与任务的完整重启。此操作暂不可用。"));
  if (MenuPage == EPlayerMenuPage::MainMenu && !OnRequestMainMenu.IsBound())
    return FText::FromString(TEXT("当前启动器尚未接通结束会话后返回场景选择。此操作暂不可用，不会以退出程序代替。"));
  if (!CachedStatus.bControlBindingAvailable || !CachedStatus.bIdentityCoherent)
    return FText::FromString(TEXT("尚未连接运行端。此预览不能结束或重启真实仿真会话。"));
  if (CachedStatus.CanRequestRecordStopCommit())
    return FText::FromString(TEXT("正在记录。请先按 R 停止并保存记录，再执行此操作。"));
  return FText::FromString(MenuPage == EPlayerMenuPage::Restart ? TEXT("机器人、场景和任务将恢复初始状态。等待运行端确认后完成。")
      : MenuPage == EPlayerMenuPage::MainMenu ? TEXT("结束当前会话，返回机器人、场景与任务选择。")
      : TEXT("结束当前仿真并退出。运行端确认安全停止后关闭窗口。"));
}

bool SLingTuSimRuntimeHUD::CanConfirmMenuAction() const {
  if (bFrontEndLoginRequired)
    return MenuPage == EPlayerMenuPage::Quit && !CachedStatus.bSessionAvailable
        && !SelectionModel->IsConfirmed() && OnRequestExit.IsBound();
  if (!CachedStatus.bControlBindingAvailable || !CachedStatus.bIdentityCoherent ||
      !HasCurrentStatus(CachedStatus) || CachedStatus.CanRequestRecordStopCommit() || LocalState->bExitRequestPending) return false;
  if (MenuPage == EPlayerMenuPage::Restart) return OnRequestRestart.IsBound();
  if (MenuPage == EPlayerMenuPage::MainMenu) return OnRequestMainMenu.IsBound();
  return MenuPage == EPlayerMenuPage::Quit && OnRequestExit.IsBound();
}

FReply SLingTuSimRuntimeHUD::ConfirmMenuAction() {
  if (!CanConfirmMenuAction()) return FReply::Handled();
  if (MenuPage == EPlayerMenuPage::Restart) OnRequestRestart.ExecuteIfBound();
  else if (MenuPage == EPlayerMenuPage::MainMenu) OnRequestMainMenu.ExecuteIfBound();
  else OnRequestExit.ExecuteIfBound();
  return FReply::Handled();
}

FText SLingTuSimRuntimeHUD::GetFieldSpeedText() const {
  return FormatFieldSpeed(CachedStatus);
}

FText SLingTuSimRuntimeHUD::GetDriveSpeedValueText() const { return FormatDriveSpeedValue(CachedStatus); }

FText SLingTuSimRuntimeHUD::FormatDriveSpeedValue(const FRuntimeUIStatusSnapshot& Status) {
  if (!Status.bObservedBaseVelocityAvailable || !Status.bIdentityCoherent ||
      (Status.bFullStatusAvailable && (!Status.bFullStatusFresh || !Status.bFullStatusIdentityCoherent)))
    return FText::FromString(TEXT("—"));
  return FText::FromString(FString::Printf(TEXT("%.1f"), Status.ObservedBaseLinearVelocityMps.Size2D()));
}

FText SLingTuSimRuntimeHUD::GetDriveNoticeText() const { return FormatDriveNotice(CachedStatus); }

FText SLingTuSimRuntimeHUD::FormatDriveNotice(const FRuntimeUIStatusSnapshot& Status) {
  return !Status.bSessionAvailable ? FText::FromString(TEXT("离线预览")) : FormatDriveLink(Status);
}

FSlateColor SLingTuSimRuntimeHUD::GetDriveNoticeColor() const {
  if (!CachedStatus.bSessionAvailable) return FLinearColor(.75F, .75F, .75F, 1);
  return HasHealthyDriveLink(CachedStatus) ? FLinearColor::White : FLinearColor(1.F, .72F, .3F, 1);
}

FText SLingTuSimRuntimeHUD::FormatFieldSpeed(const FRuntimeUIStatusSnapshot& Status) {
  return FText::FromString(Status.bObservedBaseVelocityAvailable && Status.bIdentityCoherent
      ? FString::Printf(TEXT("%.2f m/s"), Status.ObservedBaseLinearVelocityMps.Size2D())
      : TEXT("— m/s"));
}

FText SLingTuSimRuntimeHUD::GetFieldTurnRateText() const { return FormatFieldTurnRate(CachedStatus); }
FText SLingTuSimRuntimeHUD::FormatFieldTurnRate(const FRuntimeUIStatusSnapshot& Status) {
  if (!Status.bObservedBaseVelocityAvailable || !Status.bIdentityCoherent)
    return FText::FromString(TEXT("— °/s"));
  const double Degrees = FMath::RadiansToDegrees(Status.ObservedBaseAngularVelocityRadps.Z);
  return FText::FromString(FMath::Abs(Degrees) < .05 ? TEXT("0.0 °/s")
      : FString::Printf(TEXT("%+.1f °/s"), Degrees));
}

bool SLingTuSimRuntimeHUD::HasHealthyDriveLink(const FRuntimeUIStatusSnapshot& Status) {
  if (!Status.bSessionAvailable || !Status.bControlBindingAvailable || !Status.bIdentityCoherent ||
      !Status.bFullStatusAvailable || !Status.bFullStatusFresh || !Status.bFullStatusIdentityCoherent ||
      !Status.bLatestAppliedTruthAvailable || Status.FullStatus.Runtime.RuntimeState != LingTuSim::EControlStatusRuntimeState::Running ||
      Status.FullStatus.Runtime.SafeStopState == LingTuSim::EControlSafeStopState::Blocked ||
      Status.FullStatus.Runtime.SafeStopState == LingTuSim::EControlSafeStopState::Pending) return false;
  const auto& Ready = Status.FullStatus.Readiness;
  const LingTuSim::FControlStatusReadinessFacet* Facets[] = {&Ready.Physics, &Ready.Control, &Ready.Visual, &Ready.Sensors};
  for (const auto* Facet : Facets)
    if (!Facet->bRequired || Facet->State != LingTuSim::EControlBindingState::Active ||
        !Facet->Blocker.IsEmpty()) return false;
  for (const auto& Sensor : Status.FullStatus.Sensors)
    if (Sensor.State != LingTuSim::EControlSensorState::Active || !Sensor.Blocker.IsEmpty()) return false;
  return !Status.FullStatus.Sensors.IsEmpty();
}

FText SLingTuSimRuntimeHUD::GetDriveLinkText() const { return FormatDriveLink(CachedStatus); }
FText SLingTuSimRuntimeHUD::FormatDriveLink(const FRuntimeUIStatusSnapshot& Status) {
  if (!Status.bSessionAvailable) return FText::FromString(TEXT("等待仿真"));
  if (!Status.bControlBindingAvailable) return FText::FromString(TEXT("等待控制"));
  if (!Status.bIdentityCoherent || (Status.bFullStatusAvailable && !Status.bFullStatusIdentityCoherent))
    return FText::FromString(TEXT("身份不一致"));
  if (!Status.bFullStatusAvailable) return FText::FromString(TEXT("等待运行状态"));
  if (!Status.bFullStatusFresh) return FText::FromString(TEXT("运行状态过期"));
  const auto& Runtime = Status.FullStatus.Runtime;
  if (Runtime.RuntimeState == LingTuSim::EControlStatusRuntimeState::Failed) return FText::FromString(TEXT("仿真异常"));
  if (Runtime.RuntimeState == LingTuSim::EControlStatusRuntimeState::Stopped) return FText::FromString(TEXT("会话已结束"));
  if (Runtime.RuntimeState == LingTuSim::EControlStatusRuntimeState::Paused) return FText::FromString(TEXT("仿真已暂停"));
  if (Runtime.SafeStopState == LingTuSim::EControlSafeStopState::Blocked) return FText::FromString(TEXT("控制已阻断"));
  if (Runtime.SafeStopState == LingTuSim::EControlSafeStopState::Pending) return FText::FromString(TEXT("控制释放中"));
  if (!Status.bLatestAppliedTruthAvailable) return FText::FromString(TEXT("等待真值"));
  return FText::FromString(HasHealthyDriveLink(Status) ? TEXT("联动运行") : TEXT("等待运行组件"));
}

FText SLingTuSimRuntimeHUD::GetDriveCameraText() const {
  const FString Actual = CachedStatus.ActualCameraMode.ToLower();
  const TCHAR* Name = Actual == TEXT("follow") ? TEXT("跟随") : Actual == TEXT("inspection") ? TEXT("巡检") : TEXT("自由");
  if (Actual != TEXT("follow") && Actual != TEXT("inspection") && Actual != TEXT("free"))
    return FText::FromString(TEXT("视角未绑定"));
  const TCHAR* Note = !CachedStatus.bFullStatusAvailable ? TEXT(" · 待回显")
      : !CachedStatus.bFullStatusFresh ? TEXT(" · 回显过期")
      : !CachedStatus.bFullStatusIdentityCoherent ? TEXT(" · 身份不一致") : TEXT("");
  if (!*Note) {
    const auto Echo = CachedStatus.FullStatus.UI.CameraMode;
    const bool Matches = (Actual == TEXT("follow") && Echo == LingTuSim::EControlStatusCameraMode::Follow) ||
        (Actual == TEXT("inspection") && Echo == LingTuSim::EControlStatusCameraMode::Inspection) ||
        (Actual == TEXT("free") && Echo == LingTuSim::EControlStatusCameraMode::Free);
    if (!Matches) Note = TEXT(" · 回显待确认");
  }
  return FText::FromString(FString::Printf(TEXT("%s视角%s"), Name, Note));
}

FText SLingTuSimRuntimeHUD::GetDriveRecordingText() const {
  if (!HasCurrentStatus(CachedStatus)) return FText::FromString(TEXT("记录状态待确认 · Tab 详情"));
  switch (CachedStatus.FullStatus.Recording.State) {
    case LingTuSim::EControlRecordingState::Recording: return FText::FromString(TEXT("正在录制 · R 停止"));
    case LingTuSim::EControlRecordingState::Idle: return FText::FromString(TEXT("未录制 · R 记录"));
    case LingTuSim::EControlRecordingState::Committed: return FText::FromString(TEXT("记录已保存 · Tab 详情"));
    case LingTuSim::EControlRecordingState::Failed: return FText::FromString(TEXT("记录失败 · Tab 详情"));
    default: return FText::FromString(TEXT("记录状态 · Tab 详情"));
  }
}

FText SLingTuSimRuntimeHUD::GetFieldTimeText() const {
  return FormatFieldTime(CachedStatus);
}

FText SLingTuSimRuntimeHUD::FormatFieldTime(const FRuntimeUIStatusSnapshot& Status) {
  return FText::FromString(Status.bLatestAppliedTruthAvailable && Status.bIdentityCoherent
      ? FString::Printf(TEXT("仿真时间  %.1f s"), Status.SimTimeNs / 1'000'000'000.0)
      : TEXT("仿真时间  等待真值"));
}

FText SLingTuSimRuntimeHUD::GetFieldSensorText() const {
  if (!HasCurrentStatus(CachedStatus)) return FText::FromString(TEXT("传感器状态未核验"));
  int32 Active = 0;
  for (const auto& Sensor : CachedStatus.FullStatus.Sensors) {
    Active += Sensor.State == LingTuSim::EControlSensorState::Active ? 1 : 0;
  }
  return FText::FromString(FString::Printf(TEXT("传感器  %d / %d 活跃"), Active, CachedStatus.FullStatus.Sensors.Num()));
}

FText SLingTuSimRuntimeHUD::GetFieldRecordText() const {
  if (CachedStatus.CanRequestRecordStopCommit()) return FText::FromString(TEXT("停止并保存记录"));
  if (CachedStatus.CanRequestRecordStart()) return FText::FromString(TEXT("开始记录"));
  return FText::FromString(TEXT("记录不可用"));
}

FReply SLingTuSimRuntimeHUD::HandleMenuResume() {
  if (Measurements.IsValid()) Measurements->SetVisible(false);
  MenuPage = EPlayerMenuPage::Root;
  ModeController->TogglePause();
  if (ModeController->GetMode() == ERuntimeUIMode::Build) ModeController->ToggleBuild();
  if (ModeController->GetMode() == ERuntimeUIMode::Tactical) ModeController->ToggleTactical();
  return FReply::Handled();
}
FReply SLingTuSimRuntimeHUD::HandleMenuTactical() {
  ModeController->TogglePause();
  if (ModeController->GetMode() != ERuntimeUIMode::Tactical) ModeController->ToggleTactical();
  return FReply::Handled();
}
FReply SLingTuSimRuntimeHUD::HandleMenuBuild() {
  ModeController->TogglePause();
  if (ModeController->GetMode() != ERuntimeUIMode::Build) ModeController->ToggleBuild();
  return FReply::Handled();
}
EVisibility SLingTuSimRuntimeHUD::GetRuntimeMenuVisibility() const {
  if (bFrontEndLoginRequired)
    return ModeController->GetMode() == ERuntimeUIMode::Pause && !LoginModel->IsLoggedIn()
        && (MenuPage == EPlayerMenuPage::Settings || MenuPage == EPlayerMenuPage::Quit)
        ? EVisibility::Visible : EVisibility::Collapsed;
  return !bFrontEndLoginRequired && (ModeController->GetMode() == ERuntimeUIMode::Pause || IsMeasurementPageOpen())
      ? EVisibility::Visible : EVisibility::Collapsed;
}
bool SLingTuSimRuntimeHUD::IsMeasurementPageOpen() const {
  return !bFrontEndLoginRequired && MenuPage == EPlayerMenuPage::Measurements &&
      ModeController->GetMode() == ERuntimeUIMode::Tactical;
}
void SLingTuSimRuntimeHUD::FocusRuntimeMenuCTA() {
  if (bSettingsExitDecision && SettingsContinueButton.IsValid() && FSlateApplication::IsInitialized()) {
    FSlateApplication::Get().SetKeyboardFocus(SettingsContinueButton, EFocusCause::SetDirectly);
    return;
  }
  if (ResumeMenuButton.IsValid() && FSlateApplication::IsInitialized())
    FSlateApplication::Get().SetKeyboardFocus(ResumeMenuButton, EFocusCause::SetDirectly);
}
}  // namespace LingTuSim::UI
