#include "SLingTuSimSensorSelection.h"

#include "Brushes/SlateColorBrush.h"
#include "Brushes/SlateDynamicImageBrush.h"
#include "Containers/StringConv.h"
#include "Framework/Application/SlateApplication.h"
#include "Interfaces/IPluginManager.h"
#include "LingTuSimHudScreenshotContract.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScaleBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Notifications/SPopupErrorText.h"
#include <cstdlib>

namespace LingTuSim::UI {
namespace SensorSelectionStyle {
const FLinearColor Ink(.025F, .045F, .08F, 1);
const FLinearColor Muted(.19F, .24F, .31F, 1);
const FLinearColor Blue(.015F, .25F, .80F, 1);
const FSlateBrush* Fill() { return FCoreStyle::Get().GetBrush(TEXT("WhiteBrush")); }
FSlateFontInfo Font(int32 Size) { return FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), Size); }
TSharedRef<STextBlock> Text(const FText& Value, int32 Size, FLinearColor Color = Ink) {
  return SNew(STextBlock).Text(Value).Font(Font(Size)).ColorAndOpacity(Color).AutoWrapText(true);
}
const FButtonStyle* CloseStyle() {
  static const FButtonStyle Style = FButtonStyle(FCoreStyle::Get().GetWidgetStyle<FButtonStyle>(TEXT("Button")))
    .SetNormal(FSlateColorBrush(Blue))
    .SetHovered(FSlateColorBrush(FLinearColor(.025F, .34F, .95F, 1)))
    .SetPressed(FSlateColorBrush(FLinearColor(.01F, .17F, .55F, 1)));
  return &Style;
}
const FEditableTextBoxStyle* RateStyle() {
  static const FEditableTextBoxStyle Style = FEditableTextBoxStyle(FCoreStyle::Get().GetWidgetStyle<FEditableTextBoxStyle>(TEXT("NormalEditableTextBox")))
    .SetBackgroundImageNormal(FSlateColorBrush(FLinearColor(.89F, .93F, .98F, 1)))
    .SetBackgroundImageHovered(FSlateColorBrush(FLinearColor(.83F, .89F, .98F, 1)))
    .SetBackgroundImageFocused(FSlateColorBrush(FLinearColor(.75F, .85F, 1, 1)))
    .SetForegroundColor(Ink).SetFocusedForegroundColor(Ink).SetReadOnlyForegroundColor(Muted).SetPadding(FMargin(8, 6));
  return &Style;
}
const FButtonStyle* ProjectionStyle() {
  static const FButtonStyle Style = FButtonStyle(FCoreStyle::Get().GetWidgetStyle<FButtonStyle>(TEXT("Button")))
    .SetNormal(FSlateColorBrush(FLinearColor(.89F, .93F, .98F, 1)))
    .SetHovered(FSlateColorBrush(FLinearColor(.83F, .89F, .98F, 1)))
    .SetPressed(FSlateColorBrush(FLinearColor(.75F, .85F, 1, 1)))
    .SetDisabled(FSlateColorBrush(FLinearColor(.94F, .96F, .98F, 1)));
  return &Style;
}
FText ProjectionLabel(const FString& Preset) {
  if (Preset == TEXT("equidistant_fisheye")) return NSLOCTEXT("LingTuSensors", "Fisheye", "鱼眼");
  if (Preset == TEXT("equirectangular")) return NSLOCTEXT("LingTuSensors", "Panorama", "全景");
  return NSLOCTEXT("LingTuSensors", "Pinhole", "普通");
}
bool ParseFiniteInput(const FText& Text, double& Out) {
  const FString Input = Text.ToString().TrimStartAndEnd();
  const FTCHARToUTF8 Utf8(*Input);
  char* End = nullptr;
  Out = std::strtod(Utf8.Get(), &End);
  return End != Utf8.Get() && *End == '\0' && FMath::IsFinite(Out);
}
}

void SLingTuSimSensorSelection::Construct(const FArguments& Args) {
  using namespace SensorSelectionStyle;
  Model = Args._Model;
  check(Model.IsValid());
  OnChanged = Args._OnChanged;
  OnClose = Args._OnClose;
  for (const TCHAR* Preset : {TEXT("pinhole"), TEXT("equidistant_fisheye"), TEXT("equirectangular")})
    CameraPresets.Add(MakeShared<FString>(Preset));
  auto Rows = SNew(SVerticalBox);
  for (const auto& Sensor : Model->GetSetup().Sensors)
    Rows->AddSlot().AutoHeight().Padding(0, 0, 0, 8)[SensorRow(Sensor)];
  if (Model->GetSetup().Sensors.IsEmpty())
    Rows->AddSlot().AutoHeight().Padding(12, 24)
      [Text(NSLOCTEXT("LingTuSensors", "Empty", "此配置没有可选传感器。"), 15, Muted)];
  CloseButton = SNew(SButton).ButtonStyle(CloseStyle()).ContentPadding(FMargin(16, 8))
    .OnClicked_Lambda([this]() { OnClose.ExecuteIfBound(); return FReply::Handled(); })
    [Text(NSLOCTEXT("LingTuSensors", "Close", "完成"), 14, FLinearColor::White)];
  CloseButton->SetTag(TEXT("LingTu.Sensors.Close"));
  auto Scroll = SNew(SScrollBox).ScrollWhenFocusChanges(EScrollWhenFocusChanges::InstantScroll)
    + SScrollBox::Slot()[Rows];
  Scroll->SetTag(TEXT("LingTu.Sensors.Scroll"));
  ChildSlot
  [SNew(SBox).WidthOverride(620).HeightOverride(680)
    [SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor(.95F, .97F, 1, 1)).Padding(20)
      [SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
          [SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center)
              [Text(NSLOCTEXT("LingTuSensors", "Title", "传感器配置"), 23, Blue)]
            + SHorizontalBox::Slot().AutoWidth()[CloseButton.ToSharedRef()]]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
          [Text(NSLOCTEXT("LingTuSensors", "StartupOnly", "仅用于下次启动；不会修改运行中的传感器。"), 13, Muted)]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
          [Text(FText::FromString(FString::Printf(TEXT("%s  ·  %s @ %s"), *Model->GetSetup().InstanceId,
            *Model->GetSetup().RigId, *Model->GetSetup().RigVersion)), 12, Muted)]
        + SVerticalBox::Slot().FillHeight(1)
          [Scroll]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
          [Text(NSLOCTEXT("LingTuSensors", "IllustrationNotice", "类型示意，非实测数据。必选项不可关闭；声学显示固定块率，其余频率单位为 Hz。"), 12, Muted)]]]];
}

TSharedRef<SWidget> SLingTuSimSensorSelection::SensorRow(const FSensorChoice& Sensor) {
  using namespace SensorSelectionStyle;
  const FString Id = Sensor.Id;
  auto Enabled = SNew(SCheckBox).IsEnabled(!Sensor.bRequired && Sensor.UnavailableReason.IsEmpty())
    .ToolTipText(!Sensor.UnavailableReason.IsEmpty() ? FText::FromString(Sensor.UnavailableReason)
      : Sensor.bRequired ? NSLOCTEXT("LingTuSensors", "RequiredTip", "此传感器为必选项，不能关闭。")
      : NSLOCTEXT("LingTuSensors", "EnabledTip", "在下次启动时启用此传感器。"))
    .IsChecked_Lambda([this, Id]() { return Model->IsEnabled(Id) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
    .OnCheckStateChanged_Lambda([this, Id](ECheckBoxState State) {
      const bool bEnabled = State == ECheckBoxState::Checked;
      const bool bPrevious = Model->IsEnabled(Id);
      if (Model->SetEnabled(Id, bEnabled) && bPrevious != bEnabled) OnChanged.ExecuteIfBound();
    });
  Enabled->SetTag(FName(*(TEXT("LingTu.Sensors.Enabled.") + Id)));
  EnableBoxes.Add(Id, Enabled);
  auto RateError = SNew(SPopupErrorText).Visibility(EVisibility::Collapsed);
  RateError->SetTag(FName(*(TEXT("LingTu.Sensors.Error.") + Id)));
  auto Rate = SNew(SEditableTextBox).Style(RateStyle()).Font(Font(14))
    .ErrorReporting(RateError)
    .SelectAllTextWhenFocused(true).RevertTextOnEscape(true).ClearKeyboardFocusOnCommit(false)
    .IsEnabled_Lambda([this, Id, Fixed = Sensor.FixedFrequencyHz]() { return Model->IsEnabled(Id) && Fixed == 0.0; })
    .ToolTipText(Sensor.FixedFrequencyHz != 0.0
      ? NSLOCTEXT("LingTuSensors", "FixedBlockRateTip", "固定音频块率（块/s），不是音频采样率；不能在此修改。")
      : FText::Format(NSLOCTEXT("LingTuSensors", "RateTip", "{0} 的启动频率（Hz），请输入正数。"), FText::FromString(Sensor.Label)))
    .Text_Lambda([this, Id]() { return FText::FromString(FString::Printf(TEXT("%.17g"), Model->GetRate(Id))); })
    .OnTextChanged_Lambda([this, Id](const FText&) {
      const auto* Box = RateBoxes.Find(Id);
      if (Box && Box->IsValid()) (*Box)->SetError(FText::GetEmpty());
    })
    .OnTextCommitted_Lambda([this, Id](const FText& Value, ETextCommit::Type Type) { CommitRate(Id, Value, Type); });
  Rate->SetTag(FName(*(TEXT("LingTu.Sensors.Rate.") + Id)));
  RateBoxes.Add(Id, Rate);
  auto Details = SNew(SVerticalBox);
  if (!Sensor.Notice.IsEmpty()) {
    auto Notice = Text(FText::FromString(Sensor.Notice), 11, Muted);
    Notice->SetTag(FName(*(TEXT("LingTu.Sensors.Notice.") + Id)));
    Details->AddSlot().AutoHeight().Padding(0, 5, 0, 0)[Notice];
  }
  if (!Sensor.UnavailableReason.IsEmpty()) {
    auto Reason = Text(FText::FromString(Sensor.UnavailableReason), 11, Muted);
    Reason->SetTag(FName(*(TEXT("LingTu.Sensors.Unavailable.") + Id)));
    Details->AddSlot().AutoHeight().Padding(0, 5, 0, 0)[Reason];
  }
  if (!Sensor.CameraPreset.IsEmpty()) Details->AddSlot().AutoHeight().Padding(0, 8, 0, 0)[CameraControls(Sensor)];
  if (Sensor.GnssFixAvailable.IsSet()) {
    auto Fix = SNew(SCheckBox)
      .IsEnabled_Lambda([this, Id]() { return Model->IsEnabled(Id); })
      .IsChecked_Lambda([this, Id]() { return Model->GetGnssFixAvailable(Id) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
      .OnCheckStateChanged_Lambda([this, Id](ECheckBoxState State) {
        if (!Model->IsEnabled(Id)) return;
        const bool bFix = State == ECheckBoxState::Checked;
        const bool bPrevious = Model->GetGnssFixAvailable(Id);
        if (Model->SetGnssFixAvailable(Id, bFix) && bPrevious != bFix) OnChanged.ExecuteIfBound();
      })[Text(NSLOCTEXT("LingTuSensors", "SimulatedGnssFix", "模拟定位（非实测／RTK）"), 12, Muted)];
    Fix->SetTag(FName(*(TEXT("LingTu.Sensors.GnssFix.") + Id)));
    Details->AddSlot().AutoHeight().Padding(0, 8, 0, 0)[Fix];
  }
  return SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor::White).Padding(10)
    [SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 10, 0)[Enabled]
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)[TypeIllustration(Sensor)]
      + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center).Padding(0, 0, 10, 0)
        [SNew(SVerticalBox)
          + SVerticalBox::Slot().AutoHeight()[Text(FText::FromString(Sensor.Label), 15)]
          + SVerticalBox::Slot().AutoHeight().Padding(0, 3)
            [SNew(STextBlock).Text(FText::FromString(TEXT("安装：") + Sensor.ParentFrame)).Font(Font(11)).ColorAndOpacity(Muted)
              .ToolTipText(FText::FromString(Sensor.Package + TEXT(" / ") + Sensor.Id))]
          + SVerticalBox::Slot().AutoHeight().Padding(0, 3, 0, 0)
            [Text(Sensor.bRequired ? NSLOCTEXT("LingTuSensors", "Required", "必选")
              : NSLOCTEXT("LingTuSensors", "Optional", "可选"), 11, Blue)]]
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
        [SNew(SVerticalBox)
          + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
            [Text(Sensor.FixedFrequencyHz != 0.0 ? NSLOCTEXT("LingTuSensors", "BlockRate", "块率 / 块/s（固定）")
              : NSLOCTEXT("LingTuSensors", "RateHz", "频率 / Hz"), 11, Muted)]
          + SVerticalBox::Slot().AutoHeight()[SNew(SBox).WidthOverride(94)[Rate]]]]
    + SVerticalBox::Slot().AutoHeight()[Details]];
}

TSharedRef<SWidget> SLingTuSimSensorSelection::CameraControls(const FSensorChoice& Sensor) {
  using namespace SensorSelectionStyle;
  const FString Id = Sensor.Id;
  const auto* Initial = CameraPresets.FindByPredicate([this, Id](const TSharedPtr<FString>& Item) {
    return *Item == Model->GetCameraPreset(Id);
  });
  auto Projection = SNew(SComboBox<TSharedPtr<FString>>).OptionsSource(&CameraPresets)
    .ButtonStyle(ProjectionStyle()).ForegroundColor(Ink)
    .InitiallySelectedItem(Initial ? *Initial : nullptr)
    .IsEnabled_Lambda([this, Id]() { return Model->IsEnabled(Id); })
    .OnGenerateWidget_Lambda([](TSharedPtr<FString> Item) -> TSharedRef<SWidget> {
      return SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor(.89F, .93F, .98F, 1))
        .Padding(FMargin(8, 4))[Text(ProjectionLabel(*Item), 13)];
    })
    .OnSelectionChanged_Lambda([this, Id](TSharedPtr<FString> Item, ESelectInfo::Type) {
      if (!Item.IsValid() || !Model->IsEnabled(Id)) return;
      const FString Previous = Model->GetCameraPreset(Id);
      if (Model->SetCameraPreset(Id, *Item) && Previous != *Item) OnChanged.ExecuteIfBound();
    })
    [SNew(STextBlock).Font(Font(13)).ColorAndOpacity(Ink)
      .Text_Lambda([this, Id]() { return ProjectionLabel(Model->GetCameraPreset(Id)); })];
  Projection->SetTag(FName(*(TEXT("LingTu.Sensors.Projection.") + Id)));
  ProjectionBoxes.Add(Id, Projection);
  return SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight()
      [Text(NSLOCTEXT("LingTuSensors", "CameraStartup", "投影与软件云台（下次启动）"), 12, Muted)]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
      [SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 8, 0)[Projection]
        + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 8, 0)[AngleEntry(Sensor, true)]
        + SHorizontalBox::Slot().FillWidth(1)[AngleEntry(Sensor, false)]];
}

TSharedRef<SWidget> SLingTuSimSensorSelection::AngleEntry(const FSensorChoice& Sensor, bool bPan) {
  using namespace SensorSelectionStyle;
  const FString Id = Sensor.Id;
  const FVector2D Limits = bPan ? Sensor.PanLimitsDeg : Sensor.TiltLimitsDeg;
  auto Error = SNew(SPopupErrorText).Visibility(EVisibility::Collapsed);
  auto Box = SNew(SEditableTextBox).Style(RateStyle()).Font(Font(13)).ErrorReporting(Error)
    .SelectAllTextWhenFocused(true).RevertTextOnEscape(true).ClearKeyboardFocusOnCommit(false)
    .IsEnabled_Lambda([this, Id]() { return Model->IsEnabled(Id); })
    .ToolTipText(FText::FromString(FString::Printf(TEXT("%s：[%g, %g]°"), bPan ? TEXT("水平") : TEXT("俯仰"), Limits.X, Limits.Y)))
    .Text_Lambda([this, Id, bPan]() { return FText::FromString(FString::Printf(TEXT("%.17g"), bPan ? Model->GetPanDeg(Id) : Model->GetTiltDeg(Id))); })
    .OnTextChanged_Lambda([this, Id, bPan](const FText&) {
      const auto* Entry = (bPan ? PanBoxes : TiltBoxes).Find(Id);
      if (Entry && Entry->IsValid()) (*Entry)->SetError(FText::GetEmpty());
    })
    .OnTextCommitted_Lambda([this, Id, bPan](const FText& Value, ETextCommit::Type Type) { CommitAngle(Id, bPan, Value, Type); });
  Box->SetTag(FName(*((bPan ? TEXT("LingTu.Sensors.Pan.") : TEXT("LingTu.Sensors.Tilt.")) + Id)));
  (bPan ? PanBoxes : TiltBoxes).Add(Id, Box);
  return SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight()[Text(FText::Format(NSLOCTEXT("LingTuSensors", "AngleBounds", "{0} [{1}, {2}]°"),
      bPan ? NSLOCTEXT("LingTuSensors", "Pan", "水平") : NSLOCTEXT("LingTuSensors", "Tilt", "俯仰"),
      FText::AsNumber(Limits.X), FText::AsNumber(Limits.Y)), 11, Muted)]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 3, 0, 0)[Box];
}

void SLingTuSimSensorSelection::CommitAngle(const FString& Id, bool bPan, const FText& Text, ETextCommit::Type CommitType) {
  if (CommitType == ETextCommit::OnCleared || !Model->IsEnabled(Id)) return;
  const auto* Box = (bPan ? PanBoxes : TiltBoxes).Find(Id);
  if (!Box || !Box->IsValid()) return;
  double Degrees = 0.0;
  const double Previous = bPan ? Model->GetPanDeg(Id) : Model->GetTiltDeg(Id);
  if (!SensorSelectionStyle::ParseFiniteInput(Text, Degrees) ||
      !(bPan ? Model->SetPanDeg(Id, Degrees) : Model->SetTiltDeg(Id, Degrees))) {
    (*Box)->SetError(NSLOCTEXT("LingTuSensors", "InvalidAngle", "请输入限位范围内的有限角度；范围见提示。"));
    return;
  }
  (*Box)->SetError(FText::GetEmpty());
  if (Degrees != Previous) OnChanged.ExecuteIfBound();
}

TSharedRef<SWidget> SLingTuSimSensorSelection::TypeIllustration(const FSensorChoice& Sensor) {
  using namespace SensorSelectionStyle;
  TSharedPtr<FSlateDynamicImageBrush> Brush;
  if (const auto* Existing = Illustrations.Find(Sensor.Modality)) Brush = *Existing;
  else {
    const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("LingTuSim"));
    // A modality is a single local resource name, never an arbitrary file path.
    if (Plugin.IsValid() && FPaths::GetCleanFilename(Sensor.Modality) == Sensor.Modality &&
        !Sensor.Modality.Contains(TEXT("\\")) && !Sensor.Modality.Contains(TEXT(":"))) {
      const FString Path = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Resources/Sensors"), Sensor.Modality + TEXT(".png"));
      int32 Width = 0, Height = 0;
      FString Error;
      if (FPaths::FileExists(Path) && FHudScreenshotContract::ReadPngDimensions(Path, Width, Height, Error))
        Brush = MakeShared<FSlateDynamicImageBrush>(FName(*Path), FVector2D(Width, Height));
    }
    Illustrations.Add(Sensor.Modality, Brush);
  }
  TSharedRef<SWidget> Preview = SNew(SBorder).BorderImage(Fill())
    .BorderBackgroundColor(FLinearColor(.89F, .93F, .98F, 1)).Padding(6)
    [Text(NSLOCTEXT("LingTuSensors", "MissingImage", "暂无示意图"), 11, Muted)];
  if (Brush.IsValid()) Preview = SNew(SScaleBox).Stretch(EStretch::ScaleToFit)[SNew(SImage).Image(Brush.Get())];
  else Preview->SetTag(FName(*(TEXT("LingTu.Sensors.Placeholder.") + Sensor.Id)));
  return SNew(SBox).WidthOverride(104)
    [SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight()[SNew(SBox).HeightOverride(54)[Preview]]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
        [Text(NSLOCTEXT("LingTuSensors", "TypeIllustration", "类型示意"), 10, Muted)]];
}

void SLingTuSimSensorSelection::CommitRate(const FString& Id, const FText& Text, ETextCommit::Type CommitType) {
  if (CommitType == ETextCommit::OnCleared || !Model->IsEnabled(Id)) return;
  const auto* Box = RateBoxes.Find(Id);
  if (!Box || !Box->IsValid()) return;
  if (!(*Box)->IsEnabled()) return;
  double Rate = 0.0;
  const double Previous = Model->GetRate(Id);
  if (!SensorSelectionStyle::ParseFiniteInput(Text, Rate) || Rate <= 0.0) {
    (*Box)->SetError(NSLOCTEXT("LingTuSensors", "InvalidRate", "请输入有限的正数频率（Hz）。"));
    return;
  }
  if (!Model->SetRate(Id, Rate)) {
    (*Box)->SetError(NSLOCTEXT("LingTuSensors", "IncompatiblePhysicsRate",
      "频率与物理时间步不兼容或超出可校验范围；采样周期必须为整数纳秒，且是时间步的整数倍。"));
    return;
  }
  (*Box)->SetError(FText::GetEmpty());
  if (Rate != Previous) OnChanged.ExecuteIfBound();
}

void SLingTuSimSensorSelection::FocusInitialControl() {
  if (!FSlateApplication::IsInitialized()) return;
  TSharedPtr<SWidget> Target = CloseButton;
  for (const auto& Sensor : Model->GetSetup().Sensors) {
    const auto* Box = EnableBoxes.Find(Sensor.Id);
    if (Box && Box->IsValid() && (*Box)->IsEnabled()) { Target = *Box; break; }
  }
  if (Target == CloseButton) {
    for (const auto& Sensor : Model->GetSetup().Sensors) {
      const auto* Box = RateBoxes.Find(Sensor.Id);
      if (Box && Box->IsValid() && (*Box)->IsEnabled()) { Target = *Box; break; }
    }
  }
  FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
}

bool SLingTuSimSensorSelection::IsTextEntryEditing() const {
  for (const auto* Boxes : {&RateBoxes, &PanBoxes, &TiltBoxes}) {
    for (const auto& Entry : *Boxes) {
      const auto& Box = Entry.Value;
      if (Box.IsValid() && Box->IsEnabled() && (Box->HasKeyboardFocus() || Box->HasFocusedDescendants())) return true;
    }
  }
  for (const auto& Entry : ProjectionBoxes) {
    const auto& Box = Entry.Value;
    if (Box.IsValid() && Box->IsEnabled() && (Box->IsOpen() || Box->HasKeyboardFocus() || Box->HasFocusedDescendants())) return true;
  }
  return false;
}
}  // namespace LingTuSim::UI
