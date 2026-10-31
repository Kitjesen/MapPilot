#include "SLingTuSimRobotMapPicker.h"
#include "SLingTuSimShowroom.h"
#include "SLingTuSimShowroomInput.h"
#include "SLingTuSimSensorSelection.h"

#include "Brushes/SlateColorBrush.h"
#include "Brushes/SlateDynamicImageBrush.h"
#include "Framework/Application/SlateApplication.h"
#include "Interfaces/IPluginManager.h"
#include "LingTuSimHudScreenshotContract.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScaleBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace LingTuSim::UI {
namespace {
const FLinearColor Ink(.012F, .016F, .021F, 1);
const FLinearColor Muted(.16F, .19F, .23F, 1);
const FLinearColor Accent(.008F, .28F, .88F, 1);
const FLinearColor Background(.85F, .87F, .89F, 1);
const FSlateBrush* Fill() { return FCoreStyle::Get().GetBrush(TEXT("WhiteBrush")); }
FSlateFontInfo Font(int32 Size) { return FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), Size); }
const FButtonStyle* ChoiceStyle() {
  static const FButtonStyle Style = FButtonStyle()
    .SetNormal(FSlateColorBrush(FLinearColor::Transparent))
    .SetHovered(FSlateColorBrush(FLinearColor(.32F, .39F, .49F, .12F)))
    .SetPressed(FSlateColorBrush(FLinearColor(.32F, .39F, .49F, .22F)))
    .SetDisabled(FSlateColorBrush(FLinearColor::Transparent))
    .SetNormalPadding(FMargin(0)).SetPressedPadding(FMargin(0));
  return &Style;
}
const FButtonStyle* PrimaryStyle() {
  static const FButtonStyle Style = FButtonStyle(*ChoiceStyle())
    .SetNormal(FSlateColorBrush(FLinearColor(.97F, .98F, 1, 1))).SetHovered(FSlateColorBrush(FLinearColor(.80F, .88F, 1, 1)))
    .SetPressed(FSlateColorBrush(FLinearColor(.65F, .76F, .92F, 1)))
    .SetDisabled(FSlateColorBrush(FLinearColor(.62F, .65F, .69F, .5F)));
  return &Style;
}
const FButtonStyle* PreviewStyle() {
  static const FButtonStyle Style = FButtonStyle(*ChoiceStyle())
    .SetHovered(FSlateColorBrush(FLinearColor::Transparent))
    .SetPressed(FSlateColorBrush(FLinearColor::Transparent));
  return &Style;
}
const FEditableTextBoxStyle* SearchStyle() {
  static const FEditableTextBoxStyle Style = FEditableTextBoxStyle(FCoreStyle::Get().GetWidgetStyle<FEditableTextBoxStyle>(TEXT("NormalEditableTextBox")))
    .SetBackgroundImageNormal(FSlateColorBrush(FLinearColor(.45F, .48F, .55F, .20F)))
    .SetBackgroundImageHovered(FSlateColorBrush(FLinearColor(.45F, .48F, .55F, .27F)))
    .SetBackgroundImageFocused(FSlateColorBrush(FLinearColor(.40F, .55F, .75F, .20F)))
    .SetForegroundColor(Ink).SetPadding(FMargin(16, 12));
  return &Style;
}
TSharedRef<STextBlock> Text(const TCHAR* Value, int32 Size, FLinearColor Color = Ink) {
  return SNew(STextBlock).Text(FText::FromString(Value)).Font(Font(Size)).ColorAndOpacity(Color).AutoWrapText(true);
}
TSharedRef<SWidget> Line() {
  return SNew(SBox).HeightOverride(1)[SNew(SBorder).BorderImage(Fill()).Padding(0)
    .BorderBackgroundColor(FLinearColor(.25F, .29F, .34F, .18F))];
}
const TCHAR* AvailabilityName(EGameSelectionAvailability Value) {
  return Value == EGameSelectionAvailability::Runnable ? TEXT("可运行")
    : Value == EGameSelectionAvailability::PreviewOnly ? TEXT("仅预览")
    : Value == EGameSelectionAvailability::Quarantined ? TEXT("已隔离") : TEXT("不可用");
}
}

void SLingTuSimRobotMapPicker::Construct(const FArguments& Args) {
  Model = Args._Model;
  check(Model.IsValid());
  CanEdit = Args._CanEdit;
  CanConfirm = Args._CanConfirm;
  Status = Args._Status;
  OnSelectionChanged = Args._OnSelectionChanged;
  OnConfirm = Args._OnConfirm;
  OnBack = Args._OnBack;
  LoadPreviews();
  Showroom = SNew(SLingTuSimShowroom).RobotId_Lambda([this]() {
    return GetVisibleRobotCount() > 0 ? Model->GetRequestedRobotId() : FString();
  }).Visibility_Lambda([this]() { return Step == EStep::Robot ? EVisibility::HitTestInvisible : EVisibility::Collapsed; });
  RobotChoices = Model->GetRobotChoices();
  auto Robots = SNew(SVerticalBox);
  for (const auto& Package : RobotChoices)
    Robots->AddSlot().AutoHeight().Padding(0, 0, 0, 2)[Choice(Package, true)];
  Robots->AddSlot().AutoHeight().Padding(12, 24)
    [SNew(STextBlock).Text(FText::FromString(TEXT("没有匹配的机器人\n试试其他分类或关键词"))).Font(Font(15)).ColorAndOpacity(Muted)
      .Visibility_Lambda([this]() { return GetVisibleRobotCount() == 0 ? EVisibility::Visible : EVisibility::Collapsed; })];
  auto Worlds = SNew(SVerticalBox);
  for (const auto& Package : Model->GetWorldChoices())
    Worlds->AddSlot().AutoHeight().Padding(0, 0, 0, 2)[Choice(Package, false)];
  if (Model->GetWorldChoices().IsEmpty()) Worlds->AddSlot().AutoHeight()[Text(TEXT("尚未载入地图目录"), 16, Muted)];
  auto Configurations = SNew(SVerticalBox);
  for (const auto& Option : Model->GetCatalog().Options)
    Configurations->AddSlot().AutoHeight()[ConfigurationChoice(Option)];
  BackButton = SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(18, 14))
    .OnClicked_Lambda([this]() {
      if (GoBack()) return FReply::Handled();
      return OnBack.IsBound() ? OnBack.Execute() : FReply::Handled();
    })[SNew(STextBlock).Font(Font(16)).ColorAndOpacity(Ink).Text_Lambda([this]() {
      return FText::FromString(Step == EStep::Robot ? TEXT("返回") : TEXT("上一步"));
    })];
  PrimaryButton = SNew(SButton).ButtonStyle(PrimaryStyle()).ContentPadding(FMargin(36, 16))
    .IsEnabled(this, &SLingTuSimRobotMapPicker::CanAdvance)
    .OnClicked(this, &SLingTuSimRobotMapPicker::PrimaryAction)
    [SNew(STextBlock).Font(Font(17)).ColorAndOpacity(FLinearColor(.015F, .015F, .015F, 1))
      .Text_Lambda([this]() { return FText::FromString(Step == EStep::Robot ? TEXT("下一步：选择地图")
          : TEXT("进入场景")); })];
  ClosePreviewButton = SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(22, 14))
    .OnClicked_Lambda([this]() { GoBack(); return FReply::Handled(); })[Text(TEXT("关闭预览  ×"), 17)];

  SearchBox = SNew(SEditableTextBox).Style(SearchStyle()).Font(Font(15)).HintText(FText::FromString(TEXT("搜索型号或厂商")))
    .IsEnabled_Lambda([this]() { return CanEdit.Get(false) && !Model->IsConfirmed(); })
    .OnTextChanged_Lambda([this](const FText& Value) { RobotSearch = Value.ToString().TrimStartAndEnd(); ApplyRobotFilter(); })
    .OnTextCommitted_Lambda([this](const FText&, ETextCommit::Type Type) { if (Type == ETextCommit::OnEnter) FocusInitialControl(); });
  SearchBox->SetTag(TEXT("LingTu.Selection.Search"));
  RobotScroll = SNew(SScrollBox).ScrollWhenFocusChanges(EScrollWhenFocusChanges::InstantScroll)
    + SScrollBox::Slot()[Robots];
  auto Lists = SNew(SWidgetSwitcher).WidgetIndex_Lambda([this]() { return static_cast<int32>(Step); })
    + SWidgetSwitcher::Slot()[SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 18)[SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(1)[SearchBox.ToSharedRef()]
        + SHorizontalBox::Slot().AutoWidth()[SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(12, 0))
          .ToolTipText(FText::FromString(TEXT("清除搜索")))
          .Visibility_Lambda([this]() { return RobotSearch.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
          .OnClicked_Lambda([this]() { SearchBox->SetText(FText::GetEmpty()); FocusInitialControl(); return FReply::Handled(); })[Text(TEXT("×"), 20)]]]
      + SVerticalBox::Slot().FillHeight(1)[RobotScroll.ToSharedRef()]]
    + SWidgetSwitcher::Slot()[SNew(SScrollBox) + SScrollBox::Slot()[Worlds]];
  auto Left = SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
      [SNew(STextBlock).Text(this, &SLingTuSimRobotMapPicker::StepTitle).Font(Font(28)).ColorAndOpacity(Ink)]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 20)
      [SNew(STextBlock).Font(Font(13)).ColorAndOpacity(Muted).AutoWrapText(true).Text_Lambda([this]() {
        if (Step == EStep::Robot) return FText::FromString(FString::Printf(TEXT("%s  /  %02d 款"), RobotCategoryLabel(RobotCategory), GetVisibleRobotCount()));
        const int32 Count = Model->GetWorldChoices().Num();
        return FText::FromString(FString::Printf(TEXT("%s  /  %02d 个目的地"), *SelectedPackageLabel(true).ToString(), Count));
      })]
    + SVerticalBox::Slot().FillHeight(1)[Lists];
  auto Right = SNew(SVerticalBox)
    .Visibility_Lambda([this]() { return Step == EStep::Robot ? EVisibility::Collapsed : EVisibility::Visible; })
    + SVerticalBox::Slot().AutoHeight().Padding(24, 0, 24, 6)
      [SNew(STextBlock).Font(Font(13)).ColorAndOpacity(Muted).Text_Lambda([this]() {
        for (const auto& Robot : RobotChoices) if (Robot.Id == Model->GetRequestedRobotId())
          return FText::FromString(RobotCategoryLabel(Robot.Category));
        return FText::GetEmpty();
      }).Visibility_Lambda([this]() { return Step == EStep::Robot && GetVisibleRobotCount() > 0 ? EVisibility::Visible : EVisibility::Collapsed; })]
    + SVerticalBox::Slot().AutoHeight().Padding(24, 0, 24, 0)
      [SNew(STextBlock).Text_Lambda([this]() { return SelectedPackageLabel(Step == EStep::Robot); })
        .Font(Font(32)).ColorAndOpacity(Ink).AutoWrapText(true)]
    + SVerticalBox::Slot().FillHeight(1).Padding(0, 8, 0, 0)[PreviewPanel()]
    + SVerticalBox::Slot().AutoHeight().Padding(24, 0, 24, 4)
      [SNew(STextBlock).Font(Font(13)).ColorAndOpacity(Muted).Text_Lambda([this]() {
        return FText::FromString(!PreviewBrush() ? TEXT("预览待制作")
          : Step == EStep::Robot ? TEXT("本体预览  /  点击放大") : TEXT("场景预览  /  点击放大"));
      })]
    + SVerticalBox::Slot().AutoHeight().Padding(24, 10, 24, 0)
      [SNew(STextBlock).Font(Font(13)).ColorAndOpacity(Muted).Text(FText::FromString(TEXT("模型已入库 · 驾驶控制尚未接入")))
        .Visibility_Lambda([this]() {
          const bool HasSession = Model->GetCatalog().Options.ContainsByPredicate([this](const auto& Option) {
            return Option.Robot.Id == Model->GetRequestedRobotId();
          });
          return Step == EStep::Robot && GetVisibleRobotCount() > 0 && !HasSession ? EVisibility::Visible : EVisibility::Collapsed;
        })]
    + SVerticalBox::Slot().AutoHeight().Padding(24, 0, 24, 0)
      [SNew(SBox).Visibility_Lambda([this]() { return Step == EStep::World ? EVisibility::Visible : EVisibility::Collapsed; })
        [SNew(SVerticalBox)
          + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 8)[Text(TEXT("任务"), 13, Muted)]
          + SVerticalBox::Slot().AutoHeight()[SNew(SBox).MaxDesiredHeight(140)
            [SNew(SScrollBox) + SScrollBox::Slot()[Configurations]]]
          + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
            [SNew(STextBlock).Text_Lambda([this]() {
              const auto* Option = Model->GetSelectedOption();
              return FText::FromString(Option ? Option->Description : FString());
            }).Font(Font(13)).ColorAndOpacity(Muted).AutoWrapText(true)]
          + SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
            [SNew(STextBlock).Text(this, &SLingTuSimRobotMapPicker::PairAvailability).Font(Font(12)).ColorAndOpacity(Muted).AutoWrapText(true)]]];
  WorldPreviewButton = PreviewButton;
  auto Details = SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 20)
      [SNew(STextBlock).Font(Font(13)).ColorAndOpacity(Muted).Text_Lambda([this]() {
        for (const auto& Robot : RobotChoices) if (Robot.Id == Model->GetRequestedRobotId()) return FText::FromString(RobotCategoryLabel(Robot.Category));
        return FText::GetEmpty();
      })]
    + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Text_Lambda([this]() { return SelectedPackageLabel(true); })
      .Font(FCoreStyle::GetDefaultFontStyle(TEXT("Bold"), 36)).ColorAndOpacity(Ink).AutoWrapText(true)]
    + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left).Padding(0, 36, 0, 28)[SNew(SBox).WidthOverride(48)[Line()]]
    + SVerticalBox::Slot().AutoHeight()[Text(TEXT("当前接入"), 13, Muted)]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)[SNew(STextBlock).Font(Font(15)).ColorAndOpacity(Ink).AutoWrapText(true)
      .Text_Lambda([this]() {
        const bool Runnable = Model->GetCatalog().Options.ContainsByPredicate([this](const auto& Option) {
          return Option.Robot.Id == Model->GetRequestedRobotId() && Option.IsRunnable();
        });
        return FText::FromString(Runnable ? TEXT("已有可运行场景") : TEXT("模型已入库 · 驾驶控制尚未接入"));
      })]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 28, 0, 0)
      [SAssignNew(SensorsButton, SButton).ButtonStyle(PrimaryStyle()).ContentPadding(FMargin(18, 12))
        .IsEnabled_Lambda([this]() {
          const auto* Option = Model->GetSelectedOption();
          return CanEdit.Get(false) && !Model->IsConfirmed() && Option && Option->SensorSelection.IsValid();
        }).OnClicked_Lambda([this]() { OpenSensorSelection(); return FReply::Handled(); })
        [Text(TEXT("传感器配置"), 17)]]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
      [SNew(STextBlock).Font(Font(12)).ColorAndOpacity(Muted).AutoWrapText(true).Text_Lambda([this]() {
        const auto* Option = Model->GetSelectedOption();
        if (!Option || !Option->SensorSelection) return FText::FromString(TEXT("尚未声明可配置的传感器装配"));
        const auto& Setup = Option->SensorSelection->GetSetup();
        int32 Enabled = 0;
        for (const auto& Sensor : Setup.Sensors) Enabled += Option->SensorSelection->IsEnabled(Sensor.Id) ? 1 : 0;
        return FText::FromString(FString::Printf(TEXT("已选 %d / %d · %s"), Enabled, Setup.Sensors.Num(),
            Option->SensorSelection->HasChanges() ? TEXT("下次启动生效") : TEXT("装配默认配置")));
      })]
    + SVerticalBox::Slot().FillHeight(1);
  ShowroomInput = SNew(SLingTuSimShowroomInput)
    .CanInteract_Lambda([this]() { return Step == EStep::Robot && !bSensorSelectionOpen && Showroom->HasRobot(); })
    .OnGesture([this](EShowroomGesture Gesture, FVector2D Delta) {
      switch (Gesture) {
        case EShowroomGesture::Orbit: Showroom->Orbit(Delta); break;
        case EShowroomGesture::Pan: Showroom->Pan(Delta); break;
        case EShowroomGesture::Zoom: Showroom->Zoom(Delta.X); break;
        case EShowroomGesture::Reset: Showroom->SetView(EShowroomView::Perspective); break;
      }
    });
  auto RobotDetails = SNew(SHorizontalBox)
    .Visibility_Lambda([this]() { return Step == EStep::Robot ? EVisibility::Visible : EVisibility::Collapsed; })
    + SHorizontalBox::Slot().FillWidth(.43F).Padding(32, 0, 12, 0)[Details]
    + SHorizontalBox::Slot().FillWidth(.57F)
      [SNew(SOverlay)
          + SOverlay::Slot()[ShowroomInput.ToSharedRef()]
          + SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Center)
            [SNew(STextBlock).Font(Font(16)).ColorAndOpacity(Muted)
              .Visibility_Lambda([this]() { return Showroom->HasRobot() ? EVisibility::Collapsed : EVisibility::HitTestInvisible; })
              .Text_Lambda([this]() { return FText::FromString(Showroom->IsReady() ? TEXT("此模型的 3D 展示尚未就绪") : TEXT("正在准备 3D 模型…")); })]];
  auto Footer = SNew(SHorizontalBox)
    + SHorizontalBox::Slot().AutoWidth()[BackButton.ToSharedRef()]
    + SHorizontalBox::Slot().AutoWidth().Padding(18, 0)
      [SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(18, 14))
        .Visibility(Args._OnBrowseAssets.IsBound() ? EVisibility::Visible : EVisibility::Collapsed)
        .OnClicked(Args._OnBrowseAssets)[Text(TEXT("素材库"), 15, Muted)]]
    + SHorizontalBox::Slot().FillWidth(1).HAlign(HAlign_Center).VAlign(VAlign_Center)
      [Text(TEXT("↑ ↓ 选择    Enter 确认    Esc 返回"), 12, Muted)]
    + SHorizontalBox::Slot().AutoWidth()[PrimaryButton.ToSharedRef()];
  auto Page = SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight()[StepRail()]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 34, 0, 0)[CategoryRail()]
    + SVerticalBox::Slot().FillHeight(1).Padding(0, 32, 0, 0)
      [SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(.27F).Padding(0, 0, 30, 0)[Left]
        + SHorizontalBox::Slot().FillWidth(.73F)[SNew(SOverlay) + SOverlay::Slot()[Right] + SOverlay::Slot()[RobotDetails]]]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 24, 0, 18)[Line()]
    + SVerticalBox::Slot().AutoHeight()[Footer];
  ChildSlot[SNew(SOverlay)
    + SOverlay::Slot()[SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(Background)]
    + SOverlay::Slot()[Showroom.ToSharedRef()]
    + SOverlay::Slot()[MakeShowroomVeil()]
    + SOverlay::Slot()[SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(FLinearColor::Transparent).Padding(FMargin(56, 38))
      .IsEnabled_Lambda([this]() { return !bPreviewExpanded && !bSensorSelectionOpen; })
      [SNew(SScaleBox).Stretch(EStretch::ScaleToFit).StretchDirection(EStretchDirection::DownOnly)
        [SNew(SBox).WidthOverride(1760).HeightOverride(930)[Page]]]]
    + SOverlay::Slot()[SNew(SBorder).BorderImage(Fill()).BorderBackgroundColor(Background).Padding(40)
      .Visibility_Lambda([this]() { return bPreviewExpanded ? EVisibility::Visible : EVisibility::Collapsed; })
      [SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)[ClosePreviewButton.ToSharedRef()]
        + SVerticalBox::Slot().FillHeight(1)[SNew(SScaleBox).Stretch(EStretch::ScaleToFit)
          [SNew(SImage).Image(this, &SLingTuSimRobotMapPicker::PreviewBrush)]]]]
    + SOverlay::Slot().HAlign(HAlign_Right).VAlign(VAlign_Center).Padding(56, 0)
      [SAssignNew(SensorSelectionHost, SBox).Visibility_Lambda([this]() {
        return bSensorSelectionOpen ? EVisibility::Visible : EVisibility::Collapsed;
      })]];
}

bool SLingTuSimRobotMapPicker::OpenSensorSelection() {
  const auto* Option = Model->GetSelectedOption();
  if (!CanEdit.Get(false) || Model->IsConfirmed() || !Option || !Option->SensorSelection) return false;
  if (ShowroomInput) ShowroomInput->CancelDrag();
  SensorSelectionPanel = SNew(SLingTuSimSensorSelection).Model(Option->SensorSelection)
    .OnChanged(FSimpleDelegate::CreateLambda([this]() {
      OnSelectionChanged.ExecuteIfBound(); Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
    })).OnClose(FSimpleDelegate::CreateSP(this, &SLingTuSimRobotMapPicker::CloseSensorSelection));
  SensorSelectionHost->SetContent(SensorSelectionPanel.ToSharedRef());
  bSensorSelectionOpen = true;
  SlatePrepass(1.F);
  SensorSelectionPanel->FocusInitialControl();
  return true;
}

void SLingTuSimRobotMapPicker::CloseSensorSelection() {
  bSensorSelectionOpen = false;
  Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
  SlatePrepass(1.F);
  if (SensorsButton && FSlateApplication::IsInitialized())
    FSlateApplication::Get().SetKeyboardFocus(SensorsButton, EFocusCause::SetDirectly);
}

TSharedRef<SWidget> SLingTuSimRobotMapPicker::StepRail() {
  auto Rail = SNew(SHorizontalBox);
  const TCHAR* Titles[] = {TEXT("01  机器人"), TEXT("02  地图与任务")};
  Rail->AddSlot().FillWidth(.36F).VAlign(VAlign_Center)[SNew(SVerticalBox)
    + SVerticalBox::Slot().AutoHeight()[Text(TEXT("L I N G T U   /   灵途"), 22, Ink)]
    + SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)[Text(TEXT("R O B O T I C S   F O R   A   B R I G H T E R   T O M O R R O W"), 7, Muted)]];
  for (int32 Index = 0; Index < UE_ARRAY_COUNT(Titles); ++Index) {
    const EStep Target = static_cast<EStep>(Index);
    Rail->AddSlot().AutoWidth().Padding(0, 0, 28, 0)
      [SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(16, 12))
        .IsEnabled_Lambda([this, Target]() { return Target <= Step; })
        .OnClicked_Lambda([this, Target]() { if (Target <= Step) ChangeStep(Target); return FReply::Handled(); })
        [SNew(SVerticalBox)
          + SVerticalBox::Slot().AutoHeight()[SNew(STextBlock).Text(FText::FromString(Titles[Index])).Font(Font(15))
            .ColorAndOpacity_Lambda([this, Target]() { return Target == Step ? Ink : Muted; })]
          + SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)[SNew(SBox).HeightOverride(2)
            [SNew(SBorder).Padding(0).BorderImage(Fill()).BorderBackgroundColor_Lambda([this, Target]() {
              return Target == Step ? Accent : FLinearColor::Transparent;
            })]]]];
  }
  Rail->AddSlot().FillWidth(.30F)[SNew(SBox)];
  return Rail;
}

TSharedRef<SWidget> SLingTuSimRobotMapPicker::CategoryRail() {
  auto Rail = SNew(SHorizontalBox);
  for (const FString Category : {TEXT("all"), TEXT("quadruped"), TEXT("wheeled_legged"), TEXT("humanoid"), TEXT("wheeled"), TEXT("manipulator"), TEXT("other")}) {
    int32 Count = 0;
    for (const auto& Robot : RobotChoices) if (Category == TEXT("all") || Robot.Category == Category) ++Count;
    if (Category == TEXT("other") && Count == 0) continue;
    auto Button = SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(22, 10))
      .IsEnabled_Lambda([this, Count]() { return Count > 0 && CanEdit.Get(false) && !Model->IsConfirmed(); })
      .OnClicked_Lambda([this, Category]() { SetRobotCategory(Category); return FReply::Handled(); });
    Button->SetTag(FName(*(TEXT("LingTu.Selection.Category.") + Category)));
    const TWeakPtr<SButton> WeakButton = Button;
    Button->SetContent(SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
        + SHorizontalBox::Slot().AutoWidth()[SNew(STextBlock).Text(FText::FromString(RobotCategoryLabel(Category))).Font(Font(16))
          .ColorAndOpacity_Lambda([this, Category]() { return RobotCategory == Category ? Accent : Ink; })]
        + SHorizontalBox::Slot().AutoWidth().Padding(12, 2, 0, 0)[Text(*FString::Printf(TEXT("%02d"), Count), 12, Muted)]]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 11, 0, 0)[SNew(SBox).HeightOverride(2)
        [SNew(SBorder).Padding(0).BorderImage(Fill()).BorderBackgroundColor_Lambda([this, Category, WeakButton]() {
          const auto Control = WeakButton.Pin();
          return RobotCategory == Category ? Accent : Control.IsValid() && Control->HasKeyboardFocus() ? Muted : FLinearColor::Transparent;
        })]]);
    Rail->AddSlot().AutoWidth().Padding(0, 0, 10, 0)[Button];
  }
  return SNew(SBox).Visibility_Lambda([this]() { return Step == EStep::Robot ? EVisibility::Visible : EVisibility::Collapsed; })[Rail];
}

TSharedRef<SWidget> SLingTuSimRobotMapPicker::PreviewPanel() {
  PreviewButton = SNew(SButton).ButtonStyle(PreviewStyle()).ContentPadding(0)
    .IsEnabled_Lambda([this]() { return PreviewBrush() != nullptr; })
    .OnClicked_Lambda([this]() {
      if (PreviewBrush()) {
        bPreviewExpanded = true;
        SlatePrepass(1.F);
        if (FSlateApplication::IsInitialized()) FSlateApplication::Get().SetKeyboardFocus(ClosePreviewButton, EFocusCause::SetDirectly);
      }
      return FReply::Handled();
    })
    [SNew(SOverlay)
        + SOverlay::Slot()[SNew(SScaleBox).Stretch(EStretch::ScaleToFit)
          .Visibility_Lambda([this]() { return Step == EStep::Robot && Showroom.IsValid() && Showroom->HasRobot() ? EVisibility::Hidden : EVisibility::Visible; })
          [SNew(SImage).Image(this, &SLingTuSimRobotMapPicker::PreviewBrush)]]
        + SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Center)
          [SNew(SBox).Visibility_Lambda([this]() { return PreviewBrush() ? EVisibility::Collapsed : EVisibility::Visible; })
            [SNew(STextBlock).Font(Font(18)).ColorAndOpacity(Muted).Text_Lambda([this]() {
              return FText::FromString(Step == EStep::Robot && GetVisibleRobotCount() == 0 ? TEXT("修改分类或关键词以继续") : TEXT("模型预览待制作"));
            })]]];
  PreviewButton->SetTag(TEXT("LingTu.Selection.Preview"));
  return PreviewButton.ToSharedRef();
}

TSharedRef<SWidget> SLingTuSimRobotMapPicker::Choice(const FGameSelectionPackageSummary& Package, bool bRobot) {
  const FString Id = Package.Id;
  auto Button = SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(0)
    .Visibility_Lambda([this, Package, bRobot]() { return !bRobot || MatchesRobot(Package) ? EVisibility::Visible : EVisibility::Collapsed; })
    .IsEnabled_Lambda([this]() { return CanEdit.Get(false) && !Model->IsConfirmed(); })
    .OnClicked_Lambda([this, Id, bRobot]() {
      if (CanEdit.Get(false) && (bRobot ? Model->SelectRobotById(Id) : Model->SelectWorldById(Id))) RefreshSelection();
      return FReply::Handled();
    });
  (bRobot ? RobotButtons : WorldButtons).Add(Id, Button);
  const TWeakPtr<SButton> Weak = Button;
  Button->SetTag(FName(*(TEXT("LingTu.Selection.") + FString(bRobot ? TEXT("Robot.") : TEXT("World.")) + Id)));
  const bool Runnable = Model->GetCatalog().Options.ContainsByPredicate([&](const auto& Option) {
    return (bRobot ? Option.Robot.Id : Option.World.Id) == Id && Option.IsRunnable();
  });
  Button->SetContent(SNew(SBorder).BorderImage(Fill()).Padding(FMargin(10, 6))
    .BorderBackgroundColor_Lambda([this, Id, bRobot, Weak]() {
      const auto Row = Weak.Pin();
      if (Row.IsValid() && Row->HasKeyboardFocus()) return FLinearColor(.38F, .48F, .62F, .22F);
      return (bRobot ? Model->GetRequestedRobotId() : Model->GetRequestedWorldId()) == Id
          ? FLinearColor(.42F, .47F, .56F, .18F) : FLinearColor::Transparent;
    })[SNew(SHorizontalBox)
    + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, bRobot ? 12 : 0, 0)
      [SNew(SBox).WidthOverride(82).HeightOverride(64).Visibility(bRobot ? EVisibility::Visible : EVisibility::Collapsed)
        [SNew(SOverlay)
          + SOverlay::Slot()[SNew(SScaleBox).Stretch(EStretch::ScaleToFit)[SNew(SImage).Image(RobotBrush(Id))]]]]
    + SHorizontalBox::Slot().FillWidth(1).VAlign(VAlign_Center)[SNew(SVerticalBox)
      + SVerticalBox::Slot().AutoHeight()[Text(*Package.Label, bRobot ? 17 : 19)]
      + SVerticalBox::Slot().AutoHeight().Padding(0, 7, 0, 0)
        [SNew(STextBlock).Font(Font(12)).ColorAndOpacity(Muted).AutoWrapText(true)
          .Text_Lambda([this, Id, bRobot, Category = Package.Category]() {
            return bRobot ? FText::FromString(RobotCategoryLabel(Category)) : WorldCompatibility(Id);
          })]]
    + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(12, 0, 2, 0)
      [SNew(STextBlock).Font(Font(12)).ColorAndOpacity(Ink).Text(FText::FromString(TEXT("可运行")))
        .Visibility(bRobot && Runnable ? EVisibility::Visible : EVisibility::Collapsed)]]);
  return Button;
}

TSharedRef<SWidget> SLingTuSimRobotMapPicker::ConfigurationChoice(const FGameSelectionOption& Option) {
  const FString Id = Option.Id, RobotId = Option.Robot.Id, WorldId = Option.World.Id;
  return SNew(SButton).ButtonStyle(ChoiceStyle()).ContentPadding(FMargin(16, 10))
    .Visibility_Lambda([this, RobotId, WorldId]() {
      return Model->GetRequestedRobotId() == RobotId && Model->GetRequestedWorldId() == WorldId ? EVisibility::Visible : EVisibility::Collapsed;
    })
    .IsEnabled_Lambda([this]() { return CanEdit.Get(false) && !Model->IsConfirmed(); })
    .OnClicked_Lambda([this, Id]() { if (CanEdit.Get(false) && Model->SelectById(Id)) RefreshSelection(); return FReply::Handled(); })
    [SNew(SHorizontalBox)
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 16, 0)
        [SNew(SBox).WidthOverride(3).HeightOverride(20)[SNew(SBorder).BorderImage(Fill()).Padding(0)
          .BorderBackgroundColor_Lambda([this, Id]() {
            const auto* Current = Model->GetSelectedOption(); return Current && Current->Id == Id ? Ink : FLinearColor::Transparent;
          })]]
      + SHorizontalBox::Slot().FillWidth(1)[Text(*Option.Title, 16)]
      + SHorizontalBox::Slot().AutoWidth().Padding(18, 0, 0, 0)[Text(AvailabilityName(Option.Availability), 12, Muted)]];
}

void SLingTuSimRobotMapPicker::RefreshSelection() {
  if (ShowroomInput) ShowroomInput->CancelDrag();
  OnSelectionChanged.ExecuteIfBound();
  Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
  SlatePrepass(1.F);
}

bool SLingTuSimRobotMapPicker::MatchesRobot(const FGameSelectionPackageSummary& Package) const {
  return (RobotCategory == TEXT("all") || Package.Category == RobotCategory) &&
    (RobotSearch.IsEmpty() || Package.Label.Contains(RobotSearch, ESearchCase::IgnoreCase) ||
     Package.Id.Contains(RobotSearch, ESearchCase::IgnoreCase));
}

int32 SLingTuSimRobotMapPicker::GetVisibleRobotCount() const {
  int32 Count = 0;
  for (const auto& Robot : RobotChoices) if (MatchesRobot(Robot)) ++Count;
  return Count;
}

void SLingTuSimRobotMapPicker::SetRobotCategory(const FString& Category) {
  if (Step != EStep::Robot || !CanEdit.Get(false) || Model->IsConfirmed() || bPreviewExpanded) return;
  if (Category != TEXT("all") && !RobotChoices.ContainsByPredicate([&](const auto& Robot) { return Robot.Category == Category; })) return;
  RobotCategory = Category;
  ApplyRobotFilter();
}

void SLingTuSimRobotMapPicker::ApplyRobotFilter() {
  const auto* First = RobotChoices.FindByPredicate([this](const auto& Robot) { return MatchesRobot(Robot); });
  const bool CurrentVisible = RobotChoices.ContainsByPredicate([this](const auto& Robot) {
    return Robot.Id == Model->GetRequestedRobotId() && MatchesRobot(Robot);
  });
  if (First && !CurrentVisible && CanEdit.Get(false) && !Model->IsConfirmed()) Model->SelectRobotById(First->Id);
  if (RobotScroll.IsValid()) RobotScroll->ScrollToStart();
  RefreshSelection();
}

bool SLingTuSimRobotMapPicker::IsSearchEditing() const {
  return bSensorSelectionOpen || (Step == EStep::Robot && SearchBox.IsValid() &&
    (SearchBox->HasKeyboardFocus() || SearchBox->HasFocusedDescendants()));
}
bool SLingTuSimRobotMapPicker::IsShowroomReady() const {
  return Step != EStep::Robot || (Showroom.IsValid() && Showroom->IsReady());
}
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
void SLingTuSimRobotMapPicker::PreviewShowroomInspection() {
  if (Step != EStep::Robot || !Showroom->HasRobot()) return;
  // Exercise the same public camera mutations used by the live Slate surface.
  Showroom->Orbit(FVector2D(.40, .08));
  Showroom->Zoom(1);
  Showroom->Pan(FVector2D(-.1, 0));
}
#endif
void SLingTuSimRobotMapPicker::ChangeStep(EStep NewStep) {
  if (ShowroomInput) ShowroomInput->CancelDrag();
  Step = NewStep;
  bPreviewExpanded = false;
  Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
  SlatePrepass(1.F);
  FocusInitialControl();
}
bool SLingTuSimRobotMapPicker::CanAdvance() const {
  if (bPreviewExpanded || bSensorSelectionOpen) return false;
  if (Step == EStep::Robot) return GetVisibleRobotCount() > 0 && !Model->GetRequestedRobotId().IsEmpty();
  FString Blocker;
  return CanEdit.Get(false) && Model->CanConfirm(Blocker) && CanConfirm.Get(false) && OnConfirm.IsBound();
}
FReply SLingTuSimRobotMapPicker::PrimaryAction() {
  if (!CanAdvance()) return FReply::Handled();
  if (Step == EStep::World) return OnConfirm.Execute();
  ChangeStep(EStep::World);
  return FReply::Handled();
}
bool SLingTuSimRobotMapPicker::GoBack() {
  if (bSensorSelectionOpen) { CloseSensorSelection(); return true; }
  if (ShowroomInput) ShowroomInput->CancelDrag();
  if (bPreviewExpanded) {
    bPreviewExpanded = false;
    Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
    SlatePrepass(1.F);
    if (FSlateApplication::IsInitialized()) FSlateApplication::Get().SetKeyboardFocus(Step == EStep::Robot ? PreviewButton : WorldPreviewButton, EFocusCause::SetDirectly);
    return true;
  }
  if (Step == EStep::Robot) {
    if (!RobotSearch.IsEmpty()) { SearchBox->SetText(FText::GetEmpty()); FocusInitialControl(); return true; }
    return false;
  }
  ChangeStep(EStep::Robot);
  return true;
}
void SLingTuSimRobotMapPicker::FocusInitialControl() {
  const auto* Row = Step == EStep::Robot ? RobotButtons.Find(Model->GetRequestedRobotId())
      : Step == EStep::World ? WorldButtons.Find(Model->GetRequestedWorldId()) : nullptr;
  const auto Target = Row && Row->IsValid() && (*Row)->IsEnabled() && (*Row)->GetVisibility().IsVisible() ? *Row
      : PrimaryButton.IsValid() && PrimaryButton->IsEnabled() ? PrimaryButton : BackButton;
  if (Target.IsValid() && FSlateApplication::IsInitialized()) FSlateApplication::Get().SetKeyboardFocus(Target, EFocusCause::SetDirectly);
}
void SLingTuSimRobotMapPicker::LoadPreviews() {
  const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("LingTuSim"));
  if (!Plugin.IsValid()) return;
  const auto Load = [this, &Plugin](const FString& Name) {
    const FString Path = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Resources/Selection"), Name + TEXT(".png"));
    if (!FPaths::FileExists(Path)) return;
    int32 Width = 0, Height = 0;
    FString Error;
    if (FHudScreenshotContract::ReadPngDimensions(Path, Width, Height, Error))
      Previews.Add(Name, MakeShared<FSlateDynamicImageBrush>(FName(*Path), FVector2D(Width, Height)));
  };
  for (const auto& Package : Model->GetRobotChoices()) Load(TEXT("robot-") + Package.Id);
  for (const auto& Package : Model->GetWorldChoices()) Load(TEXT("world-") + Package.Id);
}
const FSlateBrush* SLingTuSimRobotMapPicker::PreviewBrush() const {
  if (Step == EStep::Robot && GetVisibleRobotCount() == 0) return nullptr;
  const FString Key = Step == EStep::Robot ? TEXT("robot-") + Model->GetRequestedRobotId() : TEXT("world-") + Model->GetRequestedWorldId();
  const auto* Brush = Previews.Find(Key);
  return Brush && Brush->IsValid() ? Brush->Get() : nullptr;
}
const FSlateBrush* SLingTuSimRobotMapPicker::RobotBrush(const FString& Id) const {
  const auto* Brush = Previews.Find(TEXT("robot-") + Id);
  return Brush && Brush->IsValid() ? Brush->Get() : nullptr;
}
FText SLingTuSimRobotMapPicker::SelectedPackageLabel(bool bRobot) const {
  if (bRobot && Step == EStep::Robot && GetVisibleRobotCount() == 0) return FText::FromString(TEXT("没有匹配结果"));
  const auto Choices = bRobot ? Model->GetRobotChoices() : Model->GetWorldChoices();
  const auto& Id = bRobot ? Model->GetRequestedRobotId() : Model->GetRequestedWorldId();
  for (const auto& Package : Choices) if (Package.Id == Id) return FText::FromString(Package.Label);
  return FText::FromString(TEXT("尚未选择"));
}
FText SLingTuSimRobotMapPicker::StepTitle() const {
  return FText::FromString(Step == EStep::Robot ? TEXT("选择机器人") : TEXT("选择地图"));
}
FText SLingTuSimRobotMapPicker::PairSummary() const {
  const auto* Option = Model->GetSelectedOption();
  if (!Option) {
    for (const auto& Candidate : Model->GetCatalog().Options)
      if (Candidate.Robot.Id == Model->GetRequestedRobotId() && Candidate.World.Id == Model->GetRequestedWorldId())
        return FText::FromString(TEXT("请选择一项任务配置"));
    return FText::FromString(TEXT("这台机器人尚未配置此地图，请选择其他地图或返回更换机器人"));
  }
  return FText::FromString(FString::Printf(TEXT("%s · 机器人 v%s / 地图 v%s"), AvailabilityName(Option->Availability), *Option->Robot.Version, *Option->World.Version));
}
FText SLingTuSimRobotMapPicker::PairAvailability() const {
  if (!CanEdit.Get(false)) return FText::FromString(TEXT("当前会话仅供查看，请结束会话后重新选择。"));
  if (Model->IsConfirmed()) return Status.Get();
  const auto* Option = Model->GetSelectedOption();
  if (!Option) return PairSummary();
  if (!Option->IsRunnable()) return FText::FromString(FString::Printf(TEXT("%s · %s"), AvailabilityName(Option->Availability), *Option->AvailabilityReason));
  return CanConfirm.Get(false) && OnConfirm.IsBound() ? FText::GetEmpty() : Status.Get();
}
FText SLingTuSimRobotMapPicker::WorldCompatibility(const FString& WorldId) const {
  int32 RunnableCount = 0;
  bool bHasPair = false, bHasPreview = false, bAllQuarantined = true;
  for (const auto& Option : Model->GetCatalog().Options) {
    if (Option.Robot.Id != Model->GetRequestedRobotId() || Option.World.Id != WorldId) continue;
    bHasPair = true;
    if (Option.IsRunnable()) ++RunnableCount;
    bHasPreview |= Option.Availability == EGameSelectionAvailability::PreviewOnly;
    bAllQuarantined &= Option.Availability == EGameSelectionAvailability::Quarantined;
  }
  if (RunnableCount > 0) return FText::FromString(FString::Printf(TEXT("可运行 · %d 项配置"), RunnableCount));
  if (!bHasPair) return FText::FromString(TEXT("当前机器人未适配"));
  if (bHasPreview) return FText::FromString(TEXT("仅预览 · 暂不可驾驶"));
  return FText::FromString(bAllQuarantined ? TEXT("已隔离 · 不可运行") : TEXT("配置不可用"));
}
}
