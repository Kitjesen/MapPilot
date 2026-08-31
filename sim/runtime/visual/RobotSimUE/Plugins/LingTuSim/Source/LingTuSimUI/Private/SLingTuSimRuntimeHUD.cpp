#include "SLingTuSimRuntimeHUD.h"

#include "Brushes/SlateDynamicImageBrush.h"
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/FileManager.h"
#include "Interfaces/IPluginManager.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScaleBox.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace LingTuSim::UI {
namespace {
// Light, airy gameplay HUD tokens. The world remains the primary surface.
const FLinearColor WarmWhiteGlass(0.97F, 0.95F, 0.90F, 0.88F);
const FLinearColor WarmWhitePanel(0.99F, 0.98F, 0.94F, 0.92F);
const FLinearColor WarmWhiteSoft(0.98F, 0.97F, 0.93F, 0.80F);
const FLinearColor TealAccent(0.02F, 0.55F, 0.50F, 1.0F);
const FLinearColor AmberAccent(0.82F, 0.49F, 0.08F, 1.0F);
const FLinearColor Ink(0.07F, 0.12F, 0.14F, 1.0F);
const FLinearColor MutedInk(0.26F, 0.34F, 0.35F, 1.0F);
const FLinearColor AlertInk(0.72F, 0.24F, 0.16F, 1.0F);
const FLinearColor MenuBackdrop(0.012F, 0.024F, 0.030F, 1.0F);
const FLinearColor DisabledButton(0.15F, 0.18F, 0.19F, 1.0F);
const FLinearColor FrontEndVoid(0.006F, 0.012F, 0.012F, 1.0F);
const FLinearColor FrontEndGlass(0.015F, 0.026F, 0.025F, 0.94F);
const FLinearColor FrontEndGlassSoft(0.025F, 0.042F, 0.039F, 0.86F);
const FLinearColor FrontEndText(0.91F, 0.92F, 0.86F, 1.0F);
const FLinearColor FrontEndMuted(0.53F, 0.59F, 0.55F, 1.0F);
const FLinearColor FieldAmber(0.94F, 0.55F, 0.12F, 1.0F);
const FLinearColor FieldAmberDim(0.32F, 0.19F, 0.07F, 1.0F);
const FLinearColor FieldTeal(0.12F, 0.58F, 0.52F, 1.0F);

const FSlateBrush *WhiteBrush() {
  return FCoreStyle::Get().GetBrush(TEXT("WhiteBrush"));
}

FSlateFontInfo Font(const int32 Size, const bool bBold = false) {
  return FCoreStyle::GetDefaultFontStyle(bBold ? TEXT("Bold") : TEXT("Regular"), Size);
}

TSharedRef<STextBlock> StaticLabel(const TCHAR *Text, const int32 Size = 10,
                                   const FLinearColor Color = MutedInk) {
  return SNew(STextBlock).Text(FText::FromString(Text)).Font(Font(Size)).ColorAndOpacity(Color);
}

const TCHAR *BindingStateName(const LingTuSim::EControlBindingState State) {
  switch (State) {
    case LingTuSim::EControlBindingState::Unbound:
      return TEXT("UNBOUND");
    case LingTuSim::EControlBindingState::Prepared:
      return TEXT("PREPARED");
    case LingTuSim::EControlBindingState::Active:
      return TEXT("ACTIVE");
    case LingTuSim::EControlBindingState::Failed:
      return TEXT("FAILED");
    case LingTuSim::EControlBindingState::Unavailable:
    default:
      return TEXT("UNAVAILABLE");
  }
}

const TCHAR *AckStatusName(const LingTuSim::EControlAckStatus Status) {
  switch (Status) {
    case LingTuSim::EControlAckStatus::Pending:
      return TEXT("PENDING");
    case LingTuSim::EControlAckStatus::Accepted:
      return TEXT("ACCEPTED");
    case LingTuSim::EControlAckStatus::Rejected:
      return TEXT("REJECTED");
    case LingTuSim::EControlAckStatus::Released:
      return TEXT("RELEASED");
    case LingTuSim::EControlAckStatus::TimeoutZero:
      return TEXT("TIMEOUT ZERO");
    case LingTuSim::EControlAckStatus::Confirmed:
      return TEXT("CONFIRMED");
    default:
      return TEXT("UNAVAILABLE");
  }
}

const TCHAR *RecordingStateName(const LingTuSim::EControlRecordingState State) {
  switch (State) {
    case LingTuSim::EControlRecordingState::Idle:
      return TEXT("IDLE");
    case LingTuSim::EControlRecordingState::Requested:
      return TEXT("REQUESTED");
    case LingTuSim::EControlRecordingState::Recording:
      return TEXT("RECORDING");
    case LingTuSim::EControlRecordingState::Committed:
      return TEXT("COMMITTED");
    case LingTuSim::EControlRecordingState::Rejected:
      return TEXT("REJECTED");
    case LingTuSim::EControlRecordingState::Failed:
      return TEXT("FAILED");
    case LingTuSim::EControlRecordingState::Unavailable:
    default:
      return TEXT("UNAVAILABLE");
  }
}

const TCHAR *CameraModeName(const LingTuSim::EControlStatusCameraMode Mode) {
  switch (Mode) {
    case LingTuSim::EControlStatusCameraMode::Follow:
      return TEXT("FOLLOW");
    case LingTuSim::EControlStatusCameraMode::Inspection:
      return TEXT("INSPECTION");
    case LingTuSim::EControlStatusCameraMode::Free:
      return TEXT("FREE");
    case LingTuSim::EControlStatusCameraMode::Unavailable:
    default:
      return TEXT("UNAVAILABLE");
  }
}

const TCHAR *AssetReviewDispositionLabel(const EAssetReviewDisposition Disposition) {
  switch (Disposition) {
    case EAssetReviewDisposition::Unverified:
      return TEXT("待验证");
    case EAssetReviewDisposition::Quarantined:
      return TEXT("已隔离");
    case EAssetReviewDisposition::ProxyOnly:
      return TEXT("仅代理");
    case EAssetReviewDisposition::Unavailable:
    default:
      return TEXT("不可用");
  }
}

FString AssetClassLabel(const FString &AssetClass) {
  const FString Normalized = AssetClass.ToLower();
  if (Normalized.Contains(TEXT("robot"))) {
    return TEXT("机器人");
  }
  if (Normalized.Contains(TEXT("payload")) || Normalized.Contains(TEXT("sensor"))) {
    return TEXT("任务装备");
  }
  if (Normalized.Contains(TEXT("tree")) || Normalized.Contains(TEXT("vegetation")) ||
      Normalized.Contains(TEXT("foliage"))) {
    return TEXT("树木");
  }
  if (Normalized.Contains(TEXT("rock")) || Normalized.Contains(TEXT("stone"))) {
    return TEXT("岩石");
  }
  if (Normalized.Contains(TEXT("human")) || Normalized.Contains(TEXT("pedestrian")) ||
      Normalized.Contains(TEXT("person"))) {
    return TEXT("行人");
  }
  if (Normalized.Contains(TEXT("vehicle")) || Normalized.Contains(TEXT("moving"))) {
    return TEXT("移动物体");
  }
  return AssetClass.IsEmpty() ? TEXT("3D 素材") : AssetClass;
}

FString AssetReviewStageLabel(const FString &ReviewStage) {
  const FString Normalized = ReviewStage.ToLower();
  if (Normalized.Contains(TEXT("catalog")) || Normalized.Contains(TEXT("review"))) {
    return TEXT("目录审阅");
  }
  if (Normalized.Contains(TEXT("source"))) {
    return TEXT("源素材检查");
  }
  if (Normalized.Contains(TEXT("qualified"))) {
    return TEXT("运行验证");
  }
  return ReviewStage.IsEmpty() ? TEXT("待检查") : ReviewStage;
}
}  // namespace

void SLingTuSimRuntimeHUD::Construct(const FArguments &InArgs) {
  World = InArgs._World;
  ModeController = InArgs._ModeController;
  LocalState = InArgs._LocalState;
  LoginModel = InArgs._LoginModel;
  SelectionModel = InArgs._SelectionModel;
  AssetReviewModel = InArgs._AssetReviewModel;
  SelectionFeedback = InArgs._SelectionFeedback;
  bFrontEndLoginRequired = InArgs._FrontEndLoginRequired;
  bSelectionIntentConfigured = InArgs._SelectionIntentConfigured;
  OnSelectionPrevious = InArgs._OnSelectionPrevious;
  OnSelectionNext = InArgs._OnSelectionNext;
  OnSelectionConfirmed = InArgs._OnSelectionConfirmed;
  OnAssetReviewPrevious = InArgs._OnAssetReviewPrevious;
  OnAssetReviewNext = InArgs._OnAssetReviewNext;
  check(ModeController.IsValid());
  check(LocalState.IsValid());
  check(LoginModel.IsValid());
  check(SelectionModel.IsValid());
  check(AssetReviewModel.IsValid());
  check(SelectionFeedback.IsValid());
  CachedStatus = FRuntimeUIStatusReader::Read(World.Get(), LocalState.Get());
  SetVisibility(EVisibility::SelfHitTestInvisible);

  if (const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("LingTuSim"))) {
    const FString BackgroundPath = FPaths::Combine(
        Plugin->GetBaseDir(), TEXT("Resources/FrontEnd/lingtu-field-ops-hero-v1.png"));
    if (IFileManager::Get().FileExists(*BackgroundPath)) {
      FrontEndBackgroundBrush =
          MakeShared<FSlateDynamicImageBrush>(FName(*BackgroundPath), FVector2D(1920.0F, 1080.0F));
    }

    struct FPreviewResource {
      const TCHAR *CardId;
      const TCHAR *FileName;
    };
    const FPreviewResource PreviewResources[] = {
        {TEXT("thunder_v4"), TEXT("preview-thunder-v4-v1.png")},
        {TEXT("rws_01"), TEXT("preview-rws-01-v1.png")},
        {TEXT("forest_pine"), TEXT("preview-forest-pine-v1.png")},
        {TEXT("forest_birch"), TEXT("preview-forest-birch-v1.png")},
        {TEXT("forest_boulder"), TEXT("preview-forest-boulder-v1.png")},
    };
    for (const FPreviewResource &PreviewResource : PreviewResources) {
      const FString PreviewPath = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Resources/FrontEnd"),
                                                  PreviewResource.FileName);
      if (IFileManager::Get().FileExists(*PreviewPath)) {
        AssetPreviewBrushes.Add(
            PreviewResource.CardId,
            MakeShared<FSlateDynamicImageBrush>(FName(*PreviewPath), FVector2D(1280.0F, 720.0F)));
      }
    }
  }

  TSharedRef<SUniformGridPanel> AssetCardGrid = SNew(SUniformGridPanel).SlotPadding(4.0F);
  const FAssetReviewCatalog &AssetCatalog = AssetReviewModel->GetCatalog();
  const int32 VisibleAssetCards = FMath::Min(AssetCatalog.Cards.Num(), 4);
  for (int32 CardIndex = 0; CardIndex < VisibleAssetCards; ++CardIndex) {
    const FAssetReviewCard &Card = AssetCatalog.Cards[CardIndex];
    const FString CardId = Card.Id;
    AssetCardGrid->AddSlot(CardIndex % 2, CardIndex / 2)
        [SNew(SButton)
             .ButtonColorAndOpacity_Lambda(
                 [this, CardId]() { return GetAssetFilmstripCardColor(CardId); })
             .IsEnabled_Lambda([this, CardId]() { return CanSelectAssetReview(CardId); })
             .OnClicked_Lambda([this, CardId]() { return HandleSelectAssetReview(CardId); })
             .ContentPadding(FMargin(10.0F, 8.0F))
                 [SNew(SVerticalBox) +
                  SVerticalBox::Slot().AutoHeight()
                      [SNew(SHorizontalBox) +
                       SHorizontalBox::Slot().FillWidth(
                           1.0F)[SNew(STextBlock)
                                     .Text(FText::FromString(AssetClassLabel(Card.AssetClass)))
                                     .Font(Font(9, true))
                                     .ColorAndOpacity(Ink)] +
                       SHorizontalBox::Slot()
                           .AutoWidth()[SNew(STextBlock)
                                            .Text(FText::FromString(
                                                AssetReviewDispositionLabel(Card.Disposition)))
                                            .Font(Font(8, true))
                                            .ColorAndOpacity(MutedInk)]] +
                  SVerticalBox::Slot().AutoHeight().Padding(
                      0.0F, 5.0F, 0.0F,
                      0.0F)[SNew(STextBlock)
                                .Text(FText::FromString(Card.Title))
                                .Font(Font(10, true))
                                .ColorAndOpacity(Ink)
                                .AutoWrapText(true)] +
                  SVerticalBox::Slot().AutoHeight().Padding(
                      0.0F, 5.0F, 0.0F,
                      0.0F)[SNew(STextBlock)
                                .Text(FText::FromString(TEXT("暂无素材预览")))
                                .Font(Font(8))
                                .ColorAndOpacity(MutedInk)]]];
  }

  ChildSlot
      [SNew(SOverlay)

       + SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Top)
             .Padding(FMargin(
                 28.0F, 22.0F, 28.0F,
                 0.0F))[SNew(SBorder)
                            .Visibility(this, &SLingTuSimRuntimeHUD::GetRuntimeHudChromeVisibility)
                            .BorderImage(WhiteBrush())
                            .BorderBackgroundColor(WarmWhiteGlass)
                            .Padding(FMargin(18.0F, 10.0F))
                                [SNew(SVerticalBox)

                                 + SVerticalBox::Slot().AutoHeight()
                                       [SNew(SHorizontalBox)

                                        + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
                                              [SNew(STextBlock)
                                                   .Text(FText::FromString(TEXT("LINGTU")))
                                                   .Font(Font(15, true))
                                                   .ColorAndOpacity(TealAccent)]

                                        + SHorizontalBox::Slot()
                                              .AutoWidth()
                                              .Padding(16.0F, 0.0F, 0.0F, 0.0F)
                                              .VAlign(VAlign_Center)
                                                  [SNew(STextBlock)
                                                       .Text(this,
                                                             &SLingTuSimRuntimeHUD::GetIdentityText)
                                                       .Font(Font(10))
                                                       .ColorAndOpacity(Ink)]

                                        + SHorizontalBox::Slot().FillWidth(1.0F)[SNew(SSpacer)]

                                        + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
                                              [SNew(STextBlock)
                                                   .Text(this, &SLingTuSimRuntimeHUD::GetModeText)
                                                   .Font(Font(11, true))
                                                   .ColorAndOpacity(TealAccent)]]

                                 + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 5.0F, 0.0F, 0.0F)
                                       [SNew(STextBlock)
                                            .Text(this, &SLingTuSimRuntimeHUD::GetReadinessText)
                                            .Font(Font(9))
                                            .ColorAndOpacity(MutedInk)]]]

       + SOverlay::Slot()
             .HAlign(HAlign_Left)
             .VAlign(VAlign_Bottom)
             .Padding(FMargin(28.0F, 0.0F, 0.0F, 66.0F))
                 [SNew(SBox).WidthOverride(430.0F).Visibility(
                     this, &SLingTuSimRuntimeHUD::GetRuntimeHudChromeVisibility)
                      [SNew(SBorder)
                           .BorderImage(WhiteBrush())
                           .BorderBackgroundColor(WarmWhitePanel)
                           .Padding(FMargin(16.0F, 13.0F))
                               [SNew(SVerticalBox)

                                + SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                      TEXT("CONTROL / REQUEST"), 10, TealAccent)]

                                + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 7.0F, 0.0F, 0.0F)
                                      [SNew(STextBlock)
                                           .Text(this, &SLingTuSimRuntimeHUD::GetInputText)
                                           .Font(Font(10))
                                           .ColorAndOpacity(Ink)]

                                + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 3.0F, 0.0F, 0.0F)
                                      [SNew(STextBlock)
                                           .Text(this, &SLingTuSimRuntimeHUD::GetRequestedAxesText)
                                           .Font(Font(11, true))
                                           .ColorAndOpacity(Ink)]

                                + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 7.0F, 0.0F, 0.0F)
                                      [SNew(STextBlock)
                                           .Text(this, &SLingTuSimRuntimeHUD::GetAckText)
                                           .Font(Font(10, true))
                                           .ColorAndOpacity(TealAccent)]

                                + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 3.0F, 0.0F, 0.0F)
                                      [SNew(STextBlock)
                                           .Text(this, &SLingTuSimRuntimeHUD::GetAcceptedMotionText)
                                           .Font(Font(9))
                                           .ColorAndOpacity(MutedInk)]]]]

       +
       SOverlay::Slot()
           .HAlign(HAlign_Right)
           .VAlign(VAlign_Bottom)
           .Padding(FMargin(0.0F, 0.0F, 28.0F, 66.0F))
               [SNew(SBox).WidthOverride(400.0F).Visibility(
                   this, &SLingTuSimRuntimeHUD::GetRuntimeHudChromeVisibility)
                    [SNew(SBorder)
                         .BorderImage(WhiteBrush())
                         .BorderBackgroundColor(WarmWhiteSoft)
                         .Padding(FMargin(16.0F, 13.0F))
                             [SNew(SVerticalBox)

                              + SVerticalBox::Slot().AutoHeight()
                                    [StaticLabel(TEXT("OBSERVED / MUJOCO TRUTH"), 10, TealAccent)]

                              + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 7.0F, 0.0F, 0.0F)
                                    [SNew(STextBlock)
                                         .Text(this, &SLingTuSimRuntimeHUD::GetObservedVelocityText)
                                         .Font(Font(10, true))
                                         .ColorAndOpacity(Ink)]

                              + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 4.0F, 0.0F, 0.0F)
                                    [SNew(STextBlock)
                                         .Text(this, &SLingTuSimRuntimeHUD::GetTruthFrameText)
                                         .Font(Font(9))
                                         .ColorAndOpacity(MutedInk)]

                              + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 7.0F, 0.0F, 0.0F)
                                    [SNew(STextBlock)
                                         .Text(this, &SLingTuSimRuntimeHUD::GetSensorsText)
                                         .Font(Font(9))
                                         .ColorAndOpacity(MutedInk)]

                              + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 4.0F, 0.0F, 0.0F)
                                    [SNew(STextBlock)
                                         .Text(this, &SLingTuSimRuntimeHUD::GetRecordingText)
                                         .Font(Font(9, true))
                                         .ColorAndOpacity(TealAccent)]

                              + SVerticalBox::Slot().AutoHeight().Padding(0.0F, 4.0F, 0.0F, 0.0F)
                                    [SNew(STextBlock)
                                         .Text(this, &SLingTuSimRuntimeHUD::GetCameraText)
                                         .Font(Font(9))
                                         .ColorAndOpacity(MutedInk)]]]]

       +
       SOverlay::Slot()
           .HAlign(HAlign_Center)
           .VAlign(VAlign_Center)
           .Padding(FMargin(24.0F, 70.0F, 24.0F, 100.0F))
               [SNew(SBox).WidthOverride(360.0F).Visibility(
                   this, &SLingTuSimRuntimeHUD::GetBuildVisibility)
                    [SNew(SBorder)
                         .BorderImage(WhiteBrush())
                         .BorderBackgroundColor(WarmWhiteGlass)
                         .Padding(22.0F)[SNew(SVerticalBox) +
                                         SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Center)
                                             [StaticLabel(TEXT("BUILD"), 16, TealAccent)] +
                                         SVerticalBox::Slot()
                                             .AutoHeight()
                                             .HAlign(HAlign_Center)
                                             .Padding(0.0F, 6.0F, 0.0F, 0.0F)[StaticLabel(
                                                 TEXT("READ-ONLY PREVIEW"), 11, AlertInk)] +
                                         SVerticalBox::Slot()
                                             .AutoHeight()
                                             .HAlign(HAlign_Center)
                                             .Padding(0.0F, 8.0F, 0.0F, 0.0F)[StaticLabel(
                                                 TEXT("No selection, transform, or publish writes"),
                                                 9, MutedInk)]]]]

       + SOverlay::Slot()
             .HAlign(HAlign_Center)
             .VAlign(VAlign_Center)
             .Padding(FMargin(24.0F, 70.0F, 24.0F, 100.0F))
                 [SNew(SBox).WidthOverride(390.0F).Visibility(
                     this, &SLingTuSimRuntimeHUD::GetTacticalVisibility)
                      [SNew(SBorder)
                           .BorderImage(WhiteBrush())
                           .BorderBackgroundColor(WarmWhiteGlass)
                           .Padding(22.0F)
                               [SNew(SVerticalBox) +
                                SVerticalBox::Slot().AutoHeight().HAlign(
                                    HAlign_Center)[StaticLabel(TEXT("TACTICAL"), 16, TealAccent)] +
                                SVerticalBox::Slot()
                                    .AutoHeight()
                                    .HAlign(HAlign_Center)
                                    .Padding(0.0F, 7.0F, 0.0F, 0.0F)[StaticLabel(
                                        TEXT("READ-ONLY WORLD OVERVIEW"), 10, MutedInk)] +
                                SVerticalBox::Slot()
                                    .AutoHeight()
                                    .HAlign(HAlign_Center)
                                    .Padding(0.0F, 10.0F, 0.0F, 0.0F)
                                        [SNew(STextBlock)
                                             .Text(this, &SLingTuSimRuntimeHUD::GetTruthFrameText)
                                             .Font(Font(10))
                                             .ColorAndOpacity(Ink)]]]]

       + SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Fill)
                 [SNew(SBorder)
                      .Visibility(this, &SLingTuSimRuntimeHUD::GetFrontEndStaticBackdropVisibility)
                      .BorderImage(WhiteBrush())
                      .BorderBackgroundColor(MenuBackdrop)]

       + SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Fill)
                 [SNew(SImage)
                      .Visibility(this, &SLingTuSimRuntimeHUD::GetFrontEndStaticBackdropVisibility)
                      .Image(FrontEndBackgroundBrush.IsValid() ? FrontEndBackgroundBrush.Get()
                                                               : WhiteBrush())
                      .ColorAndOpacity(FrontEndBackgroundBrush.IsValid()
                                           ? FLinearColor(0.48F, 0.55F, 0.48F, 1.0F)
                                           : FrontEndVoid)]

       + SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Fill)
                 [SNew(SBorder)
                      .Visibility(this, &SLingTuSimRuntimeHUD::GetFrontEndStaticBackdropVisibility)
                      .BorderImage(WhiteBrush())
                      .BorderBackgroundColor(FLinearColor(0.004F, 0.010F, 0.009F, 0.54F))]

       + SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Fill)
             .Padding(
                 FMargin(64.0F, 54.0F, 64.0F,
                         48.0F))[SNew(SBox)
                                     .Visibility(this, &SLingTuSimRuntimeHUD::GetLoginVisibility)
                                     .ToolTipText(FText::FromString(TEXT(
                                         "开始本地任务  ·  A / ENTER 确认")))[BuildLoginScreen()]]

       + SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Fill)
             .Padding(
                 FMargin(0.0F))[SNew(SBox)
                                    .Visibility(this, &SLingTuSimRuntimeHUD::GetWorkspaceVisibility)
                                    .ToolTipText(FText::FromString(TEXT(
                                        "A / ENTER 确认  ·  B / ESC 返回  ·  LB / RB 浏览素材")))
                                        [BuildWorkspaceScreen()]]

       + SOverlay::Slot()
             .HAlign(HAlign_Center)
             .VAlign(VAlign_Center)
                 [SNew(SBox).WidthOverride(1120.0F).MinDesiredHeight(500.0F).Visibility(
                     EVisibility::Collapsed)
                      [SNew(SHorizontalBox) +
                       SHorizontalBox::Slot().FillWidth(1.12F)
                           [SNew(SBorder)
                                .BorderImage(WhiteBrush())
                                .BorderBackgroundColor(FLinearColor(0.018F, 0.035F, 0.040F, 0.97F))
                                .Padding(FMargin(38.0F, 34.0F))
                                    [SNew(SVerticalBox) +
                                     SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                         TEXT("灵途  /  FIELD SIMULATION"), 12, TealAccent)] +
                                     SVerticalBox::Slot().FillHeight(1.0F).VAlign(VAlign_Center)
                                         [SNew(SVerticalBox) +
                                          SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                              TEXT("机器人巡检\n仿真工作台"), 27, FrontEndText)] +
                                          SVerticalBox::Slot().AutoHeight().Padding(
                                              0.0F, 18.0F, 32.0F,
                                              0.0F)[SNew(STextBlock)
                                                        .Text(FText::FromString(
                                                            TEXT("选择已编译的仿真会话，查看机器"
                                                                 "人、森林与场景"
                                                                 "素材审核证据，再将一份已验证的"
                                                                 "选择交给启动器。")))
                                                        .Font(Font(11))
                                                        .ColorAndOpacity(
                                                            FLinearColor(0.73F, 0.80F, 0.78F, 1.0F))
                                                        .AutoWrapText(true)]] +
                                     SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                         TEXT("MUJOCO PHYSICS AUTHORITY  ·  UE PRESENTATION"), 9,
                                         AmberAccent)]]] +
                       SHorizontalBox::Slot().FillWidth(0.88F)
                           [SNew(SBorder)
                                .BorderImage(WhiteBrush())
                                .BorderBackgroundColor(FrontEndGlass)
                                .Padding(FMargin(34.0F, 30.0F))
                                    [SNew(SVerticalBox) +
                                     SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                         TEXT("行动终端"), 17, FrontEndText)] +
                                     SVerticalBox::Slot().AutoHeight().Padding(0.0F, 7.0F, 0.0F,
                                                                               0.0F)[StaticLabel(
                                         TEXT("本地行动档案  ·  离线"), 10, TealAccent)] +
                                     SVerticalBox::Slot().AutoHeight().Padding(0.0F, 20.0F, 0.0F,
                                                                               0.0F)[StaticLabel(
                                         TEXT("显示名称  /  仅用于本次运行"), 9, MutedInk)] +
                                     SVerticalBox::Slot().AutoHeight().Padding(0.0F, 6.0F, 0.0F,
                                                                               0.0F)
                                         [SNew(SBorder)
                                              .BorderImage(WhiteBrush())
                                              .BorderBackgroundColor(TealAccent)
                                              .Padding(1.0F)
                                                  [SNew(SBorder)
                                                       .BorderImage(WhiteBrush())
                                                       .BorderBackgroundColor(
                                                           FLinearColor(0.96F, 0.97F, 0.94F, 1.0F))
                                                       .Padding(FMargin(8.0F, 4.0F))
                                                           [SNew(SEditableTextBox)
                                                                .Text(FText::FromString(
                                                                    PendingOperatorName))
                                                                .HintText(FText::FromString(
                                                                    TEXT("请输入操作员"
                                                                         "名称")))
                                                                .ForegroundColor(Ink)
                                                                .MaximumLength(64)
                                                                .IsEnabled_Lambda([this]() {
                                                                  return LoginModel->CanEdit();
                                                                })
                                                                .OnTextChanged(
                                                                    this,
                                                                    &SLingTuSimRuntimeHUD::
                                                                        HandleOperatorNameChanged)]]] +
                                     SVerticalBox::Slot().AutoHeight().Padding(
                                         0.0F, 12.0F, 0.0F,
                                         0.0F)[SNew(STextBlock)
                                                   .Text(this, &SLingTuSimRuntimeHUD::
                                                                   GetLoginCatalogStatusText)
                                                   .Font(Font(10, true))
                                                   .ColorAndOpacity(TealAccent)
                                                   .AutoWrapText(true)] +
                                     SVerticalBox::Slot().AutoHeight().Padding(
                                         0.0F, 16.0F, 0.0F,
                                         0.0F)[SNew(SButton)
                                                   .ButtonColorAndOpacity_Lambda([this]() {
                                                     return CanEnterLocalWorkspace()
                                                                ? TealAccent
                                                                : DisabledButton;
                                                   })
                                                   .ForegroundColor(FLinearColor::White)
                                                   .IsEnabled(this, &SLingTuSimRuntimeHUD::
                                                                        CanEnterLocalWorkspace)
                                                   .OnClicked(this, &SLingTuSimRuntimeHUD::
                                                                        HandleEnterLocalWorkspace)
                                                       [StaticLabel(TEXT("进入行动终端  →"), 11,
                                                                    FLinearColor::White)]] +
                                     SVerticalBox::Slot().AutoHeight().Padding(
                                         0.0F, 12.0F, 0.0F,
                                         0.0F)[SNew(SButton)
                                                   .ButtonColorAndOpacity(DisabledButton)
                                                   .IsEnabled(false)[StaticLabel(
                                                       TEXT("联网功能  ·  尚未启用"), 10,
                                                       FLinearColor(0.62F, 0.66F, 0.66F, 1.0F))]] +
                                     SVerticalBox::Slot().AutoHeight().Padding(
                                         0.0F, 9.0F, 0.0F,
                                         0.0F)[SNew(STextBlock)
                                                   .Text(FText::FromString(TEXT("当前版本不执行在"
                                                                                "线认证，也不会生"
                                                                                "成登录令牌。")))
                                                   .Font(Font(9))
                                                   .ColorAndOpacity(MutedInk)
                                                   .AutoWrapText(true)] +
                                     SVerticalBox::Slot().FillHeight(1.0F)[SNew(SSpacer)] +
                                     SVerticalBox::Slot().AutoHeight()
                                         [SNew(STextBlock)
                                              .Text(this,
                                                    &SLingTuSimRuntimeHUD::GetLoginFeedbackText)
                                              .Font(Font(10, true))
                                              .ColorAndOpacity(AlertInk)
                                              .AutoWrapText(true)]]]]]

       +
       SOverlay::Slot()
           .HAlign(HAlign_Center)
           .VAlign(VAlign_Center)
               [SNew(SBox).WidthOverride(1120.0F).MinDesiredHeight(500.0F).Visibility(
                   EVisibility::Collapsed)
                    [SNew(SBorder)
                         .BorderImage(WhiteBrush())
                         .BorderBackgroundColor(FrontEndGlass)
                         .Padding(22.0F)
                             [SNew(SVerticalBox) +
                              SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Center)
                                  [SNew(STextBlock)
                                       .Text(this, &SLingTuSimRuntimeHUD::GetFrontEndIdentityText)
                                       .Font(Font(17, true))
                                       .ColorAndOpacity(TealAccent)] +
                              SVerticalBox::Slot()
                                  .AutoHeight()
                                  .HAlign(HAlign_Center)
                                  .Padding(0.0F, 9.0F, 0.0F, 0.0F)
                                      [SNew(STextBlock)
                                           .Text(this, &SLingTuSimRuntimeHUD::GetMenuRequestText)
                                           .Font(Font(11, true))
                                           .ColorAndOpacity(Ink)] +
                              SVerticalBox::Slot().FillHeight(1.0F).Padding(0.0F, 14.0F, 0.0F, 0.0F)
                                  [SNew(SHorizontalBox) +
                                   SHorizontalBox::Slot().FillWidth(1.08F).Padding(0.0F, 0.0F, 7.0F,
                                                                                   0.0F)
                                       [SNew(SBorder)
                                            .BorderImage(WhiteBrush())
                                            .BorderBackgroundColor(WarmWhiteSoft)
                                            .Padding(FMargin(18.0F, 15.0F))
                                                [SNew(SVerticalBox) +
                                                 SVerticalBox::Slot().AutoHeight()
                                                     [SNew(SHorizontalBox) +
                                                      SHorizontalBox::Slot().FillWidth(1.0F)
                                                          [SNew(STextBlock)
                                                               .Text(this,
                                                                     &SLingTuSimRuntimeHUD::
                                                                         GetSelectionPositionText)
                                                               .Font(Font(10, true))
                                                               .ColorAndOpacity(MutedInk)] +
                                                      SHorizontalBox::Slot().AutoWidth()
                                                          [SNew(STextBlock)
                                                               .Text(
                                                                   this,
                                                                   &SLingTuSimRuntimeHUD::
                                                                       GetSelectionAvailabilityText)
                                                               .Font(Font(10, true))
                                                               .ColorAndOpacity(
                                                                   this,
                                                                   &SLingTuSimRuntimeHUD::
                                                                       GetSelectionAvailabilityColor)]] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 9.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetSelectionTitleText)
                                                          .Font(Font(15, true))
                                                          .ColorAndOpacity(Ink)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 5.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this,
                                                                &SLingTuSimRuntimeHUD::
                                                                    GetSelectionDescriptionText)
                                                          .Font(Font(10))
                                                          .ColorAndOpacity(MutedInk)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 10.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetSelectionPackagesText)
                                                          .Font(Font(10, true))
                                                          .ColorAndOpacity(Ink)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 6.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetSelectionAssetsText)
                                                          .Font(Font(10))
                                                          .ColorAndOpacity(MutedInk)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().FillHeight(
                                                     1.0F)[SNew(SSpacer)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 6.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetSelectionStatusText)
                                                          .Font(Font(10, true))
                                                          .ColorAndOpacity(
                                                              this, &SLingTuSimRuntimeHUD::
                                                                        GetSelectionStatusColor)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 12.0F, 0.0F, 0.0F)
                                                     [SNew(SHorizontalBox) +
                                                      SHorizontalBox::Slot().AutoWidth()
                                                          [SNew(SButton)
                                                               .ButtonColorAndOpacity_Lambda(
                                                                   [this]() {
                                                                     return CanBrowseSessions()
                                                                                ? TealAccent
                                                                                : DisabledButton;
                                                                   })
                                                               .IsEnabled(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              CanBrowseSessions)
                                                               .OnClicked(
                                                                   this,
                                                                   &SLingTuSimRuntimeHUD::
                                                                       HandlePreviousSelection)
                                                                   [StaticLabel(
                                                                       TEXT("↑  "
                                                                            "上一个"),
                                                                       10, FLinearColor::White)]] +
                                                      SHorizontalBox::Slot().FillWidth(
                                                          1.0F)[SNew(SSpacer)] +
                                                      SHorizontalBox::Slot().AutoWidth()
                                                          [SNew(SButton)
                                                               .ButtonColorAndOpacity_Lambda(
                                                                   [this]() {
                                                                     return CanConfirmSelection()
                                                                                ? TealAccent
                                                                                : DisabledButton;
                                                                   })
                                                               .IsEnabled(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              CanConfirmSelection)
                                                               .OnClicked(
                                                                   this, &SLingTuSimRuntimeHUD::
                                                                             HandleConfirmSelection)
                                                                   [StaticLabel(
                                                                       TEXT("确认会"
                                                                            "话"),
                                                                       10, FLinearColor::White)]] +
                                                      SHorizontalBox::Slot().FillWidth(
                                                          1.0F)[SNew(SSpacer)] +
                                                      SHorizontalBox::Slot().AutoWidth()
                                                          [SNew(SButton)
                                                               .ButtonColorAndOpacity_Lambda(
                                                                   [this]() {
                                                                     return CanBrowseSessions()
                                                                                ? TealAccent
                                                                                : DisabledButton;
                                                                   })
                                                               .IsEnabled(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              CanBrowseSessions)
                                                               .OnClicked(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              HandleNextSelection)
                                                                   [StaticLabel(
                                                                       TEXT("下一个  "
                                                                            "↓"),
                                                                       10,
                                                                       FLinearColor::White)]]]]] +
                                   SHorizontalBox::Slot().FillWidth(0.92F).Padding(7.0F, 0.0F, 0.0F,
                                                                                   0.0F)
                                       [SNew(SBorder)
                                            .BorderImage(WhiteBrush())
                                            .BorderBackgroundColor(WarmWhiteSoft)
                                            .Padding(FMargin(18.0F, 15.0F))
                                                [SNew(SVerticalBox) +
                                                 SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                                     TEXT("素材库  ·  只读审核"), 11,
                                                     AmberAccent)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 8.0F, 0.0F,
                                                     0.0F)[SNew(STextBlock)
                                                               .Text(this,
                                                                     &SLingTuSimRuntimeHUD::
                                                                         GetAssetReviewPositionText)
                                                               .Font(Font(10, true))
                                                               .ColorAndOpacity(MutedInk)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 7.0F, 0.0F, 0.0F)[AssetCardGrid] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 7.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetAssetReviewTitleText)
                                                          .Font(Font(14, true))
                                                          .ColorAndOpacity(Ink)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 4.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this,
                                                                &SLingTuSimRuntimeHUD::
                                                                    GetAssetReviewDescriptionText)
                                                          .Font(Font(10))
                                                          .ColorAndOpacity(MutedInk)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 9.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this,
                                                                &SLingTuSimRuntimeHUD::
                                                                    GetAssetReviewDispositionText)
                                                          .Font(Font(11, true))
                                                          .ColorAndOpacity(
                                                              this,
                                                              &SLingTuSimRuntimeHUD::
                                                                  GetAssetReviewDispositionColor)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 4.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetAssetReviewReasonText)
                                                          .Font(Font(10))
                                                          .ColorAndOpacity(MutedInk)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 8.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this,
                                                                &SLingTuSimRuntimeHUD::
                                                                    GetAssetReviewClassStageText)
                                                          .Font(Font(10, true))
                                                          .ColorAndOpacity(Ink)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 4.0F, 0.0F,
                                                     0.0F)[SNew(STextBlock)
                                                               .Text(this,
                                                                     &SLingTuSimRuntimeHUD::
                                                                         GetAssetReviewEvidenceText)
                                                               .Font(Font(10))
                                                               .ColorAndOpacity(MutedInk)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 8.0F, 0.0F, 0.0F)
                                                     [SNew(STextBlock)
                                                          .Text(this, &SLingTuSimRuntimeHUD::
                                                                          GetAssetReviewPolicyText)
                                                          .Font(Font(9, true))
                                                          .ColorAndOpacity(TealAccent)
                                                          .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().FillHeight(
                                                     1.0F)[SNew(SSpacer)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 8.0F, 0.0F,
                                                     0.0F)[SNew(STextBlock)
                                                               .Text(FText::FromString(
                                                                   TEXT("浏览素材不会改动已编译的 "
                                                                        "SessionBundle。")))
                                                               .Font(Font(10, true))
                                                               .ColorAndOpacity(AmberAccent)
                                                               .AutoWrapText(true)] +
                                                 SVerticalBox::Slot().AutoHeight().Padding(
                                                     0.0F, 12.0F, 0.0F, 0.0F)
                                                     [SNew(SHorizontalBox) +
                                                      SHorizontalBox::Slot().AutoWidth()
                                                          [SNew(SButton)
                                                               .ButtonColorAndOpacity_Lambda(
                                                                   [this]() {
                                                                     return CanBrowseAssets()
                                                                                ? TealAccent
                                                                                : DisabledButton;
                                                                   })
                                                               .IsEnabled(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              CanBrowseAssets)
                                                               .OnClicked(
                                                                   this,
                                                                   &SLingTuSimRuntimeHUD::
                                                                       HandlePreviousAssetReview)
                                                                   [StaticLabel(
                                                                       TEXT("←  "
                                                                            "上一项"),
                                                                       10, FLinearColor::White)]] +
                                                      SHorizontalBox::Slot().FillWidth(
                                                          1.0F)[SNew(SSpacer)] +
                                                      SHorizontalBox::Slot().AutoWidth()
                                                          [SNew(SButton)
                                                               .ButtonColorAndOpacity_Lambda(
                                                                   [this]() {
                                                                     return CanBrowseAssets()
                                                                                ? TealAccent
                                                                                : DisabledButton;
                                                                   })
                                                               .IsEnabled(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              CanBrowseAssets)
                                                               .OnClicked(this,
                                                                          &SLingTuSimRuntimeHUD::
                                                                              HandleNextAssetReview)
                                                                   [StaticLabel(
                                                                       TEXT("下一项  →"), 10,
                                                                       FLinearColor::White)]]]]]] +
                              SVerticalBox::Slot().AutoHeight().Padding(0.0F, 10.0F, 0.0F, 0.0F)
                                  [SNew(SHorizontalBox) +
                                   SHorizontalBox::Slot().FillWidth(1.0F).VAlign(VAlign_Center)
                                       [StaticLabel(TEXT("↑/↓ 选择会话  ·  ←/→ 浏览素材  "
                                                         "·  ENTER 确认会话"),
                                                    10, MutedInk)] +
                                   SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
                                       [SNew(SButton)
                                            .Visibility(this,
                                                        &SLingTuSimRuntimeHUD::GetLogoutVisibility)
                                            .IsEnabled(this, &SLingTuSimRuntimeHUD::CanLogout)
                                            .OnClicked(this, &SLingTuSimRuntimeHUD::HandleLogout)
                                                [StaticLabel(TEXT("←  更换操作员"), 9, Ink)]]]]]]

       + SOverlay::Slot()
             .HAlign(HAlign_Center)
             .VAlign(VAlign_Bottom)
             .Padding(FMargin(20.0F, 0.0F, 20.0F, 20.0F))
                 [SNew(SBorder)
                      .Visibility(this, &SLingTuSimRuntimeHUD::GetNonPauseVisibility)
                      .BorderImage(WhiteBrush())
                      .BorderBackgroundColor(WarmWhiteGlass)
                      .Padding(FMargin(
                          15.0F, 7.0F))[SNew(STextBlock)
                                            .Text(this, &SLingTuSimRuntimeHUD::GetModeHintText)
                                            .Font(Font(9, true))
                                            .ColorAndOpacity(Ink)]]];

  if (bFrontEndLoginRequired && !LoginModel->IsLoggedIn() && LocalWorkspaceButton.IsValid()) {
    FocusFrontEndLoginCTA();
  }
}

void SLingTuSimRuntimeHUD::FocusFrontEndLoginCTA() {
  if (!bFrontEndLoginRequired || LoginModel->IsLoggedIn() || !LocalWorkspaceButton.IsValid()) {
    return;
  }

  Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  const TWeakPtr<SButton> WeakLoginButton = LocalWorkspaceButton;
  RegisterActiveTimer(
      0.0F, FWidgetActiveTimerDelegate::CreateLambda([WeakLoginButton](double, float) {
        if (FSlateApplication::IsInitialized()) {
          if (const TSharedPtr<SButton> LoginButton = WeakLoginButton.Pin();
              LoginButton.IsValid() && LoginButton->GetVisibility().IsVisible() &&
              LoginButton->IsEnabled()) {
            FSlateApplication::Get().SetKeyboardFocus(LoginButton, EFocusCause::SetDirectly);
          }
        }
        return EActiveTimerReturnType::Stop;
      }));
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildLoginScreen() {
  return SNew(SOverlay)

         + SOverlay::Slot()
               .HAlign(HAlign_Left)
               .VAlign(VAlign_Top)[SNew(SVerticalBox) +
                                   SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                       TEXT("LINGTU / FIELD OPERATIONS"), 13, FieldAmber)] +
                                   SVerticalBox::Slot().AutoHeight().Padding(0.0F, 5.0F, 0.0F,
                                                                             0.0F)[StaticLabel(
                                       TEXT("森林巡检仿真系统  ·  本地离线"), 10, FrontEndMuted)]]

         +
         SOverlay::Slot()
             .HAlign(HAlign_Left)
             .VAlign(VAlign_Bottom)[SNew(SBox).WidthOverride(
                 610.0F)[SNew(SVerticalBox) +
                         SVerticalBox::Slot()
                             .AutoHeight()[StaticLabel(TEXT("林地行动终端"), 34, FrontEndText)] +
                         SVerticalBox::Slot().AutoHeight().Padding(0.0F, 10.0F, 0.0F, 0.0F)
                             [SNew(STextBlock)
                                  .Text(
                                      FText::FromString(TEXT("部署机器狗进入森林巡检场景，选择已编"
                                                             "译任务，并以只读方式核验场景素材。")))
                                  .Font(Font(12))
                                  .ColorAndOpacity(FrontEndMuted)
                                  .AutoWrapText(true)] +
                         SVerticalBox::Slot().AutoHeight().Padding(0.0F, 24.0F, 0.0F, 0.0F)
                             [SNew(SBorder)
                                  .BorderImage(WhiteBrush())
                                  .BorderBackgroundColor(FrontEndGlass)
                                  .Padding(FMargin(18.0F, 15.0F))
                                      [SNew(SHorizontalBox) +
                                       SHorizontalBox::Slot().FillWidth(1.0F).VAlign(VAlign_Center)
                                           [SNew(SVerticalBox) +
                                            SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                                TEXT("本地操作员档案"), 9, FrontEndMuted)] +
                                            SVerticalBox::Slot().AutoHeight().Padding(0.0F, 5.0F,
                                                                                      20.0F, 0.0F)
                                                [SAssignNew(OperatorNameTextBox, SEditableTextBox)
                                                     .Text(FText::FromString(PendingOperatorName))
                                                     .HintText(FText::FromString(TEXT("操作员名"
                                                                                      "称")))
                                                     .ForegroundColor(FrontEndText)
                                                     .BackgroundColor(
                                                         FLinearColor(0.02F, 0.03F, 0.03F, 0.92F))
                                                     .MaximumLength(64)
                                                     .IsEnabled_Lambda(
                                                         [this]() { return LoginModel->CanEdit(); })
                                                     .OnTextChanged(
                                                         this, &SLingTuSimRuntimeHUD::
                                                                   HandleOperatorNameChanged)]] +
                                       SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
                                           [SAssignNew(LocalWorkspaceButton, SButton)
                                                .ButtonColorAndOpacity_Lambda([this]() {
                                                  return CanEnterLocalWorkspace() ? FieldAmber
                                                                                  : DisabledButton;
                                                })
                                                .ForegroundColor(FrontEndText)
                                                .ContentPadding(FMargin(28.0F, 13.0F))
                                                .IsEnabled(this, &SLingTuSimRuntimeHUD::
                                                                     CanEnterLocalWorkspace)
                                                .OnClicked(this, &SLingTuSimRuntimeHUD::
                                                                     HandleEnterLocalWorkspace)
                                                    [StaticLabel(TEXT("开始本地任务  ▶"), 12,
                                                                 FrontEndText)]]]] +
                         SVerticalBox::Slot().AutoHeight().Padding(
                             0.0F, 12.0F, 0.0F,
                             0.0F)[SNew(STextBlock)
                                       .Text(this, &SLingTuSimRuntimeHUD::GetLoginCatalogStatusText)
                                       .Font(Font(9, true))
                                       .ColorAndOpacity(FieldTeal)
                                       .AutoWrapText(true)] +
                         SVerticalBox::Slot().AutoHeight().Padding(
                             0.0F, 5.0F, 0.0F,
                             0.0F)[SNew(STextBlock)
                                       .Text(this, &SLingTuSimRuntimeHUD::GetLoginFeedbackText)
                                       .Font(Font(10, true))
                                       .ColorAndOpacity(AlertInk)
                                       .AutoWrapText(true)] +
                         SVerticalBox::Slot().AutoHeight().Padding(
                             0.0F, 20.0F, 0.0F,
                             0.0F)[StaticLabel(TEXT("A / ENTER  确认   ·   当前版本不执行在线认证"),
                                               9, FrontEndMuted)]]]

         + SOverlay::Slot()
               .HAlign(HAlign_Right)
               .VAlign(VAlign_Bottom)
                   [SNew(SBorder)
                        .BorderImage(WhiteBrush())
                        .BorderBackgroundColor(FrontEndGlassSoft)
                        .Padding(FMargin(15.0F, 11.0F))
                            [SNew(SVerticalBox) +
                             SVerticalBox::Slot()
                                 .AutoHeight()[StaticLabel(TEXT("LOCAL SESSION"), 9, FieldAmber)] +
                             SVerticalBox::Slot().AutoHeight().Padding(0.0F, 3.0F,
                                                                       0.0F, 0.0F)[StaticLabel(
                                 TEXT("本地数据  ·  无登录令牌  ·  未联网"), 9, FrontEndText)]]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildWorkspaceScreen() {
  const TSharedRef<STextBlock> DeployButtonLabel = StaticLabel(TEXT("开始任务"), 12, FrontEndText);
  return SNew(SOverlay)

         + SOverlay::Slot().HAlign(HAlign_Fill).VAlign(VAlign_Fill)[BuildAssetHeroPreview()]

         + SOverlay::Slot()
               .HAlign(HAlign_Fill)
               .VAlign(VAlign_Top)
               .Padding(FMargin(34.0F, 26.0F))
                   [SNew(SHorizontalBox) +
                    SHorizontalBox::Slot().AutoWidth().VAlign(
                        VAlign_Center)[SNew(SVerticalBox) +
                                       SVerticalBox::Slot().AutoHeight()[StaticLabel(
                                           TEXT("灵途  ·  林地行动"), 13, FieldAmber)] +
                                       SVerticalBox::Slot().AutoHeight().Padding(0.0F, 3.0F, 0.0F,
                                                                                 0.0F)[StaticLabel(
                                           TEXT("部署准备  /  素材审阅"), 9, FrontEndMuted)]] +
                    SHorizontalBox::Slot().FillWidth(1.0F)[SNew(SSpacer)] +
                    SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
                        [SNew(SVerticalBox) +
                         SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
                             [SNew(STextBlock)
                                  .Text(this, &SLingTuSimRuntimeHUD::GetSelectionStatusText)
                                  .Font(Font(10, true))
                                  .ColorAndOpacity(
                                      this, &SLingTuSimRuntimeHUD::GetSelectionStatusColor)] +
                         SVerticalBox::Slot()
                             .AutoHeight()
                             .HAlign(HAlign_Right)
                             .Padding(0.0F, 4.0F, 0.0F, 0.0F)
                                 [SNew(STextBlock)
                                      .Text(this,
                                            &SLingTuSimRuntimeHUD::GetAssetReviewDispositionText)
                                      .Font(Font(9, true))
                                      .ColorAndOpacity(this, &SLingTuSimRuntimeHUD::
                                                                 GetAssetReviewDispositionColor)]] +
                    SHorizontalBox::Slot()
                        .AutoWidth()
                        .Padding(18.0F, 0.0F, 0.0F, 0.0F)
                        .VAlign(VAlign_Center)
                            [SNew(SButton)
                                 .ButtonColorAndOpacity(FLinearColor::Transparent)
                                 .Visibility(this, &SLingTuSimRuntimeHUD::GetLogoutVisibility)
                                 .IsEnabled(this, &SLingTuSimRuntimeHUD::CanLogout)
                                 .OnClicked(this, &SLingTuSimRuntimeHUD::HandleLogout)[StaticLabel(
                                     TEXT("切换档案"), 9, FrontEndText)]]]

         +
         SOverlay::Slot()
             .HAlign(HAlign_Left)
             .VAlign(VAlign_Bottom)
             .Padding(FMargin(38.0F, 0.0F, 0.0F, 184.0F))
                 [SNew(SVerticalBox) +
                  SVerticalBox::Slot().AutoHeight()[StaticLabel(TEXT("行动简报"), 10, FieldAmber)] +
                  SVerticalBox::Slot().AutoHeight().Padding(
                      0.0F, 5.0F, 0.0F,
                      0.0F)[SNew(STextBlock)
                                .Text(this, &SLingTuSimRuntimeHUD::GetSelectionPositionText)
                                .Font(Font(9, true))
                                .ColorAndOpacity(FrontEndMuted)] +
                  SVerticalBox::Slot().AutoHeight().Padding(
                      0.0F, 8.0F, 0.0F,
                      0.0F)[SNew(STextBlock)
                                .Text(this, &SLingTuSimRuntimeHUD::GetSelectionTitleText)
                                .Font(Font(27, true))
                                .ColorAndOpacity(FrontEndText)] +
                  SVerticalBox::Slot().AutoHeight().Padding(
                      0.0F, 5.0F, 0.0F,
                      0.0F)[SNew(STextBlock)
                                .Text(this, &SLingTuSimRuntimeHUD::GetSelectionDescriptionText)
                                .Font(Font(11))
                                .ColorAndOpacity(FrontEndMuted)] +
                  SVerticalBox::Slot().AutoHeight().Padding(0.0F, 10.0F, 0.0F, 0.0F)[StaticLabel(
                      TEXT("巡检场景  /  任务配置"), 9, FieldAmber)] +
                  SVerticalBox::Slot().AutoHeight().Padding(
                      0.0F, 4.0F, 0.0F,
                      0.0F)[SNew(STextBlock)
                                .Text(this, &SLingTuSimRuntimeHUD::GetSelectionPackagesText)
                                .Font(Font(10, true))
                                .ColorAndOpacity(FrontEndText)] +
                  SVerticalBox::Slot().AutoHeight().Padding(0.0F, 16.0F, 0.0F, 0.0F)
                      [SNew(SHorizontalBox) +
                       SHorizontalBox::Slot().AutoWidth()
                           [SNew(SButton)
                                .ButtonColorAndOpacity(FLinearColor::Transparent)
                                .IsEnabled(this, &SLingTuSimRuntimeHUD::CanBrowseSessions)
                                .OnClicked(this, &SLingTuSimRuntimeHUD::HandlePreviousSelection)
                                    [StaticLabel(TEXT("◀"), 12, FrontEndText)]] +
                       SHorizontalBox::Slot().AutoWidth().Padding(
                           10.0F,
                           0.0F)[SAssignNew(ConfirmSelectionButton, SButton)
                                     .ButtonColorAndOpacity_Lambda([this]() {
                                       return CanConfirmSelection() ? FieldAmber : DisabledButton;
                                     })
                                     .ContentPadding(FMargin(24.0F, 11.0F))
                                     .IsEnabled(this, &SLingTuSimRuntimeHUD::CanConfirmSelection)
                                     .OnClicked(this, &SLingTuSimRuntimeHUD::HandleConfirmSelection)
                                         [DeployButtonLabel]] +
                       SHorizontalBox::Slot().AutoWidth()
                           [SNew(SButton)
                                .ButtonColorAndOpacity(FLinearColor::Transparent)
                                .IsEnabled(this, &SLingTuSimRuntimeHUD::CanBrowseSessions)
                                .OnClicked(this, &SLingTuSimRuntimeHUD::HandleNextSelection)
                                    [StaticLabel(TEXT("▶"), 12, FrontEndText)]]]]

         +
         SOverlay::Slot()
             .HAlign(HAlign_Right)
             .VAlign(VAlign_Bottom)
             .Padding(FMargin(
                 0.0F, 0.0F, 36.0F,
                 186.0F))[SNew(SVerticalBox) +
                          SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
                              [SNew(STextBlock)
                                   .Text(this, &SLingTuSimRuntimeHUD::GetAssetReviewPolicyText)
                                   .Font(Font(9, true))
                                   .ColorAndOpacity(FieldTeal)] +
                          SVerticalBox::Slot()
                              .AutoHeight()
                              .HAlign(HAlign_Right)
                              .Padding(
                                  0.0F, 5.0F, 0.0F,
                                  0.0F)[SNew(STextBlock)
                                            .Text(this,
                                                  &SLingTuSimRuntimeHUD::GetAssetReviewEvidenceText)
                                            .Font(Font(8))
                                            .ColorAndOpacity(FrontEndMuted)]]

         + SOverlay::Slot()
               .HAlign(HAlign_Center)
               .VAlign(VAlign_Bottom)
               .Padding(
                   FMargin(0.0F, 0.0F, 0.0F,
                           47.0F))[SNew(SVerticalBox) +
                                   SVerticalBox::Slot()
                                       .AutoHeight()
                                       .HAlign(HAlign_Center)
                                       .Padding(0.0F, 0.0F, 0.0F, 7.0F)[StaticLabel(
                                           TEXT("离线素材缩略图  ·  非当前游戏画面  ·  未资格化"),
                                           8, FrontEndMuted)] +
                                   SVerticalBox::Slot().AutoHeight()[BuildAssetFilmstrip()]]

         +
         SOverlay::Slot()
             .HAlign(HAlign_Fill)
             .VAlign(VAlign_Bottom)
             .Padding(FMargin(
                 34.0F, 0.0F,
                 34.0F,
                 18.0F))[SNew(SHorizontalBox) +
                         SHorizontalBox::Slot().FillWidth(1.0F)[StaticLabel(
                             TEXT("A / ENTER  开始任务   ·   B / ESC  返回   ·   LB / RB  "
                                  "浏览素材   ·   方向键选择"),
                             9, FrontEndMuted)] +
                         SHorizontalBox::Slot()
                             .AutoWidth()[SNew(STextBlock)
                                              .Text(this, &SLingTuSimRuntimeHUD::GetMenuRequestText)
                                              .Font(Font(9, true))
                                              .ColorAndOpacity(FrontEndMuted)]];
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildAssetHeroPreview() {
  return SNew(SOverlay);
}

TSharedRef<SWidget> SLingTuSimRuntimeHUD::BuildAssetFilmstrip() {
  TSharedRef<SHorizontalBox> Filmstrip = SNew(SHorizontalBox);
  const FAssetReviewCatalog &Catalog = AssetReviewModel->GetCatalog();
  const int32 VisibleAssetCards = FMath::Min(Catalog.Cards.Num(), 5);
  for (int32 CardIndex = 0; CardIndex < VisibleAssetCards; ++CardIndex) {
    const FAssetReviewCard &Card = Catalog.Cards[CardIndex];
    const FString CardId = Card.Id;
    const FSlateBrush *PreviewBrush = GetAssetPreviewBrush(CardId);
    TSharedRef<SOverlay> PreviewLayer = SNew(SOverlay);
    PreviewLayer->AddSlot()[SNew(SImage)
                                .Image(PreviewBrush != nullptr ? PreviewBrush : WhiteBrush())
                                .ColorAndOpacity_Lambda([this, CardId]() {
                                  return GetAssetFilmstripPreviewColor(CardId);
                                })];
    if (PreviewBrush == nullptr) {
      PreviewLayer->AddSlot()
          .HAlign(HAlign_Center)
          .VAlign(VAlign_Center)[StaticLabel(TEXT("无预览"), 9, FrontEndMuted)];
    }

    TSharedRef<SVerticalBox> CardContent = SNew(SVerticalBox);
    CardContent->AddSlot()
        .AutoHeight()[SNew(SBox).WidthOverride(138.0F).HeightOverride(58.0F)[PreviewLayer]];
    CardContent->AddSlot().AutoHeight().Padding(1.0F, 5.0F, 1.0F,
                                                0.0F)[SNew(STextBlock)
                                                          .Text(FText::FromString(Card.Title))
                                                          .Font(Font(9, true))
                                                          .ColorAndOpacity(FrontEndText)];
    CardContent->AddSlot().AutoHeight().Padding(
        1.0F, 2.0F, 1.0F,
        0.0F)[SNew(STextBlock)
                  .Text(FText::FromString(AssetReviewDispositionLabel(Card.Disposition)))
                  .Font(Font(7, true))
                  .ColorAndOpacity(FieldAmber)];

    const TSharedRef<SWidget> CardButton =
        SNew(SButton)
            .ButtonColorAndOpacity_Lambda(
                [this, CardId]() { return GetAssetFilmstripCardColor(CardId); })
            .IsEnabled_Lambda([this, CardId]() { return CanSelectAssetReview(CardId); })
            .OnClicked_Lambda([this, CardId]() { return HandleSelectAssetReview(CardId); })
            .ContentPadding(FMargin(
                2.0F))[SNew(SBorder)
                           .BorderImage(WhiteBrush())
                           .BorderBackgroundColor_Lambda(
                               [this, CardId]() { return GetAssetFilmstripCardColor(CardId); })
                           .Padding(FMargin(2.0F))[SNew(SBorder)
                                                       .BorderImage(WhiteBrush())
                                                       .BorderBackgroundColor(FrontEndVoid)
                                                       .Padding(FMargin(3.0F))[CardContent]]];

    Filmstrip->AddSlot().AutoWidth().Padding(CardIndex == 0 ? 0.0F : 4.0F, 0.0F,
                                             CardIndex + 1 == VisibleAssetCards ? 0.0F : 4.0F,
                                             0.0F)[CardButton];
  }

  if (VisibleAssetCards == 0) {
    Filmstrip->AddSlot().AutoWidth()[StaticLabel(TEXT("素材审核目录不可用"), 10, FrontEndMuted)];
  }

  return Filmstrip;
}

void SLingTuSimRuntimeHUD::Tick(const FGeometry &AllottedGeometry, const double InCurrentTime,
                                const float InDeltaTime) {
  SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);
  CachedStatus = FRuntimeUIStatusReader::Read(World.Get(), LocalState.Get());
}

bool SLingTuSimRuntimeHUD::IsOperatorNameEditing() const {
  return OperatorNameTextBox.IsValid() &&
         (OperatorNameTextBox->HasKeyboardFocus() || OperatorNameTextBox->HasFocusedDescendants());
}

FText SLingTuSimRuntimeHUD::GetIdentityText() const {
  if (!CachedStatus.bSessionAvailable) {
    return FText::FromString(TEXT("SESSION unavailable"));
  }
  const FString Run =
      CachedStatus.bControlBindingAvailable ? CachedStatus.RunId : TEXT("unavailable");
  const FString Reset =
      CachedStatus.bControlBindingAvailable
          ? FString::Printf(TEXT("%llu"),
                            static_cast<unsigned long long>(CachedStatus.ResetGeneration))
          : TEXT("unavailable");
  return FText::FromString(
      FString::Printf(TEXT("RUN %s  ·  SESSION %s  ·  MODEL %llu  ·  RESET %s"), *Run,
                      *CachedStatus.SessionId.Left(8),
                      static_cast<unsigned long long>(CachedStatus.ModelGeneration), *Reset));
}

FText SLingTuSimRuntimeHUD::GetReadinessText() const {
  if (!CachedStatus.bFullStatusAvailable) {
    return FText::FromString(FString::Printf(
        TEXT("SESSION %s   ·   CONTROL %s   ·   VISUAL %s   ·   FULL STATUS UNAVAILABLE"),
        *CachedStatus.SessionState, *CachedStatus.ControlState, *CachedStatus.VisualState));
  }
  const LingTuSim::FControlStatusReadiness &Readiness = CachedStatus.FullStatus.Readiness;
  return FText::FromString(FString::Printf(
      TEXT("SESSION %s  ·  PHYSICS %s  ·  CONTROL %s  ·  VISUAL %s  ·  SENSORS %s  ·  STATUS %s"),
      *CachedStatus.SessionState, BindingStateName(Readiness.Physics.State),
      BindingStateName(Readiness.Control.State), BindingStateName(Readiness.Visual.State),
      BindingStateName(Readiness.Sensors.State),
      CachedStatus.bFullStatusFresh ? TEXT("CURRENT") : TEXT("STALE")));
}

FText SLingTuSimRuntimeHUD::GetModeText() const {
  const TCHAR *Name = TEXT("DRIVE");
  switch (ModeController->GetMode()) {
    case ERuntimeUIMode::Build:
      Name = TEXT("BUILD / READ-ONLY");
      break;
    case ERuntimeUIMode::Tactical:
      Name = TEXT("TACTICAL");
      break;
    case ERuntimeUIMode::Pause:
      Name = TEXT("MENU");
      break;
    case ERuntimeUIMode::Drive:
    default:
      break;
  }
  return FText::FromString(FString::Printf(TEXT("%s"), Name));
}

FText SLingTuSimRuntimeHUD::GetInputText() const {
  if (!CachedStatus.bFullStatusAvailable) {
    return FText::FromString(TEXT("OWNER unavailable  ·  authoritative input unavailable"));
  }
  return FText::FromString(FString::Printf(
      TEXT("OWNER %s  ·  DEADMAN %s  ·  AGE %.1f ms  ·  FOCUS %s"),
      *CachedStatus.FullStatus.Runtime.ControlOwner,
      CachedStatus.FullStatus.Runtime.bDeadman ? TEXT("HELD") : TEXT("RELEASED"),
      static_cast<double>(CachedStatus.FullStatus.Runtime.SampleAgeNs) / 1'000'000.0,
      CachedStatus.bInputObserved && CachedStatus.RequestedInput.bViewportFocused ? TEXT("ON")
                                                                                  : TEXT("OFF")));
}

FText SLingTuSimRuntimeHUD::GetRequestedAxesText() const {
  if (!CachedStatus.bFullStatusAvailable ||
      !CachedStatus.FullStatus.Motion.RequestedAxes.bAvailable) {
    return FText::FromString(TEXT("REQUESTED  F —  L —  YAW —"));
  }
  const LingTuSim::FControlStatusRequestedAxes &Input =
      CachedStatus.FullStatus.Motion.RequestedAxes;
  return FText::FromString(FString::Printf(TEXT("REQUESTED  F %+.2f  ·  L %+.2f  ·  YAW %+.2f"),
                                           Input.Forward, Input.Left, Input.YawLeft));
}

FText SLingTuSimRuntimeHUD::GetAckText() const {
  if (!CachedStatus.bFullStatusAvailable) {
    return FText::FromString(TEXT("CONTROL STATUS  unavailable"));
  }
  const FString Reason = CachedStatus.FullStatus.Reason.IsEmpty()
                             ? FString()
                             : TEXT("  ·  ") + CachedStatus.FullStatus.Reason;
  return FText::FromString(FString::Printf(
      TEXT("CONTROL STATUS  %s  ·  server #%llu%s"), AckStatusName(CachedStatus.FullStatus.Status),
      static_cast<unsigned long long>(CachedStatus.FullStatus.ServerStatusSequence), *Reason));
}

FText SLingTuSimRuntimeHUD::GetAcceptedMotionText() const {
  if (!CachedStatus.bFullStatusAvailable ||
      !CachedStatus.FullStatus.Motion.AdmittedTwist.bAvailable) {
    return FText::FromString(TEXT("ADMITTED  VX —  VY —  WZ —"));
  }
  const LingTuSim::FControlStatusVelocity &Twist = CachedStatus.FullStatus.Motion.AdmittedTwist;
  return FText::FromString(FString::Printf(TEXT("ADMITTED  VX %+.2f  ·  VY %+.2f  ·  WZ %+.2f"),
                                           Twist.LinearX, Twist.LinearY, Twist.AngularZ));
}

FText SLingTuSimRuntimeHUD::GetObservedVelocityText() const {
  if (!CachedStatus.bFullStatusAvailable ||
      !CachedStatus.FullStatus.Motion.ObservedBaseVelocity.bAvailable) {
    return FText::FromString(TEXT("OBSERVED  VX —  VY —  WZ —"));
  }
  const LingTuSim::FControlStatusVelocity &Velocity =
      CachedStatus.FullStatus.Motion.ObservedBaseVelocity;
  return FText::FromString(FString::Printf(TEXT("OBSERVED  VX %+.2f  ·  VY %+.2f  ·  WZ %+.2f"),
                                           Velocity.LinearX, Velocity.LinearY, Velocity.AngularZ));
}

FText SLingTuSimRuntimeHUD::GetSensorsText() const {
  if (!CachedStatus.bFullStatusAvailable) {
    return FText::FromString(TEXT("SENSORS  unavailable"));
  }
  FString Summary = TEXT("SENSORS");
  for (const LingTuSim::FControlStatusSensor &Sensor : CachedStatus.FullStatus.Sensors) {
    Summary += FString::Printf(TEXT("  ·  %s #%llu"), *Sensor.StreamId,
                               static_cast<unsigned long long>(Sensor.SampleCount));
  }
  return FText::FromString(Summary);
}

FText SLingTuSimRuntimeHUD::GetRecordingText() const {
  if (!CachedStatus.bFullStatusAvailable) {
    return FText::FromString(TEXT("RECORDING  unavailable  ·  R disabled"));
  }
  const LingTuSim::FControlStatusRecording &Recording = CachedStatus.FullStatus.Recording;
  const TCHAR *Action = CachedStatus.CanRequestRecordStart()        ? TEXT("R START")
                        : CachedStatus.CanRequestRecordStopCommit() ? TEXT("R STOP + COMMIT")
                                                                    : TEXT("R DISABLED");
  return FText::FromString(
      FString::Printf(TEXT("RECORDING  %s  ·  %.2f s  ·  %s"), RecordingStateName(Recording.State),
                      static_cast<double>(Recording.ElapsedSimTimeNs) / 1'000'000'000.0, Action));
}

FText SLingTuSimRuntimeHUD::GetCameraText() const {
  const TCHAR *Echo = CachedStatus.bFullStatusAvailable
                          ? CameraModeName(CachedStatus.FullStatus.UI.CameraMode)
                          : TEXT("UNAVAILABLE");
  return FText::FromString(FString::Printf(TEXT("CAMERA  UE %s  ·  RUNTIME ECHO %s"),
                                           *CachedStatus.ActualCameraMode.ToUpper(), Echo));
}

FText SLingTuSimRuntimeHUD::GetTruthFrameText() const {
  if (!CachedStatus.bLatestAppliedTruthAvailable) {
    return FText::FromString(TEXT("TRUTH FRAME  Unavailable"));
  }
  return FText::FromString(FString::Printf(
      TEXT("SEQ %llu  ·  SIM %.3f s  ·  BASE %s"),
      static_cast<unsigned long long>(CachedStatus.TruthSequence),
      static_cast<double>(CachedStatus.SimTimeNs) / 1'000'000'000.0,
      CachedStatus.ObservedBaseStableId.IsEmpty() ? TEXT("unavailable")
                                                  : *CachedStatus.ObservedBaseStableId));
}

FText SLingTuSimRuntimeHUD::GetModeHintText() const {
  switch (ModeController->GetMode()) {
    case ERuntimeUIMode::Build:
      return FText::FromString(
          TEXT("B  DRIVE   ·   C  CAMERA   ·   R  RECORD   ·   READ-ONLY PREVIEW"));
    case ERuntimeUIMode::Tactical:
      return FText::FromString(TEXT("TAB  DRIVE   ·   C  CAMERA   ·   R  RECORD   ·   ESC  MENU"));
    case ERuntimeUIMode::Pause:
      return FText::FromString(
          TEXT("ESC  RESUME   ·   R  RECORD   ·   X  END SESSION   ·   MOTION RELEASED"));
    case ERuntimeUIMode::Drive:
    default:
      return FText::FromString(
          TEXT("HOLD SHIFT + W/S/A/D/Q/E   ·   C  CAMERA   ·   R  RECORD   ·   B  BUILD   ·   TAB  "
               "TACTICAL   ·   ESC  MENU"));
  }
}

FText SLingTuSimRuntimeHUD::GetMenuRequestText() const {
  if (bFrontEndLoginRequired && LoginModel->IsLoggedIn()) {
    return FText::FromString(FString::Printf(TEXT("档案  %s  ·  本地模式  ·  任务素材已载入"),
                                             *LoginModel->GetDisplayName()));
  }
  if (!CachedStatus.bFullStatusAvailable) {
    return FText::FromString(TEXT("任务状态暂不可用"));
  }
  return FText::FromString(FString::Printf(TEXT("任务状态  %s  ·  画面模式  %s"),
                                           AckStatusName(CachedStatus.FullStatus.Status),
                                           CameraModeName(CachedStatus.FullStatus.UI.CameraMode)));
}

FText SLingTuSimRuntimeHUD::GetFrontEndIdentityText() const {
  return FText::FromString(bFrontEndLoginRequired ? TEXT("灵途仿真工作台")
                                                  : TEXT("INSPECTION SESSION SELECTOR"));
}

FText SLingTuSimRuntimeHUD::GetLoginCatalogStatusText() const {
  const int32 SessionCount = SelectionModel->GetCatalog().Options.Num();
  const FAssetReviewCatalog &Assets = AssetReviewModel->GetCatalog();
  if (SessionCount <= 0) {
    return FText::FromString(TEXT("BLOCKED  ·  VERIFIED SESSION CATALOG UNAVAILABLE"));
  }
  return FText::FromString(
      FString::Printf(TEXT("READY  ·  %d SESSION%s  ·  %d ASSET REVIEW CARD%s"), SessionCount,
                      SessionCount == 1 ? TEXT("") : TEXT("S"), Assets.Cards.Num(),
                      Assets.Cards.Num() == 1 ? TEXT("") : TEXT("S")));
}

FText SLingTuSimRuntimeHUD::GetLoginFeedbackText() const {
  return FText::FromString(LoginFeedback);
}

FText SLingTuSimRuntimeHUD::GetSelectionPositionText() const {
  const int32 Count = SelectionModel->GetCatalog().Options.Num();
  const int32 Index = SelectionModel->GetSelectedIndex();
  return FText::FromString(Count > 0 && Index != INDEX_NONE
                               ? FString::Printf(TEXT("任务 %02d / %02d"), Index + 1, Count)
                               : TEXT("暂无可选任务"));
}

FText SLingTuSimRuntimeHUD::GetSelectionTitleText() const {
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  return FText::FromString(Option != nullptr ? Option->Title : TEXT("任务目录不可用"));
}

FText SLingTuSimRuntimeHUD::GetSelectionDescriptionText() const {
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  return FText::FromString(Option != nullptr ? Option->Description
                                             : TEXT("未加载可用的任务配置。"));
}

FText SLingTuSimRuntimeHUD::GetSelectionAvailabilityText() const {
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  return FText::FromString(Option != nullptr ? GameSelectionAvailabilityName(Option->Availability)
                                             : TEXT("UNAVAILABLE"));
}

FText SLingTuSimRuntimeHUD::GetSelectionPackagesText() const {
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  if (Option == nullptr) {
    return FText::FromString(TEXT("机器人  —\n行动环境  —\n任务模式  —"));
  }
  const FString Scenario = Option->Scenario.IsSet()
                               ? FString::Printf(TEXT("%s v%s"), *Option->Scenario.GetValue().Label,
                                                 *Option->Scenario.GetValue().Version)
                               : TEXT("默认场景");
  return FText::FromString(
      FString::Printf(TEXT("机器人  ·  %s v%s\n行动环境  ·  %s v%s\n任务模式  ·  %s  /  %s"),
                      *Option->Robot.Label, *Option->Robot.Version, *Option->World.Label,
                      *Option->World.Version, *Scenario, *Option->Mode));
}

FText SLingTuSimRuntimeHUD::GetSelectionAssetsText() const {
  const FGameSelectionAssetSummary &Assets = SelectionModel->GetCatalog().AssetSummary;
  if (!Assets.bLibraryAvailable) {
    const FString Reason = Assets.AvailabilityReason.IsEmpty() ? TEXT("asset library not provided")
                                                               : Assets.AvailabilityReason;
    return FText::FromString(FString::Printf(TEXT("ASSETS UNAVAILABLE  ·  %s"), *Reason));
  }

  return FText::FromString(FString::Printf(
      TEXT("ASSETS  ·  %d CATALOG  ·  %d SOURCE  ·  %d QUARANTINED  ·  %d UNVERIFIED"),
      Assets.CatalogPackageCount, Assets.SourceCandidateCount,
      Assets.QuarantinedCount, Assets.UnverifiedCount));
}

FText SLingTuSimRuntimeHUD::GetSelectionStatusText() const {
  if (!SelectionFeedback->IsEmpty()) {
    return FText::FromString(*SelectionFeedback);
  }
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  if (Option == nullptr) {
    return FText::FromString(TEXT("暂无可部署的任务"));
  }
  if (Option->Availability != EGameSelectionAvailability::Runnable) {
    return FText::FromString(TEXT("当前任务尚未满足运行条件"));
  }
  if (!bSelectionIntentConfigured) {
    return FText::FromString(TEXT("任务交接路径未配置"));
  }
  if (SelectionModel->GetConfirmedOptionId() == Option->Id) {
    return FText::FromString(TEXT("任务已选定  ·  等待进入"));
  }
  FString Blocker;
  return FText::FromString(SelectionModel->CanConfirm(Blocker)
                               ? TEXT("任务已就绪  ·  会话配置已校验")
                               : TEXT("任务暂不可部署"));
}

FText SLingTuSimRuntimeHUD::GetAssetReviewPositionText() const {
  const FAssetReviewCatalog &Catalog = AssetReviewModel->GetCatalog();
  const int32 Index = AssetReviewModel->GetCurrentIndex();
  return FText::FromString(
      Catalog.bAvailable && Catalog.Cards.IsValidIndex(Index)
          ? FString::Printf(TEXT("素材 %02d / %02d"), Index + 1, Catalog.Cards.Num())
          : TEXT("素材预览不可用"));
}

FText SLingTuSimRuntimeHUD::GetAssetReviewTitleText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  return FText::FromString(Card != nullptr ? Card->Title : TEXT("Asset review unavailable"));
}

FText SLingTuSimRuntimeHUD::GetAssetReviewDescriptionText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  if (Card != nullptr) {
    return FText::FromString(Card->Description);
  }
  const FString &Reason = AssetReviewModel->GetCatalog().AvailabilityReason;
  return FText::FromString(
      Reason.IsEmpty()
          ? TEXT("No compiled asset review catalog is available for this launcher catalog.")
          : Reason);
}

FText SLingTuSimRuntimeHUD::GetAssetReviewDispositionText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  return FText::FromString(
      Card != nullptr
          ? FString::Printf(TEXT("素材状态  ·  %s"), AssetReviewDispositionLabel(Card->Disposition))
          : TEXT("素材状态  ·  不可用"));
}

FText SLingTuSimRuntimeHUD::GetAssetReviewReasonText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  if (Card == nullptr) {
    return FText::FromString(TEXT("暂时无法读取素材状态。"));
  }
  switch (Card->Disposition) {
    case EAssetReviewDisposition::Unverified:
      return FText::FromString(TEXT("可查看外观；运行前仍需完成验证。"));
    case EAssetReviewDisposition::Quarantined:
      return FText::FromString(TEXT("该素材已隔离，不会加入运行任务。"));
    case EAssetReviewDisposition::ProxyOnly:
      return FText::FromString(TEXT("仅用于代理显示，不代表最终外观。"));
    case EAssetReviewDisposition::Unavailable:
    default:
      return FText::FromString(TEXT("当前素材不可用。"));
  }
}

FText SLingTuSimRuntimeHUD::GetAssetReviewClassStageText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  return FText::FromString(Card != nullptr
                               ? FString::Printf(TEXT("类别  ·  %s\n阶段  ·  %s"),
                                                 *AssetClassLabel(Card->AssetClass),
                                                 *AssetReviewStageLabel(Card->ReviewStage))
                               : TEXT("类别  ·  —\n阶段  ·  —"));
}

FText SLingTuSimRuntimeHUD::GetAssetReviewEvidenceText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  return FText::FromString(FString::Printf(TEXT("技术详情  ·  审阅证据 %d 项  ·  本地记录"),
                                           Card != nullptr ? Card->Evidence.Num() : 0));
}

FText SLingTuSimRuntimeHUD::GetAssetReviewPolicyText() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  if (Card == nullptr) {
    return FText::FromString(TEXT("安全边界暂不可用"));
  }
  const bool bCollisionDisabled = Card->RenderPolicy.UnrealCollisionProfile.IsEmpty() ||
                                  Card->RenderPolicy.UnrealCollisionProfile.Equals(
                                      TEXT("NoCollision"), ESearchCase::IgnoreCase);
  const FString CollisionText =
      bCollisionDisabled
          ? TEXT("碰撞关闭")
          : FString::Printf(TEXT("碰撞配置为 %s"), *Card->RenderPolicy.UnrealCollisionProfile);
  const FString PhysicsAuthority = Card->RenderPolicy.PhysicsAuthority.IsEmpty()
                                       ? TEXT("MuJoCo")
                                       : Card->RenderPolicy.PhysicsAuthority;
  const FString QualificationText = Card->RenderPolicy.bQualifiedVisual
                                        ? TEXT("已取得视觉运行资格")
                                        : TEXT("未取得运行资格  ·  未资格化");
  const FString TechnicalBoundary =
      bCollisionDisabled && PhysicsAuthority.Equals(TEXT("MuJoCo"), ESearchCase::IgnoreCase)
          ? TEXT("UE 无碰撞  ·  MuJoCo 物理权威")
          : TEXT("技术边界按当前配置执行");
  return FText::FromString(FString::Printf(TEXT("%s  ·  物理由 %s 负责\n%s\n%s"), *CollisionText,
                                           *PhysicsAuthority, *QualificationText,
                                           *TechnicalBoundary));
}

FSlateColor SLingTuSimRuntimeHUD::GetAssetReviewDispositionColor() const {
  const FAssetReviewCard *Card = AssetReviewModel->GetCurrentCard();
  if (Card == nullptr) {
    return MutedInk;
  }
  switch (Card->Disposition) {
    case EAssetReviewDisposition::ProxyOnly:
      return TealAccent;
    case EAssetReviewDisposition::Unverified:
      return AmberAccent;
    case EAssetReviewDisposition::Quarantined:
      return AlertInk;
    case EAssetReviewDisposition::Unavailable:
    default:
      return MutedInk;
  }
}

FSlateColor SLingTuSimRuntimeHUD::GetSelectionAvailabilityColor() const {
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  if (Option == nullptr) {
    return MutedInk;
  }
  switch (Option->Availability) {
    case EGameSelectionAvailability::Runnable:
      return TealAccent;
    case EGameSelectionAvailability::PreviewOnly:
      return AmberAccent;
    case EGameSelectionAvailability::Quarantined:
      return AlertInk;
    case EGameSelectionAvailability::Unavailable:
    default:
      return MutedInk;
  }
}

FSlateColor SLingTuSimRuntimeHUD::GetSelectionStatusColor() const {
  if (SelectionFeedback->StartsWith(TEXT("READY")) ||
      SelectionFeedback->StartsWith(TEXT("SELECTED"))) {
    return TealAccent;
  }
  const FGameSelectionOption *Option = SelectionModel->GetSelectedOption();
  if (SelectionFeedback->IsEmpty() && Option != nullptr) {
    if (Option->Availability == EGameSelectionAvailability::Runnable &&
        bSelectionIntentConfigured) {
      return TealAccent;
    }
    if (Option->Availability == EGameSelectionAvailability::PreviewOnly) {
      return AmberAccent;
    }
  }
  return AlertInk;
}

bool SLingTuSimRuntimeHUD::CanConfirmSelection() const {
  FString Blocker;
  const bool bSelectionReady = bFrontEndLoginRequired ? LoginModel->CanConfirmSession(Blocker)
                                                      : SelectionModel->CanConfirm(Blocker);
  return bSelectionIntentConfigured && bSelectionReady;
}

FReply SLingTuSimRuntimeHUD::HandlePreviousSelection() {
  OnSelectionPrevious.ExecuteIfBound();
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::HandleNextSelection() {
  OnSelectionNext.ExecuteIfBound();
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::HandleConfirmSelection() {
  OnSelectionConfirmed.ExecuteIfBound();
  Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::HandlePreviousAssetReview() {
  OnAssetReviewPrevious.ExecuteIfBound();
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::HandleNextAssetReview() {
  OnAssetReviewNext.ExecuteIfBound();
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::HandleSelectAssetReview(FString CardId) {
  AssetReviewModel->SelectById(CardId);
  Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  return FReply::Handled();
}

FReply SLingTuSimRuntimeHUD::HandleEnterLocalWorkspace() {
  LoginFeedback.Reset();
  bool bEnteredWorkspace = false;
  if (SelectionModel->GetCatalog().Options.IsEmpty()) {
    LoginFeedback = TEXT("BLOCKED  ·  verified session catalog unavailable");
  } else {
    FString Error;
    if (!LoginModel->SubmitLocalOperator(PendingOperatorName, Error)) {
      LoginFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Error);
    } else {
      bEnteredWorkspace = true;
    }
  }
  Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  FReply Reply = FReply::Handled();
  if (bEnteredWorkspace && ConfirmSelectionButton.IsValid()) {
    Reply.SetUserFocus(ConfirmSelectionButton.ToSharedRef(), EFocusCause::SetDirectly);
  }
  return Reply;
}

FReply SLingTuSimRuntimeHUD::HandleLogout() {
  FString Error;
  LoginFeedback.Reset();
  const bool bLoggedOut = LoginModel->Logout(Error);
  if (!bLoggedOut) {
    LoginFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Error);
  }
  Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  FReply Reply = FReply::Handled();
  if (bLoggedOut && OperatorNameTextBox.IsValid()) {
    Reply.SetUserFocus(OperatorNameTextBox.ToSharedRef(), EFocusCause::SetDirectly);
  }
  return Reply;
}

void SLingTuSimRuntimeHUD::HandleOperatorNameChanged(const FText &NewText) {
  PendingOperatorName = NewText.ToString();
  LoginFeedback.Reset();
}

bool SLingTuSimRuntimeHUD::CanEnterLocalWorkspace() const {
  return LoginModel->CanEdit() && !PendingOperatorName.TrimStartAndEnd().IsEmpty() &&
         !SelectionModel->GetCatalog().Options.IsEmpty();
}

bool SLingTuSimRuntimeHUD::CanLogout() const {
  return bFrontEndLoginRequired && LoginModel->IsLoggedIn() && LoginModel->CanEdit();
}

bool SLingTuSimRuntimeHUD::CanBrowseAssets() const {
  return AssetReviewModel->GetCatalog().bAvailable &&
         !AssetReviewModel->GetCatalog().Cards.IsEmpty() && !AssetReviewModel->IsBrowsingFrozen();
}

bool SLingTuSimRuntimeHUD::CanSelectAssetReview(FString CardId) const {
  return CanBrowseAssets() &&
         AssetReviewModel->GetCatalog().Cards.ContainsByPredicate(
             [&CardId](const FAssetReviewCard &Card) { return Card.Id == CardId; });
}

const FSlateBrush *SLingTuSimRuntimeHUD::GetAssetPreviewBrush(const FString &CardId) const {
  const TSharedPtr<FSlateDynamicImageBrush> *Brush = AssetPreviewBrushes.Find(CardId);
  return Brush != nullptr && Brush->IsValid() ? Brush->Get() : nullptr;
}

FSlateColor SLingTuSimRuntimeHUD::GetAssetFilmstripCardColor(FString CardId) const {
  const FAssetReviewCard *CurrentCard = AssetReviewModel->GetCurrentCard();
  return CurrentCard != nullptr && CurrentCard->Id == CardId
             ? FieldAmber
             : FLinearColor(0.055F, 0.075F, 0.070F, 0.96F);
}

FSlateColor SLingTuSimRuntimeHUD::GetAssetFilmstripPreviewColor(FString CardId) const {
  const FAssetReviewCard *CurrentCard = AssetReviewModel->GetCurrentCard();
  const float Brightness = CurrentCard != nullptr && CurrentCard->Id == CardId ? 1.0F : 0.45F;
  return FLinearColor(Brightness, Brightness, Brightness, 1.0F);
}

bool SLingTuSimRuntimeHUD::CanBrowseSessions() const {
  return !SelectionModel->IsConfirmed() && SelectionModel->GetCatalog().Options.Num() > 1;
}

EVisibility SLingTuSimRuntimeHUD::GetBuildVisibility() const {
  return ModeController->GetMode() == ERuntimeUIMode::Build ? EVisibility::Visible
                                                            : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetTacticalVisibility() const {
  return ModeController->GetMode() == ERuntimeUIMode::Tactical ? EVisibility::Visible
                                                               : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetPauseVisibility() const {
  return ModeController->GetMode() == ERuntimeUIMode::Pause ? EVisibility::Visible
                                                            : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetLoginVisibility() const {
  return GetPauseVisibility() == EVisibility::Visible && bFrontEndLoginRequired &&
                 !LoginModel->IsLoggedIn()
             ? EVisibility::Visible
             : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetWorkspaceVisibility() const {
  return GetPauseVisibility() == EVisibility::Visible &&
                 (!bFrontEndLoginRequired || LoginModel->IsLoggedIn())
             ? EVisibility::Visible
             : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetLogoutVisibility() const {
  return bFrontEndLoginRequired ? EVisibility::Visible : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetFrontEndStaticBackdropVisibility() const {
  return GetLoginVisibility() == EVisibility::Visible ? EVisibility::HitTestInvisible
                                                      : EVisibility::Collapsed;
}

EVisibility SLingTuSimRuntimeHUD::GetRuntimeHudChromeVisibility() const {
  return bFrontEndLoginRequired ? EVisibility::Collapsed : EVisibility::Visible;
}

EVisibility SLingTuSimRuntimeHUD::GetNonPauseVisibility() const {
  return ModeController->GetMode() == ERuntimeUIMode::Pause ? EVisibility::Collapsed
                                                            : EVisibility::Visible;
}
}  // namespace LingTuSim::UI
