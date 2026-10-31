#include "SLingTuSimMeasurements.h"

#include "LingTuSimMeasurementClient.h"
#include "Brushes/SlateColorBrush.h"
#include "Engine/Texture2D.h"
#include "Slate/DeferredCleanupSlateBrush.h"
#include "Dom/JsonObject.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/PlatformProcess.h"
#include "Input/Events.h"
#include "InputCoreTypes.h"
#include "Misc/DefaultValueHelper.h"
#include "Misc/Guid.h"
#include "Serialization/JsonSerializer.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

namespace LingTuSim::UI {
namespace {
const FLinearColor Paper(.90F, .93F, .96F, 1);
const FLinearColor Muted(.55F, .61F, .68F, 1);
const FLinearColor Accent(.40F, .65F, .82F, 1);
const FLinearColor Panel(.018F, .024F, .034F, .94F);

TSharedRef<STextBlock> Text(const FString& Caption, int32 Size = 12, FLinearColor Color = Paper) {
  return SNew(STextBlock).Text(FText::FromString(Caption))
      .Font(FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), Size)).ColorAndOpacity(Color).AutoWrapText(true);
}

const FButtonStyle* QuietStyle() {
  static const FButtonStyle Style = FButtonStyle()
      .SetNormal(FSlateColorBrush(FLinearColor::Transparent))
      .SetHovered(FSlateColorBrush(FLinearColor(.08F, .10F, .13F, 1)))
      .SetPressed(FSlateColorBrush(FLinearColor(.12F, .16F, .20F, 1)))
      .SetDisabled(FSlateColorBrush(FLinearColor::Transparent));
  return &Style;
}

TSharedRef<SButton> Button(const FString& Caption, FOnClicked Click) {
  auto Result = SNew(SButton).ButtonStyle(QuietStyle()).ContentPadding(FMargin(10, 7)).OnClicked(Click);
  const TWeakPtr<SButton> Weak = Result;
  Result->SetContent(SNew(STextBlock).Text(FText::FromString(Caption))
      .Font(FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), 12))
      .ColorAndOpacity_Lambda([Weak]() {
        const auto Pinned = Weak.Pin();
        return Pinned.IsValid() && Pinned->HasKeyboardFocus() ? FLinearColor::White : Accent;
      }));
  return Result;
}

TSharedPtr<FJsonObject> Object(const TSharedPtr<FJsonObject>& Parent, const TCHAR* Name) {
  const TSharedPtr<FJsonObject>* Child = nullptr;
  return Parent.IsValid() && Parent->TryGetObjectField(Name, Child) ? *Child : nullptr;
}

TSharedPtr<FJsonObject> Noise(const TSharedPtr<FJsonObject>& Settings) {
  const auto Nested = Object(Object(Settings, TEXT("measurement_settings")), TEXT("noise"));
  return Nested.IsValid() ? Nested : Object(Settings, TEXT("noise"));
}

bool Number(const TSharedPtr<FJsonObject>& Json, const TCHAR* Name, double& Value) {
  return Json.IsValid() && Json->TryGetNumberField(Name, Value) && FMath::IsFinite(Value);
}

bool ReadNumber(const TSharedPtr<SEditableTextBox>& Edit, double& Value) {
  const FString Input = Edit->GetText().ToString().TrimStartAndEnd();
  return !Input.IsEmpty() && FDefaultValueHelper::ParseDouble(Input, Value) && FMath::IsFinite(Value);
}

FString BoolLabel(const TSharedPtr<FJsonObject>& Settings, const TCHAR* Name) {
  bool Value = false;
  return Settings.IsValid() && Settings->TryGetBoolField(Name, Value)
      ? (Value ? TEXT("开") : TEXT("关")) : TEXT("未知 / 不支持");
}

// Source frame identity, never the address of the client's replaceable optional preview.
bool SameDisplay(const FMeasurementPreview& A, const FMeasurementPreview& B) {
  return A.Status.SensorId == B.Status.SensorId && A.Kind == B.Kind
      && A.Status.Binding.ProductSessionId == B.Status.Binding.ProductSessionId
      && A.Status.Binding.SessionId == B.Status.Binding.SessionId
      && A.Status.Binding.ModelGeneration == B.Status.Binding.ModelGeneration
      && A.Status.Binding.ResetGeneration == B.Status.Binding.ResetGeneration
      && A.Status.Sequence == B.Status.Sequence && A.Status.SimTimeNs == B.Status.SimTimeNs;
}

void SetPixel(TArray<uint8>& Bytes, int32 Width, int32 X, int32 Y, const FColor& Color) {
  const int32 Offset = (Y * Width + X) * 4;
  Bytes[Offset] = Color.B; Bytes[Offset + 1] = Color.G;
  Bytes[Offset + 2] = Color.R; Bytes[Offset + 3] = Color.A;
}
}

namespace MeasurementDisplay {
bool IsValidPixel(const FMeasurementPreview& P, int32 Index) {
  if (!P.ValidMask.IsValidIndex(Index) || !P.ValidMask[Index] || !P.Primary.IsValidIndex(Index)
      || !P.Primary[Index].IsSet() || !FMath::IsFinite(P.Primary[Index].GetValue())) return false;
  return P.Kind != TEXT("semantic") || (P.Secondary.IsValidIndex(Index)
      && P.Secondary[Index].IsSet() && FMath::IsFinite(P.Secondary[Index].GetValue()));
}

TOptional<double> Temperature(const FMeasurementPreview& P, int32 Index, bool bCelsius) {
  if (P.Kind != TEXT("thermal") || !IsValidPixel(P, Index)) return {};
  return P.Primary[Index].GetValue() - (bCelsius ? 273.15 : 0.0);
}

FColor PixelColor(const FMeasurementPreview& P, int32 Index, double LowK, double HighK) {
  if (!IsValidPixel(P, Index)) {
    const int32 Row = P.Width > 0 ? Index / P.Width : 0;
    const int32 Col = P.Width > 0 ? Index % P.Width : 0;
    return ((Row + Col) % 2) ? FColor(175, 175, 175) : FColor(75, 75, 75);
  }
  if (P.Kind == TEXT("thermal")) {
    const double Level = HighK > LowK ? FMath::Clamp((P.Primary[Index].GetValue() - LowK) / (HighK - LowK), 0.0, 1.0) : .5;
    return FColor(FMath::RoundToInt(255 * Level), FMath::RoundToInt(210 * Level), FMath::RoundToInt(210 * (1 - Level)));
  }
  const int32 Hue = static_cast<int32>(FMath::Fmod(P.Primary[Index].GetValue() * 67.0, 255.0));
  return FColor(Hue, (Hue + 89) % 255, (Hue + 173) % 255);
}

TOptional<FPressureBin> PressureBin(const FMeasurementPreview& P, int32 Channel, int32 Bin) {
  if (P.Kind != TEXT("acoustic") || Channel < 0 || Channel >= P.Channels
      || P.SampleRateHz <= 0 || !P.BinStarts.IsValidIndex(Bin) || !P.BinEnds.IsValidIndex(Bin)) return {};
  const int32 Index = Channel * P.BinStarts.Num() + Bin;
  if (!P.MinPa.IsValidIndex(Index) || !P.MaxPa.IsValidIndex(Index) || !P.ValidFractions.IsValidIndex(Index)
      || !P.MinPa[Index].IsSet() || !P.MaxPa[Index].IsSet()) return {};
  FPressureBin Result;
  Result.MinPa = P.MinPa[Index].GetValue(); Result.MaxPa = P.MaxPa[Index].GetValue();
  Result.ValidFraction = P.ValidFractions[Index];
  if (!FMath::IsFinite(Result.MinPa) || !FMath::IsFinite(Result.MaxPa)
      || Result.MinPa > Result.MaxPa || !FMath::IsFinite(Result.ValidFraction)
      || Result.ValidFraction <= 0 || Result.ValidFraction > 1
      || P.BinStarts[Bin] < 0 || P.BinEnds[Bin] <= P.BinStarts[Bin]
      || P.BinEnds[Bin] > P.SourceSampleCount) return {};
  Result.StartSeconds = static_cast<double>(P.BinStarts[Bin]) / P.SampleRateHz;
  Result.EndSeconds = static_cast<double>(P.BinEnds[Bin]) / P.SampleRateHz;
  return Result;
}
}

void SLingTuSimMeasurements::Construct(const FArguments& Args) {
  Client = Args._Client; OnBack = Args._OnBack;
  BackButton = Button(TEXT("‹ 返回"), FOnClicked::CreateLambda([this]() {
    OnBack.ExecuteIfBound(); return FReply::Handled();
  }));
  const auto Inspector = [this](const TCHAR* Caption, int32* Value, bool bAcoustic, bool bSecond) {
    return SNew(SHorizontalBox)
      .Visibility_Lambda([this, bAcoustic]() {
        return CachedPreview.IsSet() && (CachedPreview->Kind == TEXT("acoustic")) == bAcoustic
            ? EVisibility::Visible : EVisibility::Collapsed;
      })
      + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)[Text(Caption)]
      + SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride(90)
        [SNew(SNumericEntryBox<int32>).MinValue(0).AllowSpin(true)
          .Value_Lambda([Value]() { return TOptional<int32>(*Value); })
          .MaxValue_Lambda([this, bAcoustic, bSecond]() -> TOptional<int32> {
            if (!CachedPreview.IsSet()) return 0;
            const auto& P = CachedPreview.GetValue();
            return FMath::Max(0, (bAcoustic ? (bSecond ? P.BinStarts.Num() : P.Channels) : (bSecond ? P.Width : P.Height)) - 1);
          })
          .OnValueChanged_Lambda([this, Value, bAcoustic, bSecond](int32 NewValue) {
            if (!CachedPreview.IsSet()) return;
            const auto& P = CachedPreview.GetValue();
            const int32 Count = bAcoustic ? (bSecond ? P.BinStarts.Num() : P.Channels) : (bSecond ? P.Width : P.Height);
            *Value = FMath::Clamp(NewValue, 0, FMath::Max(0, Count - 1));
          })]];
  };
  ChildSlot[SNew(SBox).WidthOverride(1000).HeightOverride(700)
    [SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush(TEXT("WhiteBrush"))).BorderBackgroundColor(Panel).Padding(24)
      [SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[SNew(SHorizontalBox)
          + SHorizontalBox::Slot().FillWidth(1)[Text(TEXT("传感器 / MEASUREMENTS"), 24)]
          + SHorizontalBox::Slot().AutoWidth()[BackButton.ToSharedRef()]]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 10)[SAssignNew(SensorButton, SComboButton)
          .OnGetMenuContent(this, &SLingTuSimMeasurements::BuildSensorMenu)
          .ButtonContent()[SNew(STextBlock).ColorAndOpacity(Paper)
            .Text_Lambda([this]() { return FText::FromString(Client.IsValid() && !Client->SelectedSensorId().IsEmpty()
                ? Client->SelectedSensorId() : TEXT("选择传感器 / 无已选传感器")); })]]
        + SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)[SNew(STextBlock)
          .Text(this, &SLingTuSimMeasurements::StatusText).ColorAndOpacity(Muted).AutoWrapText(true)]
        + SVerticalBox::Slot().FillHeight(1)[SNew(SHorizontalBox)
          + SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 18, 0)[SNew(SScrollBox)
            + SScrollBox::Slot()[SNew(SVerticalBox)
              + SVerticalBox::Slot().AutoHeight()[SAssignNew(Display, SVerticalBox)]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 10)[SNew(SBox).Visibility_Lambda([this]() {
                  return CachedPreview.IsSet() && CachedPreview->Kind == TEXT("thermal") ? EVisibility::Visible : EVisibility::Collapsed;
                })[Button(TEXT("温标切换 °C / K"), FOnClicked::CreateLambda([this]() {
                  bCelsius = !bCelsius; return FReply::Handled();
                }))]]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 4)[Inspector(TEXT("预览行"), &Row, false, false)]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 4)[Inspector(TEXT("预览列"), &Column, false, true)]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 4)[Inspector(TEXT("通道"), &Channel, true, false)]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 4)[Inspector(TEXT("包络 bin"), &Bin, true, true)]
              + SVerticalBox::Slot().AutoHeight().Padding(0, 10)[SNew(STextBlock).Text(this, &SLingTuSimMeasurements::InspectText)
                .ColorAndOpacity(Paper).AutoWrapText(true)]]]
          + SHorizontalBox::Slot().AutoWidth()[SNew(SBox).WidthOverride(304)[SNew(SScrollBox)
            + SScrollBox::Slot()[SAssignNew(Controls, SVerticalBox)]]]]]]];
  RefreshProjection();
}

void SLingTuSimMeasurements::FocusInitialControl() {
  RefreshProjection();
  if (FSlateApplication::IsInitialized() && BackButton.IsValid())
    FSlateApplication::Get().SetKeyboardFocus(BackButton, EFocusCause::SetDirectly);
}

FReply SLingTuSimMeasurements::OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Event) {
  if (Event.GetKey() == EKeys::Escape || Event.GetKey() == EKeys::Gamepad_FaceButton_Right) {
    OnBack.ExecuteIfBound(); return FReply::Handled();
  }
  return SCompoundWidget::OnKeyDown(Geometry, Event);
}

void SLingTuSimMeasurements::Tick(const FGeometry& Geometry, double Time, float DeltaTime) {
  SCompoundWidget::Tick(Geometry, Time, DeltaTime);
  // Local projection only. FMeasurementClient owns its HTTP ticker.
  RefreshProjection();
}

TSharedRef<SWidget> SLingTuSimMeasurements::BuildSensorMenu() {
  auto Rows = SNew(SVerticalBox);
  if (Client.IsValid()) for (const auto& Sensor : Client->Sensors()) {
    const FString Id = Sensor.SensorId;
    Rows->AddSlot().AutoHeight()[Button(Id + TEXT("  /  ") + Sensor.Modality + TEXT("  /  ") + Sensor.State,
      FOnClicked::CreateLambda([this, Id]() {
        if (Client.IsValid()) Client->SelectSensor(Id);
        SensorButton->SetIsOpen(false); RefreshProjection(); return FReply::Handled();
      }))];
  }
  if (!Client.IsValid() || Client->Sensors().IsEmpty())
    Rows->AddSlot().AutoHeight()[Text(TEXT("尚无声明的测量传感器"), 12, Muted)];
  return SNew(SBox).MaxDesiredHeight(320)[SNew(SBorder)
      .BorderImage(FCoreStyle::Get().GetBrush(TEXT("WhiteBrush"))).BorderBackgroundColor(Panel).Padding(10)
      [SNew(SScrollBox) + SScrollBox::Slot()[Rows]]];
}

FString SLingTuSimMeasurements::BindingKey() const {
  const auto* S = Client.IsValid() ? Client->SelectedStatus() : nullptr;
  if (!S) return {};
  const auto& B = Client->Binding();
  return FString::Printf(TEXT("%s|%s|%s|%llu|%llu"), *S->SensorId, *B.ProductSessionId,
      *B.SessionId, B.ModelGeneration, B.ResetGeneration);
}

bool SLingTuSimMeasurements::CanSend() const {
  return Client.IsValid() && Client->CanControl() && !Client->IsPending();
}

void SLingTuSimMeasurements::RefreshProjection() {
  const FString Key = BindingKey();
  const auto Settings = Client.IsValid() ? Client->Settings() : nullptr;
  FString Json;
  if (Settings.IsValid()) FJsonSerializer::Serialize(Settings.ToSharedRef(), TJsonWriterFactory<>::Create(&Json));
  if (Key != CachedControlKey || Json != CachedSettings || Controls->NumSlots() == 0) {
    const bool bBindingChanged = Key != CachedControlKey;
    CachedControlKey = Key; CachedSettings = Json; LocalFeedback.Reset();
    if (bBindingChanged) { Row = Column = Channel = Bin = 0; }
    RebuildControls(Settings);
  }
  const auto* Preview = Client.IsValid() ? Client->Preview() : nullptr;
  if (!Preview) {
    if (CachedPreview.IsSet() || Display->NumSlots() == 0) {
      Display->ClearChildren(); ImageBrushes.Reset(); CachedPreview.Reset();
      Display->AddSlot().AutoHeight()[Text(TEXT("暂无新鲜预览。过期、禁用、隐藏或来源变更时清空显示。"), 14, Muted)];
    }
    return;
  }
  if (!CachedPreview.IsSet() || !SameDisplay(CachedPreview.GetValue(), *Preview)) {
    CachedPreview = *Preview;
    Row = FMath::Clamp(Row, 0, FMath::Max(0, Preview->Height - 1));
    Column = FMath::Clamp(Column, 0, FMath::Max(0, Preview->Width - 1));
    Channel = FMath::Clamp(Channel, 0, FMath::Max(0, Preview->Channels - 1));
    Bin = FMath::Clamp(Bin, 0, FMath::Max(0, Preview->BinStarts.Num() - 1));
    RebuildDisplay(*Preview);
  }
}

void SLingTuSimMeasurements::RebuildDisplay(const FMeasurementPreview& P) {
  Display->ClearChildren(); ImageBrushes.Reset();
  Display->AddSlot().AutoHeight().Padding(0, 0, 0, 6)[Text(FString::Printf(
    TEXT("来源帧 %llu · 仿真时间 %llu ns"), P.Status.Sequence, P.Status.SimTimeNs), 11, Muted)];
  const auto AddImage = [this](int32 Width, int32 Height, const TArray<uint8>& Pixels, float ViewHeight) {
    const FName Name(*FString::Printf(TEXT("LingTu.Measurement.%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
    auto* Texture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8, Name, Pixels);
    if (!Texture) { Display->AddSlot().AutoHeight()[Text(TEXT("动态图像资源不可用；数值仍可检查。"), 12, Muted)]; return; }
    Texture->Filter = TF_Nearest;
    Texture->SRGB = false;
    Texture->UpdateResource();
    auto Brush = FDeferredCleanupSlateBrush::CreateBrush(Texture);
    ImageBrushes.Add(Brush);
    Display->AddSlot().AutoHeight().HAlign(HAlign_Left).Padding(0, 6)
      [SNew(SBox).HeightOverride(ViewHeight).WidthOverride(ViewHeight * Width / Height)
        [SNew(SImage).Image(Brush->GetSlateBrush())]];
  };
  if (P.Kind == TEXT("thermal") || P.Kind == TEXT("semantic")) {
    Display->AddSlot().AutoHeight()[Text(FString::Printf(TEXT("%s\n预览 %d × %d / 原始 %d × %d；最近邻采样；棋盘格 = 无效"),
      P.Kind == TEXT("thermal") ? TEXT("温度 / Kelvin 来源，°C / K 数值视图") : TEXT("语义真值 / GROUND TRUTH · 非检测结果"),
      P.Width, P.Height, P.SourceWidth, P.SourceHeight), 12, Muted)];
    double Low = TNumericLimits<double>::Max(), High = TNumericLimits<double>::Lowest();
    for (int32 I = 0; I < P.Primary.Num(); ++I) if (MeasurementDisplay::IsValidPixel(P, I)) {
      Low = FMath::Min(Low, P.Primary[I].GetValue()); High = FMath::Max(High, P.Primary[I].GetValue());
    }
    TArray<uint8> Pixels; Pixels.SetNumUninitialized(P.Width * P.Height * 4);
    for (int32 Y = 0; Y < P.Height; ++Y) for (int32 X = 0; X < P.Width; ++X)
      SetPixel(Pixels, P.Width, X, Y, MeasurementDisplay::PixelColor(P, Y * P.Width + X, Low, High));
    AddImage(P.Width, P.Height, Pixels, FMath::Min(260.F, 584.F * P.Height / FMath::Max(1, P.Width)));
    if (P.Kind == TEXT("thermal")) Display->AddSlot().AutoHeight()[SNew(STextBlock).ColorAndOpacity(Muted)
      .Text_Lambda([this, Low, High]() {
        if (Low > High) return FText::FromString(TEXT("温度范围未知：没有有效像素。"));
        const double Offset = bCelsius ? 273.15 : 0;
        return FText::FromString(FString::Printf(TEXT("显示范围 %.4f – %.4f %s · 蓝低 / 红高"),
          Low - Offset, High - Offset, bCelsius ? TEXT("°C") : TEXT("K")));
      })];
  } else if (P.Kind == TEXT("acoustic")) {
    Display->AddSlot().AutoHeight()[Text(FString::Printf(TEXT("声压包络 / 非原始波形 · %d Hz · %d 样本/通道\n横轴：相对此块起点的样本时间；空缺 = 无效，淡色 = 部分有效；min = max 显示点。"),
      P.SampleRateHz, P.SourceSampleCount), 12, Muted)];
    constexpr int32 Width = 584, Height = 96;
    for (int32 C = 0; C < P.Channels; ++C) {
      double Peak = 0;
      for (int32 B = 0; B < P.BinStarts.Num(); ++B) {
        const auto Value = MeasurementDisplay::PressureBin(P, C, B);
        if (Value.IsSet()) Peak = FMath::Max(Peak, FMath::Max(FMath::Abs(Value->MinPa), FMath::Abs(Value->MaxPa)));
      }
      Display->AddSlot().AutoHeight().Padding(0, 6)[Text(FString::Printf(TEXT("通道 %d · 纵轴 ±%.6g Pa · 0 Pa 中线"), C, Peak))];
      TArray<uint8> Pixels; Pixels.SetNumUninitialized(Width * Height * 4);
      for (int32 Y = 0; Y < Height; ++Y) for (int32 X = 0; X < Width; ++X)
        SetPixel(Pixels, Width, X, Y, Y == Height / 2 ? FColor(55, 66, 78) : FColor(5, 7, 10));
      const double Scale = Peak > 0 ? Peak : 1;
      for (int32 B = 0; B < P.BinStarts.Num(); ++B) {
        const auto Value = MeasurementDisplay::PressureBin(P, C, B);
        if (!Value.IsSet()) continue;
        const int32 X = FMath::Clamp(FMath::RoundToInt((Width - 1.0) * P.BinStarts[B] / FMath::Max(1, P.SourceSampleCount)), 0, Width - 1);
        const int32 Top = FMath::Clamp(FMath::RoundToInt(Height / 2.0 - (Height / 2 - 4) * Value->MaxPa / Scale), 0, Height - 1);
        const int32 Bottom = FMath::Clamp(FMath::RoundToInt(Height / 2.0 - (Height / 2 - 4) * Value->MinPa / Scale), 0, Height - 1);
        const FColor Color = Value->ValidFraction < 1 ? FColor(48, 79, 101) : FColor(102, 166, 209);
        // Inclusive end preserves a single visible point for a constant-pressure bin.
        for (int32 Y = Top; Y <= Bottom; ++Y) SetPixel(Pixels, Width, X, Y, Color);
      }
      AddImage(Width, Height, Pixels, Height);
      Display->AddSlot().AutoHeight()[Text(FString::Printf(TEXT("0 s                                         %.6g s (块内相对时间)"),
        P.SampleRateHz > 0 ? static_cast<double>(P.SourceSampleCount) / P.SampleRateHz : 0), 11, Muted)];
    }
  } else Display->AddSlot().AutoHeight()[Text(TEXT("未知预览类型；未生成替代图像。"), 12, Muted)];
}

FText SLingTuSimMeasurements::StatusText() const {
  if (!Client.IsValid() || !Client->IsConfigured()) return FText::FromString(TEXT("未绑定 Gateway / 来源会话；状态未知，控制不可用。"));
  const auto* S = Client->SelectedStatus();
  FString Result = S ? FString::Printf(TEXT("%s · %s · %s · 身份 %s"),
    *S->SensorId, *S->Modality, *S->State, S->bHasIdentity ? TEXT("已绑定") : TEXT("未知"))
    : TEXT("尚无传感器状态。选择已声明的传感器。");
  if (S && S->bHasIdentity) Result += FString::Printf(TEXT(" · 服务端报告年龄 %.3g s / %.3g s"), S->ReceiptAgeSeconds, S->StaleTimeoutSeconds);
  else if (S) Result += TEXT(" · 接收年龄未知");
  if (S) Result += S->bFresh && Client->Preview() ? TEXT(" · 新鲜预览") : TEXT(" · 无新鲜预览");
  if (Client->IsPending()) Result += TEXT("\n等待来源 ACK；尚未确认应用。");
  if (!Client->Message().IsEmpty()) Result += TEXT("\n") + Client->Message();
  if (!Client->Error().IsEmpty()) Result += TEXT("\n") + Client->Error();
  if (!LocalFeedback.IsEmpty()) Result += TEXT("\n") + LocalFeedback;
  return FText::FromString(Result);
}

FText SLingTuSimMeasurements::InspectText() const {
  if (!CachedPreview.IsSet() || !Client.IsValid() || !Client->Preview()) return FText::FromString(TEXT("数值未知：无新鲜预览。"));
  const auto& P = CachedPreview.GetValue();
  if (P.Kind == TEXT("acoustic")) {
    const auto Value = MeasurementDisplay::PressureBin(P, Channel, Bin);
    const FString Bounds = P.BinStarts.IsValidIndex(Bin) && P.BinEnds.IsValidIndex(Bin)
      ? FString::Printf(TEXT("样本 [%d, %d)"), P.BinStarts[Bin], P.BinEnds[Bin]) : TEXT("样本范围未知");
    if (!Value.IsSet()) return FText::FromString(FString::Printf(TEXT("通道 %d · bin %d · %s · 无效；不是 0 Pa。"), Channel, Bin, *Bounds));
    return FText::FromString(FString::Printf(TEXT("通道 %d · bin %d · %s\n相对此块 [%.9g, %.9g) s\nmin %.9g Pa / max %.9g Pa · 有效比例 %.5g"),
      Channel, Bin, *Bounds, Value->StartSeconds, Value->EndSeconds, Value->MinPa, Value->MaxPa, Value->ValidFraction));
  }
  const int32 Index = Row * P.Width + Column;
  FString Result = FString::Printf(TEXT("预览 (%d, %d) · 原始像素 (行 %s, 列 %s)\n"), Row, Column,
    P.RowIndices.IsValidIndex(Row) ? *FString::FromInt(P.RowIndices[Row]) : TEXT("未知"),
    P.ColumnIndices.IsValidIndex(Column) ? *FString::FromInt(P.ColumnIndices[Column]) : TEXT("未知"));
  if (!MeasurementDisplay::IsValidPixel(P, Index)) Result += TEXT("无效 / invalid mask；不代表黑色、0 K 或背景类别。");
  else if (P.Kind == TEXT("thermal")) Result += FString::Printf(TEXT("%.6f %s"),
    MeasurementDisplay::Temperature(P, Index, bCelsius).GetValue(), bCelsius ? TEXT("°C") : TEXT("K"));
  else if (P.Kind == TEXT("semantic")) Result += FString::Printf(TEXT("class_id %.0f · instance_id %.0f · 语义真值，非检测结果"),
    P.Primary[Index].GetValue(), P.Secondary[Index].GetValue());
  return FText::FromString(Result);
}

void SLingTuSimMeasurements::Send(const FString& Context, const FString& Operation, TSharedPtr<FJsonObject> Params) {
  if (!(Operation == TEXT("query") ? Client.IsValid() && Client->CanQuery() : CanSend()) || Context != BindingKey()) {
    LocalFeedback = TEXT("来源已变更或控制不可用；未发送。请查询当前来源设置。"); return;
  }
  LocalFeedback.Reset(); Client->SendControl(Operation, MoveTemp(Params));
}

void SLingTuSimMeasurements::RebuildControls(const TSharedPtr<FJsonObject>& Settings) {
  const bool bRestoreFocus = Controls->HasFocusedDescendants();
  Controls->ClearChildren();
  const FString Context = BindingKey();
  const auto ControlButton = [this](const FString& Caption, FOnClicked Click) {
    auto Result = Button(Caption, MoveTemp(Click));
    Result->SetEnabled(TAttribute<bool>::CreateSP(this, &SLingTuSimMeasurements::CanSend));
    return Result;
  };
  Controls->AddSlot().AutoHeight()[Text(TEXT("来源确认的设置"), 17)];
  auto Query = Button(TEXT("查询设置"), FOnClicked::CreateLambda([this, Context]() {
    Send(Context, TEXT("query"), MakeShared<FJsonObject>()); return FReply::Handled();
  }));
  Query->SetEnabled(TAttribute<bool>::CreateLambda([this]() { return Client.IsValid() && Client->CanQuery(); }));
  Controls->AddSlot().AutoHeight().Padding(0, 8)[Query];
  Controls->AddSlot().AutoHeight()[Text(TEXT("实时采样继续；手动驾驶输入已释放。自主任务仍由原生任务端管理。"), 11, Muted)];
  Controls->AddSlot().AutoHeight()[Text(TEXT("仅发送传感器设置；不获取运动所有权，不执行运动。输入框为草稿，ACK 前不代表已应用。"), 11, Muted)];
  const auto NoiseSettings = Noise(Settings);
  bool NoiseEnabled = false;
  const bool bHasNoiseEnabled = (Settings.IsValid() && Settings->TryGetBoolField(TEXT("noise_enabled"), NoiseEnabled))
    || (NoiseSettings.IsValid() && NoiseSettings->TryGetBoolField(TEXT("enabled"), NoiseEnabled));
  Controls->AddSlot().AutoHeight().Padding(0, 8)[Text(FString::Printf(TEXT("启用：%s\n噪声：%s\n录制：%s"),
    *BoolLabel(Settings, TEXT("enabled")), bHasNoiseEnabled ? (NoiseEnabled ? TEXT("开") : TEXT("关")) : TEXT("未知 / 不支持"),
    *BoolLabel(Settings, TEXT("recording"))), 12, Muted)];
  for (const FString Operation : {FString(TEXT("enabled")), FString(TEXT("noise_enabled")), FString(TEXT("recording"))}) {
    bool Confirmed = false;
    const bool bKnown = Operation == TEXT("noise_enabled") ? bHasNoiseEnabled
      : Settings.IsValid() && Settings->TryGetBoolField(Operation, Confirmed);
    if (!bKnown) continue;
    Controls->AddSlot().AutoHeight().Padding(0, 5)[SNew(SCheckBox)
      .IsEnabled_Lambda([this]() { return CanSend(); })
      .IsChecked_Lambda([this, Operation]() {
        const auto Current = Client.IsValid() ? Client->Settings() : nullptr;
        bool Value = false;
        bool bKnownNow = Current.IsValid() && Current->TryGetBoolField(Operation, Value);
        if (!bKnownNow && Operation == TEXT("noise_enabled")) {
          const auto N = Noise(Current); bKnownNow = N.IsValid() && N->TryGetBoolField(TEXT("enabled"), Value);
        }
        return !bKnownNow ? ECheckBoxState::Undetermined : Value ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
      })
      .OnCheckStateChanged_Lambda([this, Operation, Context](ECheckBoxState State) {
        if (State == ECheckBoxState::Undetermined) return;
        auto Params = MakeShared<FJsonObject>(); Params->SetBoolField(TEXT("enabled"), State == ECheckBoxState::Checked);
        Send(Context, Operation, Params);
      })[Text(Operation)]];
  }
  const auto Edit = [this](const FString& Caption, double Value) {
    TSharedPtr<SEditableTextBox> Box;
    Controls->AddSlot().AutoHeight().Padding(0, 6, 0, 2)[Text(Caption, 12, Muted)];
    Controls->AddSlot().AutoHeight()[SAssignNew(Box, SEditableTextBox).Text(FText::FromString(FString::Printf(TEXT("%.17g"), Value)))
      .IsEnabled_Lambda([this]() { return CanSend(); }).SelectAllTextWhenFocused(true)];
    return Box;
  };
  double Rate = 0;
  if (Number(Settings, TEXT("rate_hz"), Rate)) {
    const auto RateEdit = Edit(FString::Printf(TEXT("频率 Hz · 确认 %.9g"), Rate), Rate);
    Controls->AddSlot().AutoHeight()[ControlButton(TEXT("应用频率"), FOnClicked::CreateLambda([this, Context, RateEdit]() {
      double Value = 0;
      if (!ReadNumber(RateEdit, Value) || Value <= 0) { LocalFeedback = TEXT("频率必须为正的有限 Hz。"); return FReply::Handled(); }
      auto Params = MakeShared<FJsonObject>(); Params->SetNumberField(TEXT("rate_hz"), Value);
      Send(Context, TEXT("rate"), Params); return FReply::Handled();
    }))];
  }
  double Stddev = 0, Seed = 0;
  FString Unit;
  bool bNoiseOn = false;
  if (Number(NoiseSettings, TEXT("stddev"), Stddev) && Number(NoiseSettings, TEXT("seed"), Seed)
      && NoiseSettings->TryGetStringField(TEXT("unit"), Unit) && !Unit.IsEmpty()
      && NoiseSettings->TryGetBoolField(TEXT("enabled"), bNoiseOn)) {
    const auto StddevEdit = Edit(FString::Printf(TEXT("噪声 σ (%s) · 确认 %.9g"), *Unit, Stddev), Stddev);
    const auto SeedEdit = Edit(FString::Printf(TEXT("种子 · 确认 %.17g"), Seed), Seed);
    Controls->AddSlot().AutoHeight()[ControlButton(TEXT("应用噪声参数"), FOnClicked::CreateLambda([this, Context, Unit, StddevEdit, SeedEdit]() {
      double Sigma = 0, SeedValue = 0;
      if (!ReadNumber(StddevEdit, Sigma) || Sigma < 0 || !ReadNumber(SeedEdit, SeedValue)
          || SeedValue < 0 || SeedValue > 9007199254740991.0 || FMath::FloorToDouble(SeedValue) != SeedValue) {
        LocalFeedback = TEXT("σ 须非负且有限；种子须为 0 至 9007199254740991 的精确整数。未发送。"); return FReply::Handled();
      }
      const auto CurrentNoise = Client.IsValid() ? Noise(Client->Settings()) : nullptr;
      bool Enabled = false;
      if (!CurrentNoise.IsValid() || !CurrentNoise->TryGetBoolField(TEXT("enabled"), Enabled)) {
        LocalFeedback = TEXT("噪声启用状态未知；先查询设置。"); return FReply::Handled();
      }
      auto Params = MakeShared<FJsonObject>(); Params->SetBoolField(TEXT("enabled"), Enabled);
      Params->SetStringField(TEXT("unit"), Unit); Params->SetNumberField(TEXT("stddev"), Sigma);
      Params->SetNumberField(TEXT("seed"), SeedValue); Send(Context, TEXT("noise"), Params); return FReply::Handled();
    }))];
  }
  const auto Gimbal = Object(Object(Object(Settings, TEXT("measurement_settings")), TEXT("camera")), TEXT("gimbal"));
  double Pan = 0, Tilt = 0;
  if (Number(Gimbal, TEXT("pan_deg"), Pan) && Number(Gimbal, TEXT("tilt_deg"), Tilt)) {
    const auto PanEdit = Edit(FString::Printf(TEXT("Pan ° · 确认 %.9g"), Pan), Pan);
    const auto TiltEdit = Edit(FString::Printf(TEXT("Tilt ° · 确认 %.9g"), Tilt), Tilt);
    Controls->AddSlot().AutoHeight()[ControlButton(TEXT("应用云台角度"), FOnClicked::CreateLambda([this, Context, PanEdit, TiltEdit]() {
      double PanValue = 0, TiltValue = 0;
      if (!ReadNumber(PanEdit, PanValue) || !ReadNumber(TiltEdit, TiltValue)) {
        LocalFeedback = TEXT("云台角度须为有限度数。未发送。"); return FReply::Handled();
      }
      auto Params = MakeShared<FJsonObject>(); Params->SetNumberField(TEXT("pan_deg"), PanValue);
      Params->SetNumberField(TEXT("tilt_deg"), TiltValue); Send(Context, TEXT("gimbal"), Params); return FReply::Handled();
    }))];
  }
  if (!Settings.IsValid()) Controls->AddSlot().AutoHeight().Padding(0, 8)[Text(TEXT("设置未知。请先查询；不推测支持的控制项。"), 12, Muted)];
  auto Download = Button(TEXT("下载最新原始帧"), FOnClicked::CreateLambda([this]() {
    const FString Url = Client.IsValid() ? Client->RawUrl() : FString();
    if (!Url.IsEmpty()) {
      FString Error; FPlatformProcess::LaunchURL(*Url, nullptr, &Error);
      LocalFeedback = Error.IsEmpty() ? TEXT("已请求浏览器打开最新原始帧下载；尚未确认保存。") : Error;
    }
    return FReply::Handled();
  }));
  Download->SetEnabled(TAttribute<bool>::CreateLambda([this]() { return Client.IsValid() && !Client->RawUrl().IsEmpty(); }));
  Controls->AddSlot().AutoHeight().Padding(0, 12, 0, 0)[Download];
  Controls->AddSlot().AutoHeight()[Text(TEXT("完整原始 NPZ，不是采样显示数据。下载时获取最新帧，可能与当前预览不同；浏览器处理保存。"), 11, Muted)];
  if (bRestoreFocus && FSlateApplication::IsInitialized())
    FSlateApplication::Get().SetKeyboardFocus(BackButton, EFocusCause::SetDirectly);
}
}
