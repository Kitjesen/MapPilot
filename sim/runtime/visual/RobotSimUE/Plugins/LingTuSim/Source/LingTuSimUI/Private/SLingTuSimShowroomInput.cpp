#include "SLingTuSimShowroomInput.h"
#include "Framework/Application/SlateApplication.h"
#include "Framework/Application/SlateUser.h"
#include "Input/Events.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Layout/SBorder.h"

namespace LingTuSim::UI {
void SLingTuSimShowroomInput::Construct(const FArguments& Args) {
  CanInteract = Args._CanInteract;
  OnGesture = Args._OnGesture;
  SetTag(TEXT("LingTu.Selection.Model"));
  SetCursor(TAttribute<TOptional<EMouseCursor::Type>>::CreateLambda([this]() {
    return TOptional<EMouseCursor::Type>(!CanInteract.Get() ? EMouseCursor::Default : IsDragging() ? EMouseCursor::GrabHandClosed : EMouseCursor::GrabHand);
  }));
  ChildSlot[SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush(TEXT("WhiteBrush")))
    .BorderBackgroundColor(FLinearColor::Transparent).Padding(0)];
}
FReply SLingTuSimShowroomInput::OnMouseButtonDown(const FGeometry&, const FPointerEvent& Event) {
  const FKey Button = Event.GetEffectingButton();
  if (!CanInteract.Get() || !OnGesture || (Button != EKeys::LeftMouseButton &&
      Button != EKeys::RightMouseButton && Button != EKeys::MiddleMouseButton)) return FReply::Unhandled();
  if (IsDragging()) return FReply::Handled();
  DragButton = Button;
  bPan = Button == EKeys::MiddleMouseButton || (Button == EKeys::LeftMouseButton && Event.IsShiftDown());
  return FReply::Handled().CaptureMouse(SharedThis(this)).SetUserFocus(SharedThis(this), EFocusCause::Mouse);
}
FReply SLingTuSimShowroomInput::OnMouseButtonUp(const FGeometry&, const FPointerEvent& Event) {
  if (Event.GetEffectingButton() != DragButton) return FReply::Unhandled();
  DragButton = FKey();
  return FReply::Handled().ReleaseMouseCapture();
}
FReply SLingTuSimShowroomInput::OnMouseMove(const FGeometry& Geometry, const FPointerEvent& Event) {
  if (!IsDragging()) return FReply::Unhandled();
  if (!CanInteract.Get() || !Event.IsMouseButtonDown(DragButton)) {
    DragButton = FKey();
    return FReply::Handled().ReleaseMouseCapture();
  }
  const FVector2D Size = Geometry.GetLocalSize();
  if (Size.X > 0 && Size.Y > 0 && OnGesture) {
    const FVector2D LocalDelta = Geometry.AbsoluteToLocal(Event.GetScreenSpacePosition()) -
        Geometry.AbsoluteToLocal(Event.GetLastScreenSpacePosition());
    if (FMath::IsFinite(LocalDelta.X) && FMath::IsFinite(LocalDelta.Y))
      OnGesture(bPan ? EShowroomGesture::Pan : EShowroomGesture::Orbit, LocalDelta / Size);
  }
  return FReply::Handled();
}
FReply SLingTuSimShowroomInput::OnMouseWheel(const FGeometry&, const FPointerEvent& Event) {
  if (!CanInteract.Get() || !OnGesture) return FReply::Unhandled();
  if (FMath::IsFinite(Event.GetWheelDelta())) OnGesture(EShowroomGesture::Zoom, FVector2D(Event.GetWheelDelta(), 0));
  return FReply::Handled();
}
FReply SLingTuSimShowroomInput::OnMouseButtonDoubleClick(const FGeometry&, const FPointerEvent& Event) {
  if (!CanInteract.Get() || !OnGesture || Event.GetEffectingButton() != EKeys::LeftMouseButton) return FReply::Unhandled();
  DragButton = FKey();
  OnGesture(EShowroomGesture::Reset, FVector2D::ZeroVector);
  return FReply::Handled().ReleaseMouseCapture();
}
bool SLingTuSimShowroomInput::IsViewKey(const FKey& Key) {
  return Key == EKeys::Left || Key == EKeys::Right || Key == EKeys::Up || Key == EKeys::Down ||
      Key == EKeys::Home || Key == EKeys::Add || Key == EKeys::Subtract || Key == EKeys::Equals || Key == EKeys::Hyphen;
}
FReply SLingTuSimShowroomInput::OnKeyDown(const FGeometry&, const FKeyEvent& Event) {
  const auto Key = Event.GetKey();
  if (!CanInteract.Get() || !OnGesture || !IsViewKey(Key)) return FReply::Unhandled();
  if (Key == EKeys::Home) OnGesture(EShowroomGesture::Reset, FVector2D::ZeroVector);
  else if (Key == EKeys::Add || Key == EKeys::Equals || Key == EKeys::Subtract || Key == EKeys::Hyphen)
    OnGesture(EShowroomGesture::Zoom, FVector2D(Key == EKeys::Add || Key == EKeys::Equals ? 1 : -1, 0));
  else OnGesture(EShowroomGesture::Orbit, FVector2D(Key == EKeys::Left ? -.025 : Key == EKeys::Right ? .025 : 0,
      Key == EKeys::Up ? -.025 : Key == EKeys::Down ? .025 : 0));
  return FReply::Handled();
}
void SLingTuSimShowroomInput::CancelDrag() {
  DragButton = FKey();
  if (HasMouseCapture() && FSlateApplication::IsInitialized())
    FSlateApplication::Get().GetCursorUser()->ReleaseCapture(FSlateApplication::CursorPointerIndex);
}
void SLingTuSimShowroomInput::OnMouseCaptureLost(const FCaptureLostEvent&) { DragButton = FKey(); }
void SLingTuSimShowroomInput::OnFocusLost(const FFocusEvent& Event) { CancelDrag(); SCompoundWidget::OnFocusLost(Event); }
void SLingTuSimShowroomInput::Tick(const FGeometry& Geometry, double Time, float Delta) {
  SCompoundWidget::Tick(Geometry, Time, Delta);
  if (IsDragging() && (!CanInteract.Get() || !FSlateApplication::Get().IsActive())) CancelDrag();
}
}
