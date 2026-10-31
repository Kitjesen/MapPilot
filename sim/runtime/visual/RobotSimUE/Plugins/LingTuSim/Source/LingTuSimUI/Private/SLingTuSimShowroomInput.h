#pragma once

#include "Widgets/SCompoundWidget.h"

namespace LingTuSim::UI {
enum class EShowroomGesture { Orbit, Pan, Zoom, Reset };

/** Hit-test surface for a presentation camera, never a gameplay input source. */
class SLingTuSimShowroomInput final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimShowroomInput) : _CanInteract(false) {}
  SLATE_ATTRIBUTE(bool, CanInteract)
  SLATE_ARGUMENT(TFunction<void(EShowroomGesture, FVector2D)>, OnGesture)
  SLATE_END_ARGS()
  void Construct(const FArguments&);
  virtual bool SupportsKeyboardFocus() const override { return true; }
  virtual FReply OnMouseButtonDown(const FGeometry&, const FPointerEvent&) override;
  virtual FReply OnMouseButtonUp(const FGeometry&, const FPointerEvent&) override;
  virtual FReply OnMouseMove(const FGeometry&, const FPointerEvent&) override;
  virtual FReply OnMouseWheel(const FGeometry&, const FPointerEvent&) override;
  virtual FReply OnMouseButtonDoubleClick(const FGeometry&, const FPointerEvent&) override;
  virtual FReply OnKeyDown(const FGeometry&, const FKeyEvent&) override;
  virtual void OnMouseCaptureLost(const FCaptureLostEvent&) override;
  virtual void OnFocusLost(const FFocusEvent&) override;
  virtual void Tick(const FGeometry&, double, float) override;
  void CancelDrag();
  bool IsDragging() const { return DragButton.IsValid(); }
  static bool IsViewKey(const FKey&);
 private:
  TAttribute<bool> CanInteract;
  TFunction<void(EShowroomGesture, FVector2D)> OnGesture;
  FKey DragButton;
  bool bPan = false;
};
}
