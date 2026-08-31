#pragma once

#include "CoreMinimal.h"

namespace LingTuSim::UI {
class FGameSelectionModel;

enum class EFrontEndLoginMethod : uint8 {
  None,
  LocalOperator,
  OnlineAccount,
};

/**
 * Memory-only front-end identity gate. It never accepts or stores credentials,
 * tokens, or passwords, and freezes when the bound session is confirmed.
 */
class LINGTUSIMUI_API FFrontEndLoginModel final {
 public:
  void BindSessionModel(const FGameSelectionModel &InSessionModel);

  EFrontEndLoginMethod GetMethod() const { return Method; }
  const FString &GetDisplayName() const { return DisplayName; }
  const FString &GetOnlineUnavailableReason() const { return OnlineUnavailableReason; }

  bool IsLoggedIn() const { return Method != EFrontEndLoginMethod::None; }
  bool IsFrozen() const;
  bool CanEdit() const { return !IsFrozen(); }
  bool CanConfirmSession(FString &OutError) const;

  bool SubmitLocalOperator(const FString &InDisplayName, FString &OutError);
  bool SubmitOnlineAccount(FString &OutError);
  bool Logout(FString &OutError);

 private:
  EFrontEndLoginMethod Method = EFrontEndLoginMethod::None;
  FString DisplayName;
  FString OnlineUnavailableReason =
      TEXT("Online Account is unavailable in this build. Use Local Operator.");
  const FGameSelectionModel *SessionModel = nullptr;
};
}  // namespace LingTuSim::UI
