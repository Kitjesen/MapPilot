#include "LingTuSimFrontEndLogin.h"

#include "LingTuSimGameSelection.h"

namespace LingTuSim::UI {
namespace {
const TCHAR *FrozenReason =
    TEXT("Login is locked because the session selection is already confirmed.");
}

void FFrontEndLoginModel::BindSessionModel(const FGameSelectionModel &InSessionModel) {
  SessionModel = &InSessionModel;
}

bool FFrontEndLoginModel::IsFrozen() const {
  return SessionModel != nullptr && SessionModel->IsConfirmed();
}

bool FFrontEndLoginModel::CanConfirmSession(FString &OutError) const {
  OutError.Reset();
  if (SessionModel == nullptr) {
    OutError = TEXT("Login is not bound to a session selection.");
    return false;
  }
  if (!IsLoggedIn()) {
    OutError = TEXT("Log in as a Local Operator before confirming a session.");
    return false;
  }
  return SessionModel->CanConfirm(OutError);
}

bool FFrontEndLoginModel::SubmitLocalOperator(const FString &InDisplayName, FString &OutError) {
  OutError.Reset();
  if (IsFrozen()) {
    OutError = FrozenReason;
    return false;
  }

  const FString TrimmedDisplayName = InDisplayName.TrimStartAndEnd();
  if (TrimmedDisplayName.IsEmpty()) {
    OutError = TEXT("Local Operator display name is required.");
    return false;
  }
  if (TrimmedDisplayName.Len() > 64) {
    OutError = TEXT("Local Operator display name must be 64 characters or fewer.");
    return false;
  }
  for (const TCHAR Character : TrimmedDisplayName) {
    if (Character < TEXT(' ') || Character == 0x7f) {
      OutError = TEXT("Local Operator display name cannot contain control characters.");
      return false;
    }
  }

  Method = EFrontEndLoginMethod::LocalOperator;
  DisplayName = TrimmedDisplayName;
  return true;
}

bool FFrontEndLoginModel::SubmitOnlineAccount(FString &OutError) {
  OutError.Reset();
  if (IsFrozen()) {
    OutError = FrozenReason;
    return false;
  }

  OutError = OnlineUnavailableReason;
  return false;
}

bool FFrontEndLoginModel::Logout(FString &OutError) {
  OutError.Reset();
  if (IsFrozen()) {
    OutError = FrozenReason;
    return false;
  }
  if (!IsLoggedIn()) {
    OutError = TEXT("No local operator is logged in.");
    return false;
  }

  Method = EFrontEndLoginMethod::None;
  DisplayName.Reset();
  return true;
}
}  // namespace LingTuSim::UI
