# ruff: noqa: S101
"""Static contracts for the UE5 front-end gamepad input boundary."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
UI = (
    ROOT
    / "sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Source/LingTuSimUI"
)


def test_frontend_gamepad_reuses_runtime_ui_actions() -> None:
    model = (UI / "Private/LingTuSimRuntimeUIModel.cpp").read_text(encoding="utf-8")

    for key in (
        "EKeys::Gamepad_FaceButton_Bottom",
        "EKeys::Gamepad_FaceButton_Right",
        "EKeys::Gamepad_DPad_Up",
        "EKeys::Gamepad_DPad_Down",
        "EKeys::Gamepad_DPad_Left",
        "EKeys::Gamepad_DPad_Right",
        "EKeys::Gamepad_LeftShoulder",
        "EKeys::Gamepad_RightShoulder",
    ):
        assert key in model

    for action in (
        "ERuntimeUIAction::SelectPreviousGame",
        "ERuntimeUIAction::SelectNextGame",
        "ERuntimeUIAction::SelectPreviousAsset",
        "ERuntimeUIAction::SelectNextAsset",
        "ERuntimeUIAction::ConfirmGameSelection",
    ):
        assert action in model

    enum_contract = (UI / "Public/LingTuSimRuntimeUIModel.h").read_text(
        encoding="utf-8"
    )
    assert "ReturnFromFrontEnd" in enum_contract


def test_frontend_lock_swallow_precedes_robot_drive_input() -> None:
    processor = (
        UI / "Private/LingTuSimRuntimeUIInputProcessor.cpp"
    ).read_text(encoding="utf-8")

    lock_guard = "if (FrontEndLockQuery && FrontEndLockQuery())"
    assert processor.count(lock_guard) >= 4
    assert processor.index(lock_guard, processor.index("HandleKeyDownEvent")) < processor.index(
        "DriveInput.HandleKeyDown", processor.index("HandleKeyDownEvent")
    )
    assert processor.index(lock_guard, processor.index("HandleKeyUpEvent")) < processor.index(
        "DriveInput.HandleKeyUp", processor.index("HandleKeyUpEvent")
    )
    assert processor.index(
        lock_guard, processor.index("HandleAnalogInputEvent")
    ) < processor.index("DriveInput.HandleAnalog", processor.index("HandleAnalogInputEvent"))


def test_focused_button_keeps_native_keyboard_and_gamepad_activation() -> None:
    processor = (
        UI / "Private/LingTuSimRuntimeUIInputProcessor.cpp"
    ).read_text(encoding="utf-8")

    assert "bool IsConfirmKey" in processor
    assert "Key == EKeys::Enter || Key == EKeys::Gamepad_FaceButton_Bottom" in processor
    assert "Let the focused SButton handle keyboard or gamepad accept" in processor
    assert "FocusedWidget->GetVisibility().IsVisible()" in processor


def test_gamepad_back_uses_authority_resolved_frontend_action() -> None:
    processor = (
        UI / "Private/LingTuSimRuntimeUIInputProcessor.cpp"
    ).read_text(encoding="utf-8")
    subsystem = (
        UI / "Private/LingTuSimRuntimeUIWorldSubsystem.cpp"
    ).read_text(encoding="utf-8")
    subsystem_header = (
        UI / "Private/LingTuSimRuntimeUIWorldSubsystem.h"
    ).read_text(encoding="utf-8")
    hud = (UI / "Private/SLingTuSimRuntimeHUD.cpp").read_text(encoding="utf-8")
    hud_header = (UI / "Private/SLingTuSimRuntimeHUD.h").read_text(encoding="utf-8")

    assert "ActionPublisher(ERuntimeUIAction::ReturnFromFrontEnd)" in processor
    assert "case LingTuSim::UI::ERuntimeUIAction::ReturnFromFrontEnd:" in subsystem
    assert "FrontEndLoginModel->Logout(Error)" in subsystem
    assert "if (!bGameSelector" in subsystem
    assert "RuntimeHUDWidget->FocusFrontEndLoginCTA()" in subsystem
    assert "TSharedPtr<LingTuSim::UI::SLingTuSimRuntimeHUD> RuntimeHUDWidget" in subsystem_header
    assert "void FocusFrontEndLoginCTA();" in hud_header
    assert "RegisterActiveTimer(" in hud
    assert "LoginButton->GetVisibility().IsVisible()" in hud
    assert "SetKeyboardFocus(LoginButton, EFocusCause::SetDirectly)" in hud
