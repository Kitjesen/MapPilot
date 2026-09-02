# ruff: noqa: S101
"""Static design contracts for the UE5 tactical front end.

These checks deliberately inspect the checked-in Slate source.  Runtime authority
and model behavior remain covered by ``LingTuSimRuntimeUITest.cpp``; this file only
locks the presentation vocabulary and the read-only asset-browsing seam.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
HUD_CPP = (
    REPO_ROOT
    / "sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Source/LingTuSimUI/Private"
    / "SLingTuSimRuntimeHUD.cpp"
)
HUD_H = HUD_CPP.with_suffix(".h")
UI_PRIVATE = HUD_CPP.parent
INPUT_PROCESSOR_CPP = UI_PRIVATE / "LingTuSimRuntimeUIInputProcessor.cpp"
UI_MODEL_CPP = UI_PRIVATE / "LingTuSimRuntimeUIModel.cpp"
WORLD_SUBSYSTEM_CPP = UI_PRIVATE / "LingTuSimRuntimeUIWorldSubsystem.cpp"

ASSET_PREVIEW_RESOURCES = {
    "thunder_v4": "preview-thunder-v4-v1.png",
    "rws_01": "preview-rws-01-v1.png",
    "forest_pine": "preview-forest-pine-v1.png",
    "forest_birch": "preview-forest-birch-v1.png",
    "forest_boulder": "preview-forest-boulder-v1.png",
}


def _hud_source() -> str:
    return HUD_CPP.read_text(encoding="utf-8")


def _front_end_layout(source: str) -> str:
    """Return the actual Slate builders that define the front-end presentation."""

    return "\n".join(
        _function_body(source, function_name)
        for function_name in (
            "BuildLoginScreen",
            "BuildWorkspaceScreen",
            "BuildAssetHeroPreview",
            "BuildAssetFilmstrip",
        )
    )


def _workspace_player_surface(source: str) -> str:
    """Return the workspace layout plus text getters rendered on its first screen."""

    return "\n".join(
        _function_body(source, function_name)
        for function_name in (
            "BuildWorkspaceScreen",
            "BuildAssetHeroPreview",
            "BuildAssetFilmstrip",
            "GetSelectionPositionText",
            "GetSelectionPackagesText",
            "GetSelectionStatusText",
            "GetAssetReviewPositionText",
            "GetAssetReviewDispositionText",
            "GetAssetReviewClassStageText",
            "GetAssetReviewEvidenceText",
            "GetAssetReviewPolicyText",
        )
    )


def _qualified_function_body(source: str, qualified_name: str) -> str:
    definition = re.search(
        rf"^[^\s\r\n][^\r\n]*{re.escape(qualified_name)}\s*\(",
        source,
        re.MULTILINE,
    )
    if definition is None:
        raise AssertionError(f"missing function definition: {qualified_name}")
    start = definition.start()
    opening_brace = source.index("{", start)
    depth = 0
    for index in range(opening_brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening_brace : index + 1]
    raise AssertionError(f"unterminated function body: {qualified_name}")


def _function_body(source: str, function_name: str) -> str:
    return _qualified_function_body(source, f"SLingTuSimRuntimeHUD::{function_name}")


def _overlay_slot_containing(construct: str, marker: str) -> str:
    """Return the top-level overlay slot that owns one visible HUD marker."""

    marker_index = construct.index(marker)
    slot_starts = [
        match.start()
        for match in re.finditer(r"\+\s*SOverlay::Slot\(\)", construct)
    ]
    owning_starts = [start for start in slot_starts if start <= marker_index]
    if not owning_starts:
        raise AssertionError(f"missing overlay slot for HUD marker: {marker}")
    slot_start = owning_starts[-1]
    following_starts = [start for start in slot_starts if start > marker_index]
    slot_end = following_starts[0] if following_starts else -1
    return construct[slot_start : slot_end if slot_end != -1 else None]


def test_front_end_avoids_white_saas_panels_and_login_copy() -> None:
    layout = _front_end_layout(_hud_source())

    assert ".BorderBackgroundColor(WarmWhitePanel)" not in layout
    assert ".BorderBackgroundColor(WarmWhiteGlass)" not in layout
    for forbidden_copy in (
        "进入灵途仿真",
        "本地操作员  ·  OFFLINE WORKSPACE",
        "进入本地工作台",
        "在线工作区  ·  尚未接入认证",
        "生成登录令牌",
    ):
        assert forbidden_copy not in layout


def test_workspace_first_screen_hides_backend_catalog_vocabulary() -> None:
    surface = _workspace_player_surface(_hud_source())

    for forbidden_copy in (
        "COMPILED OPTION",
        "DISPOSITION",
        "CATALOG_REVIEW",
        "AUDIT PATHS ONLY",
        "NO THUMBNAIL LOAD",
    ):
        if forbidden_copy in surface:
            raise AssertionError(
                f"workspace first screen exposes backend vocabulary: {forbidden_copy}"
            )


def test_workspace_header_shows_current_context_instead_of_fake_navigation() -> None:
    workspace = _function_body(_hud_source(), "BuildWorkspaceScreen")

    assert "任务   机器人   场景   素材库   设置" not in workspace
    for current_context in ("部署准备", "素材审阅"):
        assert current_context in workspace


def test_workspace_first_screen_uses_chinese_player_facing_mission_copy() -> None:
    source = _hud_source()
    surface = _front_end_layout(source) + "\n" + _workspace_player_surface(source)

    assert any(mission_brief in surface for mission_brief in ("任务简报", "行动简报"))
    for required_copy in ("巡检场景", "开始任务"):
        if required_copy not in surface:
            raise AssertionError(
                f"workspace first screen is missing player-facing copy: {required_copy}"
            )


def test_workspace_uses_the_scene_as_its_root_interface_surface() -> None:
    workspace = _function_body(_hud_source(), "BuildWorkspaceScreen")
    return_expression = workspace[workspace.index("return") :]
    hero_index = return_expression.index("BuildAssetHeroPreview()")
    root_to_hero = return_expression[:hero_index]

    assert re.search(r"return\s+SNew\(SOverlay\)", root_to_hero)
    assert "SNew(SVerticalBox)" not in root_to_hero


def test_workspace_does_not_recreate_large_fixed_side_panels_over_the_scene() -> None:
    workspace = _function_body(_hud_source(), "BuildWorkspaceScreen")

    fixed_widths = [
        float(width)
        for width in re.findall(r"WidthOverride\(\s*([0-9]+(?:\.[0-9]+)?)F?\s*\)", workspace)
    ]
    assert all(width < 300.0 for width in fixed_widths)


def test_asset_hero_preview_is_not_wrapped_in_an_outer_frame() -> None:
    hero = _function_body(_hud_source(), "BuildAssetHeroPreview")

    assert re.search(r"return\s+SNew\(SOverlay\)", hero)


def test_asset_selector_uses_compact_auto_width_items() -> None:
    filmstrip = _function_body(_hud_source(), "BuildAssetFilmstrip")
    card_loop = filmstrip[
        filmstrip.index("for (int32 CardIndex") : filmstrip.index(
            "if (VisibleAssetCards == 0)"
        )
    ]

    assert re.search(r"Filmstrip->AddSlot\(\)\s*\.AutoWidth\(\)", card_loop)
    assert ".FillWidth(1.0F)" not in card_loop


def test_asset_selector_has_no_enclosing_panel() -> None:
    filmstrip = _function_body(_hud_source(), "BuildAssetFilmstrip")

    assert not re.search(r"return\s+SNew\(SBorder\)", filmstrip)


def test_workspace_keeps_runtime_limitations_in_secondary_technical_details() -> None:
    surface = _workspace_player_surface(_hud_source())

    assert any(label in surface for label in ("未资格化", "未取得运行资格"))
    assert "UnrealCollisionProfile" in surface
    assert 'TEXT("NoCollision")' in surface
    assert "碰撞关闭" in surface
    assert "PhysicsAuthority" in surface
    assert "MuJoCo" in surface
    assert "物理由 %s 负责" in surface
    assert any(label in surface for label in ("非当前运行画面", "非当前游戏画面"))


def test_first_screen_uses_a_chinese_local_mission_call_to_action() -> None:
    layout = _front_end_layout(_hud_source())

    assert any(call_to_action in layout for call_to_action in ("继续巡检", "开始本地任务"))
    assert not re.search(r'TEXT\("(?:LOGIN|SIGN IN|ENTER WORKSPACE)', layout, re.IGNORECASE)


def test_asset_library_declares_hero_preview_filmstrip_and_one_focus_style() -> None:
    source = HUD_H.read_text(encoding="utf-8") + _hud_source()

    assert "BuildAssetHeroPreview" in source
    assert "BuildAssetFilmstrip" in source
    assert "GetAssetFilmstripCardColor" in source


def test_front_end_exposes_keyboard_and_controller_navigation_hints() -> None:
    layout = _front_end_layout(_hud_source())

    for required_hint in ("A / ENTER", "B / ESC"):
        assert required_hint in layout
    workspace = _function_body(_hud_source(), "BuildWorkspaceScreen")
    assert "LB / RB" in workspace
    assert any(hint in workspace for hint in ("浏览素材", "切换素材"))


def test_front_end_assigns_each_focus_target_exactly_once() -> None:
    source = _hud_source()
    login = _function_body(source, "BuildLoginScreen")
    workspace = _function_body(source, "BuildWorkspaceScreen")

    assert login.count("SAssignNew(OperatorNameTextBox, SEditableTextBox)") == 1
    assert login.count("SAssignNew(LocalWorkspaceButton, SButton)") == 1
    assert workspace.count("SAssignNew(ConfirmSelectionButton, SButton)") == 1
    assert source.count("SAssignNew(OperatorNameTextBox, SEditableTextBox)") == 1
    assert source.count("SAssignNew(LocalWorkspaceButton, SButton)") == 1
    assert source.count("SAssignNew(ConfirmSelectionButton, SButton)") == 1


def test_login_logout_and_initial_focus_reference_the_assigned_controls() -> None:
    source = _hud_source()
    construct = _function_body(source, "Construct")
    focus_login_cta = _function_body(source, "FocusFrontEndLoginCTA")
    enter_workspace = _function_body(source, "HandleEnterLocalWorkspace")
    logout = _function_body(source, "HandleLogout")

    assert "FocusFrontEndLoginCTA()" in construct
    assert "LocalWorkspaceButton.IsValid()" in focus_login_cta
    assert "SetKeyboardFocus(LoginButton" in focus_login_cta
    assert "LoginModel->SubmitLocalOperator" in enter_workspace
    assert "ConfirmSelectionButton.IsValid()" in enter_workspace
    assert "SetUserFocus(ConfirmSelectionButton.ToSharedRef()" in enter_workspace
    assert "LoginModel->Logout" in logout
    assert "OperatorNameTextBox.IsValid()" in logout
    assert "SetUserFocus(OperatorNameTextBox.ToSharedRef()" in logout


def test_controller_accept_hint_is_backed_by_the_confirm_action_mapping() -> None:
    source = UI_MODEL_CPP.read_text(encoding="utf-8")
    resolve_key = _qualified_function_body(source, "FRuntimeUIActionPolicy::ResolveKey")

    assert "EKeys::Gamepad_FaceButton_Bottom" in resolve_key
    assert "ERuntimeUIAction::ConfirmGameSelection" in resolve_key


def test_front_end_back_keys_publish_return_without_toggling_the_runtime_mode() -> None:
    source = INPUT_PROCESSOR_CPP.read_text(encoding="utf-8")
    handle_key_down = _qualified_function_body(
        source, "FLingTuSimRuntimeUIInputProcessor::HandleKeyDownEvent"
    )
    front_end_branch = handle_key_down[handle_key_down.index("if (IsRuntimeUIKey(Key))") :]

    assert "FrontEndLockQuery && FrontEndLockQuery()" in front_end_branch
    assert "Key == EKeys::Escape" in front_end_branch
    assert "Key == EKeys::Gamepad_FaceButton_Right" in front_end_branch
    assert "ActionPublisher(ERuntimeUIAction::ReturnFromFrontEnd)" in front_end_branch


def test_controller_shoulder_hints_are_backed_by_selection_action_mappings() -> None:
    source = UI_MODEL_CPP.read_text(encoding="utf-8")
    resolve_key = _qualified_function_body(source, "FRuntimeUIActionPolicy::ResolveKey")

    assert "EKeys::Gamepad_LeftShoulder" in resolve_key
    assert "ERuntimeUIAction::SelectPreviousAsset" in resolve_key
    assert "EKeys::Gamepad_RightShoulder" in resolve_key
    assert "ERuntimeUIAction::SelectNextAsset" in resolve_key


def test_input_processor_publishes_actions_and_enforces_the_front_end_lock() -> None:
    source = INPUT_PROCESSOR_CPP.read_text(encoding="utf-8")
    handle_key_down = _qualified_function_body(
        source, "FLingTuSimRuntimeUIInputProcessor::HandleKeyDownEvent"
    )

    assert "FRuntimeUIActionPolicy::ResolveKey" in handle_key_down
    assert "ActionPublisher(Action)" in handle_key_down
    assert "FrontEndLockQuery && FrontEndLockQuery()" in handle_key_down


def test_selector_actions_require_a_logged_in_front_end_model() -> None:
    source = WORLD_SUBSYSTEM_CPP.read_text(encoding="utf-8")
    handle_action = _qualified_function_body(
        source, "ULingTuSimRuntimeUIWorldSubsystem::HandleUIAction"
    )

    assert "if (bGameSelector)" in handle_action
    assert "!FrontEndLoginModel.IsValid() || !FrontEndLoginModel->IsLoggedIn()" in handle_action
    assert "return;" in handle_action


def test_selector_actions_route_confirm_and_asset_navigation_after_login() -> None:
    source = WORLD_SUBSYSTEM_CPP.read_text(encoding="utf-8")
    handle_action = _qualified_function_body(
        source, "ULingTuSimRuntimeUIWorldSubsystem::HandleUIAction"
    )

    for action, handler in (
        ("SelectPreviousAsset", "HandlePreviousAssetReview"),
        ("SelectNextAsset", "HandleNextAssetReview"),
        ("ConfirmGameSelection", "HandleGameSelectionConfirm"),
    ):
        action_case = handle_action.index(
            f"case LingTuSim::UI::ERuntimeUIAction::{action}:"
        )
        next_case = handle_action.find("case LingTuSim::UI::ERuntimeUIAction::", action_case + 1)
        case_body = handle_action[action_case : next_case if next_case != -1 else None]
        assert handler in case_body


def test_front_end_back_action_logs_out_instead_of_toggling_a_hidden_mode() -> None:
    source = WORLD_SUBSYSTEM_CPP.read_text(encoding="utf-8")
    handle_action = _qualified_function_body(
        source, "ULingTuSimRuntimeUIWorldSubsystem::HandleUIAction"
    )
    action_case = handle_action.index(
        "case LingTuSim::UI::ERuntimeUIAction::ReturnFromFrontEnd:"
    )
    next_case = handle_action.find("case LingTuSim::UI::ERuntimeUIAction::", action_case + 1)
    case_body = handle_action[action_case : next_case if next_case != -1 else None]

    assert "FrontEndLoginModel->Logout" in case_body
    assert "RuntimeHUDWidget->FocusFrontEndLoginCTA()" in case_body
    assert "TogglePause" not in case_body


def test_asset_browsing_handlers_do_not_confirm_or_write_session_intent() -> None:
    source = _hud_source()
    bodies = "\n".join(
        _function_body(source, function_name)
        for function_name in (
            "HandlePreviousAssetReview",
            "HandleNextAssetReview",
            "HandleSelectAssetReview",
        )
    )

    assert "OnSelectionConfirmed" not in bodies
    assert "MarkConfirmed" not in bodies
    assert "SelectionIntent" not in bodies
    assert "FFileHelper" not in bodies


def test_hud_references_each_versioned_asset_preview_resource() -> None:
    source = HUD_H.read_text(encoding="utf-8") + _hud_source()

    for resource_name in ASSET_PREVIEW_RESOURCES.values():
        assert resource_name in source


def test_hud_maps_exact_catalog_ids_to_their_preview_resources() -> None:
    source = _hud_source()

    for asset_id, resource_name in ASSET_PREVIEW_RESOURCES.items():
        mapping = rf'TEXT\("{re.escape(asset_id)}"\).{{0,240}}{re.escape(resource_name)}'
        assert re.search(mapping, source, re.DOTALL), (
            f"missing exact preview mapping: {asset_id} -> {resource_name}"
        )


def test_workspace_hero_leaves_the_live_world_viewport_visible() -> None:
    hero = _function_body(_hud_source(), "BuildAssetHeroPreview")

    for static_preview_plate in (
        "FrontEndBackgroundBrush",
        "GetAssetPreviewBrush",
        "PreviewBrush",
        "SNew(SImage)",
        "EStretch::ScaleToFill",
    ):
        assert static_preview_plate not in hero


def test_static_front_end_backdrop_is_bound_to_login_visibility() -> None:
    source = _hud_source()
    construct = _function_body(source, "Construct")
    static_backdrop = re.search(
        r"SNew\(SImage\)"
        r"(?:(?!SNew\(SImage\)).)*"
        r"\.Visibility\(this,\s*&SLingTuSimRuntimeHUD::(\w+)\)"
        r"(?:(?!SNew\(SImage\)).)*"
        r"FrontEndBackgroundBrush",
        construct,
        re.DOTALL,
    )

    assert static_backdrop, "missing the login scene plate"
    assert static_backdrop.group(1) == "GetFrontEndStaticBackdropVisibility"


def test_static_front_end_backdrop_is_visible_for_login_only() -> None:
    source = HUD_H.read_text(encoding="utf-8") + "\n" + _hud_source()
    visibility = _qualified_function_body(
        source, "SLingTuSimRuntimeHUD::GetFrontEndStaticBackdropVisibility"
    )

    assert "GetLoginVisibility()" in visibility
    assert "EVisibility::HitTestInvisible" in visibility
    assert "EVisibility::Collapsed" in visibility
    assert "GetWorkspaceVisibility()" not in visibility


def test_game_selector_hides_all_legacy_runtime_hud_chrome_slots() -> None:
    construct = _function_body(_hud_source(), "Construct")

    for panel_marker in (
        "&SLingTuSimRuntimeHUD::GetReadinessText",
        'TEXT("CONTROL / REQUEST")',
        'TEXT("OBSERVED / MUJOCO TRUTH")',
    ):
        panel_slot = _overlay_slot_containing(construct, panel_marker)
        assert re.search(
            r"\.Visibility\(\s*this,\s*"
            r"&SLingTuSimRuntimeHUD::GetRuntimeHudChromeVisibility\)",
            panel_slot,
        ), f"legacy runtime HUD panel is still visible in GameSelector: {panel_marker}"


def test_runtime_hud_chrome_visibility_preserves_the_non_selector_hud() -> None:
    source = HUD_H.read_text(encoding="utf-8") + "\n" + _hud_source()
    visibility = _qualified_function_body(
        source, "SLingTuSimRuntimeHUD::GetRuntimeHudChromeVisibility"
    )

    assert "bFrontEndLoginRequired" in visibility
    assert "EVisibility::Collapsed" in visibility
    assert "EVisibility::Visible" in visibility
    assert "GetLoginVisibility()" not in visibility
    assert "GetWorkspaceVisibility()" not in visibility


def test_each_filmstrip_card_contains_an_asset_thumbnail() -> None:
    body = _function_body(_hud_source(), "BuildAssetFilmstrip")
    card_loop = body[body.index("for (int32 CardIndex") :]

    assert "SNew(SImage)" in card_loop
    assert ".Image(" in card_loop
    assert "PreviewBrush" in card_loop


def test_asset_thumbnail_strip_keeps_honest_non_qualification_labels() -> None:
    source = _hud_source()
    workspace = _function_body(source, "BuildWorkspaceScreen")
    filmstrip = _function_body(source, "BuildAssetFilmstrip")
    hero = _function_body(source, "BuildAssetHeroPreview")
    thumbnail_surface = workspace + "\n" + filmstrip

    assert "离线素材缩略图" in thumbnail_surface
    assert "非当前游戏画面" in thumbnail_surface
    assert "非当前游戏画面" not in hero
    assert "实机审阅截图" not in thumbnail_surface
    assert any(label in thumbnail_surface for label in ("未资格化", "未取得运行资格"))


def test_preview_surfaces_do_not_refer_to_a_removed_right_status_panel() -> None:
    surface = _workspace_player_surface(_hud_source())

    assert "安全状态见右侧" not in surface


def test_enabled_primary_actions_use_high_contrast_text_on_highlight() -> None:
    source = _hud_source()
    login = _function_body(source, "BuildLoginScreen")
    workspace = _function_body(source, "BuildWorkspaceScreen")

    for body, action_copy in (
        (login, "开始本地任务"),
        (workspace, "开始任务"),
    ):
        assert "? FieldAmber" in body
        label = re.search(
            rf'StaticLabel\(\s*TEXT\("{action_copy}[^\"]*"\),\s*\d+,\s*(\w+)\)',
            body,
            re.DOTALL,
        )
        assert label, f"missing primary action label: {action_copy}"
        assert label.group(1) in {"FrontEndText", "White"}
