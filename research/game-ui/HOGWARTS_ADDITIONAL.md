# Hogwarts Legacy additional UI evidence

Date: 2026-10-13. This is a file-level reference pass for RobotSimUE's task/status HUD. The game was not launched or modified, no key was used, and no game asset was copied into project `Content/`.

## Bounded extraction

The existing complete IoStore index at `build/game-ui-research-20261013/hogwarts/pakchunk0-iostore.csv` was filtered to five UI families. UE 5.8 `UnrealPak.exe IoStore -FindAndExtract` was then run once per precise wildcard into the new `additional/cooked/` lane:

| Package filter | Files | Extracted bytes |
| --- | ---: | ---: |
| `*/UI/HUD/Notifications/*` | 19 | 1,117,635 |
| `*/UI/HUD/QuickAction/*` | 8 | 271,251 |
| `*/UI/Common/Legend/*` | 12 | 125,576 |
| `*/UI/Common/Popup/*` | 2 | 150,591 |
| `*/UI/Menus/FieldGuide/Index/*` | 9 | 1,082,262 |
| **Total** | **50** | **2,747,315** |

The exact relative names and extracted sizes of all 50 source packages are in `additional/extracted-files.csv`. The five `*-extract.log` files are the corresponding UnrealPak logs. The Field Guide selection contains only the index and its eight named menu buttons; no portrait or world-map texture family was extracted.

## Best actual samples

| Extracted package | Bytes | What the package name/string evidence supports |
| --- | ---: | --- |
| `UI/HUD/Notifications/UI_BP_NotificationItem.uasset` | 174,578 | General item notification with count, icon, paper/glow effects and in/out animations. |
| `UI/HUD/Notifications/UI_BP_NotificationPanel.uasset` | 21,584 | Notification-panel package, separate from individual notification items. |
| `UI/HUD/Notifications/UI_BP_ErrorMessage.uasset` | 26,416 | Compact error-message package. |
| `UI/HUD/QuickAction/UI_BP_QuickHealthActions.uasset` | 140,977 | Segmented health meter, potion action, critical/low-health states and related animation names. |
| `UI/Common/Legend/UI_BP_LegendItem.uasset` | 26,084 | Input legend item with button background, text and optional progress fill. |
| `UI/Common/Popup/UI_BP_GenericPopupScreen.uasset` | 66,341 | Generic popup title/body/content plus legend actions and optional hold/wait progress. |
| `UI/Menus/FieldGuide/Index/UI_BP_FG_Index.uasset` | 203,527 | Field Guide hub referencing eight distinct category-button packages. |

These are cooked packages from the installed game, not editable UE 5.8 source assets.

## Package string evidence

The following names were read directly from the extracted package bytes. They support component/material relationships, but they are **not a reconstructed widget tree**, and do not establish exact hierarchy, anchors, opacity, timing or runtime state transitions.

### `UI_BP_NotificationItem.uasset` — 174,578 bytes

Imported paths include:

- `/Game/UI/Fonts/UI_FNT_Body`
- `/Game/UI/Fonts/UI_FNT_Header`
- `/Game/UI/FX/MI_UI_ItemNotificationGlow`
- `/Game/UI/FX/MI_UI_NotificationPaperFlutter`
- `/Game/UI/Icons/MenuAssets/UI_T_QuestItemBack`
- `/Game/UI/Menus/FieldGuide/Inventory/UI_BP_InventoryFunctionLibrary`

Observed component/property names include `CanvasPanel`, `HorizontalBox`, `VerticalBox`, `ScaleBox`, `iconBorder`, `paperBorder`, `textBorder`, `ItemCountPanel`, `NotificationIN`, `NotificationOUT`, `NotificationComplete` and `NotificationAborted`.

### `UI_BP_QuickHealthActions.uasset` — 140,977 bytes

Imported paths include:

- `/Game/UI/HUD/QuickAction/UI_T_HealthMeter_LeftEnd_4K`
- `/Game/UI/HUD/QuickAction/UI_T_HealthMeter_Middle_4K`
- `/Game/UI/HUD/QuickAction/UI_T_HealthMeter_RightEnd_4K`
- `/Game/UI/HUD/MiniMap/UI_T_HUD_HealthMeterFill`
- `/Game/UI/FX/MI_UI_LowHealthRipple`
- `/Game/UI/Icons/Potions/UI_T_WoundCleaning`

Observed names include `ProgressBar`, `healthMeterBack`, `newHealthBar`, `HealthPercent`, `HealthMax`, `HealthPotionPanel`, `leftEndScaleBox`, `middleScaleBox`, `rightEndScaleBox`, `CriticalHealth`, `LowHealthIntro`, `LowHealthLoop`, `LowHealthEnd` and `HealthItemFullyRecharged`.

### `UI_BP_LegendItem.uasset` — 26,084 bytes

Imported paths include `/Game/UI/Common/Legend/UI_MI_ButtonBG`, `UI_MI_ButtonBGSquare`, `UI_MI_ProgressBG`, `UI_MI_ProgressFill`, `/Game/UI/Fonts/UI_FNT_Header` and `/Game/UI/Styles/UI_RTS_ButtonLegend`. Observed names include `ButtonBG`, `ButtonText`, `ProgressBG`, `ProgressFill`, `ProgressCurve`, `TextString`, `UsedInputDevice` and `PhoenixRichTextBlock`.

### `UI_BP_GenericPopupScreen.uasset` — 66,341 bytes

Imported paths include `/Game/UI/Common/Legend/UI_BP_Legend_Horizontal_Tooltip`, `/Game/UI/Menus/FieldGuide/UI_BP_FG_DetailsPanelBack`, `/Game/UI/Fonts/UI_FNT_Body`, `/Game/UI/Fonts/UI_FNT_Header` and `/Game/UI/MenuAssets/UI_T_Divider4_4K`. Observed names include `popupTitle`, `popupBody`, `popupBorder`, `popupLegend`, `ContentWidget`, `LegendItems`, `OnHoldButtonPressStart`, `SetProgressUsed` and `SetPopupUsesWaitIndicator`.

### `UI_BP_FG_Index.uasset` — 203,527 bytes

The package directly references the exact button packages `UI_BP_FieldGuideButton_Collections`, `Gear`, `Inventory`, `Map`, `Owlpost`, `Quests`, `Settings` and `Talents`, plus `UI_BP_FG_ActionsButton`. Observed names include `CollectionsButton`, `GearButton`, `InventoryButton`, `MapButton`, `OwlMailButton`, `QuestsButton`, `SettingsButton`, `TalentsButton`, `CategoryLabelText`, `playerLevelText`, `IntroAnimation` and `AmbientBackgroundAnimation`.

## Genuine texture decodes

All 15 extracted `UI_T_*.uasset` files in Notifications and QuickAction matched the existing helper's deliberately narrow single-mip inline-`PF_BC7` shape. One fresh run of `research/game-ui/decode_inline_bc7.py` produced 15 PNGs under `additional/png/`; `decode-report.json` records source bytes, dimensions, pixel lengths, RGBA extrema and successful truncated-payload rejection for every file.

Most useful decoded references:

| PNG | Source bytes | Dimensions | Visual role confirmed by inspection |
| --- | ---: | ---: | --- |
| `UI_T_ChallengeComplete_Back.png` | 66,595 | 256x256 | Dark translucent notification plate with restrained corner/line detail. |
| `UI_T_ChallengeComplete_Border.png` | 66,595 | 256x256 | Separate ornate gold/red border layer with transparent center. |
| `UI_T_ChallengeUpdate_Back.png` | 17,435 | 128x128 | Smaller challenge-update backing. |
| `UI_T_HealthMeter_Middle_4K.png` | 17,597 | 256x64 | Tileable middle segment for the health/status meter. |
| `UI_T_HealthMeter_LeftEnd_4K.png` / `RightEnd_4K.png` | 5,309 each | 64x64 each | Separate end caps for the segmented meter. |
| `UI_T_NotificationGoldLeafWShadow_Top.png` | 33,843 | 512x64 | Wide top ornament/shadow layer for notification framing. |

The decoder was used unmodified. `additional/bc7-input/` contains only the 15 confirmed source-package copies supplied to that one run; `additional/decode.log` records the successful count.

## Adaptation notes

- The strongest reusable structure is separation of notification plate, border/ornament, icon/count content and in/out state—not the fantasy ornament itself.
- QuickAction demonstrates a practical three-part scalable meter with separate fill/back and explicit critical/low-health feedback. RobotSimUE can reuse that component separation with technical geometry and existing fonts.
- Legend and GenericPopup share an input-action model with optional progress/hold feedback. That is a better reference for robot action confirmation than copying a decorative popup.
- Field Guide's index supports a hub made of individually focusable category buttons with hover/click events and a separate label/player-level region. The current evidence does not prove the exact spatial layout.
- Commercial-game fonts, textures and cooked packages remain research-only and must not enter distributable product assets.
