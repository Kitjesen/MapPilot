"""Import the two repository-authored WAV loops into the actual game project."""
from pathlib import Path

import unreal

source = Path(unreal.Paths.project_dir()) / "SourceAssets" / "Audio" / "Menu"
for name in ("base-arrival", "departure"):
    task = unreal.AssetImportTask()
    task.filename = str(source / f"{name}.wav")
    task.destination_path = "/Game/RobotSim/Audio/Menu"
    task.destination_name = name
    task.automated = True
    task.replace_existing = True
    task.save = False
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    asset = unreal.load_asset(f"/Game/RobotSim/Audio/Menu/{name}")
    if not isinstance(asset, unreal.SoundWave):
        raise RuntimeError(f"SoundWave import failed: {name}")
    asset.set_editor_property("looping", True)
    asset.set_editor_property("virtualization_mode", unreal.VirtualizationMode.PLAY_WHEN_SILENT)
    if not unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False):
        raise RuntimeError(f"Could not save authored music: {name}")
    unreal.log(f"LINGTU_MENU_MUSIC_IMPORTED {name} channels={asset.get_editor_property('num_channels')}")
