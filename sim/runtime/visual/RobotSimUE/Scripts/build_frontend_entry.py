"""Create a dedicated lightweight menu world, without the old robot exhibit."""
import unreal

path = "/Game/RobotSim/Maps/LingTuFrontEnd"
levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if unreal.EditorAssetLibrary.does_asset_exist(path):
    if not levels.load_level(path):
        raise RuntimeError("Existing product entry world could not be loaded")
elif not levels.new_level(path):
    raise RuntimeError("Product entry world could not be created")
world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
if unreal.GameplayStatics.get_all_actors_of_class(world, unreal.StaticMeshActor):
    raise RuntimeError("Product entry must not contain the old robot exhibit")
if not unreal.EditorAssetLibrary.save_asset(path, only_if_is_dirty=False):
    raise RuntimeError("Product entry world could not be saved")
unreal.log("LINGTU_FRONTEND_ENTRY_SAVED no_exhibit=true no_physics_runtime=true")
