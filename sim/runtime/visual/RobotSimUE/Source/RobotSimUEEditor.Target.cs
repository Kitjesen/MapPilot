using UnrealBuildTool;

public class RobotSimUEEditorTarget : TargetRules
{
    public RobotSimUEEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
        ExtraModuleNames.Add("RobotSimUE");
    }
}
