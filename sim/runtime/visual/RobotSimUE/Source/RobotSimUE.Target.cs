using UnrealBuildTool;

public class RobotSimUETarget : TargetRules
{
    public RobotSimUETarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V7;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_8;
        ExtraModuleNames.Add("RobotSimUE");
    }
}
