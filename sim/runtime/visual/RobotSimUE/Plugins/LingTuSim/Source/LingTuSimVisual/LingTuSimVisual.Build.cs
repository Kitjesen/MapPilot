using UnrealBuildTool;

public class LingTuSimVisual : ModuleRules
{
    public LingTuSimVisual(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "LingTuSimRuntime",
            "LingTuSimSession",
        });
        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Json",
        });
    }
}
