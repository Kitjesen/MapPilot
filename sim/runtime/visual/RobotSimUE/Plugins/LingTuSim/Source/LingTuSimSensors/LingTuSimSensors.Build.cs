using UnrealBuildTool;

public class LingTuSimSensors : ModuleRules
{
    public LingTuSimSensors(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "LingTuSimRuntime",
        });
        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Json",
            "LingTuSimVisual",
            "RenderCore",
            "RHI",
        });
    }
}
