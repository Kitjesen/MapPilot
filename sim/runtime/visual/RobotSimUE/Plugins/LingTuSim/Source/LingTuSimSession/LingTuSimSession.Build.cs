using UnrealBuildTool;

public class LingTuSimSession : ModuleRules
{
    public LingTuSimSession(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "LingTuSimRuntime",
        });
        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Json",
            "Networking",
            "PlatformCryptoContext",
            "Sockets",
        });
    }
}
