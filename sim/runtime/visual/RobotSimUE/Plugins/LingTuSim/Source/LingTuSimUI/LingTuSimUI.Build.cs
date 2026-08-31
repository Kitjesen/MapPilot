using System.IO;
using UnrealBuildTool;

public class LingTuSimUI : ModuleRules
{
    public LingTuSimUI(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore",
            "LingTuSimSession",
        });
        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Json",
            "LingTuSimRuntime",
            "LingTuSimVisual",
            "PlatformCryptoContext",
            "Projects",
            "Slate",
            "SlateCore",
        });

        RuntimeDependencies.Add(
            Path.Combine(PluginDirectory, "Resources", "FrontEnd", "lingtu-field-ops-hero-v1.png"),
            StagedFileType.NonUFS);
        RuntimeDependencies.Add(
            Path.Combine(PluginDirectory, "Resources", "FrontEnd", "preview-thunder-v4-v1.png"),
            StagedFileType.NonUFS);
        RuntimeDependencies.Add(
            Path.Combine(PluginDirectory, "Resources", "FrontEnd", "preview-rws-01-v1.png"),
            StagedFileType.NonUFS);
        RuntimeDependencies.Add(
            Path.Combine(PluginDirectory, "Resources", "FrontEnd", "preview-forest-pine-v1.png"),
            StagedFileType.NonUFS);
        RuntimeDependencies.Add(
            Path.Combine(PluginDirectory, "Resources", "FrontEnd", "preview-forest-birch-v1.png"),
            StagedFileType.NonUFS);
        RuntimeDependencies.Add(
            Path.Combine(PluginDirectory, "Resources", "FrontEnd", "preview-forest-boulder-v1.png"),
            StagedFileType.NonUFS);
    }
}
