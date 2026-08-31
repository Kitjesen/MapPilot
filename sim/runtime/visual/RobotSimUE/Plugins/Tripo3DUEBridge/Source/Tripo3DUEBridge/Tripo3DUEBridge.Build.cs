// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.IO;

public class Tripo3DUEBridge : ModuleRules
{
	public Tripo3DUEBridge(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// Version detection for conditional compilation
		int MajorVersion = Target.Version.MajorVersion;
		int MinorVersion = Target.Version.MinorVersion;

		// Enable precompile support, so Blueprint projects can use precompiled binary files
		bPrecompile = true;
		PrecompileForTargets = PrecompileTargetsType.Any;

		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
		);


		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
		);


		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
			}
		);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"InterchangeEngine",
				"InterchangePipelines",
				"Slate",
				"SlateCore",
				"UnrealEd",
				"EditorStyle",
				"InputCore",
				"LevelEditor",
				"Projects",
				"AssetTools",
				"AssetRegistry",
				"ContentBrowser",
				"FBX",
				"MainFrame",
				"Json",
				"JsonUtilities",
				"HTTP",
				"Sockets",
				"Networking",
				"WebSockets",
				"DesktopPlatform",
				"MessageLog",
				"MaterialEditor",
			}
		);

		// IXWebSocket precompiled library configuration
		string ThirdPartyPath = Path.Combine(ModuleDirectory, "..", "..", "ThirdParty");
		string IXWebSocketIncludePath = Path.Combine(ThirdPartyPath, "IXWebSocket", "ixwebsocket");
		string IXWebSocketLibFile = Path.Combine(ThirdPartyPath, "IXWebSocket", "Lib", "Win64", "IXWebSocket.lib");

		if (!File.Exists(IXWebSocketLibFile))
		{
			throw new BuildException("[Tripo3DUEBridge] IXWebSocket.lib not found: " + IXWebSocketLibFile);
		}

		PublicAdditionalLibraries.Add(IXWebSocketLibFile);
		PublicIncludePaths.Add(IXWebSocketIncludePath);
		PublicDefinitions.Add("IXWEBSOCKET_USE_PRECOMPILED=1");

		// Detect zlib version based on UE version
		// Different UE versions use different zlib versions:
		// UE 5.4+: zlib 1.3
		// UE 5.1-5.3: zlib 1.2.8 (or similar 1.2.x versions)
		// UE 5.7+: zlib 1.3
		string[] PossibleVersions = (MajorVersion == 5 && MinorVersion <= 6)
			? new[] { "1.3", "1.2.13", "1.2.12", "1.2.11", "1.2.8" }
			: new[] { "1.3" };
		string ZlibVersion = null;

		foreach (string Version in PossibleVersions)
		{
			string TestPath = Path.Combine(Target.UEThirdPartySourceDirectory, "zlib", Version);
			if (Directory.Exists(TestPath))
			{
				ZlibVersion = Version;
				System.Console.WriteLine("[Tripo3DUEBridge] Found zlib version: " + Version);
				break;
			}
		}

		if (ZlibVersion == null)
		{
			throw new BuildException(
				"[Tripo3DUEBridge] Could not find a supported zlib version under " +
				Path.Combine(Target.UEThirdPartySourceDirectory, "zlib") +
				". Tried: " + string.Join(", ", PossibleVersions)
			);
		}

		// Add UE built-in zlib paths
		string ZlibPath = Path.Combine(Target.UEThirdPartySourceDirectory, "zlib", ZlibVersion, "include");
		if (!Directory.Exists(ZlibPath))
		{
			throw new BuildException("[Tripo3DUEBridge] zlib include path not found: " + ZlibPath);
		}
		PublicIncludePaths.Add(ZlibPath);

		// Link zlib library
		string ZlibLibPath = Path.Combine(Target.UEThirdPartySourceDirectory, "zlib", ZlibVersion, "lib", "Win64", "Release");
		string ZlibLibFilePath = Path.Combine(ZlibLibPath, "zlibstatic.lib");
		if (!File.Exists(ZlibLibFilePath))
		{
			throw new BuildException("[Tripo3DUEBridge] zlib library not found: " + ZlibLibFilePath);
		}
		PublicAdditionalLibraries.Add(ZlibLibFilePath);

		PublicDefinitions.Add("IXWEBSOCKET_USE_ZLIB=1");
		PublicDefinitions.Add("_WINSOCK_DEPRECATED_NO_WARNINGS");
		PublicDefinitions.Add("NOMINMAX"); // Prevent Windows.h min/max macros

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicSystemLibraries.Add("ws2_32.lib");
			PublicSystemLibraries.Add("crypt32.lib");
			PublicSystemLibraries.Add("iphlpapi.lib");
		}


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
		);
	}
}
