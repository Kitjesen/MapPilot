// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

/**
 * Main module for Tripo Bridge plugin
 */
class FTripo3DUEBridgeModule : public IModuleInterface
{
public:
	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	/** Initialize plugin style */
	void InitializeStyle();

	/** Shutdown plugin style */
	void ShutdownStyle();

	/** Register editor menu extensions */
	void RegisterMenuExtensions();

	/** Unregister editor menu extensions */
	void UnregisterMenuExtensions();

	/** Spawn the Tripo3D WebSocket window */
	TSharedRef<class SDockTab> SpawnTripoWebSocketTab(const class FSpawnTabArgs& Args);

	/** Register tab spawner */
	void RegisterTabSpawner();

	/** Unregister tab spawner */
	void UnregisterTabSpawner();

private:
	/** Tab identifier */
	static const FName TripoWebSocketTabName;
	// Test client tab removed - use external HTML client for testing

	/** Menu extension */
	TSharedPtr<class FExtender> MenuExtender;

	/** Toolbar extension */
	TSharedPtr<class FUICommandList> PluginCommands;

	/** Plugin style set */
	TSharedPtr<class FSlateStyleSet> StyleSet;
};
