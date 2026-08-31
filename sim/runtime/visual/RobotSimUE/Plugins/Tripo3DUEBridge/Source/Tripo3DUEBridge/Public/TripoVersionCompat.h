// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Runtime/Launch/Resources/Version.h"

/**
 * Version Compatibility Header for Tripo3D UE Bridge
 *
 * This header provides compatibility macros and includes for supporting
 * Unreal Engine versions 5.1 through 5.8+
 *
 * Supported Versions:
 * - UE 5.1
 * - UE 5.2
 * - UE 5.3
 * - UE 5.4
 * - UE 5.5
 * - UE 5.6
 * - UE 5.7
 * - UE 5.8+
 */

// ============================================================================
// Version Detection Macros
// ============================================================================

// Helper macros for version comparison
#ifndef UE_VERSION_OLDER_THAN
	#define UE_VERSION_OLDER_THAN(Major, Minor) \
		(ENGINE_MAJOR_VERSION < (Major) || (ENGINE_MAJOR_VERSION == (Major) && ENGINE_MINOR_VERSION < (Minor)))
#endif

#ifndef UE_VERSION_NEWER_THAN
	#define UE_VERSION_NEWER_THAN(Major, Minor) \
		(ENGINE_MAJOR_VERSION > (Major) || (ENGINE_MAJOR_VERSION == (Major) && ENGINE_MINOR_VERSION > (Minor)))
#endif

#ifndef UE_VERSION_AT_LEAST
	#define UE_VERSION_AT_LEAST(Major, Minor) \
		(ENGINE_MAJOR_VERSION > (Major) || (ENGINE_MAJOR_VERSION == (Major) && ENGINE_MINOR_VERSION >= (Minor)))
#endif

#ifndef UE_VERSION_EQUAL
	#define UE_VERSION_EQUAL(Major, Minor) \
		(ENGINE_MAJOR_VERSION == (Major) && ENGINE_MINOR_VERSION == (Minor))
#endif

// ============================================================================
// Styling API Compatibility (FEditorStyle -> FAppStyle transition in 5.1)
// ============================================================================

// FAppStyle is available in UE 5.0+, but EditorStyleSet.h header may vary
#if UE_VERSION_AT_LEAST(5, 1)
	// UE 5.1+: Use FAppStyle (FEditorStyle is deprecated)
	#include "Styling/AppStyle.h"
	#define TRIPO_STYLE_CLASS FAppStyle
#else
	// UE 5.0 and earlier: Use FEditorStyle
	#include "EditorStyleSet.h"
	#define TRIPO_STYLE_CLASS FEditorStyle
#endif

// Helper macro for getting style brushes
#define TRIPO_GET_BRUSH(BrushName) TRIPO_STYLE_CLASS::Get().GetBrush(BrushName)
#define TRIPO_GET_FONT_STYLE(FontName) TRIPO_STYLE_CLASS::Get().GetFontStyle(FontName)
#define TRIPO_GET_COLOR(ColorName) TRIPO_STYLE_CLASS::Get().GetColor(ColorName)

// ============================================================================
// Module API Compatibility
// ============================================================================

// Add future version-specific module API compatibility here
// Example:
// #if UE_VERSION_AT_LEAST(5, 3)
//     // UE 5.3+ specific module APIs
// #endif

// ============================================================================
// FBX Import API Compatibility
// ============================================================================

// FBX scale factor and legacy UFbxImportUI availability for version-specific import paths.
#if UE_VERSION_OLDER_THAN(5, 5)
	// UE 5.4 and earlier: use legacy FBX import settings with explicit scale factor
	#define TRIPO_FBX_SCALE_FACTOR 100.0f
	#define TRIPO_HAS_FBX_IMPORT_UI 1
#else
	// UE 5.5+: bare UAssetImportTask/default importer, no explicit scale needed
	#define TRIPO_FBX_SCALE_FACTOR 1.0f
	#define TRIPO_HAS_FBX_IMPORT_UI 0
#endif

// Material search location enum - API changed in UE 5.1
#if UE_VERSION_AT_LEAST(5, 1)
	#define TRIPO_MATERIAL_SEARCH_LOCATION EMaterialSearchLocation::UnderRoot
#else
	#define TRIPO_MATERIAL_SEARCH_LOCATION EMaterialSearchLocation_UnderRoot
#endif

// Interchange animation pipeline API differences
// UE 5.8+: bSnapToClosestFrameBoundary was replaced by FrameAlignment.
#if UE_VERSION_AT_LEAST(5, 8)
	#define TRIPO_INTERCHANGE_HAS_FRAME_ALIGNMENT 1
#else
	#define TRIPO_INTERCHANGE_HAS_FRAME_ALIGNMENT 0
#endif

// ============================================================================
// UE5.4-specific Bug Fix Flags
// ============================================================================

// UE5.4: UFbxImportUI does not propagate ImportUniformScale to skeletal mesh settings.
// Must be set explicitly on UFbxSkeletalMeshImportData or the mesh is 100x too small.
#if UE_VERSION_EQUAL(5, 4)
	#define TRIPO_NEEDS_SKELETAL_MESH_SCALE_FIX 1
#else
	#define TRIPO_NEEDS_SKELETAL_MESH_SCALE_FIX 0
#endif

// ============================================================================
// Build Configuration Info
// ============================================================================

// Log the detected UE version for debugging
#define TRIPO_UE_VERSION_STRING \
	FString::Printf(TEXT("UE %d.%d.%d"), ENGINE_MAJOR_VERSION, ENGINE_MINOR_VERSION, ENGINE_PATCH_VERSION)
