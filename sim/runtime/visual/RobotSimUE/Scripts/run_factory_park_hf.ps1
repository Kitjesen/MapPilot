param(
    [string]$UnrealRoot = 'D:\Program Files\Epic Games\UE_5.8',
    [string]$NetFxSdkRoot,
    [string]$BlenderExe = 'D:\Development\Blender\5.2\blender.exe',
    [string]$WorldRecipe,
    [string]$RealismRecipe,
    [string]$BlenderManifest,
    [string]$ArtifactDigest,
    [string]$EvidenceRoot,
    [string]$DerivedDataCacheRoot,
    [switch]$SkipUnrealBuild,
    [switch]$Unattended,
    [switch]$ValidateExistingEvidence,
    [switch]$RefreshScreenshotsOnly,
    [switch]$RefreshRenderingOnly,
    [ValidateRange(1, 1440)]
    [int]$UnrealBuildTimeoutMinutes = 90,
    [int]$TimeoutMinutes = 60,
    [int]$NoProgressMinutes = 8
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$errorSentinel = $null
$unrealLog = $null
$editorProcess = $null

if ($ValidateExistingEvidence -and ($RefreshScreenshotsOnly -or $RefreshRenderingOnly)) {
    throw 'ValidateExistingEvidence and refresh modes are mutually exclusive'
}
if ($RefreshScreenshotsOnly -and $RefreshRenderingOnly) {
    throw 'RefreshScreenshotsOnly and RefreshRenderingOnly are mutually exclusive'
}

function Write-LauncherErrorSentinel {
    param([string]$Message)

    if (-not $errorSentinel) {
        return
    }
    if (Test-Path -LiteralPath $errorSentinel -PathType Leaf) {
        return
    }
    $parent = Split-Path -Parent $errorSentinel
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
    $payload = [ordered]@{
        schema = 'lingtu.sim.unreal-factory-park-hf-launcher-error.v1'
        result = 'error'
        message = $Message
        log = $unrealLog
    }
    [System.IO.File]::WriteAllText(
        $errorSentinel,
        (($payload | ConvertTo-Json -Depth 5) + [Environment]::NewLine),
        [System.Text.UTF8Encoding]::new($false)
    )
}

function Read-JsonUtf8 {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "JSON file is missing: $Path"
    }
    $jsonText = [System.IO.File]::ReadAllText(
        $Path,
        [System.Text.Encoding]::UTF8
    )
    return $jsonText | ConvertFrom-Json
}

function Assert-NoReparsePointTraversal {
    param(
        [string]$LexicalPath,
        [string]$AllowedRoot,
        [string]$Context
    )

    $cursor = [System.IO.Path]::GetFullPath($LexicalPath)
    while ($cursor) {
        if (Test-Path -LiteralPath $cursor) {
            $item = Get-Item -LiteralPath $cursor -Force
            if (
                ($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0
            ) {
                throw "$Context path traverses a reparse point: $cursor"
            }
        }
        $parent = Split-Path -Parent $cursor
        if (-not $parent -or $parent -eq $cursor) {
            break
        }
        $cursor = $parent
    }
}

function Resolve-PathInsideRoot {
    param(
        [string]$Path,
        [string]$Base,
        [string]$AllowedRoot,
        [string]$Context
    )

    if (-not $Path) {
        throw "$Context path is empty"
    }
    $lexicalPath = if ([System.IO.Path]::IsPathRooted($Path)) {
        [System.IO.Path]::GetFullPath($Path)
    } else {
        [System.IO.Path]::GetFullPath((Join-Path $Base ($Path -replace '/', '\')))
    }
    $allowedPath = [System.IO.Path]::GetFullPath($AllowedRoot)
    $allowed = $allowedPath.TrimEnd('\') + '\'
    if (-not ($lexicalPath + '\').StartsWith(
        $allowed,
        [System.StringComparison]::OrdinalIgnoreCase
    )) {
        throw "$Context path escapes $AllowedRoot`: $lexicalPath"
    }
    Assert-NoReparsePointTraversal `
        -LexicalPath $lexicalPath `
        -AllowedRoot $allowedPath `
        -Context $Context
    if (Test-Path -LiteralPath $lexicalPath) {
        $resolvedPathInfo = Resolve-Path -LiteralPath $lexicalPath
        $providerPath = if ($resolvedPathInfo.ProviderPath) {
            $resolvedPathInfo.ProviderPath
        } else {
            $resolvedPathInfo.Path
        }
        $realPath = [System.IO.Path]::GetFullPath($providerPath)
        if (-not ($realPath + '\').StartsWith(
            $allowed,
            [System.StringComparison]::OrdinalIgnoreCase
        )) {
            throw "$Context resolved path escapes $AllowedRoot`: $realPath"
        }
        return $realPath
    }
    return $lexicalPath
}

function Assert-FileRecord {
    param(
        [object]$Record,
        [string]$Base,
        [string]$AllowedRoot,
        [string]$Context
    )

    $resolved = Resolve-PathInsideRoot `
        -Path ([string]$Record.path) `
        -Base $Base `
        -AllowedRoot $AllowedRoot `
        -Context $Context
    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) {
        throw "$Context file is missing: $resolved"
    }
    $item = Get-Item -LiteralPath $resolved
    if ([long]$Record.bytes -ne $item.Length) {
        throw "$Context byte count mismatch: expected=$($Record.bytes), actual=$($item.Length)"
    }
    $actualSha = (Get-FileHash -LiteralPath $resolved -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualSha -ne ([string]$Record.sha256).ToLowerInvariant()) {
        throw "$Context SHA256 mismatch: expected=$($Record.sha256), actual=$actualSha"
    }
    return $resolved
}

function Assert-NumericVectorClose {
    param(
        [object]$Actual,
        [double[]]$Expected,
        [string]$Context,
        [double]$Tolerance = 0.000001
    )

    $actualValues = @($Actual)
    if ($actualValues.Count -ne $Expected.Count) {
        throw "$Context has $($actualValues.Count) values; expected $($Expected.Count)"
    }
    for ($index = 0; $index -lt $Expected.Count; $index += 1) {
        $actualNumber = [double]$actualValues[$index]
        if (
            [double]::IsNaN($actualNumber) -or
            [double]::IsInfinity($actualNumber) -or
            [Math]::Abs($actualNumber - $Expected[$index]) -gt $Tolerance
        ) {
            throw "$Context differs at index $index`: actual=$actualNumber expected=$($Expected[$index])"
        }
    }
}

function Assert-QuaternionEquivalent {
    param(
        [object]$Actual,
        [double[]]$Expected,
        [string]$Context,
        [double]$Tolerance = 0.00001
    )

    $actualValues = @($Actual)
    if ($actualValues.Count -ne 4 -or $Expected.Count -ne 4) {
        throw "$Context must contain four quaternion values"
    }
    $directMaximum = 0.0
    $negatedMaximum = 0.0
    for ($index = 0; $index -lt 4; $index += 1) {
        $actualNumber = [double]$actualValues[$index]
        if ([double]::IsNaN($actualNumber) -or [double]::IsInfinity($actualNumber)) {
            throw "$Context has a non-finite value at index $index"
        }
        $directMaximum = [Math]::Max(
            $directMaximum,
            [Math]::Abs($actualNumber - $Expected[$index])
        )
        $negatedMaximum = [Math]::Max(
            $negatedMaximum,
            [Math]::Abs($actualNumber + $Expected[$index])
        )
    }
    if ($directMaximum -gt $Tolerance -and $negatedMaximum -gt $Tolerance) {
        throw "$Context is not equivalent to the expected quaternion"
    }
}

function ConvertTo-ComponentScaledNumericVector {
    param(
        [object]$Values,
        [double[]]$Factors,
        [string]$Context
    )

    $sourceValues = @($Values)
    if ($sourceValues.Count -ne $Factors.Count) {
        throw "$Context has $($sourceValues.Count) values; expected $($Factors.Count)"
    }
    $scaledValues = [double[]]::new($Factors.Count)
    for ($index = 0; $index -lt $Factors.Count; $index += 1) {
        $sourceNumber = [double]$sourceValues[$index]
        if ([double]::IsNaN($sourceNumber) -or [double]::IsInfinity($sourceNumber)) {
            throw "$Context has a non-finite value at index $index"
        }
        $scaledValues[$index] = [double]$Factors[$index] * $sourceNumber
    }
    return $scaledValues
}

trap {
    $failureMessage = $_.Exception.Message
    if (
        $Unattended -and
        $null -ne $editorProcess -and
        -not $editorProcess.HasExited
    ) {
        Stop-Process -Id $editorProcess.Id -Force -ErrorAction SilentlyContinue
    }
    Write-LauncherErrorSentinel -Message $failureMessage
    [Console]::Error.WriteLine($failureMessage)
    exit 1
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $projectRoot '..\..\..\..')).Path
$project = Join-Path $projectRoot 'RobotSimUE.uproject'
$pythonScript = Join-Path $PSScriptRoot 'build_factory_park_hf.py'
$editor = Join-Path $UnrealRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
$buildBat = Join-Path $UnrealRoot 'Engine\Build\BatchFiles\Build.bat'
$python = Join-Path $repoRoot '.venv\Scripts\python.exe'
$netFxSdkPreflight = Join-Path $repoRoot 'sim\toolchains\windows_netfxsdk.ps1'
$ueBuildRunner = Join-Path $repoRoot 'sim\toolchains\ue_build.py'
if (-not $ValidateExistingEvidence -and -not $SkipUnrealBuild) {
    if (-not (Test-Path -LiteralPath $netFxSdkPreflight -PathType Leaf)) {
        throw "NETFXSDK preflight is missing: $netFxSdkPreflight"
    }
    if ($NetFxSdkRoot) {
        & $netFxSdkPreflight -NetFxSdkRoot $NetFxSdkRoot | Out-Null
    } else {
        & $netFxSdkPreflight | Out-Null
    }
}
$resolvedWorldRecipe = Resolve-PathInsideRoot `
    -Path $(
        if ($WorldRecipe) {
            $WorldRecipe
        } else {
            Join-Path $repoRoot 'sim\packages\worlds\factory_park_hf\visual\ue_import.recipe.json'
        }
    ) `
    -Base $repoRoot `
    -AllowedRoot $repoRoot `
    -Context 'WorldRecipe'
$resolvedRealismRecipe = Resolve-PathInsideRoot `
    -Path $(
        if ($RealismRecipe) {
            $RealismRecipe
        } else {
            Join-Path $repoRoot 'sim\packages\worlds\factory_park_hf\visual\realism.recipe.json'
        }
    ) `
    -Base $repoRoot `
    -AllowedRoot $repoRoot `
    -Context 'RealismRecipe'
$blenderRoot = Join-Path $repoRoot 'build\factory-park-hf\blender-v2'
$resolvedBlenderManifest = Resolve-PathInsideRoot `
    -Path $(
        if ($BlenderManifest) {
            $BlenderManifest
        } else {
            Join-Path $blenderRoot 'authoring.manifest.json'
        }
    ) `
    -Base $blenderRoot `
    -AllowedRoot $blenderRoot `
    -Context 'BlenderManifest'
$resolvedArtifactDigest = Resolve-PathInsideRoot `
    -Path $(
        if ($ArtifactDigest) {
            $ArtifactDigest
        } else {
            Join-Path $blenderRoot 'artifact-set.digest.json'
        }
    ) `
    -Base $blenderRoot `
    -AllowedRoot $blenderRoot `
    -Context 'ArtifactDigest'
$allowedEvidenceRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $repoRoot 'build\factory-park-hf\unreal-v2')
)
$resolvedEvidenceRoot = Resolve-PathInsideRoot `
    -Path $(if ($EvidenceRoot) { $EvidenceRoot } else { $allowedEvidenceRoot }) `
    -Base $allowedEvidenceRoot `
    -AllowedRoot $allowedEvidenceRoot `
    -Context 'EvidenceRoot'
$successSentinel = Join-Path $resolvedEvidenceRoot 'FactoryPark_HF.success.json'
$previousSuccessSentinel = Join-Path $resolvedEvidenceRoot 'FactoryPark_HF.previous.success.json'
$errorSentinel = Join-Path $resolvedEvidenceRoot 'FactoryPark_HF.error.json'
$unrealLog = Join-Path $resolvedEvidenceRoot 'UnrealEditor.log'
$resolvedDerivedDataCacheRoot = Resolve-PathInsideRoot `
    -Path $(
        if ($DerivedDataCacheRoot) {
            $DerivedDataCacheRoot
        } else {
            Join-Path $resolvedEvidenceRoot 'ddc'
        }
    ) `
    -Base $resolvedEvidenceRoot `
    -AllowedRoot $resolvedEvidenceRoot `
    -Context 'DerivedDataCacheRoot'

$requiredFiles = @(
    $resolvedWorldRecipe,
    $resolvedRealismRecipe,
    $resolvedBlenderManifest,
    $resolvedArtifactDigest
)
if (-not $ValidateExistingEvidence) {
    $requiredFiles += @($editor, $BlenderExe, $project, $pythonScript)
    if (-not $SkipUnrealBuild) {
        $requiredFiles += @($buildBat, $python, $ueBuildRunner)
    }
}
foreach ($required in $requiredFiles) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required FactoryPark_HF tool or input is missing: $required"
    }
}

$editorVersion = $null
$blenderVersion = $null
if (-not $ValidateExistingEvidence) {
    $editorVersion = (Get-Item -LiteralPath $editor).VersionInfo.FileVersion
    if (-not $editorVersion) {
        throw "Could not read Unreal Editor version from $editor"
    }
    $blenderOutput = & $BlenderExe --version
    $blenderExitCode = $LASTEXITCODE
    $blenderVersion = ($blenderOutput | Select-Object -First 1)
    if ($blenderExitCode -ne 0 -or -not $blenderVersion) {
        throw "Blender validation failed for $BlenderExe with exit code $blenderExitCode"
    }
}

$worldRecipeJson = Read-JsonUtf8 -Path $resolvedWorldRecipe
if ($worldRecipeJson.schema -ne 'lingtu.sim.unreal-world-import-recipe.v1') {
    throw "Unsupported FactoryPark_HF Unreal recipe schema: $($worldRecipeJson.schema)"
}
if ($worldRecipeJson.world_package -ne 'factory_park_hf@1.0.0') {
    throw "FactoryPark_HF recipe resolves the wrong package: $($worldRecipeJson.world_package)"
}
if ($worldRecipeJson.binding -ne 'WorldVisual:FactoryParkHF') {
    throw "FactoryPark_HF recipe resolves the wrong visual binding: $($worldRecipeJson.binding)"
}
if ($worldRecipeJson.target_level -ne '/Game/RobotSim/Maps/FactoryPark_HF') {
    throw "FactoryPark_HF recipe targets the wrong map: $($worldRecipeJson.target_level)"
}
if (
    $worldRecipeJson.coordinate_contract.source_frame -ne 'mujoco_rh_z_up_m' -or
    $worldRecipeJson.coordinate_contract.unreal_frame -ne 'unreal_lh_z_up_cm' -or
    $worldRecipeJson.unreal_layout_import.linear_scale_m_to_cm -ne 100.0
) {
    throw 'FactoryPark_HF recipe coordinate conversion contract is invalid'
}
$resolvedRecipeSources = @{}
foreach ($property in $worldRecipeJson.sources.PSObject.Properties) {
    $resolvedRecipeSources[$property.Name] = Assert-FileRecord `
        -Record $property.Value `
        -Base $repoRoot `
        -AllowedRoot $repoRoot `
        -Context "recipe source $($property.Name)"
}
foreach ($requiredSource in @('asset_manifest', 'expanded_layout', 'terrain_obj', 'realism_recipe')) {
    if (-not $resolvedRecipeSources.ContainsKey($requiredSource)) {
        throw "FactoryPark_HF recipe is missing source $requiredSource"
    }
}
if (
    [System.IO.Path]::GetFullPath($resolvedRecipeSources['realism_recipe']) -ne
    [System.IO.Path]::GetFullPath($resolvedRealismRecipe)
) {
    throw 'Unreal world recipe does not reference the selected realism recipe'
}

$realismRecipeJson = Read-JsonUtf8 -Path $resolvedRealismRecipe
if (
    $realismRecipeJson.schema -ne 'lingtu.sim.factory-park-realism-recipe.v1' -or
    $realismRecipeJson.profile -ne 'industrial_realism_v2' -or
    $realismRecipeJson.world_package -ne 'factory_park_hf@1.0.0' -or
    $realismRecipeJson.layout_digest -ne $worldRecipeJson.layout_digest
) {
    throw 'FactoryPark_HF realism recipe has an invalid identity or layout digest'
}
if (
    $realismRecipeJson.hard_rules.added_dressing_classification -ne 'VisualOnly' -or
    $realismRecipeJson.hard_rules.added_dressing_collision -ne $false -or
    $realismRecipeJson.hard_rules.allow_new_physics_authority -ne $false -or
    $realismRecipeJson.hard_rules.layout_unchanged -ne $true -or
    $realismRecipeJson.hard_rules.mujoco_world_unchanged -ne $true -or
    $realismRecipeJson.hard_rules.physics_authority -ne 'mujoco'
) {
    throw 'FactoryPark_HF realism recipe violates VisualOnly/MuJoCo authority'
}
if (
    $realismRecipeJson.lighting.unreal.global_illumination -ne 'Lumen' -or
    $realismRecipeJson.lighting.unreal.reflections -ne 'Lumen' -or
    $realismRecipeJson.lighting.unreal.shadow_method -ne 'VirtualShadowMaps'
) {
    throw 'FactoryPark_HF realism recipe does not require Lumen and Virtual Shadow Maps'
}
$requiredPreviewTargets = @(
    'south_gate_robot_eye',
    'loading_dock_robot_eye',
    'tank_farm_inspection'
)
$previewTargetIds = @($realismRecipeJson.preview_targets | ForEach-Object { [string]$_.id })
foreach ($requiredPreviewTarget in $requiredPreviewTargets) {
    if ($previewTargetIds -notcontains $requiredPreviewTarget) {
        throw "FactoryPark_HF realism recipe omits preview target $requiredPreviewTarget"
    }
}

$blenderManifestJson = Read-JsonUtf8 -Path $resolvedBlenderManifest
if ($blenderManifestJson.schema -ne 'lingtu.sim.blender-authoring-manifest.v1') {
    throw "Unsupported Blender authoring manifest schema: $($blenderManifestJson.schema)"
}
if ($blenderManifestJson.world_package -ne 'factory_park_hf@1.0.0') {
    throw 'Blender authoring manifest resolves the wrong world package'
}
$resolvedManifestLayout = Assert-FileRecord `
    -Record $blenderManifestJson.source_layout `
    -Base $repoRoot `
    -AllowedRoot $repoRoot `
    -Context 'Blender source_layout'
if (
    [System.IO.Path]::GetFullPath($resolvedManifestLayout) -ne
    [System.IO.Path]::GetFullPath($resolvedRecipeSources['expanded_layout'])
) {
    throw 'Blender source_layout path disagrees with the world recipe'
}
if ($blenderManifestJson.source_layout.layout_digest -ne $worldRecipeJson.layout_digest) {
    throw 'Blender source_layout semantic digest disagrees with the world recipe'
}
$manifestDirectory = Split-Path -Parent $resolvedBlenderManifest
$assetRoles = @{}
foreach ($asset in @($blenderManifestJson.assets)) {
    if ($assetRoles.ContainsKey([string]$asset.role)) {
        throw "Blender authoring manifest repeats asset role $($asset.role)"
    }
    $assetRoles[[string]$asset.role] = Assert-FileRecord `
        -Record $asset `
        -Base $manifestDirectory `
        -AllowedRoot $manifestDirectory `
        -Context "Blender asset $($asset.role)"
}
if (
    -not $assetRoles.ContainsKey('unreal_scene_fbx') -and
    -not $assetRoles.ContainsKey('portable_scene_glb')
) {
    throw 'Blender authoring manifest has neither an Unreal FBX nor a portable GLB scene'
}
if (
    $blenderManifestJson.coordinate_contract.unreal.combine_meshes -ne $false -or
    $blenderManifestJson.coordinate_contract.unreal.import_scale -ne 1.0 -or
    $blenderManifestJson.coordinate_contract.unreal.import_uniform_scale -ne 1.0 -or
    $blenderManifestJson.coordinate_contract.unreal.import_uniform_scale_supported -ne $false -or
    $blenderManifestJson.coordinate_contract.unreal.fbx_actor_uniform_scale -ne 100.0 -or
    $blenderManifestJson.coordinate_contract.unreal.terrain_actor_uniform_scale -ne 1.0 -or
    $blenderManifestJson.coordinate_contract.unreal.unit_conversion_strategy -ne 'placement_actor_scale' -or
    $blenderManifestJson.coordinate_contract.fbx.unreal_transform_vertex_to_absolute -ne $false -or
    $blenderManifestJson.coordinate_contract.fbx.unreal_bake_pivot_in_vertex -ne $false -or
    $blenderManifestJson.coordinate_contract.fbx.contains_authoritative_terrain -ne $false
) {
    throw 'Blender authoring manifest would bake or combine stable scene meshes'
}
$unrealFbxRecord = @($blenderManifestJson.assets) |
    Where-Object { $_.role -eq 'unreal_scene_fbx' } |
    Select-Object -First 1
if (
    $null -ne $unrealFbxRecord -and
    (
        $unrealFbxRecord.unreal_import.import_uniform_scale -ne 1.0 -or
        $unrealFbxRecord.unreal_import.import_uniform_scale_supported -ne $false -or
        $unrealFbxRecord.unreal_placement.actor_uniform_scale -ne 100.0 -or
        $unrealFbxRecord.unreal_placement.required -ne $true
    )
) {
    throw 'Blender Unreal FBX record must require explicit actor-scale metre-to-centimetre placement'
}

if (
    $blenderManifestJson.realism.profile -ne 'industrial_realism_v2' -or
    $blenderManifestJson.realism.seed -ne $realismRecipeJson.seed -or
    $blenderManifestJson.realism.namespace -ne
        $realismRecipeJson.authoring_contract.dressing_id_namespace
) {
    throw 'Blender manifest realism identity disagrees with the shared recipe'
}
$resolvedManifestRealismRecipe = Assert-FileRecord `
    -Record $blenderManifestJson.realism.recipe `
    -Base $repoRoot `
    -AllowedRoot $repoRoot `
    -Context 'Blender realism recipe'
if (
    [System.IO.Path]::GetFullPath($resolvedManifestRealismRecipe) -ne
    [System.IO.Path]::GetFullPath($resolvedRealismRecipe)
) {
    throw 'Blender manifest points at the wrong realism recipe'
}
if (
    [int]$blenderManifestJson.realism.actor_budget.min -ne 1200 -or
    [int]$blenderManifestJson.realism.actor_budget.max -ne 1800 -or
    $blenderManifestJson.scene.material_profile -ne 'procedural_pbr_v2' -or
    $blenderManifestJson.scene.actor_budget_status -ne 'within_budget'
) {
    throw 'Blender manifest does not prove the industrial_realism_v2 material/actor budget'
}
$manifestPlacedObjects = @($blenderManifestJson.scene.layout_objects) +
    @($blenderManifestJson.scene.visual_only_objects)
$expandedLayoutJson = Read-JsonUtf8 -Path $resolvedRecipeSources['expanded_layout']
$semanticDescriptors = @($blenderManifestJson.scene.semantic_feature_descriptors)
$expectedSemanticLayoutObjects = @(
    $expandedLayoutJson.objects | Where-Object { $_.semantic_class -eq 'semantic_checkpoint' }
)
if (
    $blenderManifestJson.scene.semantic_descriptors_not_materialized -ne $true -or
    $semanticDescriptors.Count -ne $expectedSemanticLayoutObjects.Count -or
    $semanticDescriptors.Count -ne 6
) {
    throw 'Blender manifest does not prove six non-materialized semantic checkpoints'
}
$semanticDescriptorIds = @{}
foreach ($descriptor in $semanticDescriptors) {
    $stableId = [string]$descriptor.stable_id
    $expectedDescriptor = $expectedSemanticLayoutObjects |
        Where-Object { $_.id -eq $stableId } |
        Select-Object -First 1
    if (
        -not $stableId -or
        $semanticDescriptorIds.ContainsKey($stableId) -or
        $null -eq $expectedDescriptor -or
        $null -ne $descriptor.mesh_name -or
        $null -ne $descriptor.blender_object -or
        $descriptor.asset_key -ne "semantic/$stableId" -or
        $descriptor.semantic_class -ne 'semantic_checkpoint' -or
        $descriptor.source -ne 'expanded_layout_semantic' -or
        $descriptor.physics_proxy -ne 'none' -or
        $descriptor.collision -ne $false -or
        $descriptor.visual_only -ne $true -or
        $descriptor.materialized -ne $false -or
        $descriptor.exported_mesh -ne $false
    ) {
        throw "Blender semantic descriptor is invalid or duplicated: $stableId"
    }
    if (@($manifestPlacedObjects | Where-Object { $_.stable_id -eq $stableId }).Count -ne 0) {
        throw "Blender semantic checkpoint was incorrectly materialized as a mesh: $stableId"
    }
    $semanticDescriptorIds[$stableId] = $true
}
foreach ($visualRecord in @($blenderManifestJson.scene.visual_only_objects)) {
    if (
        $visualRecord.source -ne 'blender_derived_visual' -or
        $visualRecord.physics_proxy -ne 'none' -or
        $visualRecord.collision -ne $false -or
        $visualRecord.visual_only -ne $true
    ) {
        throw "Blender derived visual violates VisualOnly authority: $($visualRecord.stable_id)"
    }
}
if (
    [int]$blenderManifestJson.scene.total_mesh_actor_count -ne ($manifestPlacedObjects.Count + 1) -or
    [int]$blenderManifestJson.scene.total_mesh_actor_count -lt 1200 -or
    [int]$blenderManifestJson.scene.total_mesh_actor_count -gt 1800
) {
    throw 'Blender manifest mesh actor count lies outside the 1200-1800 realism budget'
}
$requiredDetailSemantics = @(
    'security_camera', 'barrier_arm', 'impact_bollard',
    'roller_shutter_door', 'dock_shelter', 'dock_bumper', 'wood_pallet', 'parked_forklift',
    'tank_access_ladder', 'tank_inspection_platform', 'tank_valve_pipe', 'tank_valve', 'process_pipe',
    'asphalt_patch', 'tire_mark', 'manhole_cover', 'storm_drain_grate',
    'corrugated_panel_seam', 'roof_gutter', 'downspout', 'service_door', 'hvac_unit'
)
foreach ($semanticClass in $requiredDetailSemantics) {
    $detailCount = $blenderManifestJson.scene.detail_counts.$semanticClass
    if ($null -eq $detailCount -or [int]$detailCount -le 0) {
        throw "Blender realism manifest omits required detail $semanticClass"
    }
}
$manifestToRecipeCameras = [ordered]@{
    site_aerial = 'site_aerial'
    robot_eye = 'south_gate_robot_eye'
    loading_dock = 'loading_dock_robot_eye'
    tank_area = 'tank_farm_inspection'
}
foreach ($mapping in $manifestToRecipeCameras.GetEnumerator()) {
    $manifestCamera = $blenderManifestJson.scene.cameras.($mapping.Key)
    $recipeCamera = $realismRecipeJson.preview_targets |
        Where-Object { $_.id -eq $mapping.Value } |
        Select-Object -First 1
    if ($null -eq $manifestCamera -or $null -eq $recipeCamera) {
        throw "Blender or realism recipe omits acceptance camera $($mapping.Key)"
    }
    if (
        $manifestCamera.recipe_target -ne $mapping.Value -or
        $manifestCamera.output_basename -ne $recipeCamera.output_basename -or
        [double]$manifestCamera.lens_mm -le 0.0 -or
        (
            $mapping.Key -ne 'site_aerial' -and
            -not [string]$manifestCamera.composition_adjustment
        )
    ) {
        throw "Blender acceptance camera provenance is invalid for $($mapping.Key)"
    }
    Assert-NumericVectorClose `
        -Actual $manifestCamera.position_m `
        -Expected ([double[]]@($manifestCamera.position_m)) `
        -Context "$($mapping.Key) camera position_m"
    Assert-NumericVectorClose `
        -Actual $manifestCamera.look_at_m `
        -Expected ([double[]]@($manifestCamera.look_at_m)) `
        -Context "$($mapping.Key) camera look_at_m"
}

$artifactDigestJson = Read-JsonUtf8 -Path $resolvedArtifactDigest
if ($artifactDigestJson.schema -ne 'lingtu.sim.blender-artifact-set-digest.v1') {
    throw "Unsupported Blender artifact digest schema: $($artifactDigestJson.schema)"
}
$resolvedDigestManifest = Assert-FileRecord `
    -Record $artifactDigestJson.manifest `
    -Base (Split-Path -Parent $resolvedArtifactDigest) `
    -AllowedRoot (Split-Path -Parent $resolvedArtifactDigest) `
    -Context 'artifact digest manifest'
if (
    [System.IO.Path]::GetFullPath($resolvedDigestManifest) -ne
    [System.IO.Path]::GetFullPath($resolvedBlenderManifest)
) {
    throw 'Blender artifact digest points at the wrong manifest'
}
if ($artifactDigestJson.artifact_set_digest -ne $blenderManifestJson.artifact_set_digest) {
    throw 'Blender manifest and artifact digest disagree on artifact_set_digest'
}
if ($artifactDigestJson.layout_sha256 -ne $worldRecipeJson.sources.expanded_layout.sha256) {
    throw 'Blender artifact digest has the wrong layout_sha256'
}

if (-not $ValidateExistingEvidence) {
    New-Item -ItemType Directory -Path $resolvedEvidenceRoot -Force | Out-Null
    New-Item -ItemType Directory -Path $resolvedDerivedDataCacheRoot -Force | Out-Null
    # Every child process, including UnrealBuildTool/UnrealHeaderTool, inherits a writable DDC.
    ${env:UE-LocalDataCachePath} = $resolvedDerivedDataCacheRoot
    ${env:UE-SharedDataCachePath} = 'None'
    $env:LINGTU_FACTORY_PARK_HF_DDC_FORCE_MEMORY_CACHE = '1'
    if ($RefreshScreenshotsOnly -or $RefreshRenderingOnly) {
        if (-not (Test-Path -LiteralPath $successSentinel -PathType Leaf)) {
            throw "FactoryPark refresh requires existing success evidence: $successSentinel"
        }
        Copy-Item -LiteralPath $successSentinel -Destination $previousSuccessSentinel -Force
    }
    foreach ($staleOutput in @($successSentinel, $errorSentinel, $unrealLog)) {
        $fullStalePath = [System.IO.Path]::GetFullPath($staleOutput)
        if (-not ($fullStalePath + '\').StartsWith(
            $allowedEvidenceRoot.TrimEnd('\') + '\',
            [System.StringComparison]::OrdinalIgnoreCase
        )) {
            throw "Refusing to remove output outside $allowedEvidenceRoot"
        }
        if (Test-Path -LiteralPath $fullStalePath -PathType Leaf) {
            Remove-Item -LiteralPath $fullStalePath -Force
        }
    }

    if (-not $SkipUnrealBuild) {
        $buildTimeoutSeconds = [double]$UnrealBuildTimeoutMinutes * 60.0
        & $python $ueBuildRunner `
            --build-bat $buildBat `
            --uproject $project `
            --target RobotSimUEEditor `
            --platform Win64 `
            --configuration Development `
            --timeout-seconds $buildTimeoutSeconds
        if ($LASTEXITCODE -ne 0) {
            throw "RobotSimUEEditor build failed with exit code $LASTEXITCODE"
        }
    }

    $env:LINGTU_FACTORY_PARK_HF_RECIPE = $resolvedWorldRecipe
    $env:LINGTU_FACTORY_PARK_HF_REALISM_RECIPE = $resolvedRealismRecipe
    $env:LINGTU_FACTORY_PARK_HF_BLENDER_MANIFEST = $resolvedBlenderManifest
    $env:LINGTU_FACTORY_PARK_HF_ARTIFACT_DIGEST = $resolvedArtifactDigest
    $env:LINGTU_FACTORY_PARK_HF_EVIDENCE_ROOT = $resolvedEvidenceRoot
    $env:LINGTU_FACTORY_PARK_HF_SUCCESS = $successSentinel
    $env:LINGTU_FACTORY_PARK_HF_ERROR = $errorSentinel
    $env:LINGTU_FACTORY_PARK_HF_UNATTENDED = if ($Unattended) { '1' } else { '0' }
    $env:LINGTU_FACTORY_PARK_HF_REFRESH_SCREENSHOTS_ONLY = if ($RefreshScreenshotsOnly) { '1' } else { '0' }
    $env:LINGTU_FACTORY_PARK_HF_REFRESH_RENDERING_ONLY = if ($RefreshRenderingOnly) { '1' } else { '0' }
    $env:LINGTU_FACTORY_PARK_HF_PREVIOUS_SUCCESS = $previousSuccessSentinel

    $arguments = @(
        $project,
        "-ExecCmds=`"py $pythonScript`"",
        '-NoCompile',
        '-NoSplash',
        '-nop4',
        '-DDC-ForceMemoryCache',
        '-log',
        "-abslog=`"$unrealLog`""
    )
    if ($Unattended) {
        $arguments += '-unattended'
    }
    $windowStyle = if ($Unattended) { 'Hidden' } else { 'Normal' }
    $editorProcess = Start-Process `
        -FilePath $editor `
        -ArgumentList $arguments `
        -WindowStyle $windowStyle `
        -PassThru

    $deadline = (Get-Date).AddMinutes($TimeoutMinutes)
    $lastProgressAt = Get-Date
    $lastLogWriteTime = [DateTime]::MinValue
    while (
        (Get-Date) -lt $deadline -and
        -not (Test-Path -LiteralPath $successSentinel -PathType Leaf)
    ) {
        if (Test-Path -LiteralPath $unrealLog -PathType Leaf) {
            $logItem = Get-Item -LiteralPath $unrealLog
            if ($logItem.LastWriteTime -gt $lastLogWriteTime) {
                $lastLogWriteTime = $logItem.LastWriteTime
                $lastProgressAt = Get-Date
            }
        }
        if (Test-Path -LiteralPath $errorSentinel -PathType Leaf) {
            $builderError = Get-Content -LiteralPath $errorSentinel -Raw
            throw "FactoryPark_HF Unreal Python failed: $builderError"
        }
        if ($editorProcess.HasExited) {
            throw (
                "Unreal Editor exited before FactoryPark_HF success " +
                "(exit $($editorProcess.ExitCode)); log=$unrealLog"
            )
        }
        if ((Get-Date) -gt $lastProgressAt.AddMinutes($NoProgressMinutes)) {
            throw (
                "Unreal Editor made no FactoryPark_HF log progress for " +
                "$NoProgressMinutes minutes; log=$unrealLog"
            )
        }
        Start-Sleep -Seconds 2
        $editorProcess.Refresh()
    }
}

if (-not (Test-Path -LiteralPath $successSentinel -PathType Leaf)) {
    $failureContext = if ($ValidateExistingEvidence) {
        'Existing FactoryPark_HF success evidence is missing'
    } else {
        'Timed out waiting for FactoryPark_HF success sentinel'
    }
    throw "$failureContext`: $successSentinel"
}
if (
    $ValidateExistingEvidence -and
    (Test-Path -LiteralPath $errorSentinel -PathType Leaf)
) {
    $successItem = Get-Item -LiteralPath $successSentinel
    $errorItem = Get-Item -LiteralPath $errorSentinel
    if ($errorItem.LastWriteTimeUtc -ge $successItem.LastWriteTimeUtc) {
        throw (
            'Existing FactoryPark_HF error evidence is newer than or as new as ' +
            "the success evidence: $errorSentinel"
        )
    }
}
$successJson = Read-JsonUtf8 -Path $successSentinel
if (
    $successJson.schema -ne 'lingtu.sim.unreal-factory-park-hf-evidence.v1' -or
    $successJson.result -ne 'success' -or
    $successJson.map_path -ne '/Game/RobotSim/Maps/FactoryPark_HF'
) {
    throw 'FactoryPark_HF success sentinel has an invalid identity'
}
$evidenceDerivedDataCacheRoot = [System.IO.Path]::GetFullPath(
    [string]$successJson.derived_data_cache.local_override
)
if (
    $evidenceDerivedDataCacheRoot -ne $resolvedDerivedDataCacheRoot -or
    -not ($evidenceDerivedDataCacheRoot + '\').StartsWith(
        $resolvedEvidenceRoot.TrimEnd('\') + '\',
        [System.StringComparison]::OrdinalIgnoreCase
    ) -or
    -not (Test-Path -LiteralPath $evidenceDerivedDataCacheRoot -PathType Container) -or
    $successJson.derived_data_cache.shared_override -ne 'None' -or
    $successJson.derived_data_cache.local_override_under_evidence_root -ne $true -or
    $successJson.derived_data_cache.local_override_exists_and_is_directory -ne $true -or
    $successJson.derived_data_cache.force_memory_cache_environment_contract -ne $true -or
    $successJson.derived_data_cache.force_memory_cache_argument -ne $true
) {
    throw 'FactoryPark_HF success sentinel does not prove a writable lane-local DDC'
}
if (
    $successJson.realism.profile -ne 'industrial_realism_v2' -or
    $successJson.realism.recipe_sha256 -ne
        $worldRecipeJson.sources.realism_recipe.sha256 -or
    $successJson.realism.layout_digest -ne $worldRecipeJson.layout_digest -or
    $successJson.blender_authoring.realism.profile -ne 'industrial_realism_v2'
) {
    throw 'FactoryPark_HF success sentinel has stale or invalid realism evidence'
}
if ($successJson.terrain.actor_count -ne 1) {
    throw 'FactoryPark_HF success sentinel does not prove exactly one terrain actor'
}
if (
    $successJson.terrain.collision -ne 'disabled_in_unreal' -or
    $successJson.environment.collision -ne 'disabled_in_unreal' -or
    $successJson.environment.actor_count -lt 1200 -or
    $successJson.environment.actor_count -gt 1800 -or
    $successJson.environment.physics_shared_count -ne
        @($blenderManifestJson.scene.layout_objects | Where-Object { -not $_.visual_only }).Count -or
    $successJson.environment.visual_only_count -ne
        @($blenderManifestJson.scene.layout_objects + $blenderManifestJson.scene.visual_only_objects |
            Where-Object { $_.visual_only }).Count -or
    $successJson.environment.semantic_feature_descriptor_count -ne $semanticDescriptors.Count -or
    $successJson.environment.semantic_descriptors_not_materialized -ne $true
) {
    throw 'FactoryPark_HF success sentinel violates collision authority or actor-count budgets'
}
if (
    $successJson.semantic_feature_descriptors.count -ne $semanticDescriptors.Count -or
    $successJson.semantic_feature_descriptors.actor_count -ne 0 -or
    $successJson.semantic_feature_descriptors.mesh_count -ne 0 -or
    $successJson.semantic_feature_descriptors.semantic_descriptors_not_materialized -ne $true
) {
    throw 'FactoryPark_HF success sentinel materialized semantic checkpoint descriptors'
}
$evidenceSemanticIds = @($successJson.semantic_feature_descriptors.stable_ids)
if ($evidenceSemanticIds.Count -ne $semanticDescriptorIds.Count) {
    throw 'FactoryPark_HF semantic descriptor evidence has the wrong stable-ID count'
}
foreach ($stableId in $semanticDescriptorIds.Keys) {
    if ($evidenceSemanticIds -notcontains $stableId) {
        throw "FactoryPark_HF semantic descriptor evidence omits $stableId"
    }
}
if (
    $successJson.native_pbr_materials.authority -ne 'UnrealNativePBR' -or
    $successJson.native_pbr_materials.fbx_material_networks_used -ne $false -or
    $successJson.native_pbr_materials.instance_count -le 0 -or
    $successJson.native_pbr_materials.source_identity_count -ne
        $successJson.native_pbr_materials.instance_count -or
    $successJson.native_pbr_materials.parameter_readback_verified_count -ne
        $successJson.native_pbr_materials.instance_count -or
    $successJson.native_pbr_materials.parameter_write_contract -ne
        'verified_by_getter_readback_due_ue58_false_return_value' -or
    $successJson.native_pbr_materials.environment_mesh_assignment_count -ne
        $successJson.environment.actor_count -or
    $successJson.native_pbr_materials.environment_component_assignment_count -ne
        $successJson.environment.actor_count -or
    $successJson.native_pbr_materials.terrain_assignment_count -ne 1 -or
    @($successJson.native_pbr_materials.assignments).Count -ne
        $successJson.environment.actor_count
) {
    throw 'FactoryPark_HF success sentinel does not prove complete native PBR assignment'
}
$requiredMaterialSwatches = [ordered]@{
    'transformer_dark|parked_truck' = [double[]]@(0.055, 0.065, 0.07)
    'galvanized|electrical_cabinet' = [double[]]@(0.43, 0.46, 0.47)
    'safety_black|forklift_mast' = [double[]]@(0.035, 0.04, 0.045)
    'safety_red|barrier_arm' = [double[]]@(0.55, 0.045, 0.03)
    'vehicle_blue|parked_forklift' = [double[]]@(0.08, 0.22, 0.34)
    'vehicle_white|parked_truck' = [double[]]@(0.72, 0.74, 0.72)
}
if (
    $successJson.native_pbr_materials.base_color_policy -ne
        'manifest_material_swatch_then_neutral_identity_fallback'
) {
    throw 'FactoryPark_HF success sentinel does not prove the industrial base-color policy'
}
$verifiedMaterialSwatches = @{}
foreach ($materialEvidence in @($successJson.native_pbr_materials.materials)) {
    $materialIdentity = (
        [string]$materialEvidence.source_material + '|' +
        [string]$materialEvidence.source_semantic_class
    )
    if (
        $materialEvidence.parameter_write_evidence.contract -ne
            'verified_by_getter_readback_due_ue58_false_return_value' -or
        $materialEvidence.parameter_write_evidence.all_readbacks_verified -ne $true -or
        $materialEvidence.parameter_write_evidence.vector.BaseColor.verified -ne $true -or
        @($materialEvidence.parameter_write_evidence.scalars.PSObject.Properties).Count -ne 3
    ) {
        throw 'FactoryPark_HF success sentinel has an unverified native PBR parameter write'
    }
    if (
        $materialEvidence.base_color_source -notin @(
            'manifest_material_swatch',
            'recipe_profile',
            'neutral_manifest_identity_fallback'
        )
    ) {
        throw "FactoryPark_HF material $materialIdentity has no deterministic base-color source"
    }
    if ($requiredMaterialSwatches.Contains($materialIdentity)) {
        Assert-NumericVectorClose `
            -Actual $materialEvidence.base_color_srgb `
            -Expected $requiredMaterialSwatches[$materialIdentity] `
            -Context "FactoryPark_HF material swatch $materialIdentity"
        if ($materialEvidence.base_color_source -ne 'manifest_material_swatch') {
            throw "FactoryPark_HF material $materialIdentity did not use its exact manifest swatch"
        }
        $verifiedMaterialSwatches[$materialIdentity] = $true
    }
    if ($materialEvidence.base_color_source -eq 'neutral_manifest_identity_fallback') {
        $fallbackColor = @($materialEvidence.base_color_srgb | ForEach-Object { [double]$_ })
        $fallbackMaximum = [double](
            $fallbackColor | Measure-Object -Maximum | Select-Object -ExpandProperty Maximum
        )
        $fallbackMinimum = [double](
            $fallbackColor | Measure-Object -Minimum | Select-Object -ExpandProperty Minimum
        )
        if ($fallbackMaximum - $fallbackMinimum -gt 0.040001) {
            throw "FactoryPark_HF material $materialIdentity has a saturated fallback color"
        }
    }
}
foreach ($requiredMaterialSwatch in $requiredMaterialSwatches.Keys) {
    if (-not $verifiedMaterialSwatches.ContainsKey($requiredMaterialSwatch)) {
        throw "FactoryPark_HF success sentinel omits required material swatch $requiredMaterialSwatch"
    }
}
if (
    $successJson.native_pbr_materials.master_graph.roughness_path -ne
        'clamped_parameter' -or
    $successJson.native_pbr_materials.master_graph.world_position_noise_used -ne $false -or
    $successJson.native_pbr_materials.master_graph.expression_connection_count -ne 1 -or
    $successJson.native_pbr_materials.master_graph.material_output_count -ne 4 -or
    $successJson.native_pbr_materials.master_graph.compiled -ne $true
) {
    throw 'FactoryPark_HF success sentinel does not prove the native PBR master graph'
}
if (
    $successJson.native_pbr_materials.master_graph.material_usages.InstancedStaticMeshes -ne
        $true -or
    $successJson.native_pbr_materials.master_graph.material_usages.Nanite -ne $true
) {
    throw 'FactoryPark_HF success sentinel does not prove HISM and Nanite material usage'
}
$exposureEvidence = $successJson.lighting.lumen.exposure
$expectedExposure = $realismRecipeJson.lighting.exposure
if (
    $exposureEvidence.mode -ne 'manual_physical_camera' -or
    $exposureEvidence.verified -ne $true -or
    [Math]::Abs([double]$exposureEvidence.ev100 - [double]$expectedExposure.ev100) -gt 0.000001 -or
    [Math]::Abs(
        [double]$exposureEvidence.white_balance_k -
        [double]$expectedExposure.white_balance_k
    ) -gt 0.000001
) {
    throw 'FactoryPark_HF success sentinel does not prove recipe-bound manual exposure'
}
$sunEvidence = $successJson.lighting.sun
$expectedSun = $realismRecipeJson.lighting.sun
if (
    $sunEvidence.verified -ne $true -or
    [Math]::Abs(
        [double]$sunEvidence.angular_diameter_deg -
        [double]$expectedSun.angular_diameter_deg
    ) -gt 0.000001 -or
    [Math]::Abs(
        [double]$sunEvidence.color_temperature_k -
        [double]$expectedSun.color_temperature_k
    ) -gt 0.000001 -or
    [Math]::Abs(
        [double]$sunEvidence.illuminance_lux -
        [double]$expectedSun.illuminance_lux
    ) -gt 0.000001
) {
    throw 'FactoryPark_HF success sentinel does not prove recipe-bound physical sun settings'
}
Assert-NumericVectorClose `
    -Actual $sunEvidence.base_light_color_rgba `
    -Expected ([double[]]@(255.0, 255.0, 255.0, 255.0)) `
    -Context 'FactoryPark_HF neutral physical sun color'
if (
    $successJson.nanite.requested -ne $false -or
    $successJson.nanite.enabled_mesh_count -ne 0 -or
    -not [string]$successJson.nanite.reason
) {
    throw 'FactoryPark_HF success sentinel does not match the recipe-controlled optional Nanite policy'
}
$materialAssignmentByStableId = @{}
foreach ($assignment in @($successJson.native_pbr_materials.assignments)) {
    $stableId = [string]$assignment.stable_id
    if (-not $stableId -or $materialAssignmentByStableId.ContainsKey($stableId)) {
        throw "FactoryPark_HF native PBR evidence repeats or omits stable ID $stableId"
    }
    if (
        -not [string]$assignment.source_material -or
        -not [string]$assignment.source_semantic_class -or
        -not ([string]$assignment.material_asset).StartsWith(
            '/Game/RobotSim/Worlds/FactoryParkHF/Materials/IndustrialRealismV2/',
            [System.StringComparison]::Ordinal
        )
    ) {
        throw "FactoryPark_HF native PBR evidence is incomplete for $stableId"
    }
    $materialAssignmentByStableId[$stableId] = $assignment
}
$expectedDrainageReeds = @(
    $manifestPlacedObjects |
        Where-Object {
            $_.visual_only -eq $true -and
            $_.semantic_class -eq 'drainage_reed'
        }
)
$expectedDrainageReedIds = @(
    $expectedDrainageReeds |
        ForEach-Object { [string]$_.stable_id } |
        Sort-Object -Unique
)
$visualShadowPolicy = $successJson.visual_shadow_policy
if (
    $expectedDrainageReeds.Count -ne 48 -or
    $expectedDrainageReedIds.Count -ne 48 -or
    $visualShadowPolicy.profile -ne 'visual_only_micro_dressing_v1' -or
    $visualShadowPolicy.semantic_class -ne 'drainage_reed' -or
    $visualShadowPolicy.component_property -ne
        'StaticMeshComponent render-only lighting flags' -or
    $visualShadowPolicy.expected_actor_count -ne 48 -or
    $visualShadowPolicy.actor_count -ne 48 -or
    $visualShadowPolicy.expected_cast_shadow -ne $false -or
    $visualShadowPolicy.expected_affect_distance_field_lighting -ne $false -or
    $visualShadowPolicy.expected_visible_in_ray_tracing -ne $false -or
    $visualShadowPolicy.all_readbacks_verified -ne $true -or
    $visualShadowPolicy.visible_geometry_preserved -ne $true -or
    $visualShadowPolicy.component_flag_write_count -ne 48 -or
    $visualShadowPolicy.persisted_after_map_save_reload -ne $true -or
    $visualShadowPolicy.map_reload_api -ne 'LevelEditorSubsystem.load_level' -or
    $visualShadowPolicy.environment_actor_set_changed -ne $false -or
    $visualShadowPolicy.transforms_modified -ne $false -or
    $visualShadowPolicy.meshes_modified -ne $false -or
    $visualShadowPolicy.materials_modified -ne $false -or
    $visualShadowPolicy.collision_modified -ne $false -or
    $visualShadowPolicy.physics_authority_modified -ne $false
) {
    throw 'FactoryPark_HF visual shadow policy is missing or invalid'
}
$shadowComponentProperties = @($visualShadowPolicy.component_properties)
$rayTracingSupportedActorCount = [int](
    $visualShadowPolicy.property_supported_actor_counts.visible_in_ray_tracing
)
if (
    $shadowComponentProperties.Count -ne 3 -or
    $shadowComponentProperties -notcontains 'cast_shadow' -or
    $shadowComponentProperties -notcontains 'affect_distance_field_lighting' -or
    $shadowComponentProperties -notcontains 'visible_in_ray_tracing' -or
    $visualShadowPolicy.property_supported_actor_counts.cast_shadow -ne 48 -or
    $visualShadowPolicy.property_verified_actor_counts.cast_shadow -ne 48 -or
    $visualShadowPolicy.property_supported_actor_counts.affect_distance_field_lighting -ne 48 -or
    $visualShadowPolicy.property_verified_actor_counts.affect_distance_field_lighting -ne 48 -or
    $rayTracingSupportedActorCount -notin @(0, 48) -or
    $visualShadowPolicy.property_verified_actor_counts.visible_in_ray_tracing -ne
        $rayTracingSupportedActorCount -or
    $visualShadowPolicy.property_write_count -ne (96 + $rayTracingSupportedActorCount)
) {
    throw 'FactoryPark_HF visual shadow component-property coverage is invalid'
}
$shadowPolicyStableIds = @($visualShadowPolicy.stable_ids)
$shadowReadbackByStableId = @{}
foreach ($readback in @($visualShadowPolicy.readbacks)) {
    $stableId = [string]$readback.stable_id
    if (-not $stableId -or $shadowReadbackByStableId.ContainsKey($stableId)) {
        throw "FactoryPark_HF visual shadow policy repeats or omits stable ID $stableId"
    }
    if (
        $readback.cast_shadow_supported -ne $true -or
        $readback.expected_cast_shadow -ne $false -or
        $readback.actual_cast_shadow -ne $false -or
        $readback.affect_distance_field_lighting_supported -ne $true -or
        $readback.expected_affect_distance_field_lighting -ne $false -or
        $readback.actual_affect_distance_field_lighting -ne $false -or
        $readback.expected_visible_in_ray_tracing -ne $false -or
        $readback.visible_before -ne $true -or
        $readback.visible_after -ne $true -or
        $readback.verified -ne $true
    ) {
        throw "FactoryPark_HF visual shadow readback failed for $stableId"
    }
    if (
        $readback.visible_in_ray_tracing_supported -eq $true -and
        $readback.actual_visible_in_ray_tracing -ne $false
    ) {
        throw "FactoryPark_HF ray-tracing visibility readback failed for $stableId"
    }
    if (
        $readback.visible_in_ray_tracing_supported -eq $false -and
        $null -ne $readback.actual_visible_in_ray_tracing
    ) {
        throw "FactoryPark_HF unsupported ray-tracing property has an invalid readback for $stableId"
    }
    $shadowReadbackByStableId[$stableId] = $readback
}
if (
    $shadowPolicyStableIds.Count -ne $expectedDrainageReedIds.Count -or
    $shadowReadbackByStableId.Count -ne $expectedDrainageReedIds.Count
) {
    throw 'FactoryPark_HF visual shadow policy has the wrong stable-ID coverage'
}
foreach ($stableId in $expectedDrainageReedIds) {
    if (
        $shadowPolicyStableIds -notcontains $stableId -or
        -not $shadowReadbackByStableId.ContainsKey($stableId)
    ) {
        throw "FactoryPark_HF visual shadow policy omits drainage reed $stableId"
    }
}
if (
    $successJson.lighting.lumen.requested.global_illumination -ne 'Lumen' -or
    $successJson.lighting.lumen.requested.reflections -ne 'Lumen' -or
    $successJson.lighting.lumen.requested.shadow_method -ne 'VirtualShadowMaps' -or
    $successJson.lighting.lumen.project_config_lumen_gi -ne $true -or
    $successJson.lighting.lumen.project_config_lumen_reflections -ne $true -or
    @($successJson.lighting.movable_lights).Count -lt 3
) {
    throw 'FactoryPark_HF success sentinel does not prove Lumen-compatible movable lighting'
}
$acceptanceCameraIds = @($successJson.acceptance_cameras | ForEach-Object { [string]$_.id })
foreach ($requiredPreviewTarget in $requiredPreviewTargets) {
    if ($acceptanceCameraIds -notcontains $requiredPreviewTarget) {
        throw "FactoryPark_HF success sentinel omits acceptance camera $requiredPreviewTarget"
    }
}
$tankAcceptanceCamera = @($successJson.acceptance_cameras) |
    Where-Object { $_.id -eq 'tank_farm_inspection' } |
    Select-Object -First 1
if (
    $null -eq $tankAcceptanceCamera -or
    $tankAcceptanceCamera.adjustment -ne 'avoid_east_drainage_reed_raster_occlusion'
) {
    throw 'FactoryPark_HF success sentinel does not prove the tank camera remediation'
}
Assert-NumericVectorClose `
    -Actual $tankAcceptanceCamera.requested_position_m `
    -Expected ([double[]]@(102.0, -54.0, 1.6)) `
    -Context 'tank authoring camera position_m'
Assert-NumericVectorClose `
    -Actual $tankAcceptanceCamera.requested_look_at_m `
    -Expected ([double[]]@(65.0, -36.0, 3.5)) `
    -Context 'tank authoring camera look_at_m'
Assert-NumericVectorClose `
    -Actual $tankAcceptanceCamera.actual_position_m `
    -Expected ([double[]]@(35.0, -62.0, 4.2)) `
    -Context 'tank actual QA camera position_m'
Assert-NumericVectorClose `
    -Actual $tankAcceptanceCamera.actual_look_at_m `
    -Expected ([double[]]@(65.0, -36.0, 4.8)) `
    -Context 'tank actual QA camera look_at_m'
Assert-NumericVectorClose `
    -Actual $tankAcceptanceCamera.location_cm `
    -Expected ([double[]]@(3500.0, 6200.0, 420.0)) `
    -Context 'tank actual QA camera location_cm'
Assert-NumericVectorClose `
    -Actual $tankAcceptanceCamera.look_at_cm `
    -Expected ([double[]]@(6500.0, 3600.0, 480.0)) `
    -Context 'tank actual QA camera look_at_cm'
if (
    $successJson.visual_remediation.profile -ne 'tank_east_drainage_camera_v2' -or
    $successJson.visual_remediation.reason -ne 'avoid_east_drainage_reed_raster_occlusion' -or
    $successJson.visual_remediation.environment_actors_modified -ne $true -or
    $successJson.visual_remediation.environment_component_flags_modified -ne 48 -or
    $successJson.visual_remediation.environment_actor_set_changed -ne $false -or
    $successJson.visual_remediation.environment_transforms_modified -ne $false -or
    $successJson.visual_remediation.environment_meshes_modified -ne $false -or
    $successJson.visual_remediation.environment_materials_modified -ne $false -or
    $successJson.visual_remediation.environment_collision_modified -ne $false -or
    $successJson.visual_remediation.physics_authority_modified -ne $false -or
    $successJson.visual_remediation.visual_shadow_policy.profile -ne
        $visualShadowPolicy.profile -or
    $successJson.visual_remediation.visual_shadow_policy.actor_count -ne 48 -or
    $successJson.visual_remediation.visual_shadow_policy.expected_cast_shadow -ne $false -or
    $successJson.visual_remediation.visual_shadow_policy.persisted_after_map_save_reload -ne $true
) {
    throw 'FactoryPark_HF visual-remediation evidence is incomplete'
}
$lightingRemediation = $successJson.lighting.lighting_remediation
if (
    $lightingRemediation.label -ne 'FactoryPark_HF_TankBund_QAFill' -or
    $lightingRemediation.purpose -ne 'lift_tank_bund_wall_vsm_grazing_shadow_floor' -or
    $lightingRemediation.intensity -ne 25000.0 -or
    $lightingRemediation.attenuation_radius_cm -ne 6500.0 -or
    $lightingRemediation.source_width_cm -ne 3500.0 -or
    $lightingRemediation.source_height_cm -ne 1200.0 -or
    $lightingRemediation.cast_shadows -ne $false -or
    $lightingRemediation.environment_actor -ne $false -or
    $null -ne $lightingRemediation.stable_id -or
    $successJson.visual_remediation.lighting_remediation.label -ne
        $lightingRemediation.label
) {
    throw 'FactoryPark_HF tank-bund lighting remediation is incomplete'
}
Assert-NumericVectorClose `
    -Actual $lightingRemediation.location_cm `
    -Expected ([double[]]@(10000.0, 6200.0, 450.0)) `
    -Context 'tank-bund fill-light location_cm'
Assert-NumericVectorClose `
    -Actual $lightingRemediation.look_at_cm `
    -Expected ([double[]]@(6500.0, 5350.0, 70.0)) `
    -Context 'tank-bund fill-light look_at_cm'
$screenshotRefreshProperty = $successJson.PSObject.Properties['screenshot_refresh']
if ($null -ne $screenshotRefreshProperty) {
    $screenshotRefresh = $screenshotRefreshProperty.Value
    $renderingContractRefresh = (
        $screenshotRefresh.mode -eq 'rendering_contract_refresh_v1'
    )
    $refreshCollisionPolicy = $successJson.collision_persistence_policy
    $refreshCollisionChangeCount = (
        [int]$refreshCollisionPolicy.changed_from_enabled_count +
        [int]$refreshCollisionPolicy.changed_profile_count +
        [int]$refreshCollisionPolicy.changed_overlap_count
    )
    $refreshCollisionModified = $refreshCollisionChangeCount -gt 0
    $refreshSourcePath = Resolve-PathInsideRoot `
        -Path ([string]$screenshotRefresh.source_success_path) `
        -Base $resolvedEvidenceRoot `
        -AllowedRoot $resolvedEvidenceRoot `
        -Context 'screenshot refresh source success'
    $refreshSourceSha = (Get-FileHash -LiteralPath $refreshSourcePath -Algorithm SHA256).Hash.ToLowerInvariant()
    if (
        $screenshotRefresh.mode -notin @(
            'screenshot_refresh_v1',
            'rendering_contract_refresh_v1'
        ) -or
        $screenshotRefresh.environment_actors_modified -ne $true -or
        $screenshotRefresh.environment_component_flags_modified -ne 48 -or
        $screenshotRefresh.environment_actor_set_changed -ne $false -or
        $screenshotRefresh.environment_transforms_modified -ne $false -or
        $screenshotRefresh.environment_meshes_modified -ne $false -or
        $screenshotRefresh.environment_materials_modified -ne $renderingContractRefresh -or
        $screenshotRefresh.environment_material_assignments_modified -ne $false -or
        $screenshotRefresh.rendering_contract_refreshed -ne $renderingContractRefresh -or
        $screenshotRefresh.sun_and_exposure_modified -ne $renderingContractRefresh -or
        $screenshotRefresh.master_material_usage_modified -ne $renderingContractRefresh -or
        $screenshotRefresh.environment_collision_modified -ne $refreshCollisionModified -or
        $screenshotRefresh.environment_collision_components_modified -ne
            $refreshCollisionPolicy.component_modify_count -or
        $screenshotRefresh.physics_authority_modified -ne $false -or
        $screenshotRefresh.map_saved_and_reloaded -ne $true -or
        $screenshotRefresh.environment_actor_count_before -ne
            $successJson.environment.actor_count -or
        $screenshotRefresh.environment_actor_count_after -ne
            $successJson.environment.actor_count -or
        $screenshotRefresh.cameras_replaced -ne 4 -or
        $refreshSourceSha -ne
            ([string]$screenshotRefresh.source_success_sha256).ToLowerInvariant()
    ) {
        throw 'FactoryPark_HF refresh provenance is invalid'
    }
}
$screenshotsByTarget = @{}
if (@($successJson.screenshots).Count -ne $requiredPreviewTargets.Count) {
    throw 'FactoryPark_HF success sentinel must contain exactly three acceptance screenshots'
}
foreach ($screenshot in @($successJson.screenshots)) {
    $targetId = [string]$screenshot.target_id
    if (-not $targetId -or $screenshotsByTarget.ContainsKey($targetId)) {
        throw "FactoryPark_HF screenshot evidence repeats or omits target ID $targetId"
    }
    $screenshotPath = Resolve-PathInsideRoot `
        -Path ([string]$screenshot.path) `
        -Base $resolvedEvidenceRoot `
        -AllowedRoot $resolvedEvidenceRoot `
        -Context 'acceptance screenshot'
    if (
        $screenshot.status -ne 'captured' -or
        $screenshot.unsupported -ne $false -or
        [long]$screenshot.bytes -le 0 -or
        [string]$screenshot.sha256 -notmatch '^[0-9a-f]{64}$'
    ) {
        throw "FactoryPark_HF acceptance screenshot is not captured: $targetId"
    }
    if (-not (Test-Path -LiteralPath $screenshotPath -PathType Leaf)) {
        throw "FactoryPark_HF captured screenshot is missing: $screenshotPath"
    }
    $screenshotItem = Get-Item -LiteralPath $screenshotPath
    $screenshotSha = (Get-FileHash -LiteralPath $screenshotPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if (
        $screenshotItem.Length -le 0 -or
        [long]$screenshot.bytes -ne $screenshotItem.Length -or
        [string]$screenshot.sha256 -ne $screenshotSha
    ) {
        throw "FactoryPark_HF screenshot digest evidence is invalid: $targetId"
    }
    $screenshotsByTarget[$targetId] = $screenshot
}
foreach ($requiredPreviewTarget in $requiredPreviewTargets) {
    if (-not $screenshotsByTarget.ContainsKey($requiredPreviewTarget)) {
        throw "FactoryPark_HF success sentinel omits screenshot evidence $requiredPreviewTarget"
    }
}
if (
    $successJson.coordinate_conversion.fbx_import_uniform_scale_requested -ne 1.0 -or
    $successJson.coordinate_conversion.fbx_import_uniform_scale_supported -ne $false -or
    $successJson.coordinate_conversion.environment_actor_unit_scale -ne 100.0 -or
    $successJson.coordinate_conversion.terrain_actor_unit_scale -ne 1.0 -or
    $successJson.coordinate_conversion.unit_conversion_strategy -ne 'placement_actor_scale'
) {
    throw 'FactoryPark_HF success sentinel does not prove actor-scale metre-to-centimetre placement'
}
if (
    $successJson.blender_authoring.artifact_set_digest -ne
    $blenderManifestJson.artifact_set_digest
) {
    throw 'FactoryPark_HF success sentinel has the wrong Blender artifact_set_digest'
}
if (
    @($successJson.stable_actor_world_transforms).Count -ne
    $successJson.environment.actor_count
) {
    throw 'FactoryPark_HF success sentinel omits stable actor world transforms'
}
$expectedPlacements = @($blenderManifestJson.scene.layout_objects) +
    @($blenderManifestJson.scene.visual_only_objects)
if (@($successJson.stable_actor_world_transforms).Count -ne $expectedPlacements.Count) {
    throw 'FactoryPark_HF success sentinel placement count disagrees with the Blender manifest'
}
$expectedByStableId = @{}
foreach ($expected in $expectedPlacements) {
    $stableId = [string]$expected.stable_id
    if (-not $stableId -or $expectedByStableId.ContainsKey($stableId)) {
        throw "Blender manifest has an empty or duplicate placed stable ID: $stableId"
    }
    $expectedByStableId[$stableId] = $expected
}
$seenStableIds = @{}
$transformEvidenceByStableId = @{}
foreach ($actual in @($successJson.stable_actor_world_transforms)) {
    $stableId = [string]$actual.stable_id
    if (-not $expectedByStableId.ContainsKey($stableId)) {
        throw "FactoryPark_HF success sentinel has an unexpected stable actor: $stableId"
    }
    if ($seenStableIds.ContainsKey($stableId)) {
        throw "FactoryPark_HF success sentinel repeats stable actor: $stableId"
    }
    $seenStableIds[$stableId] = $true
    $transformEvidenceByStableId[$stableId] = $actual
    $expected = $expectedByStableId[$stableId]
    if ([string]$actual.mesh_name -ne [string]$expected.mesh_name) {
        throw "FactoryPark_HF stable actor $stableId has the wrong mesh_name"
    }
    if (-not $materialAssignmentByStableId.ContainsKey($stableId)) {
        throw "FactoryPark_HF stable actor $stableId has no native PBR assignment"
    }
    $materialAssignment = $materialAssignmentByStableId[$stableId]
    if (
        [string]$materialAssignment.source_material -ne [string]$expected.material -or
        [string]$materialAssignment.source_semantic_class -ne [string]$expected.semantic_class
    ) {
        throw "FactoryPark_HF native PBR assignment identity is wrong for $stableId"
    }
    $expectedPosition = [double[]]@($expected.position_m)
    $expectedLocation = ConvertTo-ComponentScaledNumericVector `
        -Values $expectedPosition `
        -Factors ([double[]]@(100.0, -100.0, 100.0)) `
        -Context "$stableId position_m"
    $expectedQuaternionWxyz = @(
        [double]$expected.quaternion_wxyz[0],
        [double]$expected.quaternion_wxyz[1],
        [double]$expected.quaternion_wxyz[2],
        [double]$expected.quaternion_wxyz[3]
    )
    $expectedQuaternionXyzw = @(
        -$expectedQuaternionWxyz[1],
        $expectedQuaternionWxyz[2],
        -$expectedQuaternionWxyz[3],
        $expectedQuaternionWxyz[0]
    )
    $expectedSourceScale = [double[]]@($expected.scale)
    $expectedActorScale = ConvertTo-ComponentScaledNumericVector `
        -Values $expectedSourceScale `
        -Factors ([double[]]@(100.0, 100.0, 100.0)) `
        -Context "$stableId scale"
    $expectedMeshDimensionsUu = [double[]]@($expected.dimensions_m)
    $expectedWorldDimensionsCm = ConvertTo-ComponentScaledNumericVector `
        -Values $expectedMeshDimensionsUu `
        -Factors ([double[]]@(100.0, 100.0, 100.0)) `
        -Context "$stableId dimensions_m"
    Assert-NumericVectorClose `
        -Actual $actual.source_position_m `
        -Expected $expectedPosition `
        -Context "$stableId source_position_m"
    Assert-NumericVectorClose `
        -Actual $actual.source_quaternion_wxyz `
        -Expected $expectedQuaternionWxyz `
        -Context "$stableId source_quaternion_wxyz"
    Assert-NumericVectorClose `
        -Actual $actual.location_cm `
        -Expected $expectedLocation `
        -Context "$stableId location_cm"
    Assert-NumericVectorClose `
        -Actual $actual.quaternion_xyzw `
        -Expected $expectedQuaternionXyzw `
        -Context "$stableId quaternion_xyzw"
    Assert-NumericVectorClose `
        -Actual $actual.source_scale `
        -Expected $expectedSourceScale `
        -Context "$stableId source_scale"
    Assert-NumericVectorClose `
        -Actual $actual.scale `
        -Expected $expectedActorScale `
        -Context "$stableId scale"
    Assert-NumericVectorClose `
        -Actual $actual.mesh_asset_dimensions_uu `
        -Expected $expectedMeshDimensionsUu `
        -Context "$stableId mesh_asset_dimensions_uu" `
        -Tolerance 0.01
    Assert-NumericVectorClose `
        -Actual $actual.expected_world_dimensions_cm `
        -Expected $expectedWorldDimensionsCm `
        -Context "$stableId expected_world_dimensions_cm" `
        -Tolerance 0.1
    Assert-NumericVectorClose `
        -Actual $actual.world_dimensions_cm `
        -Expected $expectedWorldDimensionsCm `
        -Context "$stableId world_dimensions_cm" `
        -Tolerance 0.1
}
if ($seenStableIds.Count -ne $expectedByStableId.Count) {
    throw 'FactoryPark_HF success sentinel does not contain every expected stable actor'
}
if ($materialAssignmentByStableId.Count -ne $expectedByStableId.Count) {
    throw 'FactoryPark_HF success sentinel has incomplete native PBR stable-ID coverage'
}
$collisionPersistencePolicy = $successJson.collision_persistence_policy
if (
    $collisionPersistencePolicy.profile -ne 'persisted_no_collision_v1' -or
    $collisionPersistencePolicy.expected_actor_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.actor_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.expected_collision_enabled -ne
        '<CollisionEnabled.NO_COLLISION: 0>' -or
    $collisionPersistencePolicy.expected_collision_profile_name -ne 'NoCollision' -or
    $collisionPersistencePolicy.expected_generate_overlap_events -ne $false -or
    $collisionPersistencePolicy.component_modify_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.component_modify_return_semantics -ne
        'transaction_participation_not_persistence_proof' -or
    (
        [int]$collisionPersistencePolicy.component_modify_return_true_count +
        [int]$collisionPersistencePolicy.component_modify_return_false_count +
        [int]$collisionPersistencePolicy.component_modify_return_none_count
    ) -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.collision_enabled_write_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.collision_profile_write_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.generate_overlap_events_write_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.readback_count -ne $expectedByStableId.Count -or
    $collisionPersistencePolicy.all_readbacks_verified -ne $true -or
    $collisionPersistencePolicy.persisted_after_map_save_reload -ne $true -or
    $collisionPersistencePolicy.environment_actor_set_changed -ne $false -or
    $collisionPersistencePolicy.transforms_modified -ne $false -or
    $collisionPersistencePolicy.meshes_modified -ne $false -or
    $collisionPersistencePolicy.materials_modified -ne $false -or
    $collisionPersistencePolicy.physics_authority_modified -ne $false -or
    @($collisionPersistencePolicy.stable_ids).Count -ne $expectedByStableId.Count -or
    @($collisionPersistencePolicy.readbacks).Count -ne $expectedByStableId.Count
) {
    throw 'FactoryPark_HF persisted no-collision policy is incomplete'
}
$collisionPolicyStableIds = @{}
foreach ($collisionReadback in @($collisionPersistencePolicy.readbacks)) {
    $stableId = [string]$collisionReadback.stable_id
    $after = $collisionReadback.after
    if (
        -not $expectedByStableId.ContainsKey($stableId) -or
        $collisionPolicyStableIds.ContainsKey($stableId) -or
        $after.verified -ne $true -or
        $after.collision_disabled -ne $true -or
        $after.collision_enabled -ne '<CollisionEnabled.NO_COLLISION: 0>' -or
        $after.collision_profile_name -ne 'NoCollision' -or
        $after.generate_overlap_events -ne $false
    ) {
        throw "FactoryPark_HF persisted collision readback failed for $stableId"
    }
    $collisionPolicyStableIds[$stableId] = $true
}
$loadedMapAudit = $successJson.loaded_map_audit
if (
    $loadedMapAudit.profile -ne 'post_save_reload_runtime_v1' -or
    $loadedMapAudit.map_path -ne '/Game/RobotSim/Maps/FactoryPark_HF' -or
    $loadedMapAudit.map_saved_and_reloaded -ne $true -or
    $loadedMapAudit.observation_source -ne 'loaded_unreal_world_runtime_readback' -or
    $loadedMapAudit.stable_id_set_exact -ne $true -or
    $loadedMapAudit.expected_stable_id_count -ne $expectedByStableId.Count -or
    $loadedMapAudit.observed_stable_id_count -ne $expectedByStableId.Count -or
    $loadedMapAudit.transform_readback_count -ne $expectedByStableId.Count -or
    $loadedMapAudit.transforms_verified -ne $true -or
    $loadedMapAudit.material_assignment_readback_count -ne $expectedByStableId.Count -or
    $loadedMapAudit.materials_verified -ne $true -or
    $loadedMapAudit.collision_disabled_count -ne $expectedByStableId.Count -or
    $loadedMapAudit.collision_all_disabled -ne $true -or
    $loadedMapAudit.terrain_actor_count -ne 1 -or
    $loadedMapAudit.preplaced_robot_bindings -ne 0 -or
    $loadedMapAudit.robot_actor_count -ne 0 -or
    $loadedMapAudit.robot_mesh_references -ne 0 -or
    @($loadedMapAudit.stable_ids).Count -ne $expectedByStableId.Count -or
    @($loadedMapAudit.records).Count -ne $expectedByStableId.Count
) {
    throw 'FactoryPark_HF loaded-map runtime audit is incomplete'
}
if ($null -ne $screenshotRefreshProperty) {
    if (
        $loadedMapAudit.expected_contract_source -ne 'previous_success_evidence' -or
        [string]$loadedMapAudit.source_success_path -ne
            [string]$screenshotRefresh.source_success_path -or
        [string]$loadedMapAudit.source_success_sha256 -ne
            [string]$screenshotRefresh.source_success_sha256
    ) {
        throw 'FactoryPark_HF loaded-map audit has invalid refresh-source provenance'
    }
} elseif ($loadedMapAudit.expected_contract_source -ne 'current_build_evidence') {
    throw 'FactoryPark_HF loaded-map audit has invalid build-source provenance'
}
$auditStableIds = @($loadedMapAudit.stable_ids)
$auditByStableId = @{}
foreach ($auditRecord in @($loadedMapAudit.records)) {
    $stableId = [string]$auditRecord.stable_id
    if (
        -not $expectedByStableId.ContainsKey($stableId) -or
        $auditByStableId.ContainsKey($stableId)
    ) {
        throw "FactoryPark_HF loaded-map audit repeats or invents StableId $stableId"
    }
    $expected = $expectedByStableId[$stableId]
    $expectedTransform = $transformEvidenceByStableId[$stableId]
    $expectedAssignment = $materialAssignmentByStableId[$stableId]
    if (
        $auditStableIds -notcontains $stableId -or
        [string]$auditRecord.semantic_class -ne [string]$expected.semantic_class -or
        $auditRecord.visual_only -ne [bool]$expected.visual_only -or
        [string]$auditRecord.mesh_name -ne [string]$expected.mesh_name -or
        -not [string]$auditRecord.mesh_asset -or
        -not [string]$auditRecord.component_path -or
        $auditRecord.collision_disabled -ne $true -or
        $auditRecord.collision_enabled -ne '<CollisionEnabled.NO_COLLISION: 0>' -or
        $auditRecord.collision_profile_name -ne 'NoCollision' -or
        $auditRecord.generate_overlap_events -ne $false -or
        $auditRecord.verified -ne $true
    ) {
        throw "FactoryPark_HF loaded-map identity/collision audit failed for $stableId"
    }
    Assert-NumericVectorClose `
        -Actual $auditRecord.location_cm `
        -Expected ([double[]]@($expectedTransform.location_cm)) `
        -Context "$stableId loaded-map location_cm" `
        -Tolerance 0.05
    Assert-QuaternionEquivalent `
        -Actual $auditRecord.quaternion_xyzw `
        -Expected ([double[]]@($expectedTransform.quaternion_xyzw)) `
        -Context "$stableId loaded-map quaternion_xyzw"
    Assert-NumericVectorClose `
        -Actual $auditRecord.scale `
        -Expected ([double[]]@($expectedTransform.scale)) `
        -Context "$stableId loaded-map scale" `
        -Tolerance 0.001
    $auditMaterialPaths = @($auditRecord.material_paths)
    if (
        $auditMaterialPaths.Count -ne [int]$expectedAssignment.component_slot_count -or
        $auditMaterialPaths.Count -le 0
    ) {
        throw "FactoryPark_HF loaded-map material slot count is wrong for $stableId"
    }
    foreach ($materialPath in $auditMaterialPaths) {
        if ([string]$materialPath -ne [string]$expectedAssignment.material_asset) {
            throw "FactoryPark_HF loaded-map material path is wrong for $stableId"
        }
    }
    $auditByStableId[$stableId] = $auditRecord
}
if ($auditByStableId.Count -ne $expectedByStableId.Count) {
    throw 'FactoryPark_HF loaded-map audit does not cover the exact StableId set'
}
if ($successJson.preplaced_robot_bindings -ne 0 -or $successJson.robot_actor_count -ne 0) {
    throw 'FactoryPark_HF success sentinel contains a preplaced robot'
}
if ($Unattended -and $null -ne $editorProcess -and -not $editorProcess.HasExited) {
    $null = $editorProcess.WaitForExit(60000)
}

if ($ValidateExistingEvidence) {
    Write-Output "ValidatedExistingEvidence=$successSentinel"
} else {
    Write-Output "UnrealEditor=$editorVersion"
    Write-Output "Blender=$blenderVersion"
}
Write-Output "Map=/Game/RobotSim/Maps/FactoryPark_HF"
Write-Output $successSentinel
