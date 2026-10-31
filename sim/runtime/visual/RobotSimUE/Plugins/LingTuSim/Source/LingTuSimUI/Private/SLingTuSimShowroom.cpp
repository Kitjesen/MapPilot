#include "SLingTuSimShowroom.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/RectLightComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureCube.h"
#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Math/PerspectiveMatrix.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "ImageUtils.h"
#include "ShaderCompiler.h"
#include "DistanceFieldAtlas.h"
#include "PreviewScene.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/SLeafWidget.h"
#include "Rendering/DrawElements.h"

namespace LingTuSim::UI {
namespace {
class SShowroomVeil final : public SLeafWidget {
 public:
  SLATE_BEGIN_ARGS(SShowroomVeil) {} SLATE_END_ARGS()
  void Construct(const FArguments&) { SetVisibility(EVisibility::HitTestInvisible); }
  virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(1920, 1080); }
  virtual int32 OnPaint(const FPaintArgs&, const FGeometry& Geometry, const FSlateRect&,
      FSlateWindowElementList& Elements, int32 Layer, const FWidgetStyle& Style, bool) const override {
    const float Width = Geometry.GetLocalSize().X;
    const FLinearColor Tint = Style.GetColorAndOpacityTint();
    TArray<FSlateGradientStop> Stops;
    Stops.Emplace(FVector2D(0, 0), FLinearColor(.93F, .94F, .95F, 1.F) * Tint);
    Stops.Emplace(FVector2D(Width * .28F, 0), FLinearColor(.93F, .94F, .95F, .98F) * Tint);
    Stops.Emplace(FVector2D(Width * .48F, 0), FLinearColor(.93F, .94F, .95F, .85F) * Tint);
    Stops.Emplace(FVector2D(Width * .64F, 0), FLinearColor(.93F, .94F, .95F, 0) * Tint);
    FSlateDrawElement::MakeGradient(Elements, Layer, Geometry.ToPaintGeometry(), Stops, Orient_Vertical);
    return Layer;
  }
};
}
TSharedRef<SWidget> MakeShowroomVeil() { return SNew(SShowroomVeil); }

class FRobotShowroomScene final : public FPreviewScene {
 public:
  explicit FRobotShowroomScene(EShowroomEnvironment InEnvironment) : FPreviewScene(ConstructionValues()
      // EditorPreview components fail ShouldRender() when launched with -game.
      .SetEditor(false).SetTransactional(false).SetCreatePhysicsScene(false)
      .ShouldSimulatePhysics(false).SetLightBrightness(1.1F).SetSkyBrightness(1.1F)
      .SetLightRotation(FRotator(-58, 120, 0))), Environment(InEnvironment) {
    Surface = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/RobotSim/FrontEnd/Showroom/Materials/M_Surface.M_Surface"));
    DisplayShell = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/RobotSim/FrontEnd/Showroom/Materials/M_DisplayShell.M_DisplayShell"));
    Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    SetSkyCubemap(LoadObject<UTextureCube>(nullptr, TEXT("/Engine/MapTemplates/Sky/DaylightAmbientCubemap.DaylightAmbientCubemap")));
    DirectionalLight->SetLightSourceAngle(2.0F);
    if (Environment == EShowroomEnvironment::OperationsBase) {
      DirectionalLight->SetIntensity(2.4F);
      DirectionalLight->SetLightColor(FLinearColor(1.F, .93F, .82F));
      DirectionalLight->SetWorldRotation(FRotator(-42, -35, 0));
      DirectionalLight->SetLightSourceAngle(3.5F);
    }
    // The stage is deliberately not part of the active simulation world's entity graph.
    BuildFloor();
    if (Environment == EShowroomEnvironment::OperationsBase) BuildOperationsBase();
    else BuildGallery();
    if (Environment == EShowroomEnvironment::OperationsBase) {
      auto* DaylightFill = NewObject<UDirectionalLightComponent>();
      DaylightFill->SetIntensity(1.25F);
      DaylightFill->SetLightColor(FLinearColor(.95F, .97F, 1.F));
      DaylightFill->SetCastShadows(false);
      AddComponent(DaylightFill, FTransform(FRotator(-38, -55, 0)));
    }
    auto* Sky = NewObject<UStaticMeshComponent>();
    Sky->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")));
    auto* SkyMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/RobotSim/FrontEnd/Showroom/Materials/M_Sky.M_Sky"));
    if (Environment == EShowroomEnvironment::OperationsBase && SkyMaterial) {
      auto* DaySky = UMaterialInstanceDynamic::Create(SkyMaterial, Sky);
      DaySky->SetVectorParameterValue(TEXT("HorizonColor"), FLinearColor(.73F, .82F, .92F));
      DaySky->SetVectorParameterValue(TEXT("ZenithColor"), FLinearColor(.40F, .61F, .88F));
      Sky->SetMaterial(0, DaySky);
    } else Sky->SetMaterial(0, SkyMaterial);
    Sky->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Sky->SetCastShadow(false);
    AddComponent(Sky, FTransform(FRotator::ZeroRotator, FVector::ZeroVector,
        FVector(Environment == EShowroomEnvironment::OperationsBase ? 500.F : 100.F)));

    auto* Fill = NewObject<URectLightComponent>();
    Fill->SetIntensity(Environment == EShowroomEnvironment::OperationsBase ? 19000.F : 14000.F);
    Fill->SetLightColor(FLinearColor(1.F, .97F, .92F));
    Fill->SetSourceWidth(Environment == EShowroomEnvironment::OperationsBase ? 500.F : 400.F);
    Fill->SetSourceHeight(Environment == EShowroomEnvironment::OperationsBase ? 500.F : 400.F);
    Fill->SetAttenuationRadius(1800.F);
    const FVector FillPosition(350, -150, 330);
    AddComponent(Fill, FTransform((FVector(0, 0, 80) - FillPosition).Rotation(), FillPosition));
    // A broad, low-intensity edge light separates the body from the pale gallery.
    auto* Edge = NewObject<URectLightComponent>();
    Edge->SetIntensity(6500.F);
    Edge->SetLightColor(Environment == EShowroomEnvironment::OperationsBase ? FLinearColor(.97F, .98F, 1.F) : FLinearColor(.90F, .95F, 1.F));
    Edge->SetSourceWidth(180.F);
    Edge->SetSourceHeight(240.F);
    Edge->SetAttenuationRadius(900.F);
    Edge->SetCastShadows(false);
    const FVector EdgePosition(-180, 210, 230);
    AddComponent(Edge, FTransform((FVector(0, 0, 90) - EdgePosition).Rotation(), EdgePosition));
    Robot = NewObject<UStaticMeshComponent>();
    Robot->SetMobility(EComponentMobility::Movable);
    Robot->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    AddComponent(Robot, FTransform::Identity);
    Capture = NewObject<USceneCaptureComponent2D>();
    Capture->bCaptureEveryFrame = false;
    Capture->bCaptureOnMovement = false;
    Capture->bAlwaysPersistRenderingState = true;
    Capture->ShowFlags.SetTemporalAA(true);
    Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    Capture->FOVAngle = 45.F;
    Capture->PostProcessBlendWeight = 1.F;
    auto& PP = Capture->PostProcessSettings;
    PP.AmbientCubemap = LoadObject<UTextureCube>(nullptr, TEXT("/Engine/MapTemplates/Sky/DaylightAmbientCubemap.DaylightAmbientCubemap"));
    PP.bOverride_AmbientCubemapIntensity = true;
    PP.AmbientCubemapIntensity = Environment == EShowroomEnvironment::OperationsBase ? 1.1F : .4F;
    // FPreviewScene does not track Lumen primitives by default. Do not inherit
    // the simulation world's Lumen GI in this isolated, on-demand studio.
    PP.bOverride_DynamicGlobalIlluminationMethod = true;
    PP.DynamicGlobalIlluminationMethod = EDynamicGlobalIlluminationMethod::None;
    PP.bOverride_ReflectionMethod = true;
    PP.ReflectionMethod = EReflectionMethod::ScreenSpace;
    PP.bOverride_ScreenSpaceReflectionQuality = true;
    PP.ScreenSpaceReflectionQuality = 100.F;
    PP.bOverride_AutoExposureMethod = true;
    PP.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
    PP.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    PP.AutoExposureApplyPhysicalCameraExposure = false;
    PP.bOverride_AutoExposureBias = true;
    PP.AutoExposureBias = 0.F;
    PP.bOverride_VignetteIntensity = true;
    PP.VignetteIntensity = 0.F;
    PP.bOverride_BloomIntensity = true;
    PP.BloomIntensity = .08F;
    PP.bOverride_MotionBlurAmount = true;
    PP.MotionBlurAmount = 0.F;
    PP.bOverride_AmbientOcclusionIntensity = true;
    PP.AmbientOcclusionIntensity = .25F;
    PP.bOverride_FilmGrainIntensity = true;
    PP.FilmGrainIntensity = 0.F;
    Capture->TextureTarget = NewObject<UTextureRenderTarget2D>();
    Capture->TextureTarget->RenderTargetFormat = RTF_RGBA8_SRGB;
    Capture->TextureTarget->ClearColor = FLinearColor(.74F, .80F, .87F);
    Capture->TextureTarget->InitAutoFormat(1920, 1080);
    Capture->TextureTarget->UpdateResourceImmediate(true);
    const FVector Camera(560, 0, 155);
    AddComponent(Capture, FTransform((FVector(0, 0, 92) - Camera).Rotation(), Camera));
    UpdateCaptureContents();
  }

  void Select(const FString& Id) {
    auto* Mesh = Id.IsEmpty() ? nullptr : LoadObject<UStaticMesh>(nullptr, *SLingTuSimShowroom::MeshPath(Id), nullptr, LOAD_NoWarn);
    Robot->EmptyOverrideMaterials();
    Robot->SetStaticMesh(Mesh);
    if (!Mesh) return;
    // Neutral coated-alloy display finish; leave rubber/sensors and original
    // source packages untouched. Clear these overrides when switching models.
    if (Surface) for (int32 Slot = 0; Slot < Mesh->GetStaticMaterials().Num(); ++Slot) {
      const FString Role = Mesh->GetStaticMaterials()[Slot].MaterialSlotName.ToString();
      const bool bGraphite = Environment == EShowroomEnvironment::OperationsBase && Role.StartsWith(TEXT("graphite"));
      const bool bRubber = Environment == EShowroomEnvironment::OperationsBase && Role.StartsWith(TEXT("rubber"));
      if (Role.StartsWith(TEXT("silver")) || bGraphite || bRubber) {
        // Imported CAD shells can be open sheets. Two-sided display shading
        // is local to the preview and does not claim repaired collision topology.
        auto* Finish = UMaterialInstanceDynamic::Create(DisplayShell ? DisplayShell : Surface, Robot);
        Finish->SetVectorParameterValue(TEXT("Tint"), bRubber ? FLinearColor(.024F, .027F, .031F)
            : bGraphite ? FLinearColor(.14F, .15F, .17F)
            : Environment == EShowroomEnvironment::OperationsBase ? FLinearColor(.50F, .51F, .52F) : FLinearColor(.45F, .49F, .53F));
        Finish->SetScalarParameterValue(TEXT("Roughness"), bRubber ? .78F : bGraphite ? .50F
            : Environment == EShowroomEnvironment::OperationsBase ? .38F : .34F);
        Robot->SetMaterial(Slot, Finish);
      }
    }
    const auto Bounds = Mesh->GetBoundingBox();
    const FVector Size = Bounds.GetSize();
    // Normalize framing, never physical dimensions or simulation assets.
    ModelScale = FMath::Min(165.0 / FMath::Max(1.0, Size.Z), 150.0 / FMath::Max(1.0, FMath::Max(Size.X, Size.Y)));
    const FVector Center = Bounds.GetCenter();
    ModelOffset = FVector(-Center.X, -Center.Y, -Bounds.Min.Z) * ModelScale;
    ModelSize = Size * ModelScale;
    UE_LOG(LogTemp, Display, TEXT("LINGTU_SHOWROOM_ROBOT id=%s mesh=%s height_cm=%.2f"), *Id, *Mesh->GetPathName(), Size.Z);
  }
  void ApplyView(const FShowroomViewState& View, FVector2D Size) {
    const int32 Width = 1920;
    const int32 Height = FMath::Clamp(FMath::RoundToInt(Width * Size.Y / Size.X), 640, 1920);
    if (Target()->SizeY != Height) Target()->ResizeTarget(Width, Height);
    const double Aspect = static_cast<double>(Width) / Height;
    const double HalfFov = FMath::DegreesToRadians(22.5);
    const double Radius = FVector2D(ModelSize.X, ModelSize.Y).Size() * .5;
    const double Fit = FMath::Max(ModelSize.Z * .5 * Aspect / (FMath::Tan(HalfFov) * .58),
        Radius / (FMath::Tan(HalfFov) * .32));
    const bool bBase = Environment == EShowroomEnvironment::OperationsBase;
    const double Distance = FMath::Max(Fit / View.Zoom, ModelSize.Size() * .55 + 20) * (bBase ? .98 : 1.0);
    const double Pitch = FMath::DegreesToRadians(bBase ? FMath::Max(5.0, View.PitchDegrees - 3.0) : View.PitchDegrees);
    const FVector Pivot(0, 0, ModelSize.Z * .5);
    const FVector Camera = Pivot + FVector(Distance * FMath::Cos(Pitch), 0, Distance * FMath::Sin(Pitch));
    Capture->SetWorldLocationAndRotation(Camera, (Pivot - Camera).Rotation());
    // Off-axis framing keeps the inspection pivot in the right column at every zoom.
    Capture->bUseCustomProjectionMatrix = true;
    Capture->CustomProjectionMatrix = FReversedZPerspectiveMatrix(HalfFov, HalfFov, 1.0, Aspect, 5.0, 5.0);
    Capture->CustomProjectionMatrix.M[2][0] = (bBase ? .32 : .52) + View.Pan.X * 2;
    Capture->CustomProjectionMatrix.M[2][1] = (bBase ? -.12 : .02) - View.Pan.Y * 2;
    const FQuat Rotation(FRotator(0, View.YawDegrees, 0));
    Robot->SetWorldTransform(FTransform(Rotation, Rotation.RotateVector(ModelOffset), FVector(ModelScale)));
  }
  void Render() {
    GetWorld()->Tick(LEVELTICK_ViewportsOnly, 1.F / 30.F);
    UpdateCaptureContents();
    GetWorld()->SendAllEndOfFrameUpdates();
    Capture->CaptureScene();
  }
  void SaveDiagnostic(const FString& Id) {
#if WITH_DEV_AUTOMATION_TESTS
    FString RunId;
    if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuDevFrontEndScreenshots="), RunId)) return;
    const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Automation/LingTuSimUI/FrontEnd"), RunId);
    IFileManager::Get().MakeDirectory(*Directory, true);
    TUniquePtr<FArchive> File(IFileManager::Get().CreateFileWriter(*FPaths::Combine(Directory, TEXT("showroom-") + Id + TEXT(".png"))));
    if (File) FImageUtils::ExportRenderTarget2DAsPNG(Target(), *File);
    UE_LOG(LogTemp, Display, TEXT("LINGTU_SHOWROOM_CAPTURE world=%s visible=%d camera=%s robot=%s"),
      *GetWorld()->GetName(), Capture->IsVisible(), *Capture->GetComponentLocation().ToString(), *Robot->GetComponentLocation().ToString());
#endif
  }
  UTextureRenderTarget2D* Target() const { return Capture->TextureTarget; }
  bool HasRobot() const { return Robot->GetStaticMesh() != nullptr; }

 private:
  void BuildFloor() {
    // Reuse the project's authored world-aligned concrete maps, at low strength
    // for indoor stone. Instance overrides never mutate the source materials.
    auto* PBR = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/RobotSim/Lookdev/InspectionMaterials20261016/M_FactoryPark_IndustrialRealismV2.M_FactoryPark_IndustrialRealismV2"));
    auto* Stone = UMaterialInstanceDynamic::Create(PBR ? PBR : Surface, GetTransientPackage());
    Stone->SetVectorParameterValue(TEXT("BaseColor"), FLinearColor(.47F, .455F, .425F));
    Stone->SetScalarParameterValue(TEXT("Roughness"), .30F);
    Stone->SetScalarParameterValue(TEXT("Metallic"), 0.F);
    Stone->SetScalarParameterValue(TEXT("DetailEnabled"), .22F);
    Stone->SetVectorParameterValue(TEXT("DetailTileSizeCm"), FLinearColor(120, 120, 120));
    Stone->SetTextureParameterValue(TEXT("DetailRoughnessTexture"), LoadObject<UTexture2D>(nullptr,
        TEXT("/Game/RobotSim/Lookdev/InspectionMaterials20261016/Textures/T_FP_ConcreteWeathered_Detail.T_FP_ConcreteWeathered_Detail")));
    Stone->SetTextureParameterValue(TEXT("DetailNormalTexture"), LoadObject<UTexture2D>(nullptr,
        TEXT("/Game/RobotSim/Lookdev/InspectionMaterials20261016/Textures/T_FP_ConcreteWeathered_Normal.T_FP_ConcreteWeathered_Normal")));
    Block(FVector(-300, 0, -5), FVector(36, 30, .09), FLinearColor(.34F, .34F, .32F), .7F);
    auto* Pavers = NewObject<UInstancedStaticMeshComponent>();
    Pavers->SetStaticMesh(Cube);
    Pavers->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Pavers->SetCanEverAffectNavigation(false);
    Pavers->SetMaterial(0, Stone);
    AddComponent(Pavers, FTransform::Identity);
    auto* AlternateStone = UMaterialInstanceDynamic::Create(PBR ? PBR : Surface, GetTransientPackage());
    AlternateStone->CopyParameterOverrides(Stone);
    AlternateStone->SetVectorParameterValue(TEXT("BaseColor"), FLinearColor(.455F, .44F, .41F));
    auto* AlternatePavers = NewObject<UInstancedStaticMeshComponent>();
    AlternatePavers->SetStaticMesh(Cube);
    AlternatePavers->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    AlternatePavers->SetCanEverAffectNavigation(false);
    AlternatePavers->SetMaterial(0, AlternateStone);
    AddComponent(AlternatePavers, FTransform::Identity);
    for (int32 X = -7; X <= 4; ++X) for (int32 Y = -5; Y <= 5; ++Y) {
      // Restrained slab variation, deterministic across preview reloads.
      auto* Batch = ((X + 7) * 31 + (Y + 5) * 17) % 7 < 2 ? AlternatePavers : Pavers;
      Batch->AddInstance(FTransform(FRotator::ZeroRotator, FVector(X * 240, Y * 240, -2), FVector(2.397, 2.397, .04)));
    }

    UE_LOG(LogTemp, Display, TEXT("LINGTU_SHOWROOM_FLOOR pbr=%d instances=%d"),
        PBR != nullptr, Pavers->GetInstanceCount() + AlternatePavers->GetInstanceCount());
  }
  void BuildGallery() {
    // A recessed gallery/courtyard, rather than one flat slab behind the robot.
    const FLinearColor Plaster(.72F, .70F, .65F);
    for (int32 Panel = 0; Panel < 5; ++Panel)
      Block(FVector(-950, 270 + Panel * 150, 215), FVector(.35, 1.495, 4.3), Plaster, .82F);
    Block(FVector(-970, -280, 215), FVector(.42, .26, 4.3), Plaster, .82F);
    Block(FVector(-970, -20, 415), FVector(.50, 5.6, .24), Plaster, .82F);
    Block(FVector(-930, 205, 215), FVector(.34, .20, 4.3), Plaster, .82F);
    // Keep slender reveals at the bay edge, away from the robot's silhouette.
    const FLinearColor Reveal(.48F, .48F, .45F);
    Block(FVector(-988, -260, 208), FVector(.07, .018, 4.0), Reveal, .55F);
    Block(FVector(-988, -20, 405), FVector(.07, 4.82, .018), Reveal, .55F);

    // Distant warm facade and fine vertical timber battens establish depth.
    Block(FVector(-1350, 0, 230), FVector(.3, 15, 4.6), FLinearColor(.75F, .72F, .65F), .9F);
    for (int32 Slat = 0; Slat < 8; ++Slat)
      Block(FVector(-1330, -320 + Slat * 12, 155), FVector(.05, .035, 3.1),
          FLinearColor(.40F, .31F, .21F), .66F);
    auto* GalleryFill = NewObject<URectLightComponent>();
    GalleryFill->SetIntensity(32000.F);
    GalleryFill->SetLightColor(FLinearColor(1.F, .96F, .88F));
    GalleryFill->SetSourceWidth(500.F);
    GalleryFill->SetSourceHeight(300.F);
    GalleryFill->SetAttenuationRadius(1200.F);
    GalleryFill->SetCastShadows(false);
    const FVector GalleryLightPosition(-850, 80, 300);
    AddComponent(GalleryFill, FTransform((FVector(-1350, -40, 180) - GalleryLightPosition).Rotation(), GalleryLightPosition));

    // A low timber bench provides scale without competing with the robot.
    const FLinearColor Timber(.31F, .23F, .15F);
    for (int32 Slat = 0; Slat < 5; ++Slat)
      Block(FVector(-1066 + Slat * 8, -235, 43), FVector(.07, 1.5, .035), Timber, .55F);
    for (double Y : {-290.0, -180.0})
      Block(FVector(-1050, Y, 20), FVector(.32, .055, .4), FLinearColor(.18F, .19F, .19F), .5F);
  }
  void BuildOperationsBase() {
    const FLinearColor Plaster(.78F, .76F, .70F);
    const FLinearColor White(.84F, .83F, .79F);
    const FLinearColor Metal(.27F, .29F, .30F);
    // Receded architecture leaves the centre open; the menu has its own soft veil.
    Block(FVector(-1100, 940, 250), FVector(.35, 10.4, 5), Plaster, .82F);
    Block(FVector(-1080, 410, 250), FVector(.65, .32, 5), White, .68F);
    Block(FVector(-1060, -250, 480), FVector(.8, 13.6, .20), White, .7F);
    Block(FVector(-920, -880, 250), FVector(.70, .38, 5.3), White, .7F, FRotator(0, 0, 12));
    Block(FVector(-1090, -230, 7), FVector(.45, 12.4, .12), Plaster, .76F);
    // Narrow recessed reveals read as joints, rather than dark rails across the UI.
    for (double Y : {690.0, 1010.0})
      Block(FVector(-1081, Y, 250), FVector(.008, .007, 4.9), FLinearColor(.60F, .59F, .55F), .9F);
    Block(FVector(-1080, 940, 7), FVector(.04, 10.4, .025), FLinearColor(.55F, .54F, .50F), .8F);
    // Warm indirect wall wash, confined to this presentation world.
    auto* WallWash = NewObject<URectLightComponent>();
    WallWash->SetIntensity(28000.F);
    WallWash->SetLightColor(FLinearColor(1.F, .96F, .88F));
    WallWash->SetSourceWidth(650.F);
    WallWash->SetSourceHeight(350.F);
    WallWash->SetAttenuationRadius(1700.F);
    WallWash->SetCastShadows(false);
    const FVector WallLight(-440, 650, 310);
    AddComponent(WallWash, FTransform((FVector(-1100, 800, 210) - WallLight).Rotation(), WallLight));

    // Slatted bench and slim supports, behind the inspection footprint.
    for (int32 Slat = 0; Slat < 6; ++Slat)
      Block(FVector(-780 + Slat * 13, -500, 45), FVector(.11, 2.8, .055),
          FLinearColor(.36F + Slat % 2 * .025F, .25F, .15F), .56F);
    for (double Y : {-600.0, -400.0})
      Block(FVector(-745, Y, 21), FVector(.58, .045, .42), Metal, .55F);

    // Courtyard paving and a low, open research pavilion establish a middle distance.
    Block(FVector(-1650, -250, -12), FVector(14, 30, .14), FLinearColor(.50F, .49F, .43F), .80F);
    for (double X : {-1250.0, -1550.0, -1850.0})
      Block(FVector(X, -250, -4.8), FVector(.01, 27, .006), FLinearColor(.40F, .40F, .36F), .9F);
    Block(FVector(-2300, 70, 215), FVector(4.8, 11, .16), White, .72F);
    Block(FVector(-2350, 70, 104), FVector(.12, 10.5, 2.05), FLinearColor(.49F, .53F, .53F), .46F);
    for (double Y : {-420.0, 560.0})
      Block(FVector(-2100, Y, 100), FVector(.12, .12, 2), Plaster, .78F);
    // Slender timber screens give the pavilion scale without obscuring the landscape.
    for (int32 Fin = 0; Fin < 12; ++Fin)
      Block(FVector(-2325, 250 + Fin * 25, 104), FVector(.03, .035, 1.9), FLinearColor(.32F, .25F, .17F), .7F);

    Block(FVector(-4700, 0, -100), FVector(80, 200, .1), FLinearColor(.30F, .38F, .28F), .95F);
    auto* Terrain = LoadObject<UStaticMesh>(nullptr, TEXT("/Game/RobotSim/FrontEnd/Showroom/Environment/SM_BaseLandscape.SM_BaseLandscape"));
    PlaceScenery(Terrain, FVector(-6700, -1000, -100), FVector(4200, 14000, 520), FLinearColor(.49F, .61F, .69F));
    PlaceScenery(Terrain, FVector(-4400, -400, -100), FVector(3200, 8500, 240), FLinearColor(.31F, .43F, .32F));
    UE_LOG(LogTemp, Display, TEXT("LINGTU_WELCOME_BASE landscape=%d physics=false"), Terrain != nullptr);
  }
  void PlaceScenery(UStaticMesh* Mesh, FVector Position, FVector Size, TOptional<FLinearColor> Tint) {
    if (!Mesh) return;
    const FBox Bounds = Mesh->GetBoundingBox();
    const FVector Extent = Bounds.GetSize().ComponentMax(FVector(1));
    const FVector Scale = Size / Extent;
    auto* Component = NewObject<UStaticMeshComponent>();
    Component->SetStaticMesh(Mesh);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetCanEverAffectNavigation(false);
    if (Surface && Tint.IsSet()) {
      auto* Material = UMaterialInstanceDynamic::Create(Surface, Component);
      Material->SetVectorParameterValue(TEXT("Tint"), Tint.GetValue());
      Material->SetScalarParameterValue(TEXT("Roughness"), .95F);
      for (int32 Slot = 0; Slot < Mesh->GetStaticMaterials().Num(); ++Slot) Component->SetMaterial(Slot, Material);
    }
    const FVector Offset(-Bounds.GetCenter().X, -Bounds.GetCenter().Y, -Bounds.Min.Z);
    AddComponent(Component, FTransform(FRotator::ZeroRotator, Position + Offset * Scale, Scale));
  }
  void Block(FVector Location, FVector Scale, FLinearColor Color, float Roughness, FRotator Rotation = FRotator::ZeroRotator) {
    auto* Component = NewObject<UStaticMeshComponent>();
    Component->SetStaticMesh(Cube);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetCanEverAffectNavigation(false);
    if (Surface) {
      auto* Material = UMaterialInstanceDynamic::Create(Surface, Component);
      Material->SetVectorParameterValue(TEXT("Tint"), Color);
      Material->SetScalarParameterValue(TEXT("Roughness"), Roughness);
      Component->SetMaterial(0, Material);
    }
    AddComponent(Component, FTransform(Rotation, Location, Scale));
  }
  UMaterialInterface* Surface = nullptr;
  UMaterialInterface* DisplayShell = nullptr;
  EShowroomEnvironment Environment;
  UStaticMesh* Cube = nullptr;
  UStaticMeshComponent* Robot = nullptr;
  USceneCaptureComponent2D* Capture = nullptr;
  double ModelScale = 1;
  FVector ModelOffset = FVector::ZeroVector;
  FVector ModelSize = FVector(80, 80, 165);
};

FString SLingTuSimShowroom::MeshPath(const FString& Id) {
  // Catalog IDs are identifiers, never arbitrary object paths.
  for (TCHAR C : Id) if (!FChar::IsAlnum(C) && C != TEXT('_')) return FString();
  return Id.IsEmpty() ? FString() : FString::Printf(TEXT("/Game/RobotSim/FrontEnd/Showroom/Robots/SM_%s.SM_%s"), *Id, *Id);
}
SLingTuSimShowroom::SLingTuSimShowroom() = default;
void SLingTuSimShowroom::Construct(const FArguments& Args) {
  RobotId = Args._RobotId;
  Environment = Args._Environment;
  RenderBrush.ImageSize = FVector2D(1920, 1080);
  RenderBrush.DrawAs = ESlateBrushDrawType::Image;
  ChildSlot[SNew(SImage).Image(&RenderBrush)];
}
SLingTuSimShowroom::~SLingTuSimShowroom() {
  RenderBrush.SetResourceObject(nullptr);
  Scene.Reset();
}
bool SLingTuSimShowroom::HasRobot() const { return Scene && LoadedId == RobotId.Get() && Scene->HasRobot(); }
void SLingTuSimShowroom::ViewChanged() {
  if (!HasRobot()) return;
  Scene->ApplyView(View, RenderSize);
  WarmupFrames = FMath::Max(WarmupFrames, 8);
}
void SLingTuSimShowroom::Orbit(FVector2D Delta) { View.Orbit(Delta); ViewChanged(); }
void SLingTuSimShowroom::Pan(FVector2D Delta) { View.Translate(Delta); ViewChanged(); }
void SLingTuSimShowroom::Zoom(double Steps) { View.Dolly(Steps); ViewChanged(); }
void SLingTuSimShowroom::SetView(EShowroomView Preset) { View.SetPreset(Preset); ViewChanged(); }
void SLingTuSimShowroom::Tick(const FGeometry& Geometry, double Time, float Delta) {
  SCompoundWidget::Tick(Geometry, Time, Delta);
  if (!FApp::CanEverRender() || Geometry.GetLocalSize().X < 1 || Geometry.GetLocalSize().Y < 1) return;
  if (!Scene) {
    Scene = MakeUnique<FRobotShowroomScene>(Environment);
    RenderBrush.SetResourceObject(Scene->Target());
    WarmupFrames = 40;
    Scene->Select(RobotId.Get());
    LoadedId = RobotId.Get();
    RenderSize = Geometry.GetLocalSize();
    Scene->ApplyView(View, RenderSize);
  }
  if (LoadedId != RobotId.Get()) {
    LoadedId = RobotId.Get();
    Scene->Select(LoadedId);
    View.Reset();
    Scene->ApplyView(View, Geometry.GetLocalSize());
    WarmupFrames = 24;
  }
  if (!RenderSize.Equals(Geometry.GetLocalSize(), 1.0)) {
    RenderSize = Geometry.GetLocalSize();
    Scene->ApplyView(View, RenderSize);
    WarmupFrames = FMath::Max(WarmupFrames, 12);
  }
  // Capture during interaction/settling only; idle menus do not render a second camera.
  if (WarmupFrames > 0) {
#if WITH_EDITOR
    // In Editor -game, late-loaded preview materials also need their completed
    // shader maps finalized on the game thread before freezing the capture.
    if (GShaderCompilingManager) GShaderCompilingManager->ProcessAsyncResults(true, false);
    if (GDistanceFieldAsyncQueue) GDistanceFieldAsyncQueue->ProcessAsyncTasks();
#endif
    Scene->Render();
#if WITH_EDITOR
    if (GShaderCompilingManager && GShaderCompilingManager->IsCompiling()) return;
    if (GDistanceFieldAsyncQueue && GDistanceFieldAsyncQueue->GetNumOutstandingTasks() > 0) return;
#endif
    if (--WarmupFrames == 0) Scene->SaveDiagnostic(LoadedId);
  }
}
}
