#if WITH_DEV_AUTOMATION_TESTS
#include "SLingTuSimShowroom.h"
#include "Engine/StaticMesh.h"
#include "HAL/FileManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FShowroomAssetsTest, "LingTuSim.UI.Runtime.Showroom.InstalledAssets",
  EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FShowroomAssetsTest::RunTest(const FString&) {
  const auto Plugin = IPluginManager::Get().FindPlugin(TEXT("LingTuSim"));
  if (!TestTrue(TEXT("LingTuSim plugin exists"), Plugin.IsValid())) return false;
  TArray<FString> Images;
  IFileManager::Get().FindFiles(Images, *FPaths::Combine(Plugin->GetBaseDir(), TEXT("Resources/Selection/robot-*.png")), true, false);
  TestTrue(TEXT("real geometry preview inventory is nonempty"), Images.Num() > 0);
  for (const auto& Image : Images) {
    const FString Id = FPaths::GetBaseFilename(Image).RightChop(6);
    auto* Mesh = LoadObject<UStaticMesh>(nullptr, *LingTuSim::UI::SLingTuSimShowroom::MeshPath(Id));
    if (!TestNotNull(*FString::Printf(TEXT("%s has a real showroom mesh, not PNG fallback"), *Id), Mesh)) continue;
    TestTrue(*FString::Printf(TEXT("%s has nonzero height"), *Id), Mesh->GetBoundingBox().GetSize().Z > 1);
    for (const auto& Slot : Mesh->GetStaticMaterials())
      TestNotNull(*FString::Printf(TEXT("%s has assigned material"), *Id), Slot.MaterialInterface.Get());
  }
  return true;
}
#endif
