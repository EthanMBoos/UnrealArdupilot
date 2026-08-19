#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "UvdAircraftComponent.h"

class FUnrealVehicleDynamicsModule final : public IModuleInterface {
 public:
  void StartupModule() override {
    double FixedDtS = 0.0;
    if (FParse::Value(FCommandLine::Get(), TEXT("UvdFixedDt="), FixedDtS) &&
        FMath::IsFinite(FixedDtS) && FixedDtS > 0.0) {
      UPhysicsSettings* Settings = UPhysicsSettings::Get();
      Settings->bTickPhysicsAsync = true;
      Settings->AsyncFixedTimeStepSize = static_cast<float>(FixedDtS);
      Settings->bSubstepping = false;
      Settings->bSubsteppingAsync = false;
    }
    WorldDelegate = FWorldDelegates::OnPostWorldInitialization.AddRaw(
        this, &FUnrealVehicleDynamicsModule::OnWorldInitialized);
  }

  void ShutdownModule() override {
    FWorldDelegates::OnPostWorldInitialization.Remove(WorldDelegate);
  }

 private:
  void OnWorldInitialized(UWorld* World,
                          const UWorld::InitializationValues) const {
    if (!World || !World->IsGameWorld()) {
      return;
    }
    FString RunPath;
    if (!FParse::Value(FCommandLine::Get(), TEXT("UvdRun="), RunPath)) {
      return;
    }
    AActor* Aircraft = World->SpawnActor<AActor>();
    UBoxComponent* Body =
        NewObject<UBoxComponent>(Aircraft, TEXT("AircraftPhysicsBody"));
    Body->SetBoxExtent(FVector{40.0, 6.0, 6.0});
    Body->SetCollisionProfileName(TEXT("PhysicsActor"));
    Aircraft->SetRootComponent(Body);
    Body->RegisterComponent();
    Body->SetSimulatePhysics(true);

    UCameraComponent* Camera =
        NewObject<UCameraComponent>(Aircraft, TEXT("ChaseCamera"));
    Camera->SetupAttachment(Body);
    Camera->SetRelativeLocation(FVector{-600.0, 0.0, 220.0});
    Camera->SetRelativeRotation(FRotator{-12.0, 0.0, 0.0});
    Camera->SetFieldOfView(70.0F);
    Camera->SetActive(true);
    Camera->RegisterComponent();

    UUvdAircraftComponent* Simulation =
        NewObject<UUvdAircraftComponent>(Aircraft, TEXT("VehicleDynamics"));
    Simulation->RunConfigPath = RunPath;
    Simulation->RegisterComponent();
  }

  FDelegateHandle WorldDelegate;
};

IMPLEMENT_MODULE(FUnrealVehicleDynamicsModule, UnrealVehicleDynamics)
