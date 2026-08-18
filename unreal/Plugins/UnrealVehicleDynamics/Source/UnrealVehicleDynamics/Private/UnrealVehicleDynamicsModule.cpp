#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "UvdAircraftComponent.h"

class FUnrealVehicleDynamicsModule final : public IModuleInterface {
 public:
  void StartupModule() override {
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
    FString BundlePath;
    FParse::Value(FCommandLine::Get(), TEXT("UvdBundle="), BundlePath);

    AActor* Aircraft = World->SpawnActor<AActor>();
    UStaticMeshComponent* Body =
        NewObject<UStaticMeshComponent>(Aircraft, TEXT("AircraftBody"));
    UStaticMesh* Cube =
        LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    Body->SetStaticMesh(Cube);
    Body->SetWorldScale3D(FVector{0.8, 0.12, 0.12});
    Body->SetCollisionProfileName(TEXT("PhysicsActor"));
    Aircraft->SetRootComponent(Body);
    Body->RegisterComponent();
    Body->SetSimulatePhysics(true);

    const auto AddVisualPart = [Aircraft, Body, Cube](const TCHAR* Name,
                                                      const FVector& Location,
                                                      const FVector& Scale) {
      UStaticMeshComponent* Part =
          NewObject<UStaticMeshComponent>(Aircraft, Name);
      Part->SetStaticMesh(Cube);
      Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
      Part->SetupAttachment(Body);
      Part->SetRelativeLocation(Location);
      Part->SetRelativeScale3D(Scale);
      Part->RegisterComponent();
    };
    AddVisualPart(TEXT("MainWing"), FVector{-5.0, 0.0, 0.0},
                  FVector{0.22, 1.25, 0.035});
    AddVisualPart(TEXT("TailPlane"), FVector{-65.0, 0.0, 0.0},
                  FVector{0.15, 0.45, 0.025});
    AddVisualPart(TEXT("VerticalTail"), FVector{-65.0, 0.0, 18.0},
                  FVector{0.15, 0.025, 0.22});

    UCameraComponent* Camera =
        NewObject<UCameraComponent>(Aircraft, TEXT("ChaseCamera"));
    Camera->SetupAttachment(Body);
    Camera->SetRelativeLocation(FVector{-600.0, 0.0, 220.0});
    Camera->SetRelativeRotation(FRotator{-12.0, 0.0, 0.0});
    Camera->SetFieldOfView(70.0F);
    Camera->SetActive(true);
    Camera->RegisterComponent();

    if (APlayerController* Controller = World->GetFirstPlayerController()) {
      Controller->SetViewTarget(Aircraft);
    }

    UUvdAircraftComponent* Simulation =
        NewObject<UUvdAircraftComponent>(Aircraft, TEXT("VehicleDynamics"));
    Simulation->RunConfigPath = RunPath;
    Simulation->BundlePath = BundlePath;
    Simulation->RegisterComponent();
  }

  FDelegateHandle WorldDelegate;
};

IMPLEMENT_MODULE(FUnrealVehicleDynamicsModule, UnrealVehicleDynamics)
