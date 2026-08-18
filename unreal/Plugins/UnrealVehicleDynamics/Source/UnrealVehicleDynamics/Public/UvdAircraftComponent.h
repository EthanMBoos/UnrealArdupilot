#pragma once

#include "Components/ActorComponent.h"
#include "UvdAircraftComponent.generated.h"

class UPrimitiveComponent;
struct FUvdAircraftRuntime;

UCLASS(ClassGroup = (UnrealVehicleDynamics),
       meta = (BlueprintSpawnableComponent))
class UNREALVEHICLEDYNAMICS_API UUvdAircraftComponent : public UActorComponent {
  GENERATED_BODY()

 public:
  UUvdAircraftComponent();
  virtual ~UUvdAircraftComponent() override;

  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
  virtual void TickComponent(
      float DeltaTime, ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction) override;
  virtual void AsyncPhysicsTickComponent(float DeltaTime,
                                         float SimTime) override;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Configuration")
  FString AircraftConfigPath = TEXT("../examples/aircraft/aerosonde.json");

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Configuration")
  FString RunConfigPath;

  UPROPERTY(Transient)
  FString BundlePath;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Environment")
  double OriginAltitudeMslM = 1387.0;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Environment")
  FVector WindNedMps = FVector::ZeroVector;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Controls",
            meta = (ClampMin = "-1", ClampMax = "1"))
  double Aileron = 0.0;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Controls",
            meta = (ClampMin = "-1", ClampMax = "1"))
  double Elevator = -0.08;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Controls",
            meta = (ClampMin = "-1", ClampMax = "1"))
  double Rudder = 0.0;

  UPROPERTY(EditAnywhere, Category = "UnrealVehicleDynamics|Controls",
            meta = (ClampMin = "0", ClampMax = "1"))
  double Throttle = 0.55;

 private:
  bool LoadRun();
  bool LoadAircraft();
  bool ConfigureBody();
  void FinishRun();
  void SetInitialState();

  UPROPERTY(Transient)
  TObjectPtr<UPrimitiveComponent> UpdatedPrimitive;

  FUvdAircraftRuntime* Runtime = nullptr;
};
